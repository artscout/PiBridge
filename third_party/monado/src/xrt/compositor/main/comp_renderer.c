// Copyright 2019-2026, Collabora, Ltd.
// Copyright 2024-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Compositor rendering code.
 * @author Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @author Moshi Turner <moshiturner@protonmail.com>
 * @ingroup comp_main
 */

#include "render/render_interface.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_frame.h"
#include "xrt/xrt_compositor.h"
#include "xrt/xrt_results.h"

#include "os/os_time.h"

#include "math/m_api.h"
#include "math/m_matrix_2x2.h"
#include "math/m_space.h"

#include "util/u_misc.h"
#include "util/u_trace_marker.h"
#include "util/u_distortion_mesh.h"
#include "util/u_sink.h"
#include "util/u_var.h"
#include "util/u_frame_times_widget.h"
#include "util/u_debug.h"
#include "util/u_time.h"
#include "util/u_gpu_wedge.h"

#include "util/comp_render.h"
#include "util/comp_high_level_render.h"

#include "main/comp_frame.h"
#include "main/comp_mirror_to_debug_gui.h"
#include "main/comp_recorder.h"

#ifdef XRT_FEATURE_WINDOW_PEEK
#include "main/comp_window_peek.h"
#endif

#include "vk/vk_helpers.h"
#include "vk/vk_cmd.h"
#include "vk/vk_image_readback_to_xf_pool.h"
#include "vk/vk_submit_helpers.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <unistd.h>
#include <inttypes.h>
#include <signal.h>

DEBUG_GET_ONCE_LOG_OPTION(comp_frame_lag_level, "XRT_COMP_FRAME_LAG_LOG_AS_LEVEL", U_LOGGING_WARN)
/*!
 * Seconds the renderer waits for its GPU work before it gives up on the frame and stops submitting until the work
 * completes (0 = wait forever, upstream behaviour). A stuck present then cannot wedge the compositor thread and with
 * it the shutdown (analysis/hang-worn-debug.md).
 */
DEBUG_GET_ONCE_NUM_OPTION(comp_gpu_wait_timeout_s, "XRT_COMPOSITOR_GPU_WAIT_TIMEOUT_S", 3)
/*!
 * When a wait gives up and the frame's GPU work is done (so the present/flip is what never retired): give the
 * display back right away, without Vulkan (comp_target::release_display_wedged), so that a later SIGKILL of the
 * service cannot leave it granted to a dead process. The display then stays dark until the service is restarted.
 */
DEBUG_GET_ONCE_BOOL_OPTION(comp_stuck_flip_release, "XRT_COMPOSITOR_STUCK_FLIP_RELEASE", true)
/*!
 * After releasing the display on a stuck flip, also ask the service to shut down (as if it got SIGTERM).
 */
DEBUG_GET_ONCE_BOOL_OPTION(comp_stuck_flip_exit, "XRT_COMPOSITOR_STUCK_FLIP_EXIT", false)
#define LOG_FRAME_LAG(...) U_LOG_IFL(debug_get_log_option_comp_frame_lag_level(), u_log_get_global_level(), __VA_ARGS__)

/*
 *
 * Helper macros for adding semaphores to lists.
 *
 */

#define ADD_WAIT(LIST, SEMAPHORE, STAGE, IS_TIMELINE)                                                                  \
	do {                                                                                                           \
		VkSemaphore _semaphore = SEMAPHORE;                                                                    \
		if (_semaphore != VK_NULL_HANDLE) {                                                                    \
			if (IS_TIMELINE) {                                                                             \
				vk_semaphore_list_wait_add_timeline(&LIST, _semaphore, (uint64_t)frame_id, STAGE);     \
			} else {                                                                                       \
				vk_semaphore_list_wait_add_binary(&LIST, _semaphore, STAGE);                           \
			}                                                                                              \
		}                                                                                                      \
	} while (false)

#define ADD_SIGNAL(LIST, SEMAPHORE, IS_TIMELINE)                                                                       \
	do {                                                                                                           \
		VkSemaphore _semaphore = SEMAPHORE;                                                                    \
		if (_semaphore != VK_NULL_HANDLE) {                                                                    \
			if (IS_TIMELINE) {                                                                             \
				vk_semaphore_list_signal_add_timeline(&LIST, _semaphore, (uint64_t)frame_id);          \
			} else {                                                                                       \
				vk_semaphore_list_signal_add_binary(&LIST, _semaphore);                                \
			}                                                                                              \
		}                                                                                                      \
	} while (false)


/*
 *
 * Private struct(s).
 *
 */

/*!
 * What is the source of the FoV values used for the final image that the
 * compositor produces and is sent to the hardware (or software).
 */
enum comp_target_fov_source
{
	/*!
	 * The FoV values used for the final target is taken from the
	 * distortion information on the @ref xrt_hmd_parts struct.
	 */
	COMP_TARGET_FOV_SOURCE_DISTORTION,

	/*!
	 * The FoV values used for the final target is taken from the
	 * those returned from the device's get_views.
	 */
	COMP_TARGET_FOV_SOURCE_DEVICE_VIEWS,
};

/*!
 * Holds associated vulkan objects and state to render with a distortion.
 *
 * @ingroup comp_main
 */
struct comp_renderer
{
	//! @name Durable members
	//! @brief These don't require the images to be created and don't depend on it.
	//! @{

	//! The compositor we were created by
	struct comp_compositor *c;
	struct comp_settings *settings;

	struct comp_mirror_to_debug_gui mirror_to_debug_gui;

	//! pimax: recorder tap (NULL when PIMAX_RECORD=0), see comp_recorder.h.
	struct comp_recorder *recorder;

	//! @}

	//! @name Image-dependent members
	//! @{

	//! Index of the current buffer/image
	int32_t acquired_buffer;

	//! Which buffer was last submitted and has a fence pending.
	int32_t fenced_buffer;

	/*!
	 * The render pass used to render to the target, it depends on the
	 * target's format so will be recreated each time the target changes.
	 */
	struct render_gfx_render_pass target_render_pass;

	/*!
	 * Array of "rendering" target resources equal in size to the number of
	 * comp_target images. Each target resources holds all of the resources
	 * needed to render to that target and its views.
	 */
	struct render_gfx_target_resources *rtr_array;

	/*!
	 * Array of fences equal in size to the number of comp_target images.
	 */
	VkFence *fences;

	/*!
	 * The number of renderings/fences we've created: set from comp_target when we use that data.
	 */
	uint32_t buffer_count;

	/*!
	 * Only relevant when using present modes from VK_KHR_shared_presentable_image,
	 * tracks whether we have waited on the shared present semaphore at least once,
	 * subsequents present waits are redundant and can be skipped.
	 */
	bool shared_present_semaphore_wait_once;

	//! @}

	//! @name Bounded GPU waits (XRT_COMPOSITOR_GPU_WAIT_TIMEOUT_S)
	//! @{

	//! Signalled by an empty submit after the frame's work: replaces vkQueueWaitIdle.
	VkFence idle_fence;

	//! The last wait gave up: @ref stuck_fence is still pending, no new GPU work is submitted until it signals.
	bool gpu_stuck;
	VkFence stuck_fence;
	uint64_t stuck_since_ns;
	uint64_t stuck_skipped_frames;

	//! The display was given back after a stuck flip: never render or present again (emulated frames only).
	bool display_released;

	//! @}
};


/*
 *
 * Functions.
 *
 */

//! Short description of whether the GPU timestamps of the last frame are written (non-blocking).
static const char *
renderer_frame_gpu_state(struct comp_renderer *r)
{
	struct vk_bundle *vk = &r->c->base.vk;
	if (r->c->nr.query_pool == VK_NULL_HANDLE) {
		return "no timestamp query pool";
	}
	uint64_t ts[2] = {0};
	VkResult ret = vk->vkGetQueryPoolResults(vk->device, r->c->nr.query_pool, 0, 2, sizeof(ts), ts,
	                                         sizeof(uint64_t), VK_QUERY_RESULT_64_BIT);
	switch (ret) {
	case VK_SUCCESS: return "frame GPU work finished (timestamps written): stuck after it, in the present/flip";
	case VK_NOT_READY: return "frame GPU work NOT finished (timestamps missing): stuck in compositor GPU work";
	default: return vk_result_string(ret);
	}
}

//! True if the GPU timestamps of the last frame are written, i.e. its compositor work finished (non-blocking).
static bool
renderer_frame_gpu_work_done(struct comp_renderer *r)
{
	struct vk_bundle *vk = &r->c->base.vk;
	if (r->c->nr.query_pool == VK_NULL_HANDLE) {
		return false;
	}
	uint64_t ts[2] = {0};
	VkResult ret = vk->vkGetQueryPoolResults(vk->device, r->c->nr.query_pool, 0, 2, sizeof(ts), ts,
	                                         sizeof(uint64_t), VK_QUERY_RESULT_64_BIT);
	return ret == VK_SUCCESS;
}

/*!
 * A bounded wait just gave up. From now on nothing may wait for the GPU (u_gpu_wedge.h): the teardown leaks the
 * Vulkan objects and the service leaves with _exit(). If the frame's own GPU work is done, the flip of the presented
 * image is what never retires (analysis/nvidia-flip-hang.md): release the display while the process is alive.
 */
static void
renderer_on_gpu_stuck(struct comp_renderer *r)
{
	const bool flip_stuck = renderer_frame_gpu_work_done(r);

	u_gpu_wedge_set(flip_stuck ? "a presented image never finished flipping" : "compositor GPU work did not finish");

	if (!flip_stuck || r->display_released) {
		return;
	}

	if (!debug_get_bool_option_comp_stuck_flip_release()) {
		COMP_ERROR(r->c, "Stuck flip: not releasing the display (XRT_COMPOSITOR_STUCK_FLIP_RELEASE=0). Stop the "
		                 "service with SIGINT and let it exit by itself; a SIGKILL can wedge the display.");
		return;
	}

	if (comp_target_release_display_wedged(r->c->target)) {
		r->display_released = true;
	}

	if (debug_get_bool_option_comp_stuck_flip_exit()) {
		COMP_ERROR(r->c, "Stuck flip: shutting the service down (XRT_COMPOSITOR_STUCK_FLIP_EXIT).");
		raise(SIGTERM);
	}
}

/*!
 * While a wait has given up: true if the stuck fence is still pending (the caller must not submit or touch GPU
 * resources), false once it signalled (state cleared). Always true once the display was released.
 */
static bool
renderer_gpu_still_stuck(struct comp_renderer *r)
{
	if (!r->gpu_stuck) {
		return r->display_released;
	}
	struct vk_bundle *vk = &r->c->base.vk;
	VkResult ret = vk->vkGetFenceStatus(vk->device, r->stuck_fence);
	if (ret == VK_NOT_READY) {
		r->stuck_skipped_frames++;
		return true;
	}
	COMP_ERROR(r->c, "GPU work completed again after %.1f s (%s), %" PRIu64 " frames skipped%s.",
	           (double)(os_monotonic_get_ns() - r->stuck_since_ns) / 1e9, vk_result_string(ret),
	           r->stuck_skipped_frames,
	           r->display_released ? "; the display was released, not presenting again" : "; resuming");
	r->gpu_stuck = false;
	r->stuck_fence = VK_NULL_HANDLE;
	r->stuck_skipped_frames = 0;
	if (r->display_released) {
		// Keep the wedge: the target was not destroyed through Vulkan and must not be.
		return true;
	}
	u_gpu_wedge_clear();
	return false;
}

/*!
 * Wait for @p fence in 1 s slices, logging each slice, up to XRT_COMPOSITOR_GPU_WAIT_TIMEOUT_S. On give-up the fence is
 * remembered as stuck and false is returned; the caller must then not reset or reuse it.
 */
static bool
renderer_wait_fence_bounded(struct comp_renderer *r, VkFence fence, const char *what)
{
	struct vk_bundle *vk = &r->c->base.vk;
	const int64_t limit_s = debug_get_num_option_comp_gpu_wait_timeout_s();
	const uint64_t start_ns = os_monotonic_get_ns();

	for (int64_t s = 1;; s++) {
		VkResult ret = vk->vkWaitForFences(vk->device, 1, &fence, VK_TRUE, limit_s > 0 ? U_TIME_1S_IN_NS : UINT64_MAX);
		if (ret == VK_SUCCESS) {
			if (s > 1) {
				COMP_ERROR(r->c, "%s: GPU done after %.1f s.", what,
				           (double)(os_monotonic_get_ns() - start_ns) / 1e9);
			}
			return true;
		}
		if (ret != VK_TIMEOUT) {
			COMP_ERROR(r->c, "%s: vkWaitForFences: %s", what, vk_result_string(ret));
			return ret != VK_ERROR_DEVICE_LOST; // device lost: treat as stuck, never submit again
		}
		COMP_ERROR(r->c, "%s: GPU not done after %" PRId64 " s; %s.", what, s, renderer_frame_gpu_state(r));
		if (s >= limit_s) {
			COMP_ERROR(r->c,
			           "%s: giving up after %" PRId64 " s (XRT_COMPOSITOR_GPU_WAIT_TIMEOUT_S): no more GPU work is "
			           "submitted until it completes, so the compositor thread and shutdown are not wedged.",
			           what, s);
			r->gpu_stuck = true;
			r->stuck_fence = fence;
			r->stuck_since_ns = start_ns;
			r->stuck_skipped_frames = 0;
			renderer_on_gpu_stuck(r);
			return false;
		}
	}
}

/*!
 * Wait until all work submitted to the main queue so far is done. Upstream uses vkQueueWaitIdle here, which waits
 * forever (and with the queue lock held) when a present never retires; this submits an empty batch with a fence and
 * waits for that fence with a bound instead. Returns false if the wait gave up (or an earlier one is still stuck).
 */
static bool
renderer_wait_queue_idle(struct comp_renderer *r)
{
	COMP_TRACE_MARKER();
	struct vk_bundle *vk = &r->c->base.vk;

	if (renderer_gpu_still_stuck(r)) {
		return false;
	}

	if (debug_get_num_option_comp_gpu_wait_timeout_s() <= 0) {
		vk_queue_lock(vk->main_queue);
		vk->vkQueueWaitIdle(vk->main_queue->queue);
		vk_queue_unlock(vk->main_queue);
		return true;
	}

	VkResult ret;
	if (r->idle_fence == VK_NULL_HANDLE) {
		VkFenceCreateInfo fci = {.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
		ret = vk->vkCreateFence(vk->device, &fci, NULL, &r->idle_fence);
		if (ret != VK_SUCCESS) {
			r->idle_fence = VK_NULL_HANDLE;
		}
	} else {
		ret = vk->vkResetFences(vk->device, 1, &r->idle_fence);
	}

	if (ret == VK_SUCCESS) {
		vk_queue_lock(vk->main_queue);
		ret = vk->vkQueueSubmit(vk->main_queue->queue, 0, NULL, r->idle_fence);
		vk_queue_unlock(vk->main_queue);
	}
	if (ret != VK_SUCCESS) {
		COMP_ERROR(r->c, "bounded queue idle: %s, falling back to vkQueueWaitIdle", vk_result_string(ret));
		vk_queue_lock(vk->main_queue);
		vk->vkQueueWaitIdle(vk->main_queue->queue);
		vk_queue_unlock(vk->main_queue);
		return true;
	}

	if (!renderer_wait_fence_bounded(r, r->idle_fence, "queue idle")) {
		// The fence is still pending: it now belongs to the stuck state, never reset or destroy it while pending.
		r->idle_fence = VK_NULL_HANDLE;
		return false;
	}
	return true;
}

static void
calc_viewport_data(struct comp_renderer *r,
                   struct render_viewport_data out_viewport_data[XRT_MAX_VIEWS],
                   size_t view_count)
{
	struct comp_compositor *c = r->c;

	bool pre_rotate = false;
	if (r->c->target->surface_transform & VK_SURFACE_TRANSFORM_ROTATE_90_BIT_KHR ||
	    r->c->target->surface_transform & VK_SURFACE_TRANSFORM_ROTATE_270_BIT_KHR) {
		COMP_SPEW(c, "Swapping width and height, since we are pre rotating");
		pre_rotate = true;
	}

	int w_i32 = pre_rotate ? r->c->xdev->hmd->screens[0].h_pixels : r->c->xdev->hmd->screens[0].w_pixels;
	int h_i32 = pre_rotate ? r->c->xdev->hmd->screens[0].w_pixels : r->c->xdev->hmd->screens[0].h_pixels;

	float scale_x = (float)r->c->target->width / (float)w_i32;
	float scale_y = (float)r->c->target->height / (float)h_i32;

	for (uint32_t i = 0; i < view_count; ++i) {
		struct xrt_view *v = &r->c->xdev->hmd->views[i];
		if (pre_rotate) {
			out_viewport_data[i] = (struct render_viewport_data){
			    .x = (uint32_t)(v->viewport.y_pixels * scale_x),
			    .y = (uint32_t)(v->viewport.x_pixels * scale_y),
			    .w = (uint32_t)(v->viewport.h_pixels * scale_x),
			    .h = (uint32_t)(v->viewport.w_pixels * scale_y),
			};
		} else {
			out_viewport_data[i] = (struct render_viewport_data){
			    .x = (uint32_t)(v->viewport.x_pixels * scale_x),
			    .y = (uint32_t)(v->viewport.y_pixels * scale_y),
			    .w = (uint32_t)(v->viewport.w_pixels * scale_x),
			    .h = (uint32_t)(v->viewport.h_pixels * scale_y),
			};
		}
	}
}

static void
calc_vertex_rot_data(struct comp_renderer *r, struct xrt_matrix_2x2 out_vertex_rots[XRT_MAX_VIEWS], size_t view_count)
{
	bool pre_rotate = false;
	if (r->c->target->surface_transform & VK_SURFACE_TRANSFORM_ROTATE_90_BIT_KHR ||
	    r->c->target->surface_transform & VK_SURFACE_TRANSFORM_ROTATE_270_BIT_KHR) {
		COMP_SPEW(r->c, "Swapping width and height, since we are pre rotating");
		pre_rotate = true;
	}

	const struct xrt_matrix_2x2 rotation_90_cw = {{
	    .vecs =
	        {
	            {0, 1},
	            {-1, 0},
	        },
	}};

	for (uint32_t i = 0; i < view_count; i++) {
		// Get the view.
		struct xrt_view *v = &r->c->xdev->hmd->views[i];

		// Copy data.
		struct xrt_matrix_2x2 rot = v->rot;

		// Should we rotate.
		if (pre_rotate) {
			m_mat2x2_multiply(&rot, &rotation_90_cw, &rot);
		}

		out_vertex_rots[i] = rot;
	}
}

/*!
 * How long to spread the begin/end scanout poses over, in nanoseconds.
 *
 * This is not @ref xrt_device_compositor_info::scanout_time_ns: a globally
 * refreshing panel lights every pixel at once, so it gets 0 here even when it
 * reports a large scanout time. Returns 0 whenever no compensation applies,
 * which is not an error.
 */
static inline int64_t
calc_scanout_compensation_ns(struct comp_renderer *r)
{
	struct xrt_device *xdev = r->c->xdev;

	if (xdev == NULL || !xdev->supported.compositor_info) {
		return 0;
	}

	struct xrt_device_compositor_mode compositor_mode = {
	    .frame_interval_ns = r->c->frame_interval_ns,
	};
	struct xrt_device_compositor_info device_compositor_info = {0};
	xrt_result_t xret = xrt_device_get_compositor_info( //
	    xdev,                                           //
	    &compositor_mode,                               //
	    &device_compositor_info);                       //

	if (xret != XRT_SUCCESS) {
		COMP_WARN(r->c, "xrt_device_get_compositor_info failed, assuming 0 scanout time");
		return 0;
	}

	switch (device_compositor_info.panel_refresh_type) {
	case XRT_PANEL_REFRESH_TYPE_ROLLING:
		if (device_compositor_info.scanout_direction == XRT_SCANOUT_DIRECTION_TOP_TO_BOTTOM) {
			return device_compositor_info.scanout_time_ns;
		} else {
			COMP_SPEW(r->c,
			          "Unable to apply rolling scanout compensation, only "
			          "DIRECTION_TOP_TO_BOTTOM is supported");
			return 0;
		}
	case XRT_PANEL_REFRESH_TYPE_GLOBAL:
		/*!
		 * The entire panel refreshes at once, so there is no scanout
		 * spread to compensate for and the begin/end poses coincide.
		 */
		return 0;
	}

	//! No default case above, so a new refresh type warns at compile time.
	return 0;
}

static void
calc_pose_data(struct comp_renderer *r,
               enum comp_target_fov_source fov_source,
               struct xrt_fov out_fovs[XRT_MAX_VIEWS],
               struct xrt_pose out_world_scanout_begin[XRT_MAX_VIEWS],
               struct xrt_pose out_world_scanout_end[XRT_MAX_VIEWS],
               struct xrt_pose out_eye[XRT_MAX_VIEWS],
               uint32_t view_count)
{
	COMP_TRACE_MARKER();

	struct xrt_vec3 default_eye_relation = {
	    0.063000f, /*! @todo get actual ipd_meters */
	    0.0f,
	    0.0f,
	};

	struct xrt_space_relation head_relation[2] = XRT_SPACE_RELATION_ZERO;
	struct xrt_fov xdev_fovs[XRT_MAX_VIEWS] = XRT_STRUCT_INIT;
	struct xrt_pose xdev_poses[2][XRT_MAX_VIEWS] = XRT_STRUCT_INIT;

	// Determine view type based on view count
	enum xrt_view_type view_type = (view_count == 1) ? XRT_VIEW_TYPE_MONO : XRT_VIEW_TYPE_STEREO;

	const int64_t scanout_time_ns = calc_scanout_compensation_ns(r);

	int64_t begin_timestamp_ns = r->c->frame.rendering.predicted_display_time_ns;
	int64_t end_timestamp_ns = begin_timestamp_ns + scanout_time_ns;

	// Pose at beginning of scanout
	xrt_result_t xret = xrt_device_get_view_poses( //
	    r->c->xdev,                                //
	    &default_eye_relation,                     //
	    begin_timestamp_ns,                        // at_timestamp_ns
	    view_type,                                 //
	    view_count,                                //
	    &head_relation[0],                         // out_head_relation
	    xdev_fovs,                                 // out_fovs
	    xdev_poses[0]);                            //
	if (xret != XRT_SUCCESS) {
		struct u_pp_sink_stack_only sink;
		u_pp_delegate_t dg = u_pp_sink_stack_only_init(&sink);
		u_pp_xrt_result(dg, xret);
		U_LOG_E("xrt_device_get_view_poses failed: %s", sink.buffer);
		return;
	}

	// Pose at end of scanout
	if (scanout_time_ns != 0) {
		xret = xrt_device_get_view_poses( //
		    r->c->xdev,                   //
		    &default_eye_relation,        //
		    end_timestamp_ns,             // at_timestamp_ns
		    view_type,                    //
		    view_count,                   //
		    &head_relation[1],            // out_head_relation
		    xdev_fovs,                    // out_fovs
		    xdev_poses[1]);               // out_poses
		if (xret != XRT_SUCCESS) {
			struct u_pp_sink_stack_only sink;
			u_pp_delegate_t dg = u_pp_sink_stack_only_init(&sink);
			u_pp_xrt_result(dg, xret);
			U_LOG_E("xrt_device_get_view_poses failed: %s", sink.buffer);
			return;
		}
	} else {
		for (size_t i = 0; i < XRT_MAX_VIEWS; ++i) {
			xdev_poses[1][i] = xdev_poses[0][i];
		}
		head_relation[1] = head_relation[0];
	}

	struct xrt_fov dist_fov[XRT_MAX_VIEWS] = XRT_STRUCT_INIT;
	for (uint32_t i = 0; i < view_count; i++) {
		dist_fov[i] = r->c->xdev->hmd->distortion.fov[i];
	}

	bool use_xdev = false; // Probably what we want.

	switch (fov_source) {
	case COMP_TARGET_FOV_SOURCE_DISTORTION: use_xdev = false; break;
	case COMP_TARGET_FOV_SOURCE_DEVICE_VIEWS: use_xdev = true; break;
	}

	for (uint32_t i = 0; i < view_count; i++) {
		const struct xrt_fov fov = use_xdev ? xdev_fovs[i] : dist_fov[i];
		const struct xrt_pose eye_pose_scanout_start = xdev_poses[0][i];
		const struct xrt_pose eye_pose_scanout_end = xdev_poses[1][i];

		struct xrt_space_relation result_scanout_start = {0};
		struct xrt_space_relation result_scanout_end = {0};
		struct xrt_relation_chain xrc = {0};

		m_relation_chain_push_pose_if_not_identity(&xrc, &eye_pose_scanout_start);
		m_relation_chain_push_relation(&xrc, &head_relation[0]);
		m_relation_chain_resolve(&xrc, &result_scanout_start);

		xrc = (struct xrt_relation_chain){0};

		m_relation_chain_push_pose_if_not_identity(&xrc, &eye_pose_scanout_end);
		m_relation_chain_push_relation(&xrc, &head_relation[1]);
		m_relation_chain_resolve(&xrc, &result_scanout_end);

		// Results to callers.
		out_fovs[i] = fov;
		out_world_scanout_begin[i] = result_scanout_start.pose;
		out_world_scanout_end[i] = result_scanout_end.pose;
		out_eye[i] = eye_pose_scanout_start;

		// For remote rendering targets.
		r->c->base.frame_params.fovs[i] = fov;
		r->c->base.frame_params.poses[i] = result_scanout_start.pose;
	}
}

//! @pre comp_target_has_images(r->c->target)
static void
renderer_build_rendering_target_resources(struct comp_renderer *r,
                                          struct render_gfx_target_resources *rtr,
                                          uint32_t index)
{
	COMP_TRACE_MARKER();

	struct comp_compositor *c = r->c;

	VkImageView image_view = r->c->target->images[index].view;
	VkExtent2D extent = {r->c->target->width, r->c->target->height};

	render_gfx_target_resources_init( //
	    rtr,                          //
	    &c->nr,                       //
	    &r->target_render_pass,       //
	    image_view,                   //
	    extent);                      //
}

/*!
 * @pre comp_target_has_images(r->c->target)
 * Update r->buffer_count before calling.
 */
static void
renderer_create_renderings_and_fences(struct comp_renderer *r)
{
	assert(r->fences == NULL);
	if (r->buffer_count == 0) {
		COMP_ERROR(r->c, "Requested 0 command buffers.");
		return;
	}

	COMP_DEBUG(r->c, "Allocating %d Command Buffers.", r->buffer_count);

	struct vk_bundle *vk = &r->c->base.vk;

	bool use_compute = r->settings->use_compute;
	if (!use_compute) {
		r->rtr_array = U_TYPED_ARRAY_CALLOC(struct render_gfx_target_resources, r->buffer_count);

		render_gfx_render_pass_init(       //
		    &r->target_render_pass,        // rgrp
		    &r->c->nr,                     // struct render_resources
		    r->c->target->format,          //
		    r->c->target->present_load_op, // load_op
		    r->c->target->final_layout);   // final_layout

		for (uint32_t i = 0; i < r->buffer_count; ++i) {
			renderer_build_rendering_target_resources(r, &r->rtr_array[i], i);
		}
	}

	r->fences = U_TYPED_ARRAY_CALLOC(VkFence, r->buffer_count);

	for (uint32_t i = 0; i < r->buffer_count; i++) {
		VkFenceCreateInfo fence_info = {
		    .sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO,
		    .flags = VK_FENCE_CREATE_SIGNALED_BIT,
		};

		VkResult ret = vk->vkCreateFence( //
		    vk->device,                   //
		    &fence_info,                  //
		    NULL,                         //
		    &r->fences[i]);               //
		if (ret != VK_SUCCESS) {
			COMP_ERROR(r->c, "vkCreateFence: %s", vk_result_string(ret));
		}

		char buf[] = "Comp Renderer X_XXXX_XXXX";
		snprintf(buf, ARRAY_SIZE(buf), "Comp Renderer %u", i);
		VK_NAME_FENCE(vk, r->fences[i], buf);
	}
}

static void
renderer_close_renderings_and_fences(struct comp_renderer *r)
{
	struct vk_bundle *vk = &r->c->base.vk;
	// Renderings
	if (r->buffer_count > 0 && r->rtr_array != NULL) {
		for (uint32_t i = 0; i < r->buffer_count; i++) {
			render_gfx_target_resources_fini(&r->rtr_array[i]);
		}

		// Close the render pass used for rendering to the target.
		render_gfx_render_pass_fini(&r->target_render_pass);

		free(r->rtr_array);
		r->rtr_array = NULL;
	}

	// Fences
	if (r->buffer_count > 0 && r->fences != NULL) {
		for (uint32_t i = 0; i < r->buffer_count; i++) {
			if (r->gpu_stuck && r->stuck_fence == r->fences[i]) {
				// Never poll a destroyed fence: drop the stuck state (only on target loss / teardown).
				COMP_ERROR(r->c, "Destroying the stuck frame fence (target lost or teardown while stuck).");
				r->gpu_stuck = false;
				r->stuck_fence = VK_NULL_HANDLE;
			}
			vk->vkDestroyFence(vk->device, r->fences[i], NULL);
			r->fences[i] = VK_NULL_HANDLE;
		}
		free(r->fences);
		r->fences = NULL;
	}

	r->buffer_count = 0;
	r->acquired_buffer = -1;
	r->fenced_buffer = -1;
}

/*!
 * @brief Ensure that target images and renderings are created, if possible.
 *
 * @param r Self pointer
 * @param force_recreate If true, will tear down and re-create images and renderings, e.g. for a resize
 *
 * @returns true if images and renderings are ready and created.
 *
 * @private @memberof comp_renderer
 * @ingroup comp_main
 */
static bool
renderer_ensure_images_and_renderings(struct comp_renderer *r, bool force_recreate)
{
	struct comp_compositor *c = r->c;
	struct comp_target *target = c->target;

	if (!comp_target_check_ready(target)) {
		// Not ready, so can't render anything.
		return false;
	}

	// We will create images if we don't have any images or if we were told to recreate them.
	bool create = force_recreate || !comp_target_has_images(target) || (r->buffer_count == 0);
	if (!create) {
		return true;
	}

	COMP_DEBUG(c, "Creating images and renderings (force_recreate: %s).", force_recreate ? "true" : "false");

	/*
	 * This makes sure that any pending command buffer has completed
	 * and all resources referred by it can now be manipulated. This
	 * make sure that validation doesn't complain. This is done
	 * during resize so isn't time critical.
	 */
	if (!renderer_wait_queue_idle(r)) {
		// GPU work is stuck: the old images, renderings and fences may still be in use, do not destroy them.
		return false;
	}

	// Make we sure we destroy all dependent things before creating new images.
	renderer_close_renderings_and_fences(r);

	VkImageUsageFlags image_usage = 0;
	if (r->settings->use_compute) {
		image_usage |= VK_IMAGE_USAGE_STORAGE_BIT;
	} else {
		image_usage |= VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
	}

	if (c->peek || getenv("XRT_COMPOSITOR_DUMP_DIR")) {
		image_usage |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
	}

	struct comp_target_create_images_info info = {
	    .extent =
	        {
	            .width = r->c->settings.preferred.width,
	            .height = r->c->settings.preferred.height,
	        },
	    .image_usage = image_usage,
	    .color_space = r->settings->color_space,
	    .present_mode = r->settings->present_mode,
	};

	static_assert(ARRAY_SIZE(info.formats) == ARRAY_SIZE(r->c->settings.formats), "Miss-match format array sizes");
	for (uint32_t i = 0; i < r->c->settings.format_count; i++) {
		info.formats[info.format_count++] = r->c->settings.formats[i];
	}

	comp_target_create_images(target, &info, r->c->base.vk.main_queue);

	bool pre_rotate = false;
	if (target->surface_transform & VK_SURFACE_TRANSFORM_ROTATE_90_BIT_KHR ||
	    target->surface_transform & VK_SURFACE_TRANSFORM_ROTATE_270_BIT_KHR) {
		pre_rotate = true;
	}

	// @todo: is it safe to fail here?
	if (!render_distortion_images_ensure(&r->c->nr, &r->c->base.vk, r->c->xdev, pre_rotate))
		return false;

	r->buffer_count = r->c->target->image_count;

	renderer_create_renderings_and_fences(r);

	assert(r->buffer_count != 0);

	return true;
}

//! Create renderer and initialize non-image-dependent members
static void
renderer_init(struct comp_renderer *r, struct comp_compositor *c, VkExtent2D scratch_extent)
{
	COMP_TRACE_MARKER();

	r->c = c;
	r->settings = &c->settings;

	r->acquired_buffer = -1;
	r->fenced_buffer = -1;
	r->rtr_array = NULL;

	// Setup the scratch images.
	bool bret = chl_scratch_ensure( //
	    &c->scratch,                // scratch
	    &c->nr,                     // struct render_resources
	    c->nr.view_count,           // view_count
	    scratch_extent,             // extent
	    VK_FORMAT_R8G8B8A8_SRGB);   // format
	if (!bret) {
		COMP_ERROR(c, "chl_scratch_ensure: false");
		assert(bret && "Whelp, can't return a error. But should never really fail.");
	}

	// Try to early-allocate these, in case we can.
	renderer_ensure_images_and_renderings(r, false);

	struct vk_bundle *vk = &r->c->base.vk;

	VkResult ret = comp_mirror_init( //
	    &r->mirror_to_debug_gui,     //
	    vk,                          //
	    &c->shaders,                 //
	    scratch_extent);             //
	if (ret != VK_SUCCESS) {
		COMP_ERROR(c, "comp_mirror_init: %s", vk_result_string(ret));
		assert(false && "Whelp, can't return a error. But should never really fail.");
	}

	r->recorder = comp_recorder_create(c);
}

static void
renderer_wait_for_last_fence(struct comp_renderer *r)
{
	COMP_TRACE_MARKER();

	if (r->fenced_buffer < 0) {
		return;
	}

	struct vk_bundle *vk = &r->c->base.vk;
	VkResult ret;

	if (debug_get_num_option_comp_gpu_wait_timeout_s() > 0) {
		(void)ret;
		// On give-up the stuck state keeps watching this fence; draws are skipped until it signals.
		renderer_wait_fence_bounded(r, r->fences[r->fenced_buffer], "last frame fence");
		r->fenced_buffer = -1;
		return;
	}

	ret = vk->vkWaitForFences(vk->device, 1, &r->fences[r->fenced_buffer], VK_TRUE, UINT64_MAX);
	if (ret != VK_SUCCESS) {
		COMP_ERROR(r->c, "vkWaitForFences: %s", vk_result_string(ret));
	}

	r->fenced_buffer = -1;
}

static inline bool
requires_present_acquire_wait(struct comp_renderer *r)
{
	/*!
	 * With shared presentable images there is only one image
	 * shared between the presentation layer and the compositor,
	 *
	 * We only need to wait on the first "acquire image",
	 * subsueqent frames waiting is redundant.
	 */
	if (comp_target_is_shared_presentable_image(r->c->target)) {
		if (r->shared_present_semaphore_wait_once) {
			return false;
		}
		r->shared_present_semaphore_wait_once = true;
	}

	return true;
}

static XRT_CHECK_RESULT VkResult
renderer_submit_queue(struct comp_renderer *r, VkCommandBuffer cmd, VkPipelineStageFlags pipeline_stage_flag)
{
	COMP_TRACE_MARKER();

	struct vk_bundle *vk = &r->c->base.vk;
	int64_t frame_id = r->c->frame.rendering.id;
	VkResult ret;

	assert(frame_id >= 0);


	/*
	 * Wait for previous frame's work to complete.
	 */

	// Wait for the last fence, if any.
	renderer_wait_for_last_fence(r);
	assert(r->fenced_buffer < 0);

	assert(r->acquired_buffer >= 0);
	ret = vk->vkResetFences(vk->device, 1, &r->fences[r->acquired_buffer]);
	VK_CHK_AND_RET(ret, "vkResetFences");


	/*
	 * Semaphore and queue submit info setup.
	 */

	struct comp_target *ct = r->c->target;
	struct vk_semaphore_list_wait wait_sems = XRT_STRUCT_INIT;
	struct vk_semaphore_list_signal signal_sems = XRT_STRUCT_INIT;
	struct vk_submit_info_builder builder = XRT_STRUCT_INIT;

	if (requires_present_acquire_wait(r)) {
		ADD_WAIT(wait_sems, ct->semaphores.present_complete, pipeline_stage_flag, false);
	}

	// Add signal semaphore (render_complete to target).
	ADD_SIGNAL(signal_sems, ct->semaphores.render_complete, ct->semaphores.render_complete_is_timeline);

	// Build the submit info struct, handles all of the semaphores for us.
	vk_submit_info_builder_prepare( //
	    &builder,                   //
	    &wait_sems,                 //
	    &cmd,                       //
	    1,                          //
	    &signal_sems,               //
	    NULL);                      //

	// Everything prepared, now we are submitting.
	comp_target_mark_submit_begin(ct, frame_id, os_monotonic_get_ns());

	/*
	 * The renderer command buffer pool is only accessed from one thread,
	 * this satisfies the `_locked` requirement of the function. This lets
	 * us avoid taking a lot of locks. The queue lock will be taken by
	 * @ref vk_cmd_submit_locked tho.
	 */
	ret = vk_cmd_submit_locked(vk, vk->main_queue, 1, &builder.submit_info, r->fences[r->acquired_buffer]);

	// We have now completed the submit, even if we failed.
	comp_target_mark_submit_end(ct, frame_id, os_monotonic_get_ns());

	// Check after marking as submit complete.
	VK_CHK_AND_RET(ret, "vk_cmd_submit_locked");

	// This buffer now have a pending fence.
	r->fenced_buffer = r->acquired_buffer;

	return ret;
}

static void
renderer_acquire_swapchain_image(struct comp_renderer *r)
{
	COMP_TRACE_MARKER();

	uint32_t buffer_index = 0;
	VkResult ret;

	assert(r->acquired_buffer < 0);

	if (!renderer_ensure_images_and_renderings(r, false)) {
		// Not ready yet.
		return;
	}
	ret = comp_target_acquire(r->c->target, &buffer_index);

	while ((ret == VK_ERROR_OUT_OF_DATE_KHR) || (ret == VK_SUBOPTIMAL_KHR)) {
		COMP_DEBUG(r->c, "Received %s.", vk_result_string(ret));

		if (!renderer_ensure_images_and_renderings(r, true)) {
			// Failed on force recreate.
			COMP_ERROR(r->c,
			           "renderer_acquire_swapchain_image: comp_target_acquire was out of date, force "
			           "re-create image and renderings failed. Probably the target disappeared.");
			return;
		}

		/* Acquire image again to silence validation error */
		ret = comp_target_acquire(r->c->target, &buffer_index);
	}

	if (ret == VK_TIMEOUT) {
		// Bounded acquire gave up (presents not retiring): no image, the draw is skipped.
		COMP_ERROR(r->c, "comp_target_acquire: VK_TIMEOUT, skipping this frame");
		return;
	}
	if (ret != VK_SUCCESS) {
		COMP_ERROR(r->c, "comp_target_acquire: %s", vk_result_string(ret));
	}

	r->acquired_buffer = buffer_index;
}

static void
renderer_resize(struct comp_renderer *r)
{
	if (!comp_target_check_ready(r->c->target)) {
		// Can't create images right now.
		// Just close any existing renderings.
		renderer_close_renderings_and_fences(r);
		return;
	}
	// Force recreate.
	renderer_ensure_images_and_renderings(r, true);
}

static bool
renderer_present_swapchain_image(struct comp_renderer *r, uint64_t desired_present_time_ns, uint64_t present_slop_ns)
{
	COMP_TRACE_MARKER();

	VkResult ret;

	assert(!comp_frame_is_invalid_locked(&r->c->frame.rendering));
	uint64_t render_complete_signal_value = (uint64_t)r->c->frame.rendering.id;

	ret = comp_target_present(        //
	    r->c->target,                 //
	    r->c->base.vk.main_queue,     //
	    r->acquired_buffer,           //
	    render_complete_signal_value, //
	    desired_present_time_ns,      //
	    present_slop_ns);             //
	r->acquired_buffer = -1;

	if (ret == VK_ERROR_OUT_OF_DATE_KHR || ret == VK_SUBOPTIMAL_KHR) {
		renderer_resize(r);
		return ret != VK_ERROR_OUT_OF_DATE_KHR;
	}
	if (ret != VK_SUCCESS) {
		COMP_ERROR(r->c, "vk_swapchain_present: %s", vk_result_string(ret));
		return false;
	}
	return true;
}

static void
renderer_wait_for_present(struct comp_renderer *r, uint64_t desired_present_time_ns)
{
	struct comp_compositor *c = r->c;

	if (!comp_target_check_ready(c->target)) {
		return;
	}

	// For estimating frame misses.
	uint64_t before_ns = os_monotonic_get_ns();

	if (c->target->wait_for_present_supported) {
		// reasonable timeout
		time_duration_ns timeout_ns = c->frame_interval_ns * 2.5f;

		// @note we don't actually care about the return value, just swallow errors, anything *critical* that
		// may be returned will be handled quite soon by later calls
		VkResult result = comp_target_wait_for_present(c->target, timeout_ns);
		(void)result;

		assert(result != VK_ERROR_EXTENSION_NOT_PRESENT);
	} else {
		/*
		 * For direct mode this makes us wait until the last frame has been
		 * actually shown to the user, this avoids us missing that we have
		 * missed a frame and miss-predicting the next frame.
		 *
		 * Not all drivers follow this behaviour, so KHR_present_wait
		 * should be preferred in all circumstances.
		 *
		 * Only do this if we are ready.
		 */

		// Do the acquire
		renderer_acquire_swapchain_image(r);
	}

	// How long did it take?
	uint64_t after_ns = os_monotonic_get_ns();

	/*
	 * Make sure we at least waited 1ms before warning. Then check
	 * if we are more then 1ms behind when we wanted to present.
	 */
	if (before_ns + U_TIME_1MS_IN_NS < after_ns && //
	    desired_present_time_ns + U_TIME_1MS_IN_NS < after_ns) {
		uint64_t diff_ns = after_ns - desired_present_time_ns;
		double diff_ms_f = time_ns_to_ms_f(diff_ns);
		LOG_FRAME_LAG("Compositor probably missed frame by %.2fms", diff_ms_f);
	}
}

static void
renderer_fini(struct comp_renderer *r)
{
	struct vk_bundle *vk = &r->c->base.vk;

	if (renderer_gpu_still_stuck(r)) {
		COMP_ERROR(r->c, "Tearing down while GPU work has been stuck for %.1f s; freeing anyway so the service exits.",
		           (double)(os_monotonic_get_ns() - r->stuck_since_ns) / 1e9);
	}
	if (r->idle_fence != VK_NULL_HANDLE) {
		vk->vkDestroyFence(vk->device, r->idle_fence, NULL);
		r->idle_fence = VK_NULL_HANDLE;
	}

	// Command buffers
	renderer_close_renderings_and_fences(r);

	// Do before layer render just in case it holds any references.
	// pimax: the recorder's worker waits (bounded) for its readbacks before the scratch images go away.
	comp_recorder_destroy(&r->recorder);

	comp_mirror_fini(&r->mirror_to_debug_gui, vk);

	// Do this after the layer renderer.
	chl_scratch_free_resources(&r->c->scratch, &r->c->nr);
}


/*
 *
 * Graphics
 *
 */

/*!
 * @pre render_gfx_init(render, &c->nr)
 */
static XRT_CHECK_RESULT VkResult
dispatch_graphics(struct comp_renderer *r,
                  struct render_gfx *render,
                  struct chl_frame_state *frame_state,
                  enum comp_target_fov_source fov_source)
{
	COMP_TRACE_MARKER();

	struct comp_compositor *c = r->c;
	struct vk_bundle *vk = &c->base.vk;
	VkResult ret;

	// Basics
	const struct comp_layer *layers = c->base.layer_accum.layers;
	uint32_t layer_count = c->base.layer_accum.layer_count;

	// Resources for the distortion render target.
	struct render_gfx_target_resources *rtr = &r->rtr_array[r->acquired_buffer];

	// Viewport information.
	struct render_viewport_data viewport_datas[XRT_MAX_VIEWS];
	calc_viewport_data(r, viewport_datas, render->r->view_count);

	// Vertex rotation information.
	struct xrt_matrix_2x2 vertex_rots[XRT_MAX_VIEWS];
	calc_vertex_rot_data(r, vertex_rots, render->r->view_count);

	// Device view information.
	struct xrt_fov fovs[XRT_MAX_VIEWS];
	struct xrt_pose world_poses_scanout_begin[XRT_MAX_VIEWS];
	struct xrt_pose world_poses_scanout_end[XRT_MAX_VIEWS];
	struct xrt_pose eye_poses[XRT_MAX_VIEWS];
	calc_pose_data(                //
	    r,                         //
	    fov_source,                //
	    fovs,                      //
	    world_poses_scanout_begin, //
	    world_poses_scanout_end,   //
	    eye_poses,                 //
	    render->r->view_count);    //

	// Does everything.
	chl_frame_state_gfx_default_pipeline( //
	    frame_state,                      //
	    render,                           //
	    layers,                           //
	    layer_count,                      //
	    world_poses_scanout_begin,        //
	    world_poses_scanout_end,          //
	    eye_poses,                        //
	    fovs,                             //
	    rtr,                              //
	    viewport_datas,                   //
	    vertex_rots);                     //

	// Everything is ready, submit to the queue.
	ret = renderer_submit_queue(r, render->r->cmd, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT);
	VK_CHK_AND_RET(ret, "renderer_submit_queue");

	return ret;
}


/*
 *
 * Compute
 *
 */

/*!
 * @pre render_compute_init(render, &c->nr)
 */
static XRT_CHECK_RESULT VkResult
dispatch_compute(struct comp_renderer *r,
                 struct render_compute *render,
                 struct chl_frame_state *frame_state,
                 enum comp_target_fov_source fov_source)
{
	COMP_TRACE_MARKER();

	struct comp_compositor *c = r->c;
	struct vk_bundle *vk = &c->base.vk;
	VkResult ret;

	// Basics
	const struct comp_layer *layers = c->base.layer_accum.layers;
	uint32_t layer_count = c->base.layer_accum.layer_count;

	// Device view information.
	struct xrt_fov fovs[XRT_MAX_VIEWS];
	struct xrt_pose world_poses_scanout_begin[XRT_MAX_VIEWS];
	struct xrt_pose world_poses_scanout_end[XRT_MAX_VIEWS];
	struct xrt_pose eye_poses[XRT_MAX_VIEWS];
	calc_pose_data(                //
	    r,                         //
	    fov_source,                //
	    fovs,                      //
	    world_poses_scanout_begin, //
	    world_poses_scanout_end,   //
	    eye_poses,                 //
	    render->r->view_count);    //

	// Target Vulkan resources..
	VkImage target_image = r->c->target->images[r->acquired_buffer].handle;
	VkImageView target_storage_view = r->c->target->images[r->acquired_buffer].view;
	VkImageLayout target_final_layout = r->c->target->final_layout;

	// Target view information.
	struct render_viewport_data target_viewport_datas[XRT_MAX_VIEWS];
	calc_viewport_data(r, target_viewport_datas, render->r->view_count);

	// Does everything.
	chl_frame_state_cs_default_pipeline( //
	    frame_state,                     //
	    render,                          //
	    layers,                          //
	    layer_count,                     //
	    world_poses_scanout_begin,       //
	    world_poses_scanout_end,         //
	    eye_poses,                       //
	    fovs,                            //
	    target_image,                    //
	    target_storage_view,             //
	    target_final_layout,             //
	    target_viewport_datas);          //

	// Everything is ready, submit to the queue.
	ret = renderer_submit_queue(r, render->r->cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
	VK_CHK_AND_RET(ret, "renderer_submit_queue");

	return ret;
}



/*!
 * Debug: XRT_COMPOSITOR_DUMP_DIR=<dir> [XRT_COMPOSITOR_DUMP_FRAMES=a,b,c] writes the final target image
 * (before present) of selected frame numbers as raw files <dir>/frame<N>_<w>x<h>_fmt<vkformat>.raw.
 *
 * XRT_COMPOSITOR_DUMP_TRIGGER=<file> additionally dumps the next frame whenever <file> exists and deletes it once the
 * dump is written (or failed), so a test client can ask for a frame at the moment its pattern is on screen
 * (pimax-lenstest --capture). Without XRT_COMPOSITOR_DUMP_FRAMES the default frame list is not used when a trigger
 * is set.
 */
static void
debug_dump_target(struct comp_renderer *r, uint32_t frame_no)
{
	const char *dir = getenv("XRT_COMPOSITOR_DUMP_DIR");
	if (dir == NULL) {
		return;
	}
	const char *trigger = getenv("XRT_COMPOSITOR_DUMP_TRIGGER");
	const bool triggered = trigger != NULL && access(trigger, F_OK) == 0;
	const char *frames = getenv("XRT_COMPOSITOR_DUMP_FRAMES");
	if (frames == NULL) {
		frames = trigger != NULL ? "" : "100,101,300,301";
	}
	bool match = false;
	char *copy = strdup(frames);
	for (char *tok = strtok(copy, ","); tok != NULL; tok = strtok(NULL, ",")) {
		if ((uint32_t)atoi(tok) == frame_no) {
			match = true;
		}
	}
	free(copy);
	if (!match && !triggered) {
		return;
	}

	struct vk_bundle *vk = &r->c->base.vk;
	struct comp_target *t = r->c->target;
	VkImage image = t->images[r->acquired_buffer].handle;
	uint32_t w = t->width, h = t->height;
	VkDeviceSize size = (VkDeviceSize)w * h * 4;
	VkResult ret;

	VkBuffer buf = VK_NULL_HANDLE;
	VkDeviceMemory mem = VK_NULL_HANDLE;
	VkCommandPool pool = VK_NULL_HANDLE;
	VkCommandBuffer cmd = VK_NULL_HANDLE;
	void *map = NULL;

	VkBufferCreateInfo bi = {
	    .sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO,
	    .size = size,
	    .usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT,
	    .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
	};
	ret = vk->vkCreateBuffer(vk->device, &bi, NULL, &buf);
	if (ret != VK_SUCCESS) {
		COMP_ERROR(r->c, "dump: vkCreateBuffer %d", ret);
		if (triggered) {
			unlink(trigger);
		}
		return;
	}
	VkMemoryRequirements mr;
	vk->vkGetBufferMemoryRequirements(vk->device, buf, &mr);
	uint32_t type = 0;
	if (vk_get_memory_type(vk, mr.memoryTypeBits,
	                       VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &type) == false) {
		COMP_ERROR(r->c, "dump: no host memory type");
		goto out;
	}
	VkMemoryAllocateInfo ai = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
	    .allocationSize = mr.size,
	    .memoryTypeIndex = type,
	};
	if (vk->vkAllocateMemory(vk->device, &ai, NULL, &mem) != VK_SUCCESS ||
	    vk->vkBindBufferMemory(vk->device, buf, mem, 0) != VK_SUCCESS) {
		COMP_ERROR(r->c, "dump: alloc/bind failed");
		goto out;
	}
	VkCommandPoolCreateInfo pci = {
	    .sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
	    .queueFamilyIndex = vk->main_queue->family_index,
	};
	vk->vkCreateCommandPool(vk->device, &pci, NULL, &pool);
	VkCommandBufferAllocateInfo cai = {
	    .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
	    .commandPool = pool,
	    .level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
	    .commandBufferCount = 1,
	};
	vk->vkAllocateCommandBuffers(vk->device, &cai, &cmd);
	VkCommandBufferBeginInfo bgi = {.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
	                                .flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT};
	vk->vkBeginCommandBuffer(cmd, &bgi);
	VkImageSubresourceRange sr = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
	VkImageMemoryBarrier b = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
	    .srcAccessMask = VK_ACCESS_MEMORY_WRITE_BIT,
	    .dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT,
	    .oldLayout = t->final_layout,
	    .newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
	    .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .image = image,
	    .subresourceRange = sr,
	};
	vk->vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, NULL, 0,
	                         NULL, 1, &b);
	VkBufferImageCopy reg = {
	    .imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1},
	    .imageExtent = {w, h, 1},
	};
	vk->vkCmdCopyImageToBuffer(cmd, image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, buf, 1, &reg);
	b.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
	b.dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
	b.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
	b.newLayout = t->final_layout;
	vk->vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, NULL, 0,
	                         NULL, 1, &b);
	vk->vkEndCommandBuffer(cmd);

	VkSubmitInfo si = {
	    .sType = VK_STRUCTURE_TYPE_SUBMIT_INFO,
	    .commandBufferCount = 1,
	    .pCommandBuffers = &cmd,
	};
	vk_queue_lock(vk->main_queue);
	vk->vkQueueWaitIdle(vk->main_queue->queue);
	ret = vk->vkQueueSubmit(vk->main_queue->queue, 1, &si, VK_NULL_HANDLE);
	vk->vkQueueWaitIdle(vk->main_queue->queue);
	vk_queue_unlock(vk->main_queue);
	if (ret != VK_SUCCESS) {
		COMP_ERROR(r->c, "dump: submit %d", ret);
		goto out;
	}
	if (vk->vkMapMemory(vk->device, mem, 0, size, 0, &map) == VK_SUCCESS) {
		char path[1024];
		snprintf(path, sizeof path, "%s/frame%u_%ux%u_fmt%d.raw", dir, frame_no, w, h, (int)t->format);
		FILE *f = fopen(path, "wb");
		if (f) {
			fwrite(map, 1, size, f);
			fclose(f);
			COMP_INFO(r->c, "dumped %s", path);
		}
		vk->vkUnmapMemory(vk->device, mem);
	}
out:
	if (pool != VK_NULL_HANDLE) {
		vk->vkDestroyCommandPool(vk->device, pool, NULL);
	}
	if (buf != VK_NULL_HANDLE) {
		vk->vkDestroyBuffer(vk->device, buf, NULL);
	}
	if (mem != VK_NULL_HANDLE) {
		vk->vkFreeMemory(vk->device, mem, NULL);
	}
	if (triggered) {
		unlink(trigger); // tells the requester the dump is finished
	}
}

/*
 *
 * Interface functions.
 *
 */

XRT_CHECK_RESULT xrt_result_t
comp_renderer_draw(struct comp_renderer *r)
{
	COMP_TRACE_MARKER();

	struct comp_target *ct = r->c->target;
	struct comp_compositor *c = r->c;

	// Check that we don't have any bad data.
	assert(!comp_frame_is_invalid_locked(&c->frame.waited));
	assert(comp_frame_is_invalid_locked(&c->frame.rendering));

	// Move waited frame to rendering frame, clear waited.
	comp_frame_move_and_clear_locked(&c->frame.rendering, &c->frame.waited);

	// Tell the target we are starting to render, for frame timing.
	comp_target_mark_begin(ct, c->frame.rendering.id, os_monotonic_get_ns());

	// Are we ready to render? No - skip rendering. Also skip while earlier GPU work is stuck (bounded waits).
	if (renderer_gpu_still_stuck(r) || !comp_target_check_ready(r->c->target)) {
		// Need to emulate rendering for the timing.
		//! @todo This should be discard.
		comp_target_mark_submit_begin(ct, c->frame.rendering.id, os_monotonic_get_ns());
		comp_target_mark_submit_end(ct, c->frame.rendering.id, os_monotonic_get_ns());

		// Clear the rendering frame.
		comp_frame_clear_locked(&c->frame.rendering);
		return XRT_SUCCESS;
	}

	comp_target_flush(ct);

	comp_target_update_timings(ct);

	if (r->acquired_buffer < 0) {
		// Ensures that renderings are created.
		renderer_acquire_swapchain_image(r);
	}

	if (r->acquired_buffer < 0 && debug_get_num_option_comp_gpu_wait_timeout_s() > 0) {
		// No image (bounded acquire gave up, or images could not be made): emulate rendering for the timing.
		comp_target_mark_submit_begin(ct, c->frame.rendering.id, os_monotonic_get_ns());
		comp_target_mark_submit_end(ct, c->frame.rendering.id, os_monotonic_get_ns());
		comp_frame_clear_locked(&c->frame.rendering);
		return XRT_SUCCESS;
	}

	comp_target_update_timings(ct);

	// Hardcoded for now.
	const uint32_t view_count = c->nr.view_count;
	enum comp_target_fov_source fov_source = COMP_TARGET_FOV_SOURCE_DISTORTION;

	bool fast_path = c->base.frame_params.one_projection_layer_fast_path;
	bool do_timewarp = !c->debug.atw_off;

	// Consistency check.
	assert(!fast_path || c->base.layer_accum.layer_count >= 1);

	// For scratch image debugging.
	struct chl_frame_state frame_state;
	chl_frame_state_init( //
	    &frame_state,     //
	    &c->nr,           //
	    view_count,       //
	    do_timewarp,      //
	    fast_path,        //
	    &c->scratch);     //

	bool use_compute = r->settings->use_compute;
	struct render_gfx render_g = {0};
	struct render_compute render_c = {0};

	VkResult res = VK_SUCCESS;
	if (use_compute) {
		render_compute_init(&render_c, &c->nr);
		res = dispatch_compute(r, &render_c, &frame_state, fov_source);
	} else {
		render_gfx_init(&render_g, &c->nr);
		res = dispatch_graphics(r, &render_g, &frame_state, fov_source);
	}
	if (res != VK_SUCCESS) {
		return XRT_ERROR_VULKAN;
	}

#ifdef XRT_FEATURE_WINDOW_PEEK
	if (c->peek) {
		switch (comp_window_peek_get_eye(c->peek)) {
		case COMP_WINDOW_PEEK_EYE_LEFT: {
			uint32_t scratch_index = frame_state.scratch_state.views[0].index;
			struct comp_scratch_single_images *view = &c->scratch.views[0].cssi;

			comp_window_peek_blit(                 //
			    c->peek,                           //
			    view->images[scratch_index].image, //
			    view->info.width,                  //
			    view->info.height);                //
		} break;
		case COMP_WINDOW_PEEK_EYE_RIGHT: {
			uint32_t scratch_index = frame_state.scratch_state.views[1].index;
			struct comp_scratch_single_images *view = &c->scratch.views[1].cssi;

			comp_window_peek_blit(                 //
			    c->peek,                           //
			    view->images[scratch_index].image, //
			    view->info.width,                  //
			    view->info.height);                //
		} break;
		case COMP_WINDOW_PEEK_EYE_BOTH:
			/* TODO: display the undistorted image */
			comp_window_peek_blit(c->peek, c->target->images[r->acquired_buffer].handle, c->target->width,
			                      c->target->height);
			break;
		}
	}
#endif

	{
		static uint32_t dump_frame_no = 0;
		debug_dump_target(r, dump_frame_no++);
	}

	bool present_success = renderer_present_swapchain_image(r, c->frame.rendering.desired_present_time_ns,
	                                                        c->frame.rendering.present_slop_ns);

	// Save for timestamps below.
	uint64_t frame_id = c->frame.rendering.id;
	uint64_t desired_present_time_ns = c->frame.rendering.desired_present_time_ns;
	uint64_t predicted_display_time_ns = c->frame.rendering.predicted_display_time_ns;

	// Clear the rendered frame.
	comp_frame_clear_locked(&c->frame.rendering);

	xrt_result_t xret = XRT_SUCCESS;
	comp_mirror_fixup_ui_state(&r->mirror_to_debug_gui, c);
	if (comp_mirror_is_ready_and_active(&r->mirror_to_debug_gui, c, predicted_display_time_ns)) {

		uint32_t scratch_index = frame_state.scratch_state.views[0].index;
		struct comp_scratch_single_images *view = &c->scratch.views[0].cssi;
		struct render_scratch_color_image *rsci = &view->images[scratch_index];
		VkExtent2D extent = {view->info.width, view->info.width};

		// Used for both, want clamp to edge to no bring in black.
		VkSampler clamp_to_edge = c->nr.samplers.clamp_to_edge;

		// Covers the whole view.
		struct xrt_normalized_rect rect = {0, 0, 1.0f, 1.0f};

		xret = comp_mirror_do_blit(    //
		    &r->mirror_to_debug_gui,   //
		    &c->base.vk,               //
		    frame_id,                  //
		    predicted_display_time_ns, //
		    rsci->image,               //
		    rsci->srgb_view,           //
		    clamp_to_edge,             //
		    extent,                    //
		    rect);                     //
	}

	/*
	 * This fixes a lot of validation issues as it makes sure that the
	 * command buffer has completed and all resources referred by it can
	 * now be manipulated.
	 *
	 * This is done after a swap so isn't time critical.
	 */
	bool queue_idle = renderer_wait_queue_idle(r);

	// pimax: hand the squashed eye images to the recorder (never waits; drops the frame if it cannot keep up).
	if (queue_idle && r->recorder != NULL) {
		comp_recorder_capture(r->recorder, &frame_state.scratch_state, frame_id, predicted_display_time_ns);
	}

	/*
	 * Free any resources and finalize the scratch images,
	 * which sends them send to debug UI if it is active.
	 */
	chl_frame_state_fini(&frame_state);

	// Check timestamps (not while stuck: the query read waits for the GPU).
	if (xret == XRT_SUCCESS && queue_idle) {
		/*
		 * Get timestamps of GPU work (if available).
		 */

		uint64_t gpu_start_ns, gpu_end_ns;
		if (render_resources_get_timestamps(&c->nr, &gpu_start_ns, &gpu_end_ns)) {
			uint64_t now_ns = os_monotonic_get_ns();
			comp_target_info_gpu(ct, frame_id, gpu_start_ns, gpu_end_ns, now_ns);
		}
	}


	/*
	 * Free resources.
	 */

	if (use_compute) {
		render_compute_fini(&render_c);
	} else {
		render_gfx_fini(&render_g);
	}

	if (present_success) {
		renderer_wait_for_present(r, desired_present_time_ns);
	}

	comp_target_update_timings(ct);

	return xret;
}

struct comp_renderer *
comp_renderer_create(struct comp_compositor *c, VkExtent2D scratch_extent)
{
	struct comp_renderer *r = U_TYPED_CALLOC(struct comp_renderer);

	renderer_init(r, c, scratch_extent);

	return r;
}

void
comp_renderer_destroy(struct comp_renderer **ptr_r)
{
	if (ptr_r == NULL) {
		return;
	}

	struct comp_renderer *r = *ptr_r;
	if (r == NULL) {
		return;
	}

	renderer_fini(r);

	free(r);
	*ptr_r = NULL;
}

void
comp_renderer_add_debug_vars(struct comp_renderer *self)
{
	struct comp_renderer *r = self;

	comp_mirror_add_debug_vars(&r->mirror_to_debug_gui, r->c);
}
