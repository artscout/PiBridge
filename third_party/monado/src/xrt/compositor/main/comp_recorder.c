// Copyright 2026, pimax-re contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Recorder tap, see comp_recorder.h and comp_recorder_shm.h. Design notes: pimax-re docs/RECORD.md.
 * @ingroup comp_main
 */

#include "main/comp_recorder.h"
#include "main/comp_recorder_shm.h"
#include "main/comp_compositor.h"

#include "cache/render_compute_pipeline_cache.h"
#include "cache/render_shader_specialization_helpers.h"
#include "render/render_interface.h"
#include "util/comp_high_level_scratch.h"
#include "util/u_misc.h"
#include "util/u_trace_marker.h"
#include "os/os_time.h"
#include "util/u_logging.h"
#include "vk/vk_helpers.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

#define REC_GPU_SLOTS 3
#define REC_MAX_EYES 2
#define REC_STALE_NS (3ull * 1000 * 1000 * 1000)
#define REC_FENCE_WAIT_NS (2ull * 1000 * 1000)
#define REC_DRAIN_WAIT_NS (1000ull * 1000 * 1000)
#define REC_MAX_ERRORS 10

#define REC_LOG(...) U_LOG_I("recorder: " __VA_ARGS__)
#define REC_WARN(...) U_LOG_W("recorder: " __VA_ARGS__)
#define REC_ERR(...) U_LOG_E("recorder: " __VA_ARGS__)

enum rec_state
{
	REC_IDLE = 0,     //!< no consumer, no Vulkan resources
	REC_READY = 1,    //!< the compositor thread may capture
	REC_DRAINING = 2, //!< the worker is tearing down, the compositor thread keeps out
};

enum gpu_slot_state
{
	GPU_FREE = 0,      //!< the compositor thread may record into it
	GPU_SUBMITTED = 1, //!< submitted, the worker waits for its fence
};

struct rec_gpu_slot
{
	_Atomic int state;
	VkCommandPool pool;
	VkCommandBuffer cmd;
	VkFence fence;
	VkBuffer buffer;
	VkDeviceMemory mem;
	void *map;
	VkDescriptorSet sets[REC_MAX_EYES];
	uint64_t seq;
	uint64_t timestamp_ns;
};

struct comp_recorder
{
	struct comp_compositor *c;
	struct vk_bundle *vk;

	pthread_t thread;
	bool thread_started;
	_Atomic bool quit;

	//! enum rec_state
	_Atomic int state;
	//! The compositor thread is inside comp_recorder_capture.
	_Atomic bool busy;
	//! Vulkan errors on the compositor thread; the worker detaches after REC_MAX_ERRORS.
	_Atomic int errors;

	char path[512];

	// Attached consumer (worker-owned, read-only for the compositor thread while READY).
	int fd;
	struct pimax_rec_header *hdr;
	size_t map_size;
	uint32_t eyes;
	uint64_t interval_ns;
	VkExtent2D eye_extent; //!< per-eye output size
	VkExtent2D out_extent; //!< whole frame
	VkExtent2D src_extent;

	// Compositor-thread pacing.
	uint64_t next_due_ns;
	uint64_t seq;

	// Vulkan resources (created and destroyed by the worker while not READY).
	VkImage bounce;
	VkDeviceMemory bounce_mem;
	VkImageView bounce_view;
	VkDescriptorPool descriptor_pool;
	VkDescriptorSetLayout set_layout;
	VkPipelineLayout pipeline_layout;
	VkPipelineCache driver_cache;
	struct render_blit_pipeline_cache *pipeline_cache;
	VkPipeline pipeline;
	struct rec_gpu_slot slots[REC_GPU_SLOTS];
	bool leaked; //!< a fence never signalled: the resources are left alone
};

static _Atomic bool g_squash = false;


/*
 *
 * Helpers.
 *
 */

static inline uint64_t
now_ns(void)
{
	return os_monotonic_get_ns();
}

static inline uint64_t
hdr_load64(uint64_t *p)
{
	return atomic_load_explicit((_Atomic uint64_t *)p, memory_order_acquire);
}

static inline void
hdr_store64(uint64_t *p, uint64_t v)
{
	atomic_store_explicit((_Atomic uint64_t *)p, v, memory_order_release);
}

static inline void
hdr_add64(uint64_t *p, uint64_t v)
{
	atomic_fetch_add_explicit((_Atomic uint64_t *)p, v, memory_order_relaxed);
}

static inline uint32_t
hdr_load32(uint32_t *p)
{
	return atomic_load_explicit((_Atomic uint32_t *)p, memory_order_acquire);
}

static inline void
hdr_store32(uint32_t *p, uint32_t v)
{
	atomic_store_explicit((_Atomic uint32_t *)p, v, memory_order_release);
}

static inline bool
hdr_cas32(uint32_t *p, uint32_t expect, uint32_t v)
{
	return atomic_compare_exchange_strong_explicit((_Atomic uint32_t *)p, &expect, v, memory_order_acq_rel,
	                                               memory_order_acquire);
}

static uint32_t
even(uint32_t v)
{
	return v & ~1u;
}

/*!
 * Output size for an eye of `src` (pure; mirrored by pimax-record's tests).
 */
static VkExtent2D
pick_eye_extent(VkExtent2D src, uint32_t max_height, uint32_t scale_permille, uint32_t eyes, uint64_t slot_size)
{
	double h = src.height;
	if (scale_permille > 0 && scale_permille <= 1000) {
		h = h * scale_permille / 1000.0;
	}
	if (max_height > 0 && h > max_height) {
		h = max_height;
	}
	double aspect = (double)src.width / (double)src.height;
	for (;;) {
		uint32_t eh = even((uint32_t)h);
		uint32_t ew = even((uint32_t)(h * aspect + 0.5));
		if (eh < 64 || ew < 64) {
			return (VkExtent2D){0, 0};
		}
		if ((uint64_t)ew * eyes * eh * 4 <= slot_size && ew * eyes <= 4096) {
			return (VkExtent2D){ew, eh};
		}
		h -= 16;
	}
}


/*
 *
 * Vulkan resources (worker thread, never while READY).
 *
 */

static void
destroy_resources(struct comp_recorder *rec)
{
	struct vk_bundle *vk = rec->vk;
	if (rec->leaked) {
		return;
	}
	for (int i = 0; i < REC_GPU_SLOTS; i++) {
		struct rec_gpu_slot *s = &rec->slots[i];
		if (s->map != NULL) {
			vk->vkUnmapMemory(vk->device, s->mem);
		}
		if (s->buffer != VK_NULL_HANDLE) {
			vk->vkDestroyBuffer(vk->device, s->buffer, NULL);
		}
		if (s->mem != VK_NULL_HANDLE) {
			vk->vkFreeMemory(vk->device, s->mem, NULL);
		}
		if (s->fence != VK_NULL_HANDLE) {
			vk->vkDestroyFence(vk->device, s->fence, NULL);
		}
		if (s->pool != VK_NULL_HANDLE) {
			vk->vkDestroyCommandPool(vk->device, s->pool, NULL); // frees s->cmd
		}
		memset(s, 0, sizeof(*s));
	}
	if (rec->pipeline_cache != NULL) {
		rec->pipeline = VK_NULL_HANDLE;
		render_blit_pipeline_cache_destroy(&rec->pipeline_cache, vk);
	}
	if (rec->pipeline_layout != VK_NULL_HANDLE) {
		vk->vkDestroyPipelineLayout(vk->device, rec->pipeline_layout, NULL);
	}
	if (rec->driver_cache != VK_NULL_HANDLE) {
		vk->vkDestroyPipelineCache(vk->device, rec->driver_cache, NULL);
	}
	if (rec->descriptor_pool != VK_NULL_HANDLE) {
		vk->vkDestroyDescriptorPool(vk->device, rec->descriptor_pool, NULL);
	}
	if (rec->set_layout != VK_NULL_HANDLE) {
		vk->vkDestroyDescriptorSetLayout(vk->device, rec->set_layout, NULL);
	}
	if (rec->bounce_view != VK_NULL_HANDLE) {
		vk->vkDestroyImageView(vk->device, rec->bounce_view, NULL);
	}
	if (rec->bounce != VK_NULL_HANDLE) {
		vk->vkDestroyImage(vk->device, rec->bounce, NULL);
	}
	if (rec->bounce_mem != VK_NULL_HANDLE) {
		vk->vkFreeMemory(vk->device, rec->bounce_mem, NULL);
	}
	rec->pipeline_layout = VK_NULL_HANDLE;
	rec->driver_cache = VK_NULL_HANDLE;
	rec->descriptor_pool = VK_NULL_HANDLE;
	rec->set_layout = VK_NULL_HANDLE;
	rec->bounce_view = VK_NULL_HANDLE;
	rec->bounce = VK_NULL_HANDLE;
	rec->bounce_mem = VK_NULL_HANDLE;
}

static bool
create_resources(struct comp_recorder *rec)
{
	struct vk_bundle *vk = rec->vk;
	VkResult ret;

#define CHK(x, what)                                                                                                   \
	do {                                                                                                           \
		VkResult r_ = (x);                                                                                     \
		if (r_ != VK_SUCCESS) {                                                                                \
			REC_ERR("%s: %s", what, vk_result_string(r_));                                                 \
			destroy_resources(rec);                                                                        \
			return false;                                                                                  \
		}                                                                                                      \
	} while (false)

	// The bounce image the compute blit writes (storage) and the copy reads (transfer src).
	CHK(vk_create_image_mutable_rgba(vk, rec->out_extent,
	                                 VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
	                                     VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT,
	                                 &rec->bounce_mem, &rec->bounce),
	    "bounce image");
	VkImageSubresourceRange range = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
	CHK(vk_create_view_usage(vk, rec->bounce, VK_IMAGE_VIEW_TYPE_2D, VK_FORMAT_R8G8B8A8_UNORM,
	                         VK_IMAGE_USAGE_STORAGE_BIT, range, &rec->bounce_view),
	    "bounce view");

	// Same layout and push constants as the debug-GUI mirror's blit.
	VkDescriptorSetLayoutBinding bindings[2] = {
	    {.binding = 0,
	     .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
	     .descriptorCount = 1,
	     .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT},
	    {.binding = 1,
	     .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
	     .descriptorCount = 1,
	     .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT},
	};
	VkDescriptorSetLayoutCreateInfo sli = {
	    .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
	    .bindingCount = 2,
	    .pBindings = bindings,
	};
	CHK(vk->vkCreateDescriptorSetLayout(vk->device, &sli, NULL, &rec->set_layout), "set layout");

	VkPushConstantRange pcr = {
	    .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT,
	    .offset = 0,
	    .size = sizeof(struct render_compute_blit_push_data),
	};
	VkPipelineLayoutCreateInfo pli = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO,
	    .setLayoutCount = 1,
	    .pSetLayouts = &rec->set_layout,
	    .pushConstantRangeCount = 1,
	    .pPushConstantRanges = &pcr,
	};
	CHK(vk->vkCreatePipelineLayout(vk->device, &pli, NULL, &rec->pipeline_layout), "pipeline layout");

	struct vk_descriptor_pool_info dpi = {
	    .sampler_per_descriptor_count = 1,
	    .storage_image_per_descriptor_count = 1,
	    .descriptor_count = REC_GPU_SLOTS * REC_MAX_EYES,
	    .freeable = false,
	};
	CHK(vk_create_descriptor_pool(vk, &dpi, &rec->descriptor_pool), "descriptor pool");
	CHK(vk_create_pipeline_cache(vk, &rec->driver_cache), "pipeline cache");

	if (render_blit_pipeline_cache_create("comp_recorder blit", &rec->pipeline_cache) != XRT_SUCCESS ||
	    render_blit_pipeline_cache_init(rec->pipeline_cache, rec->c->shaders.blit_comp, rec->pipeline_layout,
	                                    rec->driver_cache) != XRT_SUCCESS) {
		REC_ERR("blit pipeline cache");
		destroy_resources(rec);
		return false;
	}
	// The scratch images hold sRGB-encoded bytes; sampling their UNORM view and writing UNORM copies them as is.
	const struct render_blit_spec spec = render_make_blit_spec(0);
	if (render_blit_pipeline_cache_get(rec->pipeline_cache, vk, &spec, &rec->pipeline) != XRT_SUCCESS) {
		REC_ERR("blit pipeline");
		destroy_resources(rec);
		return false;
	}

	VkDeviceSize bytes = (VkDeviceSize)rec->out_extent.width * rec->out_extent.height * 4;
	for (int i = 0; i < REC_GPU_SLOTS; i++) {
		struct rec_gpu_slot *s = &rec->slots[i];
		VkCommandPoolCreateInfo cpi = {
		    .sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
		    .flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT,
		    .queueFamilyIndex = vk->main_queue->family_index,
		};
		CHK(vk->vkCreateCommandPool(vk->device, &cpi, NULL, &s->pool), "command pool");
		VkCommandBufferAllocateInfo cai = {
		    .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
		    .commandPool = s->pool,
		    .level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
		    .commandBufferCount = 1,
		};
		CHK(vk->vkAllocateCommandBuffers(vk->device, &cai, &s->cmd), "command buffer");
		VkFenceCreateInfo fci = {.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
		CHK(vk->vkCreateFence(vk->device, &fci, NULL, &s->fence), "fence");

		// Host-cached memory makes the worker's memcpy fast; coherent because there is no invalidate here.
		bool ok = vk_buffer_init(vk, bytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		                         VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT |
		                             VK_MEMORY_PROPERTY_HOST_CACHED_BIT,
		                         &s->buffer, &s->mem);
		if (!ok) {
			if (s->buffer != VK_NULL_HANDLE) {
				vk->vkDestroyBuffer(vk->device, s->buffer, NULL);
				s->buffer = VK_NULL_HANDLE;
			}
			ok = vk_buffer_init(vk, bytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
			                    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
			                    &s->buffer, &s->mem);
		}
		if (!ok) {
			REC_ERR("readback buffer");
			destroy_resources(rec);
			return false;
		}
		CHK(vk->vkMapMemory(vk->device, s->mem, 0, VK_WHOLE_SIZE, 0, &s->map), "map");
		for (uint32_t e = 0; e < REC_MAX_EYES; e++) {
			CHK(vk_create_descriptor_set(vk, rec->descriptor_pool, rec->set_layout, &s->sets[e]),
			    "descriptor set");
		}
		atomic_store(&s->state, GPU_FREE);
	}
#undef CHK
	(void)ret;
	return true;
}


/*
 *
 * Attach / detach (worker thread).
 *
 */

static void
unmap_consumer(struct comp_recorder *rec)
{
	if (rec->hdr != NULL) {
		hdr_store32(&rec->hdr->producer_pid, 0);
		munmap(rec->hdr, rec->map_size);
		rec->hdr = NULL;
	}
	if (rec->fd >= 0) {
		close(rec->fd);
		rec->fd = -1;
	}
}

/*!
 * Wait (bounded) for a submitted slot. Returns false if it never signalled.
 */
static bool
wait_slot(struct comp_recorder *rec, struct rec_gpu_slot *s, uint64_t timeout_ns)
{
	if (atomic_load(&s->state) != GPU_SUBMITTED) {
		return true;
	}
	VkResult r = rec->vk->vkWaitForFences(rec->vk->device, 1, &s->fence, VK_TRUE, timeout_ns);
	return r == VK_SUCCESS;
}

static void
detach(struct comp_recorder *rec, const char *why)
{
	if (atomic_load(&rec->state) == REC_IDLE && rec->hdr == NULL) {
		return;
	}
	REC_LOG("detaching (%s)", why);
	atomic_store(&rec->state, REC_DRAINING);
	atomic_store(&g_squash, false);
	// The compositor thread leaves comp_recorder_capture within one frame.
	for (int i = 0; i < 1000 && atomic_load(&rec->busy); i++) {
		usleep(1000);
	}
	bool all = !atomic_load(&rec->busy);
	for (int i = 0; i < REC_GPU_SLOTS; i++) {
		if (rec->slots[i].fence != VK_NULL_HANDLE && !wait_slot(rec, &rec->slots[i], REC_DRAIN_WAIT_NS)) {
			all = false;
		}
	}
	if (!all) {
		// A readback that never finished (GPU stuck): never destroy what the GPU may still use.
		REC_ERR("a readback did not finish: leaving its resources alone");
		rec->leaked = true;
	}
	destroy_resources(rec);
	unmap_consumer(rec);
	atomic_store(&rec->errors, 0);
	atomic_store(&rec->state, REC_IDLE);
}

static void
try_attach(struct comp_recorder *rec)
{
	if (rec->leaked) {
		return; // after a stuck readback, no more recording in this process
	}
	int fd = open(rec->path, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		return;
	}
	struct stat st;
	if (fstat(fd, &st) != 0 || st.st_uid != getuid() || (size_t)st.st_size < PIMAX_REC_HEADER_SIZE) {
		close(fd);
		return;
	}
	struct pimax_rec_header *h = mmap(NULL, (size_t)st.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (h == MAP_FAILED) {
		close(fd);
		return;
	}
	rec->fd = fd;
	rec->hdr = h;
	rec->map_size = (size_t)st.st_size;

	const char *bad = NULL;
	uint64_t hb = hdr_load64(&h->consumer_heartbeat_ns);
	if (h->magic != PIMAX_REC_MAGIC || h->version != PIMAX_REC_VERSION || h->header_size != PIMAX_REC_HEADER_SIZE) {
		bad = "bad magic or version";
	} else if (h->slot_count == 0 || h->slot_count > PIMAX_REC_MAX_SLOTS ||
	           PIMAX_REC_HEADER_SIZE + h->slot_count * h->slot_size > (uint64_t)st.st_size) {
		bad = "bad slots";
	} else if (h->eyes < 1 || h->eyes > 2 || h->fps < 1 || h->fps > 240) {
		bad = "bad request";
	} else if (hdr_load32(&h->stop) != 0 || hb == 0 || now_ns() - hb > REC_STALE_NS) {
		bad = NULL; // a consumer that is gone or leaving: ignore quietly
		unmap_consumer(rec);
		return;
	}
	if (bad != NULL) {
		REC_WARN("ignoring %s: %s", rec->path, bad);
		unmap_consumer(rec);
		return;
	}
	if (hdr_load32(&h->producer_pid) != 0) {
		unmap_consumer(rec); // another compositor owns it
		return;
	}

	struct comp_compositor *c = rec->c;
	if (c->scratch.view_count < h->eyes) {
		REC_WARN("only %u views", c->scratch.view_count);
		unmap_consumer(rec);
		return;
	}
	rec->src_extent = (VkExtent2D){c->scratch.views[0].cssi.info.width, c->scratch.views[0].cssi.info.height};
	rec->eyes = h->eyes;
	rec->eye_extent = pick_eye_extent(rec->src_extent, h->max_height, h->scale_permille, h->eyes, h->slot_size);
	if (rec->eye_extent.width == 0) {
		REC_WARN("no output size fits the slots");
		unmap_consumer(rec);
		return;
	}
	rec->out_extent = (VkExtent2D){rec->eye_extent.width * rec->eyes, rec->eye_extent.height};
	rec->interval_ns = 1000000000ull / h->fps;

	if (!create_resources(rec)) {
		unmap_consumer(rec);
		return;
	}

	h->width = rec->out_extent.width;
	h->height = rec->out_extent.height;
	h->src_width = rec->src_extent.width;
	h->src_height = rec->src_extent.height;
	snprintf(h->producer_note, sizeof(h->producer_note), "monado comp_recorder v1, %u eye(s)", rec->eyes);
	hdr_store64(&h->producer_heartbeat_ns, now_ns());
	hdr_store32(&h->producer_pid, (uint32_t)getpid());

	REC_LOG("attached: %u eye(s) %ux%u from %ux%u at %u fps", rec->eyes, rec->out_extent.width,
	        rec->out_extent.height, rec->src_extent.width, rec->src_extent.height, h->fps);
	rec->next_due_ns = 0;
	atomic_store(&g_squash, true);
	atomic_store(&rec->state, REC_READY);
}

/*!
 * Copy finished readbacks into free shared-memory slots.
 */
static void
drain_finished(struct comp_recorder *rec)
{
	struct pimax_rec_header *h = rec->hdr;
	size_t bytes = (size_t)rec->out_extent.width * rec->out_extent.height * 4;
	for (int i = 0; i < REC_GPU_SLOTS; i++) {
		struct rec_gpu_slot *s = &rec->slots[i];
		if (atomic_load(&s->state) != GPU_SUBMITTED) {
			continue;
		}
		VkResult r = rec->vk->vkWaitForFences(rec->vk->device, 1, &s->fence, VK_TRUE, REC_FENCE_WAIT_NS);
		if (r == VK_TIMEOUT) {
			continue;
		}
		if (r != VK_SUCCESS) {
			atomic_fetch_add(&rec->errors, REC_MAX_ERRORS); // device lost or similar: detach
			return;
		}
		bool put = false;
		for (uint32_t k = 0; k < h->slot_count; k++) {
			struct pimax_rec_slot *ss = &h->slots[k];
			if (!hdr_cas32(&ss->state, PIMAX_REC_SLOT_FREE, PIMAX_REC_SLOT_WRITING)) {
				continue;
			}
			uint8_t *dst = (uint8_t *)h + PIMAX_REC_HEADER_SIZE + (size_t)k * h->slot_size;
			memcpy(dst, s->map, bytes);
			ss->width = rec->out_extent.width;
			ss->height = rec->out_extent.height;
			ss->seq = s->seq;
			ss->timestamp_ns = s->timestamp_ns;
			hdr_store32(&ss->state, PIMAX_REC_SLOT_READY);
			hdr_add64(&h->produced, 1);
			put = true;
			break;
		}
		if (!put) {
			hdr_add64(&h->dropped_full, 1);
		}
		rec->vk->vkResetFences(rec->vk->device, 1, &s->fence);
		atomic_store(&s->state, GPU_FREE);
	}
}

static void *
worker(void *ptr)
{
	struct comp_recorder *rec = ptr;

	// Never compete with the compositor: plain scheduling (threads inherit SCHED_FIFO from their creator), nice 10.
	struct sched_param sp = {.sched_priority = 0};
	pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp);
	setpriority(PRIO_PROCESS, (id_t)syscall(SYS_gettid), 10);
	pthread_setname_np(pthread_self(), "comp_recorder");

	uint64_t last_poll = 0;
	while (!atomic_load(&rec->quit)) {
		int st = atomic_load(&rec->state);
		if (st == REC_IDLE) {
			uint64_t t = now_ns();
			if (t - last_poll >= 500ull * 1000 * 1000) {
				last_poll = t;
				try_attach(rec);
			}
			if (atomic_load(&rec->state) == REC_IDLE) {
				usleep(100 * 1000);
			}
			continue;
		}

		struct pimax_rec_header *h = rec->hdr;
		uint64_t t = now_ns();
		hdr_store64(&h->producer_heartbeat_ns, t);
		struct stat stt;
		if (hdr_load32(&h->stop) != 0) {
			detach(rec, "the recorder asked to stop");
			continue;
		}
		if (t - hdr_load64(&h->consumer_heartbeat_ns) > REC_STALE_NS) {
			detach(rec, "the recorder stopped answering");
			continue;
		}
		if (fstat(rec->fd, &stt) == 0 && stt.st_nlink == 0) {
			detach(rec, "the shared-memory file was removed");
			continue;
		}
		if (atomic_load(&rec->errors) >= REC_MAX_ERRORS) {
			detach(rec, "Vulkan errors");
			continue;
		}
		drain_finished(rec);
		usleep(2000);
	}
	detach(rec, "compositor shutting down");
	return NULL;
}


/*
 *
 * Compositor thread.
 *
 */

static bool
record_and_submit(struct comp_recorder *rec, struct rec_gpu_slot *s, const struct chl_scratch_state *ss)
{
	struct vk_bundle *vk = rec->vk;
	struct comp_compositor *c = rec->c;
	VkResult ret;

	ret = vk->vkResetCommandPool(vk->device, s->pool, 0);
	if (ret != VK_SUCCESS) {
		return false;
	}

	for (uint32_t e = 0; e < rec->eyes; e++) {
		struct comp_scratch_single_images *view = &c->scratch.views[e].cssi;
		VkDescriptorImageInfo src = {
		    .sampler = c->nr.samplers.clamp_to_edge,
		    .imageView = view->images[ss->views[e].index].unorm_view,
		    .imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
		};
		VkDescriptorImageInfo dst = {
		    .imageView = rec->bounce_view,
		    .imageLayout = VK_IMAGE_LAYOUT_GENERAL,
		};
		VkWriteDescriptorSet w[2] = {
		    {.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
		     .dstSet = s->sets[e],
		     .dstBinding = 0,
		     .descriptorCount = 1,
		     .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
		     .pImageInfo = &src},
		    {.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
		     .dstSet = s->sets[e],
		     .dstBinding = 1,
		     .descriptorCount = 1,
		     .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
		     .pImageInfo = &dst},
		};
		vk->vkUpdateDescriptorSets(vk->device, 2, w, 0, NULL);
	}

	VkCommandBufferBeginInfo bi = {
	    .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
	    .flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT,
	};
	VkCommandBuffer cmd = s->cmd;
	if (vk->vkBeginCommandBuffer(cmd, &bi) != VK_SUCCESS) {
		return false;
	}

	VkImageSubresourceRange range = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};

	// The frame's writes to the scratch images are done (same queue, earlier submit); the previous capture's copy
	// out of the bounce image too. Discard the bounce image's contents.
	VkMemoryBarrier mb = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER,
	    .srcAccessMask = VK_ACCESS_MEMORY_WRITE_BIT,
	    .dstAccessMask = VK_ACCESS_SHADER_READ_BIT,
	};
	VkImageMemoryBarrier to_general = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
	    .srcAccessMask = 0,
	    .dstAccessMask = VK_ACCESS_SHADER_WRITE_BIT,
	    .oldLayout = VK_IMAGE_LAYOUT_UNDEFINED,
	    .newLayout = VK_IMAGE_LAYOUT_GENERAL,
	    .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .image = rec->bounce,
	    .subresourceRange = range,
	};
	vk->vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &mb,
	                         0, NULL, 1, &to_general);

	vk->vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, rec->pipeline);
	for (uint32_t e = 0; e < rec->eyes; e++) {
		vk->vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, rec->pipeline_layout, 0, 1, &s->sets[e],
		                            0, NULL);
		struct render_compute_blit_push_data pc = {
		    .source_rect = {0.0f, 0.0f, 1.0f, 1.0f},
		    .target_rect = {.offset = {(int32_t)(e * rec->eye_extent.width), 0},
		                    .extent = {(int32_t)rec->eye_extent.width, (int32_t)rec->eye_extent.height}},
		};
		vk->vkCmdPushConstants(cmd, rec->pipeline_layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(pc), &pc);
		vk->vkCmdDispatch(cmd, (rec->eye_extent.width + 7) / 8, (rec->eye_extent.height + 7) / 8, 1);
	}

	VkImageMemoryBarrier to_src = to_general;
	to_src.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
	to_src.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
	to_src.oldLayout = VK_IMAGE_LAYOUT_GENERAL;
	to_src.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
	vk->vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, NULL, 0,
	                         NULL, 1, &to_src);

	VkBufferImageCopy region = {
	    .imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1},
	    .imageExtent = {rec->out_extent.width, rec->out_extent.height, 1},
	};
	vk->vkCmdCopyImageToBuffer(cmd, rec->bounce, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, s->buffer, 1, &region);

	VkBufferMemoryBarrier to_host = {
	    .sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER,
	    .srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT,
	    .dstAccessMask = VK_ACCESS_HOST_READ_BIT,
	    .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .buffer = s->buffer,
	    .offset = 0,
	    .size = VK_WHOLE_SIZE,
	};
	vk->vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_HOST_BIT, 0, 0, NULL, 1, &to_host,
	                         0, NULL);

	if (vk->vkEndCommandBuffer(cmd) != VK_SUCCESS) {
		return false;
	}

	VkSubmitInfo si = {
	    .sType = VK_STRUCTURE_TYPE_SUBMIT_INFO,
	    .commandBufferCount = 1,
	    .pCommandBuffers = &cmd,
	};
	vk_queue_lock(vk->main_queue);
	ret = vk->vkQueueSubmit(vk->main_queue->queue, 1, &si, s->fence);
	vk_queue_unlock(vk->main_queue);
	return ret == VK_SUCCESS;
}

void
comp_recorder_capture(struct comp_recorder *rec,
                      const struct chl_scratch_state *ss,
                      uint64_t frame_id,
                      uint64_t predicted_display_time_ns)
{
	if (rec == NULL || atomic_load_explicit(&rec->state, memory_order_acquire) != REC_READY) {
		return;
	}
	atomic_store(&rec->busy, true);
	if (atomic_load(&rec->state) != REC_READY) {
		atomic_store(&rec->busy, false);
		return;
	}
	COMP_TRACE_MARKER();
	(void)frame_id;

	struct pimax_rec_header *h = rec->hdr;
	uint64_t t = predicted_display_time_ns;
	const uint64_t slop = 2ull * 1000 * 1000;

	if (rec->next_due_ns != 0 && t + slop < rec->next_due_ns) {
		goto out; // not due yet (fps pacing)
	}
	// Keep the cadence; after a gap (or at the start) restart it from this frame.
	rec->next_due_ns = (rec->next_due_ns != 0 && t < rec->next_due_ns + rec->interval_ns)
	                       ? rec->next_due_ns + rec->interval_ns
	                       : t + rec->interval_ns;

	struct comp_compositor *c = rec->c;
	for (uint32_t e = 0; e < rec->eyes; e++) {
		if (!ss->views[e].used || c->scratch.views[e].cssi.info.width != rec->src_extent.width ||
		    c->scratch.views[e].cssi.info.height != rec->src_extent.height) {
			hdr_add64(&h->skipped, 1);
			goto out;
		}
	}

	struct rec_gpu_slot *slot = NULL;
	for (int i = 0; i < REC_GPU_SLOTS; i++) {
		if (atomic_load(&rec->slots[i].state) == GPU_FREE) {
			slot = &rec->slots[i];
			break;
		}
	}
	if (slot == NULL) {
		hdr_add64(&h->dropped_busy, 1);
		goto out;
	}

	slot->seq = rec->seq++;
	slot->timestamp_ns = t;
	if (record_and_submit(rec, slot, ss)) {
		atomic_store(&slot->state, GPU_SUBMITTED);
	} else {
		atomic_fetch_add(&rec->errors, 1);
	}

out:
	atomic_store(&rec->busy, false);
}

bool
comp_recorder_wants_layer_squash(void)
{
	return atomic_load_explicit(&g_squash, memory_order_relaxed);
}


/*
 *
 * Create / destroy.
 *
 */

struct comp_recorder *
comp_recorder_create(struct comp_compositor *c)
{
	const char *env = getenv("PIMAX_RECORD");
	if (env != NULL && strcmp(env, "0") == 0) {
		return NULL;
	}

	struct comp_recorder *rec = U_TYPED_CALLOC(struct comp_recorder);
	rec->c = c;
	rec->vk = &c->base.vk;
	rec->fd = -1;

	const char *p = getenv("PIMAX_RECORD_SHM");
	if (p != NULL && p[0] != '\0') {
		snprintf(rec->path, sizeof(rec->path), "%s", p);
	} else {
		const char *rt = getenv("XDG_RUNTIME_DIR");
		snprintf(rec->path, sizeof(rec->path), "%s/pimax-record.shm", rt != NULL ? rt : "/tmp");
	}

	if (pthread_create(&rec->thread, NULL, worker, rec) != 0) {
		REC_ERR("pthread_create failed: recording disabled");
		free(rec);
		return NULL;
	}
	rec->thread_started = true;
	REC_LOG("tap ready (waiting for %s)", rec->path);
	return rec;
}

void
comp_recorder_destroy(struct comp_recorder **rec_ptr)
{
	struct comp_recorder *rec = *rec_ptr;
	if (rec == NULL) {
		return;
	}
	atomic_store(&rec->quit, true);
	if (rec->thread_started) {
		pthread_join(rec->thread, NULL); // the worker detaches on its way out
	}
	free(rec);
	*rec_ptr = NULL;
}
