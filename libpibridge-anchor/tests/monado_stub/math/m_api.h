// SPDX-License-Identifier: MIT
// Test stub: the few Monado math functions pimax_anchor.h calls (Monado implements them with Eigen in m_base.cpp).
// Plain float maths; the test checks layouts and parsed values, not Monado's rounding.
#pragma once
#include "xrt/xrt_defines.h"
#include <math.h>

static inline void math_quat_normalize(struct xrt_quat *q)
{
	float n = sqrtf(q->x * q->x + q->y * q->y + q->z * q->z + q->w * q->w);
	q->x /= n; q->y /= n; q->z /= n; q->w /= n;
}

static inline void math_quat_rotate(const struct xrt_quat *a, const struct xrt_quat *b, struct xrt_quat *r)
{
	struct xrt_quat o;
	o.w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
	o.x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
	o.y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
	o.z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;
	*r = o;
}

static inline void math_quat_rotate_vec3(const struct xrt_quat *q, const struct xrt_vec3 *v, struct xrt_vec3 *r)
{
	struct xrt_quat p = {v->x, v->y, v->z, 0.0f};
	struct xrt_quat c = {-q->x, -q->y, -q->z, q->w};
	struct xrt_quat t, u;
	math_quat_rotate(q, &p, &t);
	math_quat_rotate(&t, &c, &u);
	r->x = u.x; r->y = u.y; r->z = u.z;
}

static inline void math_quat_slerp(const struct xrt_quat *a, const struct xrt_quat *b, float t, struct xrt_quat *r)
{
	// normalised lerp (enough for a stub)
	float d = a->x * b->x + a->y * b->y + a->z * b->z + a->w * b->w;
	float s = d < 0.0f ? -1.0f : 1.0f;
	r->x = a->x * (1 - t) + s * b->x * t;
	r->y = a->y * (1 - t) + s * b->y * t;
	r->z = a->z * (1 - t) + s * b->z * t;
	r->w = a->w * (1 - t) + s * b->w * t;
	math_quat_normalize(r);
}

static inline void math_pose_transform(const struct xrt_pose *t, const struct xrt_pose *p, struct xrt_pose *o)
{
	struct xrt_pose r;
	math_quat_rotate_vec3(&t->orientation, &p->position, &r.position);
	r.position.x += t->position.x;
	r.position.y += t->position.y;
	r.position.z += t->position.z;
	math_quat_rotate(&t->orientation, &p->orientation, &r.orientation);
	*o = r;
}
