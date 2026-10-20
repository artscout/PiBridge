// SPDX-License-Identifier: MIT
// Monado's side of the anchor files for test_monado.cpp: omarchyvr's forks/monado pimax_anchor.h itself (the header
// the Pimax driver includes, from the checkout), driven through two plain C functions.
#include "pimax_anchor.h"

int64_t pibridge_test_clock_shift_ns = 0;

/// One compositor head query at `at_ns` with the raw pose `p` (px py pz qx qy qz qw) and the relation flags; the
/// pose after the hook (offset applied) goes back into `out`.
void pibridge_monado_anchor_query(int64_t at_ns, const float p[7], uint32_t flags, float out[7])
{
	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	rel.relation_flags = (enum xrt_space_relation_flags)flags;
	rel.pose.position = (struct xrt_vec3){p[0], p[1], p[2]};
	rel.pose.orientation = (struct xrt_quat){p[3], p[4], p[5], p[6]};
	pimax_anchor_process(at_ns, &rel);
	out[0] = rel.pose.position.x;
	out[1] = rel.pose.position.y;
	out[2] = rel.pose.position.z;
	out[3] = rel.pose.orientation.x;
	out[4] = rel.pose.orientation.y;
	out[5] = rel.pose.orientation.z;
	out[6] = rel.pose.orientation.w;
}

/// The hook's state: enabled, an offset parsed from the file (its 7 floats), the applied offset.
int pibridge_monado_anchor_state(float offset[7], float applied[7])
{
	struct pimax_anchor_state *s = &g_pimax_anchor;
	const struct xrt_pose *o[2] = {&s->offset, &s->applied};
	float *d[2] = {offset, applied};
	for (int i = 0; i < 2; i++) {
		d[i][0] = o[i]->position.x;
		d[i][1] = o[i]->position.y;
		d[i][2] = o[i]->position.z;
		d[i][3] = o[i]->orientation.x;
		d[i][4] = o[i]->orientation.y;
		d[i][5] = o[i]->orientation.z;
		d[i][6] = o[i]->orientation.w;
	}
	return (s->enabled ? 1 : 0) | (s->have_offset ? 2 : 0) | (s->have_applied ? 4 : 0);
}

void pibridge_monado_clock_shift(int64_t ns)
{
	pibridge_test_clock_shift_ns += ns;
}
