// Copyright 2026, pimax-re.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief PIMAX_RTTIME_PRELOAD: LD_PRELOAD for SteamVR started on the host (`pimax vr start`, Games run with: SteamVR).
 *
 * vrserver's scheduler setup sets RLIMIT_RTTIME to 0. Inside Steam's container the compositor is started by Steam's
 * launcher service and does not inherit that; started on the host it is vrserver's child, inherits 0, gets
 * realtime threads through vrcompositor-launcher's cap_sys_nice, and the kernel SIGKILLs it within a second of
 * "Startup Complete" (no log, no dump; measured: rttime 0 -> killed after 1 s, unlimited -> runs). This library
 * makes setrlimit/prlimit calls on RLIMIT_RTTIME no-ops, so the limit stays what `pimax vr start` gave (unlimited).
 * Every other call goes to libc unchanged.
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdio.h>
#include <sys/resource.h>
#include <sys/types.h>

static void
note(const char *fn)
{
	static int said = 0;
	if (!said) {
		said = 1;
		fprintf(stderr, "PIMAX_RTTIME_PRELOAD: %s(RLIMIT_RTTIME) ignored (keeps the compositor alive)\n", fn);
	}
}

int
setrlimit(__rlimit_resource_t resource, const struct rlimit *rlim)
{
	if (resource == RLIMIT_RTTIME) {
		note("setrlimit");
		return 0;
	}
	int (*real)(__rlimit_resource_t, const struct rlimit *) = dlsym(RTLD_NEXT, "setrlimit");
	return real != NULL ? real(resource, rlim) : -1;
}

int
setrlimit64(__rlimit_resource_t resource, const struct rlimit64 *rlim)
{
	if (resource == RLIMIT_RTTIME) {
		note("setrlimit64");
		return 0;
	}
	int (*real)(__rlimit_resource_t, const struct rlimit64 *) = dlsym(RTLD_NEXT, "setrlimit64");
	return real != NULL ? real(resource, rlim) : -1;
}

int
prlimit(pid_t pid, __rlimit_resource_t resource, const struct rlimit *new_limit, struct rlimit *old_limit)
{
	int (*real)(pid_t, __rlimit_resource_t, const struct rlimit *, struct rlimit *) = dlsym(RTLD_NEXT, "prlimit");
	if (resource == RLIMIT_RTTIME && new_limit != NULL) {
		note("prlimit");
		new_limit = NULL; // only report the old value
	}
	return real != NULL ? real(pid, resource, new_limit, old_limit) : -1;
}

int
prlimit64(pid_t pid, __rlimit_resource_t resource, const struct rlimit64 *new_limit, struct rlimit64 *old_limit)
{
	int (*real)(pid_t, __rlimit_resource_t, const struct rlimit64 *, struct rlimit64 *) = dlsym(RTLD_NEXT, "prlimit64");
	if (resource == RLIMIT_RTTIME && new_limit != NULL) {
		note("prlimit64");
		new_limit = NULL;
	}
	return real != NULL ? real(pid, resource, new_limit, old_limit) : -1;
}
