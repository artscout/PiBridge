// SPDX-License-Identifier: MIT
// Test stub: Monado's log macros, to stderr.
#pragma once
#include <stdio.h>
#define U_LOG_I(...) (fprintf(stderr, "monado I: " __VA_ARGS__), fputc('\n', stderr))
#define U_LOG_W(...) (fprintf(stderr, "monado W: " __VA_ARGS__), fputc('\n', stderr))
#define U_LOG_E(...) (fprintf(stderr, "monado E: " __VA_ARGS__), fputc('\n', stderr))
