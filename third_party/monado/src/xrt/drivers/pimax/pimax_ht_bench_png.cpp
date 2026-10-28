// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PNG loader for pimax_ht_bench (EuRoC recordings from PIMAX_SLAM_RECORD), via OpenCV.
 * @ingroup drv_pimax
 */

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cstdint>
#include <cstring>

extern "C" int
pimax_ht_bench_load_grey(const char *path, uint8_t *dst, uint32_t w, uint32_t h, uint32_t stride)
{
	cv::Mat m = cv::imread(path, cv::IMREAD_GRAYSCALE);
	if (m.empty() || (uint32_t)m.cols != w || (uint32_t)m.rows != h) {
		return -1;
	}
	for (uint32_t y = 0; y < h; y++) {
		std::memcpy(dst + (size_t)y * stride, m.ptr<uint8_t>((int)y), w);
	}
	return 0;
}
