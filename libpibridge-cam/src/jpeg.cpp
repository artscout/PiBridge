// SPDX-License-Identifier: MIT
#include "pibridge/cam/jpeg.h"

#include <csetjmp>
#include <cstdio>
#include <cstdlib>
#include <string>

// libjpeg's error handler longjmps out of the library, so nothing with a destructor may be live between the setjmp
// and the call that may fail: the decompress struct is created and destroyed by hand on both paths.
extern "C" {
#include <jpeglib.h>
}

namespace pibridge::cam {
namespace {

struct ErrMgr {
    jpeg_error_mgr pub;
    std::jmp_buf jb;
    char msg[JMSG_LENGTH_MAX] = {};
};

void on_error(j_common_ptr c) {
    ErrMgr* e = reinterpret_cast<ErrMgr*>(c->err);
    (*c->err->format_message)(c, e->msg);
    std::longjmp(e->jb, 1);
}

/// Warnings ("Corrupt JPEG data" on a dropped UVC buffer, for example) would otherwise go to stderr on every frame.
void say_nothing(j_common_ptr, int) {}

/// jpeg_std_error installs its own handlers, so both are set through here after it ran.
void setup_err(ErrMgr& em) {
    em.pub.error_exit = on_error;
    em.pub.emit_message = say_nothing;
}

} // namespace

bool looks_like_jpeg(std::span<const std::uint8_t> bytes) {
    return bytes.size() >= 2 && bytes[0] == 0xff && bytes[1] == 0xd8;
}

Result<void> decode_grey_jpeg(std::span<const std::uint8_t> jpeg, std::span<std::uint8_t> out, std::uint32_t& w,
                              std::uint32_t& h) {
    if (!looks_like_jpeg(jpeg)) {
        return fail(IoError::other("not a JPEG image"));
    }
    ErrMgr em;
    jpeg_decompress_struct ci{};
    ci.err = jpeg_std_error(&em.pub);   // resets the handlers, so they go on afterwards
    setup_err(em);
    if (setjmp(em.jb)) {
        jpeg_destroy_decompress(&ci);
        return fail(IoError::other(std::string("jpeg: ") + em.msg));
    }
    jpeg_create_decompress(&ci);
    jpeg_mem_src(&ci, jpeg.data(), static_cast<unsigned long>(jpeg.size()));
    if (jpeg_read_header(&ci, TRUE) != JPEG_HEADER_OK) {
        jpeg_destroy_decompress(&ci);
        return fail(IoError::other("jpeg: no image in the stream"));
    }
    ci.out_color_space = JCS_GRAYSCALE;
    jpeg_start_decompress(&ci);
    const std::uint32_t dw = ci.output_width, dh = ci.output_height;
    if (dw == 0 || dh == 0 || ci.output_components != 1) {
        jpeg_destroy_decompress(&ci);
        return fail(IoError::other("jpeg: not a single-channel image"));
    }
    if (out.size() < static_cast<std::size_t>(dw) * dh) {
        jpeg_destroy_decompress(&ci);
        return fail(IoError::other("jpeg: output buffer too small"));
    }
    std::uint8_t* at = out.data();
    while (ci.output_scanline < dh) {
        JSAMPROW row = at;
        (void)jpeg_read_scanlines(&ci, &row, 1);
        at += dw;
    }
    jpeg_finish_decompress(&ci);
    jpeg_destroy_decompress(&ci);
    w = dw;
    h = dh;
    return {};
}

} // namespace pibridge::cam
