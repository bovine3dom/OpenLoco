/*
 * Copyright (c) 2014 Clément Bœsch
 *
 * This file is part of FFmpeg.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * @file
 * hqx magnification filters (hq2x, hq3x, hq4x)
 *
 * Originally designed by Maxim Stepin.
 *
 * @see http://en.wikipedia.org/wiki/Hqx
 * @see http://web.archive.org/web/20131114143602/http://www.hiend3d.com/hq3x.html
 * @see http://blog.pkh.me/p/19-butchering-hqx-scaling-filters.html
 */

/*
 * Ported from FFmpeg's libavfilter/vf_hqx.c at commit
 * f7368f97b92a0afe8dc8368a4b6749704b740317:
 * https://github.com/FFmpeg/FFmpeg/blob/f7368f97b92a0afe8dc8368a4b6749704b740317/libavfilter/vf_hqx.c
 */

#include "Graphics/PixelScaling.h"

#include <array>
#include <limits>

namespace OpenLoco::Gfx
{
    namespace
    {
        constexpr size_t kPaletteSize = 1U << std::numeric_limits<PaletteIndex_t>::digits;

        struct HqxPixel
        {
            uint32_t argb;
            uint32_t yuv;

            constexpr operator uint32_t() const { return argb; }
        };

        static bool yuvDiff(uint32_t yuv1, uint32_t yuv2)
        {
            constexpr uint32_t kYMask = 0xFF0000;
            constexpr uint32_t kUMask = 0x00FF00;
            constexpr uint32_t kVMask = 0x0000FF;

            const auto absDiff = [](uint32_t a, uint32_t b) { return a > b ? a - b : b - a; };
            return absDiff(yuv1 & kYMask, yuv2 & kYMask) > (48U << 16)
                || absDiff(yuv1 & kUMask, yuv2 & kUMask) > (7U << 8)
                || absDiff(yuv1 & kVMask, yuv2 & kVMask) > 6U;
        }

        /* (c1*w1 + c2*w2) >> s */
        static uint32_t interp2Px(uint32_t c1, int w1, uint32_t c2, int w2, int s)
        {
            return (((((c1 & 0xFF00FF00) >> 8) * w1 + ((c2 & 0xFF00FF00) >> 8) * w2) << (8 - s)) & 0xFF00FF00)
                | (((((c1 & 0x00FF00FF)) * w1 + ((c2 & 0x00FF00FF)) * w2) >> s) & 0x00FF00FF);
        }

        /* (c1*w1 + c2*w2 + c3*w3) >> s */
        static uint32_t interp3Px(uint32_t c1, int w1, uint32_t c2, int w2, uint32_t c3, int w3, int s)
        {
            return (((((c1 & 0xFF00FF00) >> 8) * w1 + ((c2 & 0xFF00FF00) >> 8) * w2 + ((c3 & 0xFF00FF00) >> 8) * w3) << (8 - s)) & 0xFF00FF00)
                | (((((c1 & 0x00FF00FF)) * w1 + ((c2 & 0x00FF00FF)) * w2 + ((c3 & 0x00FF00FF)) * w3) >> s) & 0x00FF00FF);
        }

/* m is the mask of diff with the center pixel that matters in the pattern, and
 * r is the expected result (bit set to 1 if there is difference with the
 * center, 0 otherwise) */
#define P(m, r) ((kShuffled & (m)) == (r))

/* adjust 012345678 to 01235678: the mask doesn't contain the (null) diff
 * between the center/current pixel and itself */
#define DROP4(z) ((z) > 4 ? (z) - 1 : (z))

/* shuffle the input mask: move bit n (4-adjusted) to position stored in p<n> */
#define SHF(x, rot, n) ((((x) >> ((rot) ? 7 - DROP4(n) : DROP4(n))) & 1) << DROP4(p##n))

/* used to check if there is YUV difference between 2 pixels */
#define WDIFF(c1, c2) yuvDiff((c1).yuv, (c2).yuv)

/* bootstrap template for every interpolation code. It defines the shuffled
 * masks and surrounding pixels. The rot flag is used to indicate if it's a
 * rotation; its basic effect is to shuffle k using p8..p0 instead of p0..p8 */
#define INTERP_BOOTSTRAP(rot)                                                                                \
    const int kShuffled = SHF(k, rot, 0) | SHF(k, rot, 1) | SHF(k, rot, 2) | SHF(k, rot, 3) | SHF(k, rot, 5) \
        | SHF(k, rot, 6) | SHF(k, rot, 7) | SHF(k, rot, 8);                                                  \
    const HqxPixel w0 = w[p0], w1 = w[p1], w3 = w[p3], w4 = w[p4], w5 = w[p5], w7 = w[p7]

        /* Assuming p0..p8 is mapped to pixels 0..8, this function interpolates the
         * top-left pixel in the total of the 2x2 pixels to interpolates. The function
         * is also used for the 3 other pixels */
        static uint32_t hq2xInterp1x1(
            int k,
            const HqxPixel* w,
            int p0,
            int p1,
            int p2,
            int p3,
            int p4,
            int p5,
            int p6,
            int p7,
            int p8)
        {
            INTERP_BOOTSTRAP(0);

            if ((P(0xBF, 0x37) || P(0xDB, 0x13)) && WDIFF(w1, w5))
            {
                return interp2Px(w4, 3, w3, 1, 2);
            }
            if ((P(0xDB, 0x49) || P(0xEF, 0x6D)) && WDIFF(w7, w3))
            {
                return interp2Px(w4, 3, w1, 1, 2);
            }
            if ((P(0x0B, 0x0B) || P(0xFE, 0x4A) || P(0xFE, 0x1A)) && WDIFF(w3, w1))
            {
                return w4;
            }
            if ((P(0x6F, 0x2A) || P(0x5B, 0x0A) || P(0xBF, 0x3A) || P(0xDF, 0x5A)
                 || P(0x9F, 0x8A) || P(0xCF, 0x8A) || P(0xEF, 0x4E) || P(0x3F, 0x0E)
                 || P(0xFB, 0x5A) || P(0xBB, 0x8A) || P(0x7F, 0x5A) || P(0xAF, 0x8A)
                 || P(0xEB, 0x8A))
                && WDIFF(w3, w1))
            {
                return interp2Px(w4, 3, w0, 1, 2);
            }
            if (P(0x0B, 0x08))
            {
                return interp3Px(w4, 2, w0, 1, w1, 1, 2);
            }
            if (P(0x0B, 0x02))
            {
                return interp3Px(w4, 2, w0, 1, w3, 1, 2);
            }
            if (P(0x2F, 0x2F))
            {
                return interp3Px(w4, 14, w3, 1, w1, 1, 4);
            }
            if (P(0xBF, 0x37) || P(0xDB, 0x13))
            {
                return interp3Px(w4, 5, w1, 2, w3, 1, 3);
            }
            if (P(0xDB, 0x49) || P(0xEF, 0x6D))
            {
                return interp3Px(w4, 5, w3, 2, w1, 1, 3);
            }
            if (P(0x1B, 0x03) || P(0x4F, 0x43) || P(0x8B, 0x83) || P(0x6B, 0x43))
            {
                return interp2Px(w4, 3, w3, 1, 2);
            }
            if (P(0x4B, 0x09) || P(0x8B, 0x89) || P(0x1F, 0x19) || P(0x3B, 0x19))
            {
                return interp2Px(w4, 3, w1, 1, 2);
            }
            if (P(0x7E, 0x2A) || P(0xEF, 0xAB) || P(0xBF, 0x8F) || P(0x7E, 0x0E))
            {
                return interp3Px(w4, 2, w3, 3, w1, 3, 3);
            }
            if (P(0xFB, 0x6A) || P(0x6F, 0x6E) || P(0x3F, 0x3E) || P(0xFB, 0xFA)
                || P(0xDF, 0xDE) || P(0xDF, 0x1E))
            {
                return interp2Px(w4, 3, w0, 1, 2);
            }
            if (P(0x0A, 0x00) || P(0x4F, 0x4B) || P(0x9F, 0x1B) || P(0x2F, 0x0B)
                || P(0xBE, 0x0A) || P(0xEE, 0x0A) || P(0x7E, 0x0A) || P(0xEB, 0x4B)
                || P(0x3B, 0x1B))
            {
                return interp3Px(w4, 2, w3, 1, w1, 1, 2);
            }
            return interp3Px(w4, 6, w3, 1, w1, 1, 3);
        }

        /* Assuming p0..p8 is mapped to pixels 0..8, this function interpolates the
         * top-left and top-center pixel in the total of the 3x3 pixels to
         * interpolates. The function is also used for the 3 other couples of pixels
         * defining the outline. The center pixel is not defined through this function,
         * since it's just the same as the original value. */
        static void hq3xInterp2x1(
            uint32_t* dst,
            size_t dstPitch,
            int k,
            const HqxPixel* w,
            int pos00,
            int pos01,
            int p0,
            int p1,
            int p2,
            int p3,
            int p4,
            int p5,
            int p6,
            int p7,
            int p8,
            int rotate)
        {
            INTERP_BOOTSTRAP(rotate);

            uint32_t* dst00 = &dst[dstPitch * (pos00 >> 1) + (pos00 & 1)];
            uint32_t* dst01 = &dst[dstPitch * (pos01 >> 1) + (pos01 & 1)];

            if ((P(0xDB, 0x49) || P(0xEF, 0x6D)) && WDIFF(w7, w3))
            {
                *dst00 = interp2Px(w4, 3, w1, 1, 2);
            }
            else if ((P(0xBF, 0x37) || P(0xDB, 0x13)) && WDIFF(w1, w5))
            {
                *dst00 = interp2Px(w4, 3, w3, 1, 2);
            }
            else if ((P(0x0B, 0x0B) || P(0xFE, 0x4A) || P(0xFE, 0x1A)) && WDIFF(w3, w1))
            {
                *dst00 = w4;
            }
            else if ((P(0x6F, 0x2A) || P(0x5B, 0x0A) || P(0xBF, 0x3A) || P(0xDF, 0x5A) || P(0x9F, 0x8A) || P(0xCF, 0x8A) || P(0xEF, 0x4E) || P(0x3F, 0x0E) || P(0xFB, 0x5A) || P(0xBB, 0x8A) || P(0x7F, 0x5A) || P(0xAF, 0x8A) || P(0xEB, 0x8A)) && WDIFF(w3, w1))
            {
                *dst00 = interp2Px(w4, 3, w0, 1, 2);
            }
            else if (P(0x4B, 0x09) || P(0x8B, 0x89) || P(0x1F, 0x19) || P(0x3B, 0x19))
            {
                *dst00 = interp2Px(w4, 3, w1, 1, 2);
            }
            else if (P(0x1B, 0x03) || P(0x4F, 0x43) || P(0x8B, 0x83) || P(0x6B, 0x43))
            {
                *dst00 = interp2Px(w4, 3, w3, 1, 2);
            }
            else if (P(0x7E, 0x2A) || P(0xEF, 0xAB) || P(0xBF, 0x8F) || P(0x7E, 0x0E))
            {
                *dst00 = interp2Px(w3, 1, w1, 1, 1);
            }
            else if (P(0x4F, 0x4B) || P(0x9F, 0x1B) || P(0x2F, 0x0B) || P(0xBE, 0x0A) || P(0xEE, 0x0A) || P(0x7E, 0x0A) || P(0xEB, 0x4B) || P(0x3B, 0x1B))
            {
                *dst00 = interp3Px(w4, 2, w3, 7, w1, 7, 4);
            }
            else if (P(0x0B, 0x08) || P(0xF9, 0x68) || P(0xF3, 0x62) || P(0x6D, 0x6C) || P(0x67, 0x66) || P(0x3D, 0x3C) || P(0x37, 0x36) || P(0xF9, 0xF8) || P(0xDD, 0xDC) || P(0xF3, 0xF2) || P(0xD7, 0xD6) || P(0xDD, 0x1C) || P(0xD7, 0x16) || P(0x0B, 0x02))
            {
                *dst00 = interp2Px(w4, 3, w0, 1, 2);
            }
            else
            {
                *dst00 = interp3Px(w4, 2, w3, 1, w1, 1, 2);
            }

            if ((P(0xFE, 0xDE) || P(0x9E, 0x16) || P(0xDA, 0x12) || P(0x17, 0x16)
                 || P(0x5B, 0x12) || P(0xBB, 0x12))
                && WDIFF(w1, w5))
            {
                *dst01 = w4;
            }
            else if ((P(0x0F, 0x0B) || P(0x5E, 0x0A) || P(0xFB, 0x7B) || P(0x3B, 0x0B) || P(0xBE, 0x0A) || P(0x7A, 0x0A)) && WDIFF(w3, w1))
            {
                *dst01 = w4;
            }
            else if (P(0xBF, 0x8F) || P(0x7E, 0x0E) || P(0xBF, 0x37) || P(0xDB, 0x13))
            {
                *dst01 = interp2Px(w1, 3, w4, 1, 2);
            }
            else if (P(0x02, 0x00) || P(0x7C, 0x28) || P(0xED, 0xA9) || P(0xF5, 0xB4) || P(0xD9, 0x90))
            {
                *dst01 = interp2Px(w4, 3, w1, 1, 2);
            }
            else if (P(0x4F, 0x4B) || P(0xFB, 0x7B) || P(0xFE, 0x7E) || P(0x9F, 0x1B) || P(0x2F, 0x0B) || P(0xBE, 0x0A) || P(0x7E, 0x0A) || P(0xFB, 0x4B) || P(0xFB, 0xDB) || P(0xFE, 0xDE) || P(0xFE, 0x56) || P(0x57, 0x56) || P(0x97, 0x16) || P(0x3F, 0x1E) || P(0xDB, 0x12) || P(0xBB, 0x12))
            {
                *dst01 = interp2Px(w4, 7, w1, 1, 3);
            }
            else
            {
                *dst01 = w4;
            }
        }

        /* Assuming p0..p8 is mapped to pixels 0..8, this function interpolates the
         * top-left block of 2x2 pixels in the total of the 4x4 pixels (or 4 blocks) to
         * interpolates. The function is also used for the 3 other blocks of 2x2
         * pixels. */
        static void hq4xInterp2x2(
            uint32_t* dst,
            size_t dstPitch,
            int k,
            const HqxPixel* w,
            int pos00,
            int pos01,
            int pos10,
            int pos11,
            int p0,
            int p1,
            int p2,
            int p3,
            int p4,
            int p5,
            int p6,
            int p7,
            int p8)
        {
            INTERP_BOOTSTRAP(0);

            uint32_t* dst00 = &dst[dstPitch * (pos00 >> 1) + (pos00 & 1)];
            uint32_t* dst01 = &dst[dstPitch * (pos01 >> 1) + (pos01 & 1)];
            uint32_t* dst10 = &dst[dstPitch * (pos10 >> 1) + (pos10 & 1)];
            uint32_t* dst11 = &dst[dstPitch * (pos11 >> 1) + (pos11 & 1)];

            const int cond00 = (P(0xBF, 0x37) || P(0xDB, 0x13)) && WDIFF(w1, w5);
            const int cond01 = (P(0xDB, 0x49) || P(0xEF, 0x6D)) && WDIFF(w7, w3);
            const int cond02 = (P(0x6F, 0x2A) || P(0x5B, 0x0A) || P(0xBF, 0x3A)
                                || P(0xDF, 0x5A) || P(0x9F, 0x8A) || P(0xCF, 0x8A)
                                || P(0xEF, 0x4E) || P(0x3F, 0x0E) || P(0xFB, 0x5A)
                                || P(0xBB, 0x8A) || P(0x7F, 0x5A) || P(0xAF, 0x8A)
                                || P(0xEB, 0x8A))
                && WDIFF(w3, w1);
            const int cond03 = P(0xDB, 0x49) || P(0xEF, 0x6D);
            const int cond04 = P(0xBF, 0x37) || P(0xDB, 0x13);
            const int cond05 = P(0x1B, 0x03) || P(0x4F, 0x43) || P(0x8B, 0x83)
                || P(0x6B, 0x43);
            const int cond06 = P(0x4B, 0x09) || P(0x8B, 0x89) || P(0x1F, 0x19)
                || P(0x3B, 0x19);
            const int cond07 = P(0x0B, 0x08) || P(0xF9, 0x68) || P(0xF3, 0x62)
                || P(0x6D, 0x6C) || P(0x67, 0x66) || P(0x3D, 0x3C)
                || P(0x37, 0x36) || P(0xF9, 0xF8) || P(0xDD, 0xDC)
                || P(0xF3, 0xF2) || P(0xD7, 0xD6) || P(0xDD, 0x1C)
                || P(0xD7, 0x16) || P(0x0B, 0x02);
            const int cond08 = (P(0x0F, 0x0B) || P(0x2B, 0x0B) || P(0xFE, 0x4A)
                                || P(0xFE, 0x1A))
                && WDIFF(w3, w1);
            const int cond09 = P(0x2F, 0x2F);
            const int cond10 = P(0x0A, 0x00);
            const int cond11 = P(0x0B, 0x09);
            const int cond12 = P(0x7E, 0x2A) || P(0xEF, 0xAB);
            const int cond13 = P(0xBF, 0x8F) || P(0x7E, 0x0E);
            const int cond14 = P(0x4F, 0x4B) || P(0x9F, 0x1B) || P(0x2F, 0x0B)
                || P(0xBE, 0x0A) || P(0xEE, 0x0A) || P(0x7E, 0x0A)
                || P(0xEB, 0x4B) || P(0x3B, 0x1B);
            const int cond15 = P(0x0B, 0x03);

            if (cond00)
            {
                *dst00 = interp2Px(w4, 5, w3, 3, 3);
            }
            else if (cond01)
            {
                *dst00 = interp2Px(w4, 5, w1, 3, 3);
            }
            else if ((P(0x0B, 0x0B) || P(0xFE, 0x4A) || P(0xFE, 0x1A)) && WDIFF(w3, w1))
            {
                *dst00 = w4;
            }
            else if (cond02)
            {
                *dst00 = interp2Px(w4, 5, w0, 3, 3);
            }
            else if (cond03)
            {
                *dst00 = interp2Px(w4, 3, w3, 1, 2);
            }
            else if (cond04)
            {
                *dst00 = interp2Px(w4, 3, w1, 1, 2);
            }
            else if (cond05)
            {
                *dst00 = interp2Px(w4, 5, w3, 3, 3);
            }
            else if (cond06)
            {
                *dst00 = interp2Px(w4, 5, w1, 3, 3);
            }
            else if (P(0x0F, 0x0B) || P(0x5E, 0x0A) || P(0x2B, 0x0B) || P(0xBE, 0x0A) || P(0x7A, 0x0A) || P(0xEE, 0x0A))
            {
                *dst00 = interp2Px(w1, 1, w3, 1, 1);
            }
            else if (cond07)
            {
                *dst00 = interp2Px(w4, 5, w0, 3, 3);
            }
            else
            {
                *dst00 = interp3Px(w4, 2, w1, 1, w3, 1, 2);
            }

            if (cond00)
            {
                *dst01 = interp2Px(w4, 7, w3, 1, 3);
            }
            else if (cond08)
            {
                *dst01 = w4;
            }
            else if (cond02)
            {
                *dst01 = interp2Px(w4, 3, w0, 1, 2);
            }
            else if (cond09)
            {
                *dst01 = w4;
            }
            else if (cond10)
            {
                *dst01 = interp3Px(w4, 5, w1, 2, w3, 1, 3);
            }
            else if (P(0x0B, 0x08))
            {
                *dst01 = interp3Px(w4, 5, w1, 2, w0, 1, 3);
            }
            else if (cond11)
            {
                *dst01 = interp2Px(w4, 5, w1, 3, 3);
            }
            else if (cond04)
            {
                *dst01 = interp2Px(w1, 3, w4, 1, 2);
            }
            else if (cond12)
            {
                *dst01 = interp3Px(w1, 2, w4, 1, w3, 1, 2);
            }
            else if (cond13)
            {
                *dst01 = interp2Px(w1, 5, w3, 3, 3);
            }
            else if (cond05)
            {
                *dst01 = interp2Px(w4, 7, w3, 1, 3);
            }
            else if (P(0xF3, 0x62) || P(0x67, 0x66) || P(0x37, 0x36) || P(0xF3, 0xF2) || P(0xD7, 0xD6) || P(0xD7, 0x16) || P(0x0B, 0x02))
            {
                *dst01 = interp2Px(w4, 3, w0, 1, 2);
            }
            else if (cond14)
            {
                *dst01 = interp2Px(w1, 1, w4, 1, 1);
            }
            else
            {
                *dst01 = interp2Px(w4, 3, w1, 1, 2);
            }

            if (cond01)
            {
                *dst10 = interp2Px(w4, 7, w1, 1, 3);
            }
            else if (cond08)
            {
                *dst10 = w4;
            }
            else if (cond02)
            {
                *dst10 = interp2Px(w4, 3, w0, 1, 2);
            }
            else if (cond09)
            {
                *dst10 = w4;
            }
            else if (cond10)
            {
                *dst10 = interp3Px(w4, 5, w3, 2, w1, 1, 3);
            }
            else if (P(0x0B, 0x02))
            {
                *dst10 = interp3Px(w4, 5, w3, 2, w0, 1, 3);
            }
            else if (cond15)
            {
                *dst10 = interp2Px(w4, 5, w3, 3, 3);
            }
            else if (cond03)
            {
                *dst10 = interp2Px(w3, 3, w4, 1, 2);
            }
            else if (cond13)
            {
                *dst10 = interp3Px(w3, 2, w4, 1, w1, 1, 2);
            }
            else if (cond12)
            {
                *dst10 = interp2Px(w3, 5, w1, 3, 3);
            }
            else if (cond06)
            {
                *dst10 = interp2Px(w4, 7, w1, 1, 3);
            }
            else if (P(0x0B, 0x08) || P(0xF9, 0x68) || P(0x6D, 0x6C) || P(0x3D, 0x3C) || P(0xF9, 0xF8) || P(0xDD, 0xDC) || P(0xDD, 0x1C))
            {
                *dst10 = interp2Px(w4, 3, w0, 1, 2);
            }
            else if (cond14)
            {
                *dst10 = interp2Px(w3, 1, w4, 1, 1);
            }
            else
            {
                *dst10 = interp2Px(w4, 3, w3, 1, 2);
            }

            if ((P(0x7F, 0x2B) || P(0xEF, 0xAB) || P(0xBF, 0x8F) || P(0x7F, 0x0F))
                && WDIFF(w3, w1))
            {
                *dst11 = w4;
            }
            else if (cond02)
            {
                *dst11 = interp2Px(w4, 7, w0, 1, 3);
            }
            else if (cond15)
            {
                *dst11 = interp2Px(w4, 7, w3, 1, 3);
            }
            else if (cond11)
            {
                *dst11 = interp2Px(w4, 7, w1, 1, 3);
            }
            else if (P(0x0A, 0x00) || P(0x7E, 0x2A) || P(0xEF, 0xAB) || P(0xBF, 0x8F) || P(0x7E, 0x0E))
            {
                *dst11 = interp3Px(w4, 6, w3, 1, w1, 1, 3);
            }
            else if (cond07)
            {
                *dst11 = interp2Px(w4, 7, w0, 1, 3);
            }
            else
            {
                *dst11 = w4;
            }
        }

#undef INTERP_BOOTSTRAP
#undef WDIFF
#undef SHF
#undef DROP4
#undef P

        static bool containsImage(size_t spanSize, size_t pitch, size_t width, size_t height)
        {
            if (pitch < width)
            {
                return false;
            }

            const auto rowsBeforeLast = height - 1;
            if (rowsBeforeLast > (std::numeric_limits<size_t>::max() - width) / pitch)
            {
                return false;
            }
            return rowsBeforeLast * pitch + width <= spanSize;
        }
    }

    bool scaleHqx(
        std::span<const PaletteIndex_t> src,
        int32_t width,
        int32_t height,
        size_t srcPitch,
        std::span<uint32_t> dst,
        size_t dstPitch,
        std::span<const PaletteEntry> palette,
        uint8_t factor)
    {
        if (factor < 2 || factor > 4 || width <= 0 || height <= 0 || palette.size() < kPaletteSize)
        {
            return false;
        }

        const auto srcWidth = static_cast<size_t>(width);
        const auto srcHeight = static_cast<size_t>(height);
        const auto scale = static_cast<size_t>(factor);
        constexpr auto kMaxDimension = static_cast<size_t>(std::numeric_limits<int32_t>::max());
        if (srcWidth > kMaxDimension / scale || srcHeight > kMaxDimension / scale)
        {
            return false;
        }

        const auto dstWidth = srcWidth * scale;
        const auto dstHeight = srcHeight * scale;
        if (!containsImage(src.size(), srcPitch, srcWidth, srcHeight)
            || !containsImage(dst.size(), dstPitch, dstWidth, dstHeight))
        {
            return false;
        }

        std::array<HqxPixel, kPaletteSize> hqxPalette{};
        for (size_t i = 0; i < hqxPalette.size(); ++i)
        {
            const auto& entry = palette[i];
            const int rg = static_cast<int>(entry.r) - entry.g;
            const int bg = static_cast<int>(entry.b) - entry.g;
            const auto y = static_cast<uint32_t>((299 * rg + 1000 * entry.g + 114 * bg) / 1000);
            const auto u = static_cast<uint32_t>((-169 * rg + 500 * bg) / 1000) + 128;
            const auto v = static_cast<uint32_t>((500 * rg - 81 * bg) / 1000) + 128;
            hqxPalette[i] = {
                0xFF000000U | (static_cast<uint32_t>(entry.r) << 16) | (static_cast<uint32_t>(entry.g) << 8) | entry.b,
                (y << 16) + (u << 8) + v,
            };
        }

        for (size_t y = 0; y < srcHeight; ++y)
        {
            const auto prevRow = (y == 0 ? y : y - 1) * srcPitch;
            const auto row = y * srcPitch;
            const auto nextRow = (y + 1 == srcHeight ? y : y + 1) * srcPitch;
            auto* dstRow = dst.data() + y * scale * dstPitch;

            for (size_t x = 0; x < srcWidth; ++x)
            {
                const auto prevCol = x == 0 ? x : x - 1;
                const auto nextCol = x + 1 == srcWidth ? x : x + 1;
                const HqxPixel w[9] = {
                    hqxPalette[src[prevRow + prevCol]],
                    hqxPalette[src[prevRow + x]],
                    hqxPalette[src[prevRow + nextCol]],
                    hqxPalette[src[row + prevCol]],
                    hqxPalette[src[row + x]],
                    hqxPalette[src[row + nextCol]],
                    hqxPalette[src[nextRow + prevCol]],
                    hqxPalette[src[nextRow + x]],
                    hqxPalette[src[nextRow + nextCol]],
                };
                const auto differs = [&](size_t index) {
                    return w[4].argb != w[index].argb && yuvDiff(w[4].yuv, w[index].yuv);
                };
                const int pattern = static_cast<int>(differs(0))
                    | static_cast<int>(differs(1)) << 1
                    | static_cast<int>(differs(2)) << 2
                    | static_cast<int>(differs(3)) << 3
                    | static_cast<int>(differs(5)) << 4
                    | static_cast<int>(differs(6)) << 5
                    | static_cast<int>(differs(7)) << 6
                    | static_cast<int>(differs(8)) << 7;

                auto* dstPixel = dstRow + x * scale;
                switch (factor)
                {
                    case 2:
                        dstPixel[dstPitch * 0 + 0] = hq2xInterp1x1(pattern, w, 0, 1, 2, 3, 4, 5, 6, 7, 8); // 00
                        dstPixel[dstPitch * 0 + 1] = hq2xInterp1x1(pattern, w, 2, 1, 0, 5, 4, 3, 8, 7, 6); // 01 (vert mirrored)
                        dstPixel[dstPitch * 1 + 0] = hq2xInterp1x1(pattern, w, 6, 7, 8, 3, 4, 5, 0, 1, 2); // 10 (horiz mirrored)
                        dstPixel[dstPitch * 1 + 1] = hq2xInterp1x1(pattern, w, 8, 7, 6, 5, 4, 3, 2, 1, 0); // 11 (center mirrored)
                        break;

                    case 3:
                        hq3xInterp2x1(dstPixel, dstPitch, pattern, w, 0, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0);                // 00 01
                        hq3xInterp2x1(dstPixel + 1, dstPitch, pattern, w, 1, 3, 2, 5, 8, 1, 4, 7, 0, 3, 6, 1);            // 02 12 (rotated to the right)
                        hq3xInterp2x1(dstPixel + dstPitch, dstPitch, pattern, w, 2, 0, 6, 3, 0, 7, 4, 1, 8, 5, 2, 1);     // 20 10 (rotated to the left)
                        hq3xInterp2x1(dstPixel + dstPitch + 1, dstPitch, pattern, w, 3, 2, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0); // 22 21 (center mirrored)
                        dstPixel[dstPitch + 1] = w[4];                                                                    // 11
                        break;

                    case 4:
                        hq4xInterp2x2(dstPixel, dstPitch, pattern, w, 0, 1, 2, 3, 0, 1, 2, 3, 4, 5, 6, 7, 8);                    // 00 01 10 11
                        hq4xInterp2x2(dstPixel + 2, dstPitch, pattern, w, 1, 0, 3, 2, 2, 1, 0, 5, 4, 3, 8, 7, 6);                // 02 03 12 13 (vert mirrored)
                        hq4xInterp2x2(dstPixel + 2 * dstPitch, dstPitch, pattern, w, 2, 3, 0, 1, 6, 7, 8, 3, 4, 5, 0, 1, 2);     // 20 21 30 31 (horiz mirrored)
                        hq4xInterp2x2(dstPixel + 2 * dstPitch + 2, dstPitch, pattern, w, 3, 2, 1, 0, 8, 7, 6, 5, 4, 3, 2, 1, 0); // 22 23 32 33 (center mirrored)
                        break;
                }
            }
        }

        return true;
    }
}
