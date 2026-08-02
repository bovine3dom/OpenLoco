/*
 * Copyright 2020 Morgan McGuire & Mara Gagiu
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// Adapted from supplement/code/source/cppPerf.cpp in the official supplement:
// https://jcgt.org/published/0010/02/04/supplement.zip.

#include "Graphics/PixelScaling.h"

#include <algorithm>
#include <limits>

namespace OpenLoco::Gfx
{
    namespace
    {
        template<typename... T>
        constexpr bool allEqual(PaletteIndex_t value, T... candidates)
        {
            return ((value == candidates) && ...);
        }

        template<typename... T>
        constexpr bool anyEqual(PaletteIndex_t value, T... candidates)
        {
            return ((value == candidates) || ...);
        }

        template<typename... T>
        constexpr bool noneEqual(PaletteIndex_t value, T... candidates)
        {
            return ((value != candidates) && ...);
        }

        constexpr bool isValidBuffer(size_t size, size_t rowWidth, size_t rowCount, size_t pitch)
        {
            if (pitch < rowWidth)
            {
                return false;
            }

            const auto precedingRows = rowCount - 1;
            if (precedingRows != 0 && pitch > (std::numeric_limits<size_t>::max() - rowWidth) / precedingRows)
            {
                return false;
            }

            return precedingRows * pitch + rowWidth <= size;
        }
    }

    bool scaleMmpx2x(
        std::span<const PaletteIndex_t> src,
        int32_t width,
        int32_t height,
        size_t srcPitch,
        std::span<PaletteIndex_t> dst,
        size_t dstPitch,
        std::span<const PaletteEntry> palette)
    {
        if (width <= 0 || height <= 0 || palette.size() < 256)
        {
            return false;
        }

        constexpr size_t kScale = 2;
        const auto srcWidth = static_cast<size_t>(width);
        const auto srcHeight = static_cast<size_t>(height);
        constexpr auto kMaxDimension = static_cast<size_t>(std::numeric_limits<int32_t>::max());
        if (srcWidth > kMaxDimension / kScale || srcHeight > kMaxDimension / kScale)
        {
            return false;
        }

        const auto dstWidth = srcWidth * kScale;
        const auto dstHeight = srcHeight * kScale;
        if (!isValidBuffer(src.size(), srcWidth, srcHeight, srcPitch)
            || !isValidBuffer(dst.size(), dstWidth, dstHeight, dstPitch))
        {
            return false;
        }

        const auto sample = [&](int64_t x, int64_t y) {
            x = std::clamp<int64_t>(x, 0, width - 1);
            y = std::clamp<int64_t>(y, 0, height - 1);
            return src[static_cast<size_t>(y) * srcPitch + static_cast<size_t>(x)];
        };
        const auto luminance = [&](PaletteIndex_t index) {
            const auto& colour = palette[index];
            return static_cast<uint32_t>(colour.r) + colour.g + colour.b + 1;
        };

        for (int64_t srcY = 0; srcY < height; ++srcY)
        {
            int64_t srcX = 0;

            auto A = sample(srcX - 1, srcY - 1);
            auto B = sample(srcX, srcY - 1);
            auto C = sample(srcX + 1, srcY - 1);
            auto D = sample(srcX - 1, srcY);
            auto E = sample(srcX, srcY);
            auto F = sample(srcX + 1, srcY);
            auto G = sample(srcX - 1, srcY + 1);
            auto H = sample(srcX, srcY + 1);
            auto I = sample(srcX + 1, srcY + 1);
            auto Q = sample(srcX - 2, srcY);
            auto R = sample(srcX + 2, srcY);

            for (; srcX < width; ++srcX)
            {
                auto J = E;
                auto K = E;
                auto L = E;
                auto M = E;

                if (A != E || B != E || C != E || D != E || F != E || G != E || H != E || I != E)
                {
                    const auto P = sample(srcX, srcY - 2);
                    const auto S = sample(srcX, srcY + 2);
                    const auto Bl = luminance(B);
                    const auto Dl = luminance(D);
                    const auto El = luminance(E);
                    const auto Fl = luminance(F);
                    const auto Hl = luminance(H);

                    // 1:1 slope rules
                    if (D == B && D != H && D != F && (El >= Dl || E == A) && anyEqual(E, A, C, G) && (El < Dl || A != D || E != P || E != Q))
                    {
                        J = D;
                    }
                    if (B == F && B != D && B != H && (El >= Bl || E == C) && anyEqual(E, A, C, I) && (El < Bl || C != B || E != P || E != R))
                    {
                        K = B;
                    }
                    if (H == D && H != F && H != B && (El >= Hl || E == G) && anyEqual(E, A, G, I) && (El < Hl || G != H || E != S || E != Q))
                    {
                        L = H;
                    }
                    if (F == H && F != B && F != D && (El >= Fl || E == I) && anyEqual(E, C, G, I) && (El < Fl || I != H || E != R || E != S))
                    {
                        M = F;
                    }

                    // Intersection rules
                    if (E != F && allEqual(E, C, I, D, Q) && allEqual(F, B, H) && F != sample(srcX + 3, srcY))
                    {
                        K = M = F;
                    }
                    if (E != D && allEqual(E, A, G, F, R) && allEqual(D, B, H) && D != sample(srcX - 3, srcY))
                    {
                        J = L = D;
                    }
                    if (E != H && allEqual(E, G, I, B, P) && allEqual(H, D, F) && H != sample(srcX, srcY + 3))
                    {
                        L = M = H;
                    }
                    if (E != B && allEqual(E, A, C, H, S) && allEqual(B, D, F) && B != sample(srcX, srcY - 3))
                    {
                        J = K = B;
                    }
                    if (Bl < El && allEqual(E, G, H, I, S) && noneEqual(E, A, D, C, F))
                    {
                        J = K = B;
                    }
                    if (Hl < El && allEqual(E, A, B, C, P) && noneEqual(E, D, G, I, F))
                    {
                        L = M = H;
                    }
                    if (Fl < El && allEqual(E, A, D, G, Q) && noneEqual(E, B, C, I, H))
                    {
                        K = M = F;
                    }
                    if (Dl < El && allEqual(E, C, F, I, R) && noneEqual(E, B, A, G, H))
                    {
                        J = L = D;
                    }

                    // 2:1 slope rules
                    if (H != B)
                    {
                        if (H != A && H != E && H != C)
                        {
                            if (allEqual(H, G, F, R) && noneEqual(H, D, sample(srcX + 2, srcY - 1)))
                            {
                                L = M;
                            }
                            if (allEqual(H, I, D, Q) && noneEqual(H, F, sample(srcX - 2, srcY - 1)))
                            {
                                M = L;
                            }
                        }

                        if (B != I && B != G && B != E)
                        {
                            if (allEqual(B, A, F, R) && noneEqual(B, D, sample(srcX + 2, srcY + 1)))
                            {
                                J = K;
                            }
                            if (allEqual(B, C, D, Q) && noneEqual(B, F, sample(srcX - 2, srcY + 1)))
                            {
                                K = J;
                            }
                        }
                    }

                    if (F != D)
                    {
                        if (D != I && D != E && D != C)
                        {
                            if (allEqual(D, A, H, S) && noneEqual(D, B, sample(srcX + 1, srcY + 2)))
                            {
                                J = L;
                            }
                            if (allEqual(D, G, B, P) && noneEqual(D, H, sample(srcX + 1, srcY - 2)))
                            {
                                L = J;
                            }
                        }

                        if (F != E && F != A && F != G)
                        {
                            if (allEqual(F, C, H, S) && noneEqual(F, B, sample(srcX - 1, srcY + 2)))
                            {
                                K = M;
                            }
                            if (allEqual(F, I, B, P) && noneEqual(F, H, sample(srcX - 1, srcY - 2)))
                            {
                                M = K;
                            }
                        }
                    }
                }

                auto* dstTop = dst.data() + static_cast<size_t>(srcY * 2) * dstPitch + static_cast<size_t>(srcX * 2);
                auto* dstBottom = dstTop + dstPitch;
                dstTop[0] = J;
                dstTop[1] = K;
                dstBottom[0] = L;
                dstBottom[1] = M;

                A = B;
                B = C;
                C = sample(srcX + 2, srcY - 1);
                Q = D;
                D = E;
                E = F;
                F = R;
                R = sample(srcX + 3, srcY);
                G = H;
                H = I;
                I = sample(srcX + 2, srcY + 1);
            }
        }

        return true;
    }
}
