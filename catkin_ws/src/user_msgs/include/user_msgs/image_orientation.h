#pragma once

#include <algorithm>
#include <cstddef>
#include <stdexcept>

namespace jzzj_image {

// Reflect image rows without touching channel order, depth units or row padding.
template <typename Image>
void flip_vertical(Image& image) {
    const std::size_t rows = image.height, step = image.step;
    if (!rows || !step || image.data.size() / step < rows)
        throw std::invalid_argument("invalid image dimensions/step/data");
    for (std::size_t y = 0; y < rows / 2; ++y) {
        auto first = image.data.begin() + y * step;
        auto opposite = image.data.begin() + (rows - 1 - y) * step;
        std::swap_ranges(first, first + step, opposite);
    }
}

// K' = F K and P' = F P, F = [1 0 0; 0 -1 H-1; 0 0 1].
// The optical frame and its physical extrinsics stay the same; fy becomes signed.
template <typename CameraInfo>
void flip_camera_info(CameraInfo& info) {
    if (!info.height) throw std::invalid_argument("camera_info height must be positive");
    const double last_row = info.height - 1.;
    for (unsigned c = 0; c < 3; ++c)
        info.K[3 + c] = last_row * info.K[6 + c] - info.K[3 + c];
    for (unsigned c = 0; c < 4; ++c)
        info.P[4 + c] = last_row * info.P[8 + c] - info.P[4 + c];
}

}  // namespace jzzj_image
