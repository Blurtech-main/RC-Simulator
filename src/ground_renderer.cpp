#include "ground_renderer.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>

namespace {
std::uint8_t channel(std::uint32_t pixel, unsigned shift) {
  return static_cast<std::uint8_t>((pixel >> shift) & 255u);
}

std::uint32_t bgra(unsigned b, unsigned g, unsigned r) {
  return (255u << 24) | (r << 16) | (g << 8) | b;
}

}

GroundRenderer::GroundRenderer() {
  wchar_t executable[32768]{};
  GetModuleFileNameW(nullptr, executable, 32768);
  const auto path = std::filesystem::path(executable).parent_path() / L"assets" / L"grass-field-cheap.bmp";
  std::ifstream file(path, std::ios::binary);
  BITMAPFILEHEADER fileHeader{};
  BITMAPINFOHEADER info{};
  if (!file || !file.read(reinterpret_cast<char*>(&fileHeader), sizeof(fileHeader)) ||
      !file.read(reinterpret_cast<char*>(&info), sizeof(info)) || fileHeader.bfType != 0x4d42 ||
      info.biBitCount != 24 || info.biCompression != BI_RGB || info.biWidth <= 0 || info.biHeight == 0) {
    error_ = L"missing or invalid assets/grass-field-cheap.bmp";
    return;
  }

  Mip base;
  base.width = info.biWidth;
  base.height = std::abs(info.biHeight);
  base.bgra.resize(static_cast<size_t>(base.width) * base.height);
  const size_t stride = (static_cast<size_t>(base.width) * 3u + 3u) & ~3u;
  std::vector<std::uint8_t> row(stride);
  file.seekg(fileHeader.bfOffBits);
  for (int sourceY = 0; sourceY < base.height; ++sourceY) {
    if (!file.read(reinterpret_cast<char*>(row.data()), static_cast<std::streamsize>(row.size()))) {
      error_ = L"truncated assets/grass-field-cheap.bmp";
      return;
    }
    const int y = info.biHeight > 0 ? base.height - 1 - sourceY : sourceY;
    for (int x = 0; x < base.width; ++x) {
      base.bgra[static_cast<size_t>(y) * base.width + x] = bgra(row[x * 3], row[x * 3 + 1], row[x * 3 + 2]);
    }
  }
  mips_.push_back(std::move(base));

  while (mips_.back().width > 1 || mips_.back().height > 1) {
    const Mip& source = mips_.back();
    Mip next;
    next.width = std::max(1, source.width / 2);
    next.height = std::max(1, source.height / 2);
    next.bgra.resize(static_cast<size_t>(next.width) * next.height);
    for (int y = 0; y < next.height; ++y) for (int x = 0; x < next.width; ++x) {
      unsigned sums[3]{};
      for (int oy = 0; oy < 2; ++oy) for (int ox = 0; ox < 2; ++ox) {
        const int sx = std::min(source.width - 1, x * 2 + ox);
        const int sy = std::min(source.height - 1, y * 2 + oy);
        const auto pixel = source.bgra[static_cast<size_t>(sy) * source.width + sx];
        sums[0] += channel(pixel, 0); sums[1] += channel(pixel, 8); sums[2] += channel(pixel, 16);
      }
      next.bgra[static_cast<size_t>(y) * next.width + x] = bgra(sums[0] / 4, sums[1] / 4, sums[2] / 4);
    }
    mips_.push_back(std::move(next));
  }
}

void GroundRenderer::draw(HDC target, int width, int horizon, int bottom, float cameraHeight,
                          float worldX, float worldZ, float forwardX, float forwardZ) {
  const int height = bottom - horizon;
  if (!available() || width <= 0 || height <= 0) return;
  // Grass is stochastic and does not benefit from one sample per display pixel.
  // Render at half size, then let GDI do one filtered upscale: one quarter of
  // the shading work without reducing the HUD or aircraft resolution.
  const int renderWidth = (width + 1) / 2, renderHeight = (height + 1) / 2;
  frame_.resize(static_cast<size_t>(renderWidth) * renderHeight);

  const float forwardLength = std::sqrt(forwardX * forwardX + forwardZ * forwardZ);
  if (forwardLength > .0001f) { forwardX /= forwardLength; forwardZ /= forwardLength; }
  else { forwardX = 0.f; forwardZ = 1.f; }
  const float rightX = forwardZ, rightZ = -forwardX;
  const float focal = renderWidth / (2.f * std::tan(65.f * 3.14159265f / 360.f));
  constexpr float texelsPerMetre = 192.f; // Dense enough for the very low, wheel-height camera.
  const auto wrap = [](int value, int size) { value %= size; return value < 0 ? value + size : value; };
  struct Sample { float b{}, g{}, r{}; };
  const auto bilinear = [&](const Mip& mip, float textureX, float textureY) {
    const int ix = static_cast<int>(std::floor(textureX)), iy = static_cast<int>(std::floor(textureY));
    const float blendX = textureX - std::floor(textureX), blendY = textureY - std::floor(textureY);
    const auto at = [&](int px, int py) {
      return mip.bgra[static_cast<size_t>(wrap(py, mip.height)) * mip.width + wrap(px, mip.width)];
    };
    const auto p00 = at(ix, iy), p10 = at(ix + 1, iy), p01 = at(ix, iy + 1), p11 = at(ix + 1, iy + 1);
    const auto filtered = [&](unsigned shift) {
      const float top = channel(p00, shift) * (1.f - blendX) + channel(p10, shift) * blendX;
      const float bottom = channel(p01, shift) * (1.f - blendX) + channel(p11, shift) * blendX;
      return top * (1.f - blendY) + bottom * blendY;
    };
    return Sample{filtered(0), filtered(8), filtered(16)};
  };

  for (int y = 0; y < renderHeight; ++y) {
    const float screenBelowHorizon = y + .5f;
    const float distance = std::min(4000.f, cameraHeight * focal / screenBelowHorizon);
    const float metresPerPixel = distance / focal;
    const float footprint = std::max(1.f, metresPerPixel * texelsPerMetre);
    const int lod = std::clamp(static_cast<int>(std::log2(footprint)), 0, static_cast<int>(mips_.size()) - 1);
    const Mip& mip = mips_[lod];
    const float mipScale = texelsPerMetre / static_cast<float>(1u << lod);
    const float baseX = worldX + forwardX * distance;
    const float baseZ = worldZ + forwardZ * distance;

    for (int x = 0; x < renderWidth; ++x) {
      const float lateral = (x + .5f - renderWidth * .5f) * metresPerPixel;
      const float sampleX = baseX + rightX * lateral;
      const float sampleZ = baseZ + rightZ * lateral;
      const Sample fine = bilinear(mip, sampleX * mipScale, sampleZ * mipScale);

      // The fine texture correctly collapses toward its average at altitude,
      // but by itself that produces a flat green sheet.  Reuse the baked grass
      // at a 170 m repeat scale for broad field variation that survives distant
      // mip filtering.  The offset avoids lining up its features with the fine layer.
      constexpr float macroTexelsPerMetre = 1.5f;
      const float macroFootprint = std::max(1.f, metresPerPixel * macroTexelsPerMetre);
      const int macroLod = std::clamp(static_cast<int>(std::log2(macroFootprint)), 0, static_cast<int>(mips_.size()) - 1);
      const Mip& macroMip = mips_[macroLod];
      const float macroScale = macroTexelsPerMetre / static_cast<float>(1u << macroLod);
      const Sample macro = bilinear(macroMip, (sampleX + 37.f) * macroScale, (sampleZ - 23.f) * macroScale);
      float b = fine.b * .72f + macro.b * .28f;
      float g = fine.g * .72f + macro.g * .28f;
      float r = fine.r * .72f + macro.r * .28f;

      // A mown grass runway is part of the same world plane, so it turns and
      // recedes correctly instead of being a screen-space trapezoid.
      const float acrossRunway = std::abs(sampleX);
      if (acrossRunway < 9.f) {
        r = r * .72f + 22.f; g = g * .72f + 26.f; b = b * .66f + 18.f;
        const bool edge = acrossRunway > 8.35f && acrossRunway < 8.78f;
        if (edge) { r = r * .66f + 35.f; g = g * .66f + 40.f; b = b * .66f + 30.f; }
      }

      frame_[static_cast<size_t>(y) * renderWidth + x] = bgra(
        static_cast<unsigned>(std::clamp(b, 0.f, 255.f)),
        static_cast<unsigned>(std::clamp(g, 0.f, 255.f)),
        static_cast<unsigned>(std::clamp(r, 0.f, 255.f)));
    }
  }

  BITMAPINFO bitmap{};
  bitmap.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
  bitmap.bmiHeader.biWidth = renderWidth;
  bitmap.bmiHeader.biHeight = -renderHeight;
  bitmap.bmiHeader.biPlanes = 1;
  bitmap.bmiHeader.biBitCount = 32;
  bitmap.bmiHeader.biCompression = BI_RGB;
  SetStretchBltMode(target, HALFTONE);
  SetBrushOrgEx(target, 0, 0, nullptr);
  StretchDIBits(target, 0, horizon, width, height, 0, 0, renderWidth, renderHeight,
                frame_.data(), &bitmap, DIB_RGB_COLORS, SRCCOPY);
}
