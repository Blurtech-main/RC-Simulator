#include "environment_renderer.h"

#include <objidl.h>
#include <gdiplus.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>

EnvironmentRenderer::EnvironmentRenderer() {
  Gdiplus::GdiplusStartupInput startup;
  if (Gdiplus::GdiplusStartup(&gdiplusToken_, &startup, nullptr) != Gdiplus::Ok) {
    error_ = L"GDI+ startup failed";
    return;
  }
  const bool skyOk = loadPng(L"assets/environment/sky-rural-v1.png", sky_, false);
  const bool hillsOk = loadPng(L"assets/environment/hills-treeline-v2.png", horizon_, true);
  if (!skyOk || !hillsOk) error_ = L"missing or invalid baked environment PNG assets";
}

EnvironmentRenderer::~EnvironmentRenderer() {
  release(sky_);
  release(horizon_);
  if (gdiplusToken_) Gdiplus::GdiplusShutdown(gdiplusToken_);
}

void EnvironmentRenderer::release(Layer& layer) {
  if (layer.dc && layer.previous) SelectObject(layer.dc, layer.previous);
  if (layer.dc) DeleteDC(layer.dc);
  if (layer.bitmap) DeleteObject(layer.bitmap);
  layer = {};
}

bool EnvironmentRenderer::loadPng(const std::wstring& relativePath, Layer& layer, bool premultiplyAlpha) {
  wchar_t executable[32768]{};
  GetModuleFileNameW(nullptr, executable, 32768);
  const auto path = std::filesystem::path(executable).parent_path() / relativePath;
  Gdiplus::Bitmap source(path.c_str());
  if (source.GetLastStatus() != Gdiplus::Ok || !source.GetWidth() || !source.GetHeight()) return false;

  layer.width = static_cast<int>(source.GetWidth());
  layer.height = static_cast<int>(source.GetHeight());
  BITMAPINFO info{};
  info.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
  info.bmiHeader.biWidth = layer.width;
  info.bmiHeader.biHeight = -layer.height;
  info.bmiHeader.biPlanes = 1;
  info.bmiHeader.biBitCount = 32;
  info.bmiHeader.biCompression = BI_RGB;
  void* destination{};
  layer.bitmap = CreateDIBSection(nullptr, &info, DIB_RGB_COLORS, &destination, nullptr, 0);
  if (!layer.bitmap || !destination) { release(layer); return false; }

  Gdiplus::Rect rectangle(0, 0, layer.width, layer.height);
  Gdiplus::BitmapData data{};
  if (source.LockBits(&rectangle, Gdiplus::ImageLockModeRead, PixelFormat32bppARGB, &data) != Gdiplus::Ok) {
    release(layer); return false;
  }
  auto* output = static_cast<std::uint8_t*>(destination);
  const auto* scanZero = static_cast<const std::uint8_t*>(data.Scan0);
  for (int y = 0; y < layer.height; ++y) {
    const auto* input = scanZero + static_cast<ptrdiff_t>(y) * data.Stride;
    auto* row = output + static_cast<size_t>(y) * layer.width * 4u;
    for (int x = 0; x < layer.width; ++x) {
      unsigned alpha = input[x * 4 + 3];
      // The source panorama deliberately has a deep lower feather. At runtime
      // that broad ramp reads as a blurred green strip between the projected
      // grass and the trees. Preserve the irregular cut-out, but confine its
      // lower anti-aliasing to a narrow, photographic occlusion edge.
      if (premultiplyAlpha && y > layer.height * 17 / 20 && alpha > 0 && alpha < 255)
        alpha = static_cast<unsigned>(std::clamp((static_cast<int>(alpha) - 92) * 255 / 72, 0, 255));
      row[x * 4 + 0] = static_cast<std::uint8_t>(premultiplyAlpha ? input[x * 4 + 0] * alpha / 255u : input[x * 4 + 0]);
      row[x * 4 + 1] = static_cast<std::uint8_t>(premultiplyAlpha ? input[x * 4 + 1] * alpha / 255u : input[x * 4 + 1]);
      row[x * 4 + 2] = static_cast<std::uint8_t>(premultiplyAlpha ? input[x * 4 + 2] * alpha / 255u : input[x * 4 + 2]);
      row[x * 4 + 3] = static_cast<std::uint8_t>(alpha);
    }
  }
  source.UnlockBits(&data);
  layer.dc = CreateCompatibleDC(nullptr);
  if (!layer.dc) { release(layer); return false; }
  layer.previous = SelectObject(layer.dc, layer.bitmap);
  return true;
}

int EnvironmentRenderer::panoramaOffset(int width, float forwardX, float forwardZ,
                                         float worldX, float worldZ, float effectiveRadius) {
  constexpr float TwoPi = 6.28318530718f;
  float turns = std::atan2(forwardX, forwardZ) / TwoPi;
  if (effectiveRadius > 1.f) {
    // Sky is effectively infinite. Hills are not: translating sideways should
    // move the nearer tree belt slightly against the field. This small-angle
    // finite-distance approximation supplies that missing parallax.
    const float rightX = forwardZ, rightZ = -forwardX;
    const float lateral = worldX * rightX + worldZ * rightZ;
    turns += lateral / (TwoPi * effectiveRadius);
  }
  turns -= std::floor(turns);
  return static_cast<int>(turns * width);
}

// The supplied panoramas are photographic strips rather than cube-map faces,
// so their first and last texels cannot meet perfectly.  Keep the established
// composition and only soften the 32-pixel wrap strip when it enters view.
// This is deliberately performed at the tile boundary, never across the rest
// of the sky or landscape.
static void blendPanoramaSeams(HDC target, int viewportWidth, int top, int height,
                               int tileWidth, int offset, HDC source, int sourceWidth,
                               int sourceHeight, bool sourceHasAlpha) {
  constexpr int halfBlendPixels = 16;
  const int blendPixels = halfBlendPixels * 2;
  const int sourceSpan = std::max(1, (blendPixels * sourceWidth + tileWidth - 1) / tileWidth);
  for (int seam = -offset; seam <= viewportWidth; seam += tileWidth) {
    const int first = std::max(0, seam - halfBlendPixels);
    const int last = std::min(viewportWidth, seam + halfBlendPixels);
    for (int x = first; x < last; ++x) {
      const float t = std::clamp((static_cast<float>(x - (seam - halfBlendPixels)) + .5f) /
                                 static_cast<float>(blendPixels), 0.f, 1.f);
      const int sample = std::min(sourceSpan - 1, static_cast<int>(t * sourceSpan));
      const int leftX = std::max(0, sourceWidth - sourceSpan + sample);
      const int rightX = std::min(sourceWidth - 1, sample);
      if (sourceHasAlpha) {
        const BLENDFUNCTION opaque{AC_SRC_OVER, 0, 255, AC_SRC_ALPHA};
        AlphaBlend(target, x, top, 1, height, source, leftX, 0, 1, sourceHeight, opaque);
      } else {
        StretchBlt(target, x, top, 1, height, source, leftX, 0, 1, sourceHeight, SRCCOPY);
      }
      const BLENDFUNCTION fade{AC_SRC_OVER, 0, static_cast<BYTE>(std::lround(t * 255.f)), AC_SRC_ALPHA};
      AlphaBlend(target, x, top, 1, height, source, rightX, 0, 1, sourceHeight, fade);
    }
  }
}

void EnvironmentRenderer::drawSky(HDC target, int width, int bottom, float forwardX, float forwardZ) const {
  if (!skyAvailable() || width <= 0 || bottom <= 0) return;
  const int offset = panoramaOffset(width, forwardX, forwardZ);
  SetStretchBltMode(target, HALFTONE);
  SetBrushOrgEx(target, 0, 0, nullptr);
  for (int x : {-offset, width - offset})
    StretchBlt(target, x, 0, width, bottom, sky_.dc, 0, 0, sky_.width, sky_.height, SRCCOPY);
  blendPanoramaSeams(target, width, 0, bottom, width, offset,
                     sky_.dc, sky_.width, sky_.height, false);
}

void EnvironmentRenderer::drawHorizon(HDC target, int width, int horizonY, float forwardX, float forwardZ,
                                      float worldX, float worldZ) const {
  if (!horizonAvailable() || width <= 0) return;
  const int offset = panoramaOffset(width, forwardX, forwardZ, worldX, worldZ, 850.f);
  const int drawHeight = std::max(160, static_cast<int>(width * (static_cast<float>(horizon_.height) / horizon_.width) * .72f));
  const int top = horizonY + 58 - drawHeight;
  BLENDFUNCTION blend{AC_SRC_OVER, 0, 255, AC_SRC_ALPHA};
  SetStretchBltMode(target, HALFTONE);
  for (int x : {-offset, width - offset})
    AlphaBlend(target, x, top, width, drawHeight, horizon_.dc, 0, 0, horizon_.width, horizon_.height, blend);
  blendPanoramaSeams(target, width, top, drawHeight, width, offset,
                     horizon_.dc, horizon_.width, horizon_.height, true);
}
