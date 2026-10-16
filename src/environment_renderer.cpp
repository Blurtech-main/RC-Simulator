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
      const unsigned alpha = input[x * 4 + 3];
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

int EnvironmentRenderer::panoramaOffset(int width, float forwardX, float forwardZ) {
  constexpr float TwoPi = 6.28318530718f;
  float turns = std::atan2(forwardX, forwardZ) / TwoPi;
  turns -= std::floor(turns);
  return static_cast<int>(turns * width);
}

void EnvironmentRenderer::drawSky(HDC target, int width, int bottom, float forwardX, float forwardZ) const {
  if (!skyAvailable() || width <= 0 || bottom <= 0) return;
  const int offset = panoramaOffset(width, forwardX, forwardZ);
  SetStretchBltMode(target, HALFTONE);
  SetBrushOrgEx(target, 0, 0, nullptr);
  for (int x : {-offset, width - offset})
    StretchBlt(target, x, 0, width, bottom, sky_.dc, 0, 0, sky_.width, sky_.height, SRCCOPY);
}

void EnvironmentRenderer::drawHorizon(HDC target, int width, int horizonY, float forwardX, float forwardZ) const {
  if (!horizonAvailable() || width <= 0) return;
  const int offset = panoramaOffset(width, forwardX, forwardZ);
  const int drawHeight = std::max(160, static_cast<int>(width * (static_cast<float>(horizon_.height) / horizon_.width) * .72f));
  const int top = horizonY + 58 - drawHeight;
  BLENDFUNCTION blend{AC_SRC_OVER, 0, 255, AC_SRC_ALPHA};
  SetStretchBltMode(target, HALFTONE);
  for (int x : {-offset, width - offset})
    AlphaBlend(target, x, top, width, drawHeight, horizon_.dc, 0, 0, horizon_.width, horizon_.height, blend);
}
