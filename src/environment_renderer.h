#pragma once

#include <windows.h>
#include <string>

class EnvironmentRenderer {
public:
  EnvironmentRenderer();
  ~EnvironmentRenderer();
  EnvironmentRenderer(const EnvironmentRenderer&) = delete;
  EnvironmentRenderer& operator=(const EnvironmentRenderer&) = delete;

  bool skyAvailable() const { return sky_.bitmap != nullptr; }
  bool horizonAvailable() const { return horizon_.bitmap != nullptr; }
  const std::wstring& error() const { return error_; }
  void drawSky(HDC target, int width, int bottom, float forwardX, float forwardZ) const;
  void drawHorizon(HDC target, int width, int horizonY, float forwardX, float forwardZ) const;

private:
  struct Layer {
    HBITMAP bitmap{};
    HDC dc{};
    HGDIOBJ previous{};
    int width{}, height{};
  };
  bool loadPng(const std::wstring& relativePath, Layer& layer, bool premultiplyAlpha);
  static int panoramaOffset(int width, float forwardX, float forwardZ);
  static void release(Layer& layer);

  Layer sky_, horizon_;
  ULONG_PTR gdiplusToken_{};
  std::wstring error_;
};
