#pragma once

#include <windows.h>
#include <cstdint>
#include <string>
#include <vector>

class GroundRenderer {
public:
  GroundRenderer();
  bool available() const { return !mips_.empty(); }
  const std::wstring& error() const { return error_; }
  void draw(HDC target, int width, int horizon, int bottom, float cameraHeight,
            float worldX, float worldZ, float forwardX, float forwardZ);

private:
  struct Mip {
    int width{}, height{};
    std::vector<std::uint32_t> bgra;
  };
  std::vector<Mip> mips_;
  std::vector<std::uint32_t> frame_;
  std::wstring error_;
};
