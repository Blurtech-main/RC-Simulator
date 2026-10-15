#pragma once
#include <windows.h>
#include <string>
#include <vector>

class TexturedAircraftRenderer {
public:
  static TexturedAircraftRenderer& instance();
  bool available() const{return loaded_;}
  const std::wstring& error() const{return error_;}
  void draw(HDC target,int centreX,int centreY,float pixelsPerMetre,float qw,float qx,float qy,float qz);
private:
  struct Vertex{float x,y,z,nx,ny,nz,u,v;};
  struct Triangle{Vertex v[3];unsigned material;};
  struct Texture{unsigned flags{},width{},height{};std::vector<unsigned char> rgba;};
  TexturedAircraftRenderer();
  bool loaded_{};std::wstring error_;std::vector<Triangle> triangles_;std::vector<Texture> textures_;
  std::vector<unsigned char> pixels_;std::vector<float> depth_;
};
