#pragma once
#include <windows.h>
#include <string>
#include <vector>

class TexturedAircraftRenderer {
public:
  static TexturedAircraftRenderer& instance();
  bool available() const{return loaded_;}
  const std::wstring& error() const{return error_;}
  bool pitchDirectionMatchesPhysics() const;
  void drawGroundShadow(HDC target,int centreX,int groundY,float pixelsPerMetre,float altitudeAgl,unsigned wheelsOnGround);
  void draw(HDC target,int centreX,int centreY,float pixelsPerMetre,float qw,float qx,float qy,float qz,float propellerAngleRad,float propellerRpm);
private:
  struct Vertex{float x,y,z,nx,ny,nz,u,v;};
  struct Triangle{Vertex v[3];unsigned material{},part{};};
  struct Texture{unsigned flags{},width{},height{};std::vector<unsigned char> rgba;};
  TexturedAircraftRenderer();
  bool loaded_{};std::wstring error_;std::vector<Triangle> triangles_;std::vector<Texture> textures_;
  // The converter bakes the source node transform into its vertices, so the
  // tagged rotor does not necessarily have its hub at model-space (0, 0).
  float propellerHubX_{},propellerHubY_{};
  std::vector<unsigned char> pixels_,shadowPixels_;std::vector<float> depth_;
};
