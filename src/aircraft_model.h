#pragma once
#include <cstdint>
#include <vector>

// Body axes: +X right wing, +Y up, +Z through the spinner. Units are metres.
struct ModelVec3 { float x{}, y{}, z{}; };
enum class CollisionShape : uint8_t { Box, Sphere };
struct CollisionPrimitive { const wchar_t* name{}; CollisionShape shape{}; ModelVec3 centre{},halfExtents{}; bool landingGear{}; };
enum class AeroSurfaceRole : uint8_t { MainWing,Aileron,HorizontalTail,Elevator,Fin,Rudder };
struct AeroReferenceSurface {
  const wchar_t* name{}; AeroSurfaceRole role{}; ModelVec3 centre{},spanAxis{};
  float span{},chord{},incidenceDeg{},maxDeflectionDeg{};
};

class TrainerAircraftModel {
public:
  static const TrainerAircraftModel& instance();
  float wingspan() const{return 1.72f;} float length() const{return 1.294f;} float groundClearance() const{return .20f;}
  const std::vector<CollisionPrimitive>& collision() const{return collision_;}
  const std::vector<AeroReferenceSurface>& aeroSurfaces() const{return aero_;}
  float lowestCollisionY(float qw,float qx,float qy,float qz) const;
private:
  TrainerAircraftModel();
  std::vector<CollisionPrimitive> collision_;
  std::vector<AeroReferenceSurface> aero_;
};
