#pragma once
#include "propulsion.h"
#include <cstdint>
#include <vector>

// Body axes: +X right wing, +Y up, +Z through the spinner. Units are metres.
struct ModelVec3 { float x{}, y{}, z{}; };
enum class CollisionShape : uint8_t { Box, Sphere };
struct CollisionPrimitive { const wchar_t* name{}; CollisionShape shape{}; ModelVec3 centre{},halfExtents{}; bool landingGear{}; };
struct LandingGearContact {
  const wchar_t* name{};
  ModelVec3 wheelCentre{};
  float wheelRadius{};
  float springRateNPerM{};
  float dampingNPerMS{};
  float rollingResistance{};
  float lateralFriction{};
  float brakingFriction{};
  float maxSteerDeg{};
  bool braked{};
};
enum class AeroSurfaceRole : uint8_t { MainWing,Aileron,HorizontalTail,Elevator,Fin,Rudder };
struct AeroReferenceSurface {
  const wchar_t* name{}; AeroSurfaceRole role{}; ModelVec3 centre{},spanAxis{};
  float span{},chord{},incidenceDeg{},maxDeflectionDeg{};
};
struct AerodynamicConfiguration {
  float referenceChordM{},aspectRatio{},spanEfficiency{},dihedralDeg{},wingIncidenceDeg{};
  float wingAreaM2{},rollStabilityCoefficient{},tailLiftSlopePerRad{},tailStallDeg{},fuselageDragAreaM2{};
};

class AircraftModel {
public:
  virtual ~AircraftModel()=default;
  virtual const wchar_t* name() const=0;
  virtual float wingspan() const=0;
  virtual float length() const=0;
  virtual float groundClearance() const=0;
  float massKg() const{return massKg_;}
  V3 inertiaKgM2() const{return inertiaKgM2_;}
  const std::vector<CollisionPrimitive>& collision() const{return collision_;}
  const std::vector<LandingGearContact>& landingGear() const{return landingGear_;}
  const std::vector<AeroReferenceSurface>& aeroSurfaces() const{return aero_;}
  const AerodynamicConfiguration& aerodynamics() const{return aerodynamics_;}
  const std::vector<ElectricPropulsionDefinition>& propulsionUnits() const{return propulsion_;}
  float lowestCollisionY(float qw,float qx,float qy,float qz) const;
  float lowestAirframeCollisionY(float qw,float qx,float qy,float qz) const;
protected:
  float massKg_{};V3 inertiaKgM2_{};
  AerodynamicConfiguration aerodynamics_{};
  std::vector<CollisionPrimitive> collision_;
  std::vector<LandingGearContact> landingGear_;
  std::vector<AeroReferenceSurface> aero_;
  std::vector<ElectricPropulsionDefinition> propulsion_;
};

class TrainerAircraftModel final : public AircraftModel {
public:
  static const TrainerAircraftModel& instance();
  const wchar_t* name() const override{return L"Cessna 172 1:6.395 electric replica";}
  float wingspan() const override{return 1.72f;} float length() const override{return 1.294f;} float groundClearance() const override{return .20f;}
private:
  TrainerAircraftModel();
};
