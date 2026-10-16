#pragma once
#include "flight_math.h"
#include <cstddef>
#include <vector>

// Measured fixed-pitch propeller data. J is the axial advance ratio, CT is
// thrust coefficient and CP is shaft-power coefficient.
struct PropellerCurvePoint { float advanceRatio{},ct{},cp{}; };
struct StaticPropellerPoint { float rpm{},ct{},cp{}; };

struct ElectricPropulsionDefinition {
  const wchar_t* name{};
  V3 mountBody{};
  V3 axisBody{0,0,1};
  int spinDirection{1}; // +1: angular momentum points along axisBody.
  float diameterM{},nominalPitchM{}; // Pitch identity is already embodied by the measured curve.
  float propellerInertiaKgM2{};
  float motorKvRpmPerVolt{};
  float motorResistanceOhm{};
  float motorNoLoadCurrentA{};
  float escCurrentLimitA{};
  float batteryOpenCircuitV{};
  float batteryResistanceOhm{};
  float drivetrainEfficiency{1.f};
  std::vector<PropellerCurvePoint> curve;
  std::vector<StaticPropellerPoint> staticCurve;
};

struct PropulsionState {
  float shaftOmegaRadSec{};
  float shaftAngleRad{};
};

struct PropulsionTelemetry {
  float rpm{},advanceRatio{},ct{},cp{};
  float thrustN{},propellerTorqueNm{},motorTorqueNm{},shaftAccelerationRadSec2{};
  float motorCurrentA{},batteryVoltageV{},electricalPowerW{},shaftPowerW{},propulsiveEfficiency{};
  float inducedVelocityMS{},curveCoverage{};
};

struct PropulsionEvaluation {
  V3 forceBody{},momentBody{};
  V3 mountMomentBody{},motorReactionMomentBody{},gyroscopicMomentBody{};
  float shaftAccelerationRadSec2{};
  int spinDirection{1};
  PropulsionTelemetry telemetry{};
};

// Evaluates one reusable propulsor without allocating. Multiple engines are
// handled by evaluating one definition/state pair per installed unit.
PropulsionEvaluation evaluatePropulsion(const ElectricPropulsionDefinition&,
  const PropulsionState&,float throttle,V3 relativeAirBody,V3 omegaBody,float density);
void integratePropulsion(PropulsionState&,const PropulsionEvaluation&,float dt);
V3 propwashVelocityAt(const ElectricPropulsionDefinition&,
  const PropulsionEvaluation&,V3 pointBody);
bool validatePropulsionDefinition(const ElectricPropulsionDefinition&);
