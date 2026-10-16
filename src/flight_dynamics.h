#pragma once
#include "flight_math.h"
#include "propulsion.h"
#include <array>

// A deliberately explicit atmosphere boundary: a later weather provider can
// fill this from location, altitude, live observations, deterministic gusts,
// turbulence and rain without changing the aerodynamic solver.
struct AtmosphereSample {
  float density{1.225f};
  float pressure{101325.f};
  float temperatureK{288.15f};
  float dynamicViscosity{1.7894e-5f};
  V3 windWorld{};
  V3 gustWorld{};
  float rainRateMmHr{};
  float turbulenceIntensity{};
};

struct FlightState {
  V3 positionWorld{},velocityWorld{},omegaBody{};
  Quat attitude{};
  static constexpr size_t MaxPropulsionUnits=4;
  std::array<PropulsionState,MaxPropulsionUnits> propulsion{};
};

struct PropulsionCommand {
  std::array<float,FlightState::MaxPropulsionUnits> throttle{};
  size_t count{}; // zero broadcasts Controls::thr; otherwise overrides [0,count).
};

struct AeroTelemetry {
  float airspeed{},aoa{},beta{},reynolds{};
  float lift{},drag{},sideForce{},thrust{},propTorque{};
  float propRpm{},motorCurrent{},batteryVoltage{},shaftPower{},propEfficiency{},advanceRatio{},curveCoverage{};
  float cl{},cd{},stallFraction{},loadFactor{},groundEffect{};
  V3 forceBody{},momentBody{},relativeAirBody{};
};

struct AeroEvaluation {
  V3 forceBody{},momentBody{};
  AeroTelemetry telemetry{};
  std::array<PropulsionEvaluation,FlightState::MaxPropulsionUnits> propulsion{};
  size_t propulsionCount{};
  bool modelValid{true};
};

struct AirfoilCoefficients { float cl{},cd{},cm{}; };
class AircraftModel;

// Public pure functions make the coefficient source and force model directly
// testable without a window, controller, renderer or integrator.
AirfoilCoefficients naca2412LowRe(float alphaRad,float reynolds);
AeroEvaluation evaluateAerodynamics(const FlightState&,Controls,const AtmosphereSample&);
AeroEvaluation evaluateAerodynamics(const AircraftModel&,const FlightState&,Controls,const AtmosphereSample&);
AeroEvaluation evaluateAerodynamics(const AircraftModel&,const FlightState&,Controls,const AtmosphereSample&,const PropulsionCommand&);
void integrateFlightState(FlightState&,const AeroEvaluation&,float dt);
void integrateFlightState(const AircraftModel&,FlightState&,const AeroEvaluation&,float dt);
