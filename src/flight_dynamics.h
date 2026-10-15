#pragma once
#include "flight_math.h"

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
  float propRpm{};
};

struct AeroTelemetry {
  float airspeed{},aoa{},beta{},reynolds{};
  float lift{},drag{},sideForce{},thrust{},propTorque{};
  float cl{},cd{},stallFraction{},loadFactor{};
  V3 forceBody{},momentBody{},relativeAirBody{};
};

struct AeroEvaluation {
  V3 forceBody{},momentBody{};
  AeroTelemetry telemetry{};
  float targetPropRpm{};
};

inline constexpr float TrainerMassKg=3.65f;
inline constexpr V3 TrainerInertiaKgM2{.49f,1.08f,.91f};
inline constexpr float TrainerPropRollCoupling=1.f;

struct AirfoilCoefficients { float cl{},cd{},cm{}; };

// Public pure functions make the coefficient source and force model directly
// testable without a window, controller, renderer or integrator.
AirfoilCoefficients naca2412LowRe(float alphaRad,float reynolds);
AeroEvaluation evaluateAerodynamics(const FlightState&,Controls,const AtmosphereSample&);
void integrateFlightState(FlightState&,const AeroEvaluation&,float dt);
