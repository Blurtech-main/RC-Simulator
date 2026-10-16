#pragma once
#include "aircraft_model.h"
#include "flight_dynamics.h"
#include <array>

struct WheelContactTelemetry {
  bool contacting{};
  float compressionM{};
  float normalForceN{};
  float longitudinalSlipMS{};
  float lateralSlipMS{};
};

struct GroundContactEvaluation {
  V3 forceBody{};
  V3 momentBody{};
  std::array<WheelContactTelemetry,4> wheels{};
  size_t wheelCount{};
  float totalNormalForceN{};
  bool anyContact{};
};

GroundContactEvaluation evaluateGroundContact(const AircraftModel& model,
  const FlightState& state,Controls controls,float brakeAmount);
