#pragma once

#include "flight_math.h"
#include <cstdint>
#include <string>
#include <vector>

struct FlightTestSample {
  double timeSeconds{};
  uint64_t frame{};
  Controls controls{};
  V3 positionWorld{},velocityWorld{},angularRateBody{};
  Quat attitude{};
  float aoa{},beta{},airspeed{},lift{},drag{},thrust{},stallFraction{};
  bool crashed{};
};

struct FlightTestReportSummary {
  bool wrote{};
  bool mandatoryPassed{};
  std::string verdict;
  std::string elevatorFinding;
  std::string aileronFinding;
  std::string rudderFinding;
};

struct FlightTestEnvelopeCheck {
  std::string name;
  std::string expected;
  std::string observed;
  std::string diagnosis;
  bool passed{};
  bool mandatory{true};
};

FlightTestReportSummary writeFlightTestReport(const std::string& htmlPath,
  const std::string& csvPath,const std::string& title,const std::string& source,
  const std::vector<FlightTestSample>& samples,bool scriptedScenario,
  const std::vector<FlightTestEnvelopeCheck>& envelopeChecks={});
