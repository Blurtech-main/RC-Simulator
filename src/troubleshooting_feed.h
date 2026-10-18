#pragma once

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>
#include "flight_math.h"

struct TroubleshootingInput {
  uint64_t frame{};
  std::string mode{"live"},controller{"unknown"},route{"held_controls_1s"};
  bool trainerAssist{},crashed{};
  Controls controls{};
  V3 position{},velocity{},omegaBody{};
  Quat attitude{};
  float aoa{},beta{},airspeed{},lift{},drag{},thrust{},stallFraction{},loadFactor{};
  float propRpm{},motorCurrent{},batteryVoltage{},shaftPower{},advanceRatio{},curveCoverage{};
  float groundNormal{},maxGearCompression{},groundEffect{},trajectoryRms{},trajectoryMax{};
  unsigned wheelsOnGround{};
};

struct TroubleshootingSample {
  uint64_t frame{};
  float timeSeconds{};
  Controls controls{};
  V3 position{},velocity{},omegaBody{};
  float pitchDeg{},bankDeg{},headingDeg{},aoaDeg{},betaDeg{},airspeed{},stallFraction{};
  float lift{},drag{},thrust{},verticalAcceleration{},angularJerk{};
  float trajectoryRms{},trajectoryMax{};
  float raisingScore{},bumpinessScore{},sluggishScore{},oscillationScore{};
  bool raising{},bumpy{},sluggish{},oscillating{};
};

class TroubleshootingFeed {
  std::ofstream archive_;
  std::string sessionId_,archivePath_,lastIssue_{"none"};
  std::vector<TroubleshootingSample> history_;
  TroubleshootingSample current_{};
  V3 previousVelocity_{},previousOmega_{};
  float smoothVerticalAcceleration_{},sluggishSeconds_{},oscillationScore_{};
  int previousRollSign_{},previousPitchSign_{},previousYawSign_{};
  uint64_t lastSampleFrame_{UINT64_MAX},sequence_{};
  bool started_{},ended_{};

  void writeEvent(const std::string& json,bool flush=false);
  void publishSnapshot(const TroubleshootingInput& input);
public:
  ~TroubleshootingFeed();
  bool begin();
  void end(const std::string& reason="clean_exit");
  void sample(const TroubleshootingInput& input);
  void markIssue(const std::string& type,const TroubleshootingInput& input);
  void lifecycle(const std::string& event,uint64_t frame);

  const TroubleshootingSample& current() const{return current_;}
  const std::vector<TroubleshootingSample>& history() const{return history_;}
  const std::string& sessionId() const{return sessionId_;}
  const std::string& archivePath() const{return archivePath_;}
  const std::string& lastIssue() const{return lastIssue_;}
};
