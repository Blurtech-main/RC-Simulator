#include "troubleshooting_feed.h"
#include <windows.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace {
std::string escaped(const std::string& value){std::string out;for(char c:value){if(c=='\\'||c=='\"')out.push_back('\\');if(c=='\n'){out+="\\n";continue;}out.push_back(c);}return out;}
std::string number(double value){if(!std::isfinite(value))value=0;std::ostringstream out;out<<std::setprecision(9)<<value;return out.str();}
std::string boolean(bool value){return value?"true":"false";}
std::string timestamp(){SYSTEMTIME t{};GetSystemTime(&t);char b[48]{};sprintf_s(b,"%04u%02u%02uT%02u%02u%02u.%03uZ",t.wYear,t.wMonth,t.wDay,t.wHour,t.wMinute,t.wSecond,t.wMilliseconds);return b;}
float degrees(float radians){return radians*180.f/PI;}
int activeSign(float value){return value>.025f?1:value<-.025f?-1:0;}
}

TroubleshootingFeed::~TroubleshootingFeed(){end();}

bool TroubleshootingFeed::begin(){
  if(started_&&!ended_)return true;std::filesystem::create_directories("reports/troubleshooting/archive");sessionId_=timestamp()+"-"+std::to_string(GetCurrentProcessId());archivePath_="reports/troubleshooting/archive/session-"+sessionId_+".jsonl";archive_.open(archivePath_,std::ios::trunc);if(!archive_)return false;
  started_=true;ended_=false;sequence_=0;history_.clear();lastSampleFrame_=UINT64_MAX;writeEvent("{\"schema\":1,\"kind\":\"session_start\",\"session_id\":\""+escaped(sessionId_)+"\",\"fixed_dt_s\":0.008333333333}",true);
  std::ofstream pointer("reports/troubleshooting/current.ptr.tmp",std::ios::trunc);pointer<<archivePath_<<'\n';pointer.flush();pointer.close();MoveFileExA("reports/troubleshooting/current.ptr.tmp","reports/troubleshooting/current.ptr",MOVEFILE_REPLACE_EXISTING|MOVEFILE_WRITE_THROUGH);return true;
}

void TroubleshootingFeed::writeEvent(const std::string& json,bool){
  if(!archive_)return;
  archive_<<json<<'\n';
  // This feed exists for post-crash diagnosis. At its fixed 10 Hz sample rate,
  // keeping rows in the C++ stream buffer saves negligible work but can lose
  // the entire flight window while the live snapshot continues to update.
  // Make every event readable by an external diagnostic immediately.
  archive_.flush();
}

void TroubleshootingFeed::end(const std::string& reason){if(!started_||ended_)return;writeEvent("{\"schema\":1,\"kind\":\"session_end\",\"session_id\":\""+escaped(sessionId_)+"\",\"seq\":"+std::to_string(sequence_)+",\"reason\":\""+escaped(reason)+"\"}",true);archive_.close();ended_=true;}

void TroubleshootingFeed::lifecycle(const std::string& event,uint64_t frame){if(!started_)begin();writeEvent("{\"schema\":1,\"kind\":\"lifecycle\",\"session_id\":\""+escaped(sessionId_)+"\",\"seq\":"+std::to_string(++sequence_)+",\"frame\":"+std::to_string(frame)+",\"event\":\""+escaped(event)+"\"}",true);}

void TroubleshootingFeed::sample(const TroubleshootingInput& in){
  if(!started_&&!begin())return;if(lastSampleFrame_!=UINT64_MAX&&in.frame>=lastSampleFrame_&&in.frame-lastSampleFrame_<12)return;if(lastSampleFrame_!=UINT64_MAX&&in.frame<lastSampleFrame_){lifecycle("frame_reset",in.frame);lastSampleFrame_=UINT64_MAX;}
  float sampleDt=lastSampleFrame_==UINT64_MAX?.1f:std::max(.0083333f,float(in.frame-lastSampleFrame_)/120.f);V3 forward=rotate(in.attitude,{0,0,1}),right=rotate(in.attitude,{1,0,0}),up=rotate(in.attitude,{0,1,0});float pitch=degrees(std::asin(clampf(forward.y,-1.f,1.f))),bank=degrees(std::atan2(right.y,up.y)),heading=degrees(std::atan2(forward.x,forward.z));
  float verticalAcceleration=(in.velocity.y-previousVelocity_.y)/sampleDt,angularJerk=len(in.omegaBody-previousOmega_)/sampleDt;smoothVerticalAcceleration_+=(verticalAcceleration-smoothVerticalAcceleration_)*.12f;float highPassVertical=std::abs(verticalAcceleration-smoothVerticalAcceleration_);
  float commanded=std::max({std::abs(in.controls.ail),std::abs(in.controls.ele),std::abs(in.controls.rud)}),response=std::max({std::abs(in.omegaBody.x),std::abs(in.omegaBody.y),std::abs(in.omegaBody.z)});if(in.airspeed>7.f&&commanded>.35f&&response<.08f)sluggishSeconds_+=sampleDt;else sluggishSeconds_=std::max(0.f,sluggishSeconds_-sampleDt*2.f);
  int rollSign=activeSign(in.omegaBody.z),pitchSign=activeSign(in.omegaBody.x),yawSign=activeSign(in.omegaBody.y);int flips=(rollSign&&previousRollSign_&&rollSign!=previousRollSign_)+(pitchSign&&previousPitchSign_&&pitchSign!=previousPitchSign_)+(yawSign&&previousYawSign_&&yawSign!=previousYawSign_);oscillationScore_=clampf(oscillationScore_*.90f+flips*.24f,0.f,1.f);if(rollSign)previousRollSign_=rollSign;if(pitchSign)previousPitchSign_=pitchSign;if(yawSign)previousYawSign_=yawSign;
  float raisingScore=clampf((in.velocity.y-.45f)/1.8f,0.f,1.f)*clampf((.14f-std::abs(in.controls.ele))/.14f,0.f,1.f)*clampf((in.airspeed-6.f)/5.f,0.f,1.f);float bumpinessScore=clampf(std::max(highPassVertical/2.2f,angularJerk/2.8f),0.f,1.f);float sluggishScore=clampf(sluggishSeconds_/.6f,0.f,1.f);
  current_={in.frame,float(in.frame)/120.f,in.controls,in.position,in.velocity,in.omegaBody,pitch,bank,heading,degrees(in.aoa),degrees(in.beta),in.airspeed,in.stallFraction,in.lift,in.drag,in.thrust,verticalAcceleration,angularJerk,in.trajectoryRms,in.trajectoryMax,raisingScore,bumpinessScore,sluggishScore,oscillationScore_,raisingScore>.68f,bumpinessScore>.72f,sluggishScore>.72f,oscillationScore_>.68f};
  history_.push_back(current_);if(history_.size()>900)history_.erase(history_.begin(),history_.begin()+150);lastSampleFrame_=in.frame;previousVelocity_=in.velocity;previousOmega_=in.omegaBody;
  std::ostringstream row;row<<"{\"schema\":1,\"kind\":\"sample\",\"session_id\":\""<<escaped(sessionId_)<<"\",\"seq\":"<<++sequence_<<",\"frame\":"<<in.frame<<",\"time_s\":"<<number(current_.timeSeconds)<<",\"mode\":\""<<escaped(in.mode)<<"\",\"controller\":\""<<escaped(in.controller)<<"\",\"trainer_assist\":"<<boolean(in.trainerAssist)<<",\"route\":\""<<escaped(in.route)<<"\",\"controls\":{\"ail\":"<<number(in.controls.ail)<<",\"ele\":"<<number(in.controls.ele)<<",\"thr\":"<<number(in.controls.thr)<<",\"rud\":"<<number(in.controls.rud)<<"},\"state\":{\"x_m\":"<<number(in.position.x)<<",\"y_m\":"<<number(in.position.y)<<",\"z_m\":"<<number(in.position.z)<<",\"vx_mps\":"<<number(in.velocity.x)<<",\"vy_mps\":"<<number(in.velocity.y)<<",\"vz_mps\":"<<number(in.velocity.z)<<",\"pitch_deg\":"<<number(pitch)<<",\"bank_deg\":"<<number(bank)<<",\"heading_deg\":"<<number(heading)<<",\"roll_rate_dps\":"<<number(degrees(-in.omegaBody.z))<<",\"pitch_rate_dps\":"<<number(degrees(-in.omegaBody.x))<<",\"yaw_rate_dps\":"<<number(degrees(in.omegaBody.y))<<",\"aoa_deg\":"<<number(degrees(in.aoa))<<",\"beta_deg\":"<<number(degrees(in.beta))<<",\"airspeed_mps\":"<<number(in.airspeed)<<",\"stall_fraction\":"<<number(in.stallFraction)<<",\"lift_n\":"<<number(in.lift)<<",\"drag_n\":"<<number(in.drag)<<",\"thrust_n\":"<<number(in.thrust)<<",\"prop_rpm\":"<<number(in.propRpm)<<",\"motor_current_a\":"<<number(in.motorCurrent)<<",\"battery_v\":"<<number(in.batteryVoltage)<<",\"shaft_power_w\":"<<number(in.shaftPower)<<",\"advance_ratio\":"<<number(in.advanceRatio)<<",\"prop_map_coverage\":"<<number(in.curveCoverage)<<",\"ground_normal_n\":"<<number(in.groundNormal)<<",\"wheels_on_ground\":"<<in.wheelsOnGround<<",\"gear_compression_m\":"<<number(in.maxGearCompression)<<"},\"derived\":{\"vertical_accel_mps2\":"<<number(verticalAcceleration)<<",\"angular_jerk_radps2\":"<<number(angularJerk)<<",\"trajectory_rms_m\":"<<number(in.trajectoryRms)<<",\"trajectory_max_m\":"<<number(in.trajectoryMax)<<",\"raising_score\":"<<number(raisingScore)<<",\"bumpiness_score\":"<<number(bumpinessScore)<<",\"sluggish_score\":"<<number(sluggishScore)<<",\"oscillation_score\":"<<number(oscillationScore_)<<"},\"flags\":{\"raising\":"<<boolean(current_.raising)<<",\"bumpy\":"<<boolean(current_.bumpy)<<",\"sluggish\":"<<boolean(current_.sluggish)<<",\"oscillating\":"<<boolean(current_.oscillating)<<"}}";writeEvent(row.str(),in.frame%120==0);publishSnapshot(in);
}

void TroubleshootingFeed::publishSnapshot(const TroubleshootingInput& in){
  std::ofstream out("reports/troubleshooting/live-flight-snapshot.json.tmp",std::ios::trunc);if(!out)return;out<<"{\n  \"schema\": 1,\n  \"session_id\": \""<<escaped(sessionId_)<<"\",\n  \"archive\": \""<<escaped(archivePath_)<<"\",\n  \"updated_frame\": "<<current_.frame<<",\n  \"updated_time_s\": "<<number(current_.timeSeconds)<<",\n  \"mode\": \""<<escaped(in.mode)<<"\",\n  \"route\": \""<<escaped(in.route)<<"\",\n  \"last_user_issue\": \""<<escaped(lastIssue_)<<"\",\n  \"state\": {\"altitude_m\": "<<number(current_.position.y)<<", \"vertical_speed_mps\": "<<number(current_.velocity.y)<<", \"airspeed_mps\": "<<number(current_.airspeed)<<", \"pitch_deg\": "<<number(current_.pitchDeg)<<", \"bank_deg\": "<<number(current_.bankDeg)<<", \"aoa_deg\": "<<number(current_.aoaDeg)<<"},\n  \"diagnostics\": {\"trajectory_rms_m\": "<<number(current_.trajectoryRms)<<", \"trajectory_max_m\": "<<number(current_.trajectoryMax)<<", \"vertical_accel_mps2\": "<<number(current_.verticalAcceleration)<<", \"angular_jerk_radps2\": "<<number(current_.angularJerk)<<", \"raising_score\": "<<number(current_.raisingScore)<<", \"bumpiness_score\": "<<number(current_.bumpinessScore)<<", \"sluggish_score\": "<<number(current_.sluggishScore)<<", \"oscillation_score\": "<<number(current_.oscillationScore)<<"},\n  \"flags\": {\"raising\": "<<boolean(current_.raising)<<", \"bumpy\": "<<boolean(current_.bumpy)<<", \"sluggish\": "<<boolean(current_.sluggish)<<", \"oscillating\": "<<boolean(current_.oscillating)<<"}\n}\n";out.flush();out.close();MoveFileExA("reports/troubleshooting/live-flight-snapshot.json.tmp","reports/troubleshooting/live-flight-snapshot.json",MOVEFILE_REPLACE_EXISTING|MOVEFILE_WRITE_THROUGH);
}

void TroubleshootingFeed::markIssue(const std::string& type,const TroubleshootingInput& in){
  if(!started_)begin();lastIssue_=type;std::ostringstream row;row<<"{\"schema\":1,\"kind\":\"user_issue\",\"session_id\":\""<<escaped(sessionId_)<<"\",\"seq\":"<<++sequence_<<",\"frame\":"<<in.frame<<",\"time_s\":"<<number(double(in.frame)/120.0)<<",\"type\":\""<<escaped(type)<<"\",\"position\":{\"x_m\":"<<number(in.position.x)<<",\"y_m\":"<<number(in.position.y)<<",\"z_m\":"<<number(in.position.z)<<"},\"controls\":{\"ail\":"<<number(in.controls.ail)<<",\"ele\":"<<number(in.controls.ele)<<",\"thr\":"<<number(in.controls.thr)<<",\"rud\":"<<number(in.controls.rud)<<"}}";writeEvent(row.str(),true);publishSnapshot(in);
}
