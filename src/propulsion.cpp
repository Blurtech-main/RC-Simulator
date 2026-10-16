#include "propulsion.h"
#include <algorithm>
#include <cmath>

namespace {
PropellerCurvePoint sampleCurve(const std::vector<PropellerCurvePoint>& curve,float j,float& coverage){
  coverage=0.f;if(curve.empty())return {};
  if(j<=curve.front().advanceRatio){coverage=j>=curve.front().advanceRatio?1.f:0.f;return curve.front();}
  if(j>=curve.back().advanceRatio){coverage=j<=curve.back().advanceRatio?1.f:0.f;return curve.back();}
  coverage=1.f;
  auto hi=std::lower_bound(curve.begin(),curve.end(),j,[](const auto&p,float value){return p.advanceRatio<value;});
  auto lo=hi-1;float t=(j-lo->advanceRatio)/std::max(1e-6f,hi->advanceRatio-lo->advanceRatio);
  return {j,lo->ct+(hi->ct-lo->ct)*t,lo->cp+(hi->cp-lo->cp)*t};
}
StaticPropellerPoint sampleStatic(const std::vector<StaticPropellerPoint>& curve,float rpm){
  if(curve.empty())return {};
  if(rpm<=curve.front().rpm)return curve.front();if(rpm>=curve.back().rpm)return curve.back();
  auto hi=std::lower_bound(curve.begin(),curve.end(),rpm,[](const auto&p,float value){return p.rpm<value;});auto lo=hi-1;
  float t=(rpm-lo->rpm)/std::max(1e-6f,hi->rpm-lo->rpm);return {rpm,lo->ct+(hi->ct-lo->ct)*t,lo->cp+(hi->cp-lo->cp)*t};
}
}

bool validatePropulsionDefinition(const ElectricPropulsionDefinition& d){
  auto finite=[](float v){return std::isfinite(v);};
  if(d.spinDirection==0||!finite(d.diameterM)||d.diameterM<=0.f||!finite(d.nominalPitchM)||d.nominalPitchM<=0.f||!finite(d.propellerInertiaKgM2)||d.propellerInertiaKgM2<=0.f||!finite(d.motorKvRpmPerVolt)||d.motorKvRpmPerVolt<=0.f||!finite(d.motorResistanceOhm)||d.motorResistanceOhm<=0.f||!finite(d.escCurrentLimitA)||d.escCurrentLimitA<=0.f||!finite(d.batteryOpenCircuitV)||d.batteryOpenCircuitV<=0.f||d.curve.empty())return false;
  float last=-1e30f;for(const auto&p:d.curve){if(!finite(p.advanceRatio)||!finite(p.ct)||!finite(p.cp)||p.advanceRatio<=last)return false;last=p.advanceRatio;}
  last=-1.f;for(const auto&p:d.staticCurve){if(!finite(p.rpm)||!finite(p.ct)||!finite(p.cp)||p.rpm<=last)return false;last=p.rpm;}
  return true;
}

PropulsionEvaluation evaluatePropulsion(const ElectricPropulsionDefinition& d,
  const PropulsionState& state,float throttle,V3 relativeAirBody,V3 omegaBody,float density){
  PropulsionEvaluation out{};auto axis=norm(d.axisBody);if(!validatePropulsionDefinition(d)||len2(axis)<.5f)return out;
  throttle=clampf(throttle,0.f,1.f);float omega=std::max(0.f,state.shaftOmegaRadSec);
  float n=omega/(2.f*PI),rpm=n*60.f;
  float axialSpeed=dot(relativeAirBody+cross(omegaBody,d.mountBody),axis);
  float j=n>.5f?axialSpeed/(n*d.diameterM):0.f;
  float coverage{};auto coeff=sampleCurve(d.curve,j,coverage);
  if(!d.staticCurve.empty()&&j<=d.curve.front().advanceRatio){auto s=sampleStatic(d.staticCurve,rpm);float blend=clampf(j/std::max(1e-4f,d.curve.front().advanceRatio),0.f,1.f);coeff={j,s.ct+(d.curve.front().ct-s.ct)*blend,s.cp+(d.curve.front().cp-s.cp)*blend};coverage=j>=0.f?1.f:0.f;}
  float n2=n*n,d4=std::pow(d.diameterM,4.f),d5=d4*d.diameterM;
  float thrust=coeff.ct*density*n2*d4;
  float propTorque=coeff.cp/(2.f*PI)*density*n2*d5;

  // Average-value BLDC/ESC model. Battery sag is solved in the same algebraic
  // step, which is stable at the 120 Hz rigid-body rate and avoids pretending
  // to resolve microsecond PWM switching.
  float kvRadSecPerVolt=d.motorKvRpmPerVolt*(2.f*PI/60.f);
  float ke=kvRadSecPerVolt>1e-6f?1.f/kvRadSecPerVolt:0.f,kt=ke;
  float resistance=std::max(.001f,d.motorResistanceOhm+throttle*throttle*d.batteryResistanceOhm);
  float current=throttle>0.f?std::max(0.f,(throttle*d.batteryOpenCircuitV-ke*omega)/resistance):0.f;
  current=std::min(current,std::max(0.f,d.escCurrentLimitA));
  float batteryCurrent=throttle*current;
  float batteryVoltage=std::max(0.f,d.batteryOpenCircuitV-batteryCurrent*d.batteryResistanceOhm);
  float motorTorque=kt*std::max(0.f,current-d.motorNoLoadCurrentA)*clampf(d.drivetrainEfficiency,0.f,1.f);
  float alpha=(motorTorque-propTorque)/d.propellerInertiaKgM2;
  if(omega<=0.f&&alpha<0.f)alpha=0.f;

  V3 force=axis*thrust;
  V3 spinAxis=axis*float(d.spinDirection>=0?1:-1);
  V3 angularMomentum=spinAxis*(d.propellerInertiaKgM2*omega);
  // The airframe receives the equal/opposite motor reaction plus the bearing
  // moment needed to precess the spinning propeller with the aircraft.
  out.mountMomentBody=cross(d.mountBody,force);out.motorReactionMomentBody=spinAxis*(-motorTorque);out.gyroscopicMomentBody=-cross(omegaBody,angularMomentum);
  out.forceBody=force;out.momentBody=out.mountMomentBody+out.motorReactionMomentBody+out.gyroscopicMomentBody;out.shaftAccelerationRadSec2=alpha;out.spinDirection=d.spinDirection>=0?1:-1;
  auto& t=out.telemetry;t.rpm=rpm;t.advanceRatio=j;t.ct=coeff.ct;t.cp=coeff.cp;t.thrustN=thrust;t.propellerTorqueNm=propTorque;t.motorTorqueNm=motorTorque;t.shaftAccelerationRadSec2=alpha;t.motorCurrentA=current;t.batteryVoltageV=batteryVoltage;t.electricalPowerW=batteryVoltage*batteryCurrent;t.shaftPowerW=propTorque*omega;t.propulsiveEfficiency=t.shaftPowerW>1e-3f?clampf(thrust*std::max(0.f,axialSpeed)/t.shaftPowerW,0.f,1.f):0.f;t.curveCoverage=coverage;
  float diskArea=PI*d.diameterM*d.diameterM*.25f;
  if(thrust>0.f&&density>0.f&&diskArea>0.f)t.inducedVelocityMS=.5f*(-axialSpeed+std::sqrt(std::max(0.f,axialSpeed*axialSpeed+2.f*thrust/(density*diskArea))));
  return out;
}

void integratePropulsion(PropulsionState& state,const PropulsionEvaluation& e,float dt){
  float previous=std::max(0.f,state.shaftOmegaRadSec);
  state.shaftOmegaRadSec=std::max(0.f,previous+e.shaftAccelerationRadSec2*dt);
  state.shaftAngleRad=std::fmod(state.shaftAngleRad+float(e.spinDirection)*.5f*(previous+state.shaftOmegaRadSec)*dt,2.f*PI);
  if(state.shaftAngleRad<0.f)state.shaftAngleRad+=2.f*PI;
}

V3 propwashVelocityAt(const ElectricPropulsionDefinition& d,const PropulsionEvaluation& e,V3 pointBody){
  if(e.telemetry.inducedVelocityMS<=0.f)return {};
  auto axis=norm(d.axisBody);V3 fromDisk=d.mountBody-pointBody;
  float downstream=dot(fromDisk,axis);if(downstream<=0.f)return {};
  V3 radial=fromDisk-axis*downstream;float radius=.5f*d.diameterM;
  float contracted=radius*(.82f+.18f*std::exp(-downstream/std::max(.01f,d.diameterM)));
  float u=len(radial)/std::max(.01f,contracted);if(u>=1.f)return {};
  float radialWeight=1.f-u*u;radialWeight*=radialWeight;
  float developed=1.f+clampf(downstream/(4.f*d.diameterM),0.f,1.f);
  return axis*(e.telemetry.inducedVelocityMS*developed*radialWeight);
}
