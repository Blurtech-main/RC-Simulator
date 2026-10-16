#include "ground_dynamics.h"
#include <cmath>

namespace {
V3 horizontalUnit(V3 v,V3 fallback){
  v.y=0.f;
  V3 n=norm(v);
  if(len2(n)>.5f)return n;
  fallback.y=0.f;
  return norm(fallback);
}

float smoothOpposition(float speed,float fullForce,float transitionSpeed){
  return -clampf(speed/std::max(.01f,transitionSpeed),-1.f,1.f)*fullForce;
}
}

GroundContactEvaluation evaluateGroundContact(const AircraftModel& model,
  const FlightState& state,Controls controls,float brakeAmount){
  GroundContactEvaluation out{};
  const auto& gear=model.landingGear();
  out.wheelCount=std::min(gear.size(),out.wheels.size());
  brakeAmount=clampf(brakeAmount,0.f,1.f);
  V3 bodyForwardWorld=rotate(state.attitude,{0,0,1});
  V3 bodyRightWorld=rotate(state.attitude,{1,0,0});
  for(size_t i=0;i<out.wheelCount;i++){
    const auto& wheel=gear[i];
    V3 centreBody{wheel.wheelCentre.x,wheel.wheelCentre.y,wheel.wheelCentre.z};
    V3 centreWorld=state.positionWorld+rotate(state.attitude,centreBody);
    float compression=wheel.wheelRadius-centreWorld.y;
    if(compression<=0.f)continue;

    V3 pointVelocityWorld=state.velocityWorld+rotate(state.attitude,cross(state.omegaBody,centreBody));
    float normalForce=wheel.springRateNPerM*compression-wheel.dampingNPerMS*pointVelocityWorld.y;
    normalForce=clampf(normalForce,0.f,model.massKg()*9.80665f*8.f);
    if(normalForce<=0.f)continue;

    float steer=wheel.maxSteerDeg*PI/180.f*clampf(controls.rud,-1.f,1.f);
    V3 steeredForwardBody{std::sin(steer),0.f,std::cos(steer)};
    V3 forward=horizontalUnit(rotate(state.attitude,steeredForwardBody),bodyForwardWorld);
    V3 right=horizontalUnit(cross({0,1,0},forward),bodyRightWorld);
    float longitudinal=dot(pointVelocityWorld,forward);
    float lateral=dot(pointVelocityWorld,right);

    float rolling=smoothOpposition(longitudinal,wheel.rollingResistance*normalForce,.18f);
    float brakeForce=wheel.braked?smoothOpposition(longitudinal,wheel.brakingFriction*normalForce*brakeAmount,.30f):0.f;
    float lateralForce=clampf(-lateral*normalForce*2.2f,-wheel.lateralFriction*normalForce,wheel.lateralFriction*normalForce);
    V3 forceWorld={0,normalForce,0};
    forceWorld+=forward*(rolling+brakeForce)+right*lateralForce;
    V3 forceBody=invrot(state.attitude,forceWorld);
    out.forceBody+=forceBody;
    out.momentBody+=cross(centreBody,forceBody);
    out.totalNormalForceN+=normalForce;
    out.anyContact=true;
    out.wheels[i]={true,compression,normalForce,longitudinal,lateral};
  }
  return out;
}
