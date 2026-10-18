#include "route_pilot.h"
#include <algorithm>
#include <cmath>

namespace {
float wrap(float a){while(a>PI)a-=2.f*PI;while(a<-PI)a+=2.f*PI;return a;}
float horizontal(V3 v){return std::sqrt(v.x*v.x+v.z*v.z);}
}

Controls RoutePilot::update(const RoutePilotState& s,const std::vector<V3>& route,float dt){
  Controls c{};if(!active_||route.size()<3)return c;elapsed_+=dt;
  // Search only forward. This prevents a figure-eight crossing from jumping
  // to the other branch and makes every run repeatable.
  size_t begin=progress_>3?progress_-3:0,end=std::min(route.size()-1,progress_+28),nearest=begin;float best=1e30f;
  for(size_t i=begin;i<=end;i++){V3 d=route[i]-s.position;float q=d.x*d.x+d.y*d.y+d.z*d.z;if(q<best){best=q;nearest=i;}}
  progress_=std::max(progress_,nearest);size_t look=progress_;float distance=0;
  while(look+1<route.size()&&distance<18.f){distance+=len(route[look+1]-route[look]);++look;}
  size_t ahead=std::min(route.size()-1,look+2),behind=look?look-1:0;V3 tangent=route[ahead]-route[behind];float hlen=std::max(.01f,horizontal(tangent));
  V3 target=route[look];if(progress_+2>=route.size()){V3 direction{tangent.x/hlen,0,tangent.z/hlen};float passed=std::max(0.f,dot(s.position-route.back(),direction));target=route.back()+direction*(passed+18.f);}V3 to=target-s.position;float targetHeading=std::atan2(tangent.x,tangent.z);V3 forward=rotate(s.attitude,{0,0,1}),right=rotate(s.attitude,{1,0,0}),up=rotate(s.attitude,{0,1,0});
  float heading=std::atan2(forward.x,forward.z),pitch=std::asin(clampf(forward.y,-1.f,1.f)),bank=std::atan2(right.y,up.y);
  V3 pathRight{tangent.z/hlen,0,-tangent.x/hlen};float cross=to.x*pathRight.x+to.z*pathRight.z;float speed=std::max(8.f,horizontal(s.velocity));
  float headingError=wrap(targetHeading-heading+std::atan2(cross*.35f,speed));float turnRate=clampf(2.f*speed/18.f*std::sin(headingError),-.55f,.55f);float desiredBank=clampf(-std::atan2(speed*turnRate,9.80665f),-25.f*PI/180.f,25.f*PI/180.f);
  c.ail=clampf((bank-desiredBank)*3.0f+s.omegaBody.z*.9f,-.45f,.45f);
  // The model needs positive geometric pitch in level flight to carry its
  // normal angle of attack. Omitting this trim made a mathematically "level"
  // command lower the nose and sink before altitude feedback could recover.
  float pathAngle=std::atan2(tangent.y,hlen),heightError=target.y-s.position.y;float trimPitch=6.5f*PI/180.f;float desiredPitch=clampf(trimPitch+pathAngle+std::atan2(heightError*.35f-s.velocity.y*.65f,speed),-6.f*PI/180.f,16.f*PI/180.f);
  c.ele=clampf((desiredPitch-pitch)*6.5f+s.omegaBody.x*1.0f,-.55f,.55f);
  c.rud=clampf((turnRate-s.omegaBody.y)*.32f-s.beta*.18f,-.28f,.28f);
  float flightAngle=std::atan2(s.velocity.y,speed);c.thr=clampf(.68f+(15.2f-s.airspeed)*.05f+(pathAngle-flightAngle)*.35f,.40f,.92f);
  // Ease in for a human-like capture instead of an instantaneous stick step.
  float capture=clampf(elapsed_/1.2f,.25f,1.f);c.ail*=capture;c.ele*=capture;c.rud*=capture;c.thr=.48f+(c.thr-.48f)*capture;
  return c;
}
