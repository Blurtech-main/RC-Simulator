#include "aircraft_model.h"
#include <algorithm>
#include <cmath>

namespace {
ModelVec3 add(ModelVec3 a,ModelVec3 b){return{a.x+b.x,a.y+b.y,a.z+b.z};}
ModelVec3 mul(ModelVec3 a,float s){return{a.x*s,a.y*s,a.z*s};}
ModelVec3 cross(ModelVec3 a,ModelVec3 b){return{a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
ModelVec3 rotate(float w,float x,float y,float z,ModelVec3 v){ModelVec3 q{x,y,z};ModelVec3 t=mul(cross(q,v),2);return add(v,add(mul(t,w),cross(q,t)));}
}

const TrainerAircraftModel& TrainerAircraftModel::instance(){static TrainerAircraftModel model;return model;}

TrainerAircraftModel::TrainerAircraftModel(){
  // Seven stable collision proxies follow the imported C172's 1:6.395 scale.
  // The high-detail visual mesh remains independent from collision geometry.
  collision_={
    {L"fuselage",CollisionShape::Box,{0,0,.01f},{.11f,.115f,.62f},false},
    {L"main wing",CollisionShape::Box,{0,.105f,.03f},{.86f,.035f,.125f},false},
    {L"tailplane",CollisionShape::Box,{0,.04f,-.53f},{.28f,.02f,.10f},false},
    {L"fin",CollisionShape::Box,{0,.11f,-.50f},{.02f,.11f,.12f},false},
    {L"left main wheel",CollisionShape::Sphere,{-.25f,-.16f,-.02f},{.04f,.04f,.04f},true},
    {L"right main wheel",CollisionShape::Sphere,{.25f,-.16f,-.02f},{.04f,.04f,.04f},true},
    {L"nose wheel",CollisionShape::Sphere,{0,-.16f,.48f},{.04f,.04f,.04f},true}
  };
  // Slice 2 will evaluate forces at these scale-accurate reference panels.
  aero_={
    // The .203 m fixed-wing chord plus the two .06 m aileron panels totals
    // 0.396 m^2, matching the full-scale 16.17 m^2 planform at 1:6.395.
    {L"left wing",AeroSurfaceRole::MainWing,{-.43f,.105f,.03f},{1,0,0},.86f,.203f,1.5f,0},
    {L"right wing",AeroSurfaceRole::MainWing,{.43f,.105f,.03f},{1,0,0},.86f,.203f,1.5f,0},
    {L"left aileron",AeroSurfaceRole::Aileron,{-.59f,.11f,-.06f},{1,0,0},.39f,.06f,0,20},
    {L"right aileron",AeroSurfaceRole::Aileron,{.59f,.11f,-.06f},{1,0,0},.39f,.06f,0,20},
    {L"stabiliser",AeroSurfaceRole::HorizontalTail,{0,.04f,-.50f},{1,0,0},.56f,.14f,-1,0},
    {L"elevator",AeroSurfaceRole::Elevator,{0,.04f,-.60f},{1,0,0},.52f,.06f,0,25},
    {L"fin",AeroSurfaceRole::Fin,{0,.11f,-.48f},{0,1,0},.22f,.16f,0,0},
    {L"rudder",AeroSurfaceRole::Rudder,{0,.12f,-.58f},{0,1,0},.19f,.06f,0,28}
  };
}

float TrainerAircraftModel::lowestCollisionY(float w,float x,float y,float z)const{
  float low=1e9f;
  for(const auto&p:collision_){
    ModelVec3 c=rotate(w,x,y,z,p.centre);
    if(p.shape==CollisionShape::Sphere){low=std::min(low,c.y-p.halfExtents.x);continue;}
    ModelVec3 ex=rotate(w,x,y,z,{p.halfExtents.x,0,0}),ey=rotate(w,x,y,z,{0,p.halfExtents.y,0}),ez=rotate(w,x,y,z,{0,0,p.halfExtents.z});
    low=std::min(low,c.y-std::abs(ex.y)-std::abs(ey.y)-std::abs(ez.y));
  }
  return low;
}
