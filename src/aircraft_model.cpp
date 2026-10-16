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
  massKg_=3.65f;inertiaKgM2_={.49f,1.08f,.91f};
  aerodynamics_={.23f,7.48f,.72f,1.5f,1.5f,.396f,.55f,4.15f,14.f,.022f};
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
  // Individual tyre contacts. Spring rates give roughly 12 mm static travel
  // at this mass; damping is near critical for the share carried by each leg.
  // The nose wheel follows rudder input and all three wheels accept the
  // explicit brake command, while rolling resistance remains present unbraked.
  landingGear_={
    {L"left main",{-.25f,-.16f,-.02f},.04f,1250.f,55.f,.025f,.78f,.92f,0.f,true},
    {L"right main",{.25f,-.16f,-.02f},.04f,1250.f,55.f,.025f,.78f,.92f,0.f,true},
    // The mains sit only 20 mm behind the current CG, so the nose leg carries
    // about four percent of static weight. Match that geometry instead of
    // assigning an arbitrary equal spring rate that would pitch the model up.
    {L"nose",{0,-.16f,.48f},.04f,105.f,6.f,.030f,.68f,.80f,25.f,true}
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
  // APC 14x7 Thin Electric dynamic coefficients at 5963 rpm, plus the
  // measured 7480 rpm static point. The electrical constants describe a
  // replaceable 4S / 620 Kv RC installation; they are calibration inputs, not
  // properties hidden in the solver.
  propulsion_.push_back({
    L"nose electric tractor",{0,0,.64f},{0,0,1},1,
    .3556f,.1778f,.00064f,620.f,.050f,1.2f,55.f,14.8f,.025f,.94f,
    {{.111033f,.093305f,.031728f},{.125391f,.092150f,.031940f},
     {.139988f,.090862f,.032129f},{.162224f,.089342f,.032559f},{.181818f,.087312f,.032806f},
     {.197378f,.085901f,.033029f},{.214888f,.083903f,.033136f},{.232524f,.081569f,.033181f},
     {.250516f,.079287f,.033214f},{.266970f,.075992f,.032819f},{.284421f,.073049f,.032492f},
     {.301351f,.069333f,.031884f},{.318902f,.066182f,.031355f},{.335613f,.062181f,.030400f},
     {.350282f,.058672f,.029534f},{.367670f,.054562f,.028415f},{.385405f,.050437f,.027243f},
     {.403608f,.046249f,.025957f},{.420496f,.042168f,.024652f},{.438207f,.038050f,.023322f},
     {.455837f,.033937f,.021947f},{.472980f,.030231f,.020639f},{.492191f,.026470f,.018999f},
     {.504629f,.023959f,.018018f},{.523562f,.019936f,.016434f},{.543274f,.015928f,.014759f},
     {.560355f,.012149f,.013067f},{.577140f,.008623f,.011231f},{.594034f,.004995f,.009515f},
     {.609775f,.001437f,.007810f}},
    {{980.f,.073866f,.030143f},{1540.f,.084541f,.029408f},{2026.667f,.089267f,.028904f},
     {2460.f,.091007f,.028589f},{2953.333f,.092014f,.028369f},{3446.667f,.093638f,.028524f},
     {3980.f,.094392f,.028692f},{4533.333f,.093622f,.028787f},{4953.333f,.094381f,.029123f},
     {5540.f,.095177f,.029413f},{5980.f,.096625f,.029841f},{6513.333f,.097815f,.030281f},
     {6960.f,.099331f,.030769f},{7480.f,.100331f,.031150f}}
  });
}

float AircraftModel::lowestCollisionY(float w,float x,float y,float z)const{
  float low=1e9f;
  for(const auto&p:collision_){
    ModelVec3 c=rotate(w,x,y,z,p.centre);
    if(p.shape==CollisionShape::Sphere){low=std::min(low,c.y-p.halfExtents.x);continue;}
    ModelVec3 ex=rotate(w,x,y,z,{p.halfExtents.x,0,0}),ey=rotate(w,x,y,z,{0,p.halfExtents.y,0}),ez=rotate(w,x,y,z,{0,0,p.halfExtents.z});
    low=std::min(low,c.y-std::abs(ex.y)-std::abs(ey.y)-std::abs(ez.y));
  }
  return low;
}

float AircraftModel::lowestAirframeCollisionY(float w,float x,float y,float z)const{
  float low=1e9f;
  for(const auto&p:collision_){
    if(p.landingGear)continue;
    ModelVec3 c=rotate(w,x,y,z,p.centre);
    if(p.shape==CollisionShape::Sphere){low=std::min(low,c.y-p.halfExtents.x);continue;}
    ModelVec3 ex=rotate(w,x,y,z,{p.halfExtents.x,0,0}),ey=rotate(w,x,y,z,{0,p.halfExtents.y,0}),ez=rotate(w,x,y,z,{0,0,p.halfExtents.z});
    low=std::min(low,c.y-std::abs(ex.y)-std::abs(ey.y)-std::abs(ez.y));
  }
  return low;
}
