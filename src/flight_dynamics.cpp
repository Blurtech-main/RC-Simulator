#include "flight_dynamics.h"
#include "aircraft_model.h"
#include <array>
#include <cmath>

namespace {
constexpr float deg(float d){return d*PI/180.f;}

// Digitised anchors from Kent State's CC-BY Airfoil 360 v2022 wind-tunnel
// NACA 2412 sheets (Re 50k and 100k).  The table preserves the measured
// low-Re stall onset and drag rise.  Negative angles are obtained using the
// NACA 2412 zero-lift angle (-2 degrees), and full-angle behaviour is blended
// to the flat-plate/Viterna asymptote rather than clipping forces at stall.
struct PolarPoint { float alphaDeg,cl,cd; };
constexpr std::array<PolarPoint,12> polar50{{
  {-2.f,0.f,.033f},{0.f,.184f,.0332f},{5.124f,.6371f,.0494f},{8.291f,.8689f,.0651f},
  {10.012f,.9680f,.0795f},{11.163f,1.0299f,.0894f},{12.315f,1.0609f,.1088f},
  {12.962f,.9915f,.1382f},{16.009f,.9465f,.1992f},{20.f,.76f,.39f},{30.f,.62f,.72f},{45.f,.54f,1.10f}
}};
constexpr std::array<PolarPoint,12> polar100{{
  {-2.f,0.f,.029f},{0.f,.31f,.031f},{5.057f,.8613f,.0620f},{7.973f,1.0835f,.0849f},
  {10.288f,1.1958f,.1055f},{10.990f,1.2132f,.1138f},{12.090f,1.2113f,.1270f},
  {12.978f,1.1652f,.1712f},{14.050f,.9846f,.2541f},{16.234f,.8638f,.3200f},{20.f,.86f,.40f},{45.f,.58f,1.10f}
}};

template<size_t N> AirfoilCoefficients interpolatePolar(const std::array<PolarPoint,N>& p,float alphaDeg){
  bool mirror=alphaDeg<-2.f;
  float sample=mirror?-4.f-alphaDeg:alphaDeg; // symmetry about measured zero-lift angle
  sample=clampf(sample,-2.f,90.f);
  if(sample>45.f){
    float a=deg(sample),cl=.82f*std::sin(2*a),cd=.025f+1.12f*std::sin(a)*std::sin(a);
    if(mirror)cl=-cl;
    return {cl,cd,-.045f};
  }
  size_t hi=1;while(hi<N&&p[hi].alphaDeg<sample)++hi;hi=std::min(hi,N-1);size_t lo=hi-1;
  float t=(sample-p[lo].alphaDeg)/std::max(.001f,p[hi].alphaDeg-p[lo].alphaDeg);
  float cl=p[lo].cl+(p[hi].cl-p[lo].cl)*t;
  float cd=p[lo].cd+(p[hi].cd-p[lo].cd)*t;
  if(mirror)cl=-cl;
  return {cl,cd,-.045f};
}

AirfoilCoefficients finiteWingNaca2412(float alphaRad,float reynolds,float aspect,float efficiency,float inducedDragFactor=1.f){
  float ad=alphaRad*180.f/PI;auto a=interpolatePolar(polar50,ad),b=interpolatePolar(polar100,ad);
  float t=clampf(std::log(std::max(50000.f,reynolds)/50000.f)/std::log(2.f),0.f,1.f);
  AirfoilCoefficients out{a.cl+(b.cl-a.cl)*t,a.cd+(b.cd-a.cd)*t,a.cm+(b.cm-a.cm)*t};
  aspect=std::max(.1f,aspect);efficiency=std::max(.1f,efficiency);out.cl*=aspect/(aspect+2.f);out.cd+=inducedDragFactor*out.cl*out.cl/(PI*efficiency*aspect);return out;
}

struct Accumulator { V3 force{},moment{};float lift{},drag{},side{}; };
void addAt(Accumulator& a,V3 force,V3 centre){a.force+=force;a.moment+=cross(centre,force);}

V3 localVelocity(const FlightState&s,V3 centre,const AtmosphereSample&atm,V3 propwash={}){
  V3 airWorld=s.velocityWorld-atm.windWorld-atm.gustWorld;
  return invrot(s.attitude,airWorld)+cross(s.omegaBody,centre)+propwash;
}

float controlFor(AeroSurfaceRole role,float centreX,Controls c){
  switch(role){
    case AeroSurfaceRole::Aileron:return centreX<0.f?c.ail:-c.ail;
    // Positive transmitter elevator is the conventional pull-up command:
    // trailing edge up, reducing tail lift and producing a nose-up moment.
    case AeroSurfaceRole::Elevator:return -c.ele;
    case AeroSurfaceRole::Rudder:return c.rud;
    default:return 0.f;
  }
}

void horizontalPanel(Accumulator& out,const AeroReferenceSurface&s,const FlightState&st,Controls c,const AtmosphereSample&atm,const AerodynamicConfiguration& config,float inducedAlpha,float inducedDragFactor,V3 propwash){
  V3 vel=localVelocity(st,{s.centre.x,s.centre.y,s.centre.z},atm,propwash);float speed=len(vel);if(speed<.25f)return;
  float q=.5f*atm.density*speed*speed,area=s.span*s.chord;
  float ctrl=controlFor(s.role,s.centre.x,c);
  // A hinged control changes local camber; it is not an injected torque.
  float hingeEffect=s.role==AeroSurfaceRole::Elevator?.30f:s.role==AeroSurfaceRole::Aileron?.70f:0.f;
  float alpha=std::atan2(-vel.y,std::max(.05f,vel.z))+deg(s.incidenceDeg)+ctrl*deg(s.maxDeflectionDeg)*hingeEffect-inducedAlpha;
  float re=atm.density*speed*s.chord/std::max(1e-7f,atm.dynamicViscosity);
  bool tail=s.role==AeroSurfaceRole::HorizontalTail||s.role==AeroSurfaceRole::Elevator;
  auto cf=finiteWingNaca2412(alpha,re,config.aspectRatio,config.spanEfficiency,inducedDragFactor);
  // The stabiliser/elevator is a symmetric section. Use its finite-tail slope
  // and smooth post-stall continuation instead of the cambered wing polar.
  if(tail){
    float aa=std::abs(alpha),linear=config.tailLiftSlopePerRad*alpha;
    cf.cl=aa<deg(config.tailStallDeg)?clampf(linear,-1.02f,1.02f):.78f*std::sin(2.f*alpha);
    cf.cd=.018f+.075f*cf.cl*cf.cl+clampf((aa-deg(12.f))/deg(35.f),0.f,1.f)*.65f;
    cf.cm=0.f;
  }
  bool wing=s.role==AeroSurfaceRole::MainWing||s.role==AeroSurfaceRole::Aileron;
  float dih=wing?deg(config.dihedralDeg):0.f;
  V3 spanAxis{std::cos(dih),(s.centre.x>=0.f?1.f:-1.f)*std::sin(dih),0};
  V3 flow=norm(vel),liftDir=norm(cross(flow,spanAxis));
  V3 lf=liftDir*(q*area*cf.cl),df=-flow*(q*area*cf.cd);
  addAt(out,lf+df,{s.centre.x,s.centre.y,s.centre.z});
  out.lift+=dot(lf,{0,1,0});out.drag+=-dot(df,flow);
  // Conventional negative Cm is nose-down; +X is nose-down in these axes.
  out.moment.x+=-cf.cm*q*area*s.chord;
}

void verticalPanel(Accumulator& out,const AeroReferenceSurface&s,const FlightState&st,Controls c,const AtmosphereSample&atm,V3 propwash){
  V3 vel=localVelocity(st,{s.centre.x,s.centre.y,s.centre.z},atm,propwash);float speed=len(vel);if(speed<.25f)return;
  float q=.5f*atm.density*speed*speed,area=s.span*s.chord;
  float ctrl=controlFor(s.role,s.centre.x,c);
  float beta=std::atan2(vel.x,std::max(.05f,vel.z))+ctrl*deg(s.maxDeflectionDeg)*.72f;
  float cl=clampf(4.1f*beta,-1.05f,1.05f),cd=.018f+.075f*cl*cl;
  V3 flow=norm(vel),sideDir=norm(cross({0,1,0},flow));
  // Fin force opposes lateral flow; this also damps yaw rate at the aft tail.
  // Keep positive rudder producing a positive yaw moment.
  V3 sf=sideDir*(-q*area*cl),df=-flow*(q*area*cd);
  addAt(out,sf+df,{s.centre.x,s.centre.y,s.centre.z});out.side+=sf.x;out.drag+=-dot(df,flow);
}
}

AirfoilCoefficients naca2412LowRe(float alphaRad,float reynolds){
  return finiteWingNaca2412(alphaRad,reynolds,7.48f,.72f);
}

AeroEvaluation evaluateAerodynamics(const AircraftModel&model,const FlightState&st,Controls c,const AtmosphereSample&atm,const PropulsionCommand& command){
  AeroEvaluation r{};Accumulator acc{};
  const auto& config=model.aerodynamics();
  V3 relWorld=st.velocityWorld-atm.windWorld-atm.gustWorld;V3 relBody=invrot(st.attitude,relWorld);
  float speed=len(relBody);r.telemetry.relativeAirBody=relBody;r.telemetry.airspeed=speed;
  r.telemetry.aoa=std::atan2(-relBody.y,std::max(.05f,relBody.z));
  r.telemetry.beta=std::atan2(relBody.x,std::sqrt(relBody.y*relBody.y+relBody.z*relBody.z));
  r.telemetry.reynolds=atm.density*speed*config.referenceChordM/std::max(1e-7f,atm.dynamicViscosity);
  auto wingCf=finiteWingNaca2412(r.telemetry.aoa+deg(config.wingIncidenceDeg),r.telemetry.reynolds,config.aspectRatio,config.spanEfficiency);
  // McCormick-style ground-effect factor for induced flow. At large wing
  // height it tends to one; near the runway it reduces induced drag/downwash.
  V3 wingCentreWorld=st.positionWorld+rotate(st.attitude,{0,.105f,.03f});
  float heightOverSpan=std::max(0.f,wingCentreWorld.y)/std::max(.01f,model.wingspan());
  float groundRatio=16.f*heightOverSpan;
  float inducedDragFactor=(groundRatio*groundRatio)/(1.f+groundRatio*groundRatio);
  r.telemetry.groundEffect=1.f-inducedDragFactor;
  float induced=wingCf.cl/(PI*config.spanEfficiency*config.aspectRatio)*inducedDragFactor;
  float tailDownwash=2.f*wingCf.cl/(PI*config.aspectRatio)*inducedDragFactor;
  const auto& units=model.propulsionUnits();if(units.size()>FlightState::MaxPropulsionUnits){r.modelValid=false;return r;}r.propulsionCount=units.size();
  for(size_t i=0;i<r.propulsionCount;i++){
    float throttle=i<command.count?command.throttle[i]:c.thr;
    r.propulsion[i]=evaluatePropulsion(units[i],st.propulsion[i],throttle,relBody,st.omegaBody,atm.density);
    acc.force+=r.propulsion[i].forceBody;acc.moment+=r.propulsion[i].momentBody;
    const auto&t=r.propulsion[i].telemetry;r.telemetry.thrust+=t.thrustN;r.telemetry.propTorque+=t.propellerTorqueNm;
    if(i==0){r.telemetry.propRpm=t.rpm;r.telemetry.motorCurrent=t.motorCurrentA;r.telemetry.batteryVoltage=t.batteryVoltageV;r.telemetry.shaftPower=t.shaftPowerW;r.telemetry.propEfficiency=t.propulsiveEfficiency;r.telemetry.advanceRatio=t.advanceRatio;r.telemetry.curveCoverage=t.curveCoverage;}
  }
  for(const auto&s:model.aeroSurfaces()){
    V3 wash{};V3 point{s.centre.x,s.centre.y,s.centre.z};for(size_t i=0;i<r.propulsionCount;i++)wash+=propwashVelocityAt(units[i],r.propulsion[i],point);
    if(s.role==AeroSurfaceRole::Fin||s.role==AeroSurfaceRole::Rudder)verticalPanel(acc,s,st,c,atm,wash);
    else horizontalPanel(acc,s,st,c,atm,config,(s.role==AeroSurfaceRole::MainWing||s.role==AeroSurfaceRole::Aileron)?induced:tailDownwash,inducedDragFactor,wash);
  }
  // The local panels provide roll-rate damping, but the previous slice omitted
  // the Cessna trainer's high-wing/dihedral restoring tendency. Consequently
  // any non-zero propeller reaction torque accumulated bank forever. Model the
  // missing lateral stability as a dynamic-pressure-scaled rolling moment;
  // deliberate aileron input progressively releases the self-righting force.
  V3 worldUpBody=invrot(st.attitude,{0,1,0});
  float bank=std::atan2(worldUpBody.x,worldUpBody.y);
  float neutralAileron=clampf(1.f-std::abs(c.ail),0.f,1.f);
  float rollRestoring=-bank*(.5f*atm.density*speed*speed)*(config.wingAreaM2*model.wingspan())*config.rollStabilityCoefficient*neutralAileron;
  acc.moment.z+=rollRestoring;
  if(speed>.25f){
    V3 flow=norm(relBody);float q=.5f*atm.density*speed*speed;
    V3 fuselageDrag=-flow*(q*config.fuselageDragAreaM2);addAt(acc,fuselageDrag,{0,0,.02f});acc.drag+=q*config.fuselageDragAreaM2;
  }
  // Optional trainer assistance compensates the physical powertrain reaction
  // at centred aileron. The unassisted path retains the complete motor torque.
  // Without trim a persistent small bank still creates an endless gentle turn.
  // The caller elects to add this compensation; the generic solver does not.
  r.forceBody=acc.force;r.momentBody=acc.moment;
  r.telemetry.forceBody=acc.force;r.telemetry.momentBody=acc.moment;r.telemetry.lift=acc.lift;r.telemetry.drag=acc.drag;
  r.telemetry.sideForce=acc.side;r.telemetry.cl=wingCf.cl;r.telemetry.cd=wingCf.cd;
  r.telemetry.stallFraction=clampf((std::abs(r.telemetry.aoa)-deg(9.f))/deg(10.f),0.f,1.f);
  r.telemetry.loadFactor=acc.lift/(model.massKg()*9.80665f);
  return r;
}

void integrateFlightState(const AircraftModel&model,FlightState&s,const AeroEvaluation&a,float dt){
  for(size_t i=0;i<a.propulsionCount;i++)integratePropulsion(s.propulsion[i],a.propulsion[i],dt);
  float mass=model.massKg();V3 inertia=model.inertiaKgM2();V3 forceWorld=rotate(s.attitude,a.forceBody)+V3{0,-mass*9.80665f,0};
  s.velocityWorld+=forceWorld/mass*dt;s.positionWorld+=s.velocityWorld*dt;
  V3 iw{inertia.x*s.omegaBody.x,inertia.y*s.omegaBody.y,inertia.z*s.omegaBody.z};
  V3 rhs=a.momentBody-cross(s.omegaBody,iw);
  V3 alpha{rhs.x/inertia.x,rhs.y/inertia.y,rhs.z/inertia.z};
  s.omegaBody+=alpha*dt;Quat spin{0,s.omegaBody.x,s.omegaBody.y,s.omegaBody.z};Quat dq=qmul(s.attitude,spin);
  s.attitude=qnorm({s.attitude.w+.5f*dq.w*dt,s.attitude.x+.5f*dq.x*dt,s.attitude.y+.5f*dq.y*dt,s.attitude.z+.5f*dq.z*dt});
}

AeroEvaluation evaluateAerodynamics(const AircraftModel&model,const FlightState&st,Controls c,const AtmosphereSample&atm){return evaluateAerodynamics(model,st,c,atm,{});}
AeroEvaluation evaluateAerodynamics(const FlightState&st,Controls c,const AtmosphereSample&atm){return evaluateAerodynamics(TrainerAircraftModel::instance(),st,c,atm,{});}
void integrateFlightState(FlightState&s,const AeroEvaluation&a,float dt){integrateFlightState(TrainerAircraftModel::instance(),s,a,dt);}
