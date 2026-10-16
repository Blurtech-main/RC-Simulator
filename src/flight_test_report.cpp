#include "flight_test_report.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace {
constexpr float radToDeg=180.f/PI;

struct Derived { float pitch{},roll{},heading{}; };
Derived attitude(const FlightTestSample&s){
  V3 forward=rotate(s.attitude,{0,0,1}),right=rotate(s.attitude,{1,0,0}),up=rotate(s.attitude,{0,1,0});
  return {std::asin(clampf(forward.y,-1.f,1.f))*radToDeg,
    std::atan2(right.y,up.y)*radToDeg,std::atan2(forward.x,forward.z)*radToDeg};
}

std::string htmlEscape(const std::string&s){
  std::string out;out.reserve(s.size());for(char c:s){switch(c){case '&':out+="&amp;";break;case '<':out+="&lt;";break;case '>':out+="&gt;";break;case '"':out+="&quot;";break;default:out+=c;}}return out;
}

std::string f(float v,int decimals=2){std::ostringstream o;o<<std::fixed<<std::setprecision(decimals)<<v;return o.str();}

float correlation(const std::vector<float>&a,const std::vector<float>&b){
  if(a.size()!=b.size()||a.size()<12)return std::numeric_limits<float>::quiet_NaN();
  double ma=0,mb=0;for(size_t i=0;i<a.size();++i){ma+=a[i];mb+=b[i];}ma/=a.size();mb/=b.size();
  double aa=0,bb=0,ab=0;for(size_t i=0;i<a.size();++i){double x=a[i]-ma,y=b[i]-mb;aa+=x*x;bb+=y*y;ab+=x*y;}
  return aa>1e-8&&bb>1e-8?float(ab/std::sqrt(aa*bb)):std::numeric_limits<float>::quiet_NaN();
}

float spread(const std::vector<float>&v){if(v.empty())return 0;auto [lo,hi]=std::minmax_element(v.begin(),v.end());return *hi-*lo;}

struct Series { std::string name,color;std::vector<float> values;float fixedLow{},fixedHigh{};bool fixed{}; };

std::string chart(const std::string&title,const std::vector<FlightTestSample>&samples,std::vector<Series> series){
  constexpr float width=920,height=250,left=48,right=18,top=24,bottom=35;
  std::ostringstream o;o<<"<section class='chart'><h2>"<<htmlEscape(title)<<"</h2><svg viewBox='0 0 "<<width<<" "<<height<<"' role='img' aria-label='"<<htmlEscape(title)<<"'>";
  o<<"<rect x='"<<left<<"' y='"<<top<<"' width='"<<(width-left-right)<<"' height='"<<(height-top-bottom)<<"' class='plot'/>";
  for(int i=0;i<=4;i++){float y=top+(height-top-bottom)*i/4.f;o<<"<line x1='"<<left<<"' y1='"<<y<<"' x2='"<<(width-right)<<"' y2='"<<y<<"' class='grid'/>";}
  const double end=samples.empty()?0:samples.back().timeSeconds;
  o<<"<text x='"<<left<<"' y='"<<(height-9)<<"'>0 s</text><text text-anchor='end' x='"<<(width-right)<<"' y='"<<(height-9)<<"'>"<<f(float(end),1)<<" s</text>";
  size_t stride=std::max<size_t>(1,samples.size()/700);
  for(auto& s:series){
    if(s.values.empty())continue;float lo=s.fixedLow,hi=s.fixedHigh;if(!s.fixed){auto mm=std::minmax_element(s.values.begin(),s.values.end());lo=*mm.first;hi=*mm.second;if(hi-lo<1e-5f){lo-=1;hi+=1;}}
    o<<"<polyline fill='none' stroke='"<<s.color<<"' stroke-width='2' points='";
    for(size_t i=0;i<s.values.size();i+=stride){float x=left+(width-left-right)*float(i)/std::max<size_t>(1,s.values.size()-1);float y=top+(height-top-bottom)*(1.f-clampf((s.values[i]-lo)/(hi-lo),0.f,1.f));o<<x<<','<<y<<' ';}
    if((s.values.size()-1)%stride)o<<(width-right)<<','<<(top+(height-top-bottom)*(1.f-clampf((s.values.back()-lo)/(hi-lo),0.f,1.f)));
    o<<"'/><g class='legend'><rect width='10' height='3' fill='"<<s.color<<"'/><text x='14' y='4'>"<<htmlEscape(s.name)<<" ["<<f(lo,1)<<" to "<<f(hi,1)<<"]</text></g>";
  }
  o<<"</svg><div class='key'>";for(auto&s:series)o<<"<span style='color:"<<s.color<<"'>"<<htmlEscape(s.name)<<"</span> ";o<<"</div></section>";return o.str();
}

std::string statusCard(const std::string&status,const std::string&name,const std::string&detail){
  return "<article class='card "+status+"'><b>"+htmlEscape(status)+" - "+htmlEscape(name)+"</b><p>"+htmlEscape(detail)+"</p></article>";
}

std::string controlFinding(const char* name,const std::vector<float>&input,float corr){
  if(input.size()<12||spread(input)<.12f)return std::string(name)+": NOT EXERCISED (not enough stick movement at flying speed)";
  if(!std::isfinite(corr))return std::string(name)+": INCONCLUSIVE";
  if(corr<-.18f)return std::string(name)+": LIKELY REVERSED (response correlation "+f(corr)+")";
  if(corr>.18f)return std::string(name)+": expected sign (response correlation "+f(corr)+")";
  return std::string(name)+": weak/inconclusive response (correlation "+f(corr)+")";
}
}

FlightTestReportSummary writeFlightTestReport(const std::string&htmlPath,const std::string&csvPath,
  const std::string&title,const std::string&source,const std::vector<FlightTestSample>&samples,bool scriptedScenario,
  const std::vector<FlightTestEnvelopeCheck>& envelopeChecks){
  FlightTestReportSummary result{};if(samples.size()<2)return result;
  std::filesystem::path hp(htmlPath),cp(csvPath);if(hp.has_parent_path())std::filesystem::create_directories(hp.parent_path());if(cp.has_parent_path())std::filesystem::create_directories(cp.parent_path());
  std::ofstream csv(cp,std::ios::trunc);csv<<"time_s,frame,aileron,elevator,throttle,rudder,x_m,y_m,z_m,vx_mps,vy_mps,vz_mps,pitch_deg,roll_deg,heading_deg,omega_x_dps,omega_y_dps,omega_z_dps,aoa_deg,beta_deg,airspeed_mps,lift_n,drag_n,thrust_n,stall_fraction,crashed\n";
  std::vector<float> ail,ele,thr,rud,x,y,z,vx,vy,vz,pitch,roll,heading,ox,oy,oz,aoa,airspeed,stall,lift,drag,thrust;
  float peakAltitude=-1e9f,peakSpeed=0,maxAbsRoll=0;bool crashed=false,finite=true;
  for(const auto&s:samples){auto d=attitude(s);ail.push_back(s.controls.ail);ele.push_back(s.controls.ele);thr.push_back(s.controls.thr);rud.push_back(s.controls.rud);x.push_back(s.positionWorld.x);y.push_back(s.positionWorld.y);z.push_back(s.positionWorld.z);vx.push_back(s.velocityWorld.x);vy.push_back(s.velocityWorld.y);vz.push_back(s.velocityWorld.z);pitch.push_back(d.pitch);roll.push_back(d.roll);heading.push_back(d.heading);ox.push_back(s.angularRateBody.x*radToDeg);oy.push_back(s.angularRateBody.y*radToDeg);oz.push_back(s.angularRateBody.z*radToDeg);aoa.push_back(s.aoa*radToDeg);airspeed.push_back(s.airspeed);stall.push_back(s.stallFraction*100);lift.push_back(s.lift);drag.push_back(s.drag);thrust.push_back(s.thrust);peakAltitude=std::max(peakAltitude,s.positionWorld.y);peakSpeed=std::max(peakSpeed,s.airspeed);maxAbsRoll=std::max(maxAbsRoll,std::abs(d.roll));crashed|=s.crashed;finite&=std::isfinite(s.positionWorld.x)&&std::isfinite(s.positionWorld.y)&&std::isfinite(s.positionWorld.z)&&std::isfinite(d.pitch);csv<<std::setprecision(9)<<s.timeSeconds<<','<<s.frame<<','<<s.controls.ail<<','<<s.controls.ele<<','<<s.controls.thr<<','<<s.controls.rud<<','<<s.positionWorld.x<<','<<s.positionWorld.y<<','<<s.positionWorld.z<<','<<s.velocityWorld.x<<','<<s.velocityWorld.y<<','<<s.velocityWorld.z<<','<<d.pitch<<','<<d.roll<<','<<d.heading<<','<<s.angularRateBody.x*radToDeg<<','<<s.angularRateBody.y*radToDeg<<','<<s.angularRateBody.z*radToDeg<<','<<s.aoa*radToDeg<<','<<s.beta*radToDeg<<','<<s.airspeed<<','<<s.lift<<','<<s.drag<<','<<s.thrust<<','<<s.stallFraction<<','<<s.crashed<<'\n';}
  csv.close();

  std::vector<float> responsePitch,responseRoll,responseYaw,inputEle,inputAil,inputRud;
  for(size_t i=0;i<samples.size();++i)if(samples[i].airspeed>8.f&&!samples[i].crashed){inputEle.push_back(ele[i]);responsePitch.push_back(-ox[i]);inputAil.push_back(ail[i]);responseRoll.push_back(-oz[i]);inputRud.push_back(rud[i]);responseYaw.push_back(oy[i]);}
  float ce=correlation(inputEle,responsePitch),ca=correlation(inputAil,responseRoll),cr=correlation(inputRud,responseYaw);
  result.elevatorFinding=controlFinding("Elevator",inputEle,ce);result.aileronFinding=controlFinding("Aileron",inputAil,ca);result.rudderFinding=controlFinding("Rudder",inputRud,cr);
  bool reversed=result.elevatorFinding.find("LIKELY REVERSED")!=std::string::npos||result.aileronFinding.find("LIKELY REVERSED")!=std::string::npos||result.rudderFinding.find("LIKELY REVERSED")!=std::string::npos;

  bool pitchUp=false,pitchDown=false,climbed=peakAltitude>2.5f;
  for(size_t i=0;i<samples.size();++i){
    if(samples[i].airspeed<10.f)continue;size_t later=std::min(samples.size()-1,i+60);float noseUpRateNow=-ox[i],noseUpRateLater=-ox[later];
    bool startsPull=ele[i]>.15f&&(i==0||ele[i-1]<=.15f),startsPush=ele[i]<-.15f&&(i==0||ele[i-1]>=-.15f);
    // Attitude has inertia: a correct push can first slow an existing climb
    // before the geometric pitch angle falls. Test the commanded angular-rate
    // change, while plotting both pitch and vertical trajectory for inspection.
    if(startsPull&&noseUpRateLater-noseUpRateNow>.8f)pitchUp=true;
    if(startsPush&&noseUpRateLater-noseUpRateNow<-.8f)pitchDown=true;
  }
  bool envelopePassed=true;for(const auto& check:envelopeChecks)if(check.mandatory&&!check.passed)envelopePassed=false;
  result.mandatoryPassed=finite&&!crashed&&envelopePassed&&(!scriptedScenario||(climbed&&pitchUp&&pitchDown));
  result.verdict=!result.mandatoryPassed?"FAIL":reversed?"CAUTION":"PASS";

  std::ofstream html(hp,std::ios::trunc);if(!html)return result;
  html<<"<!doctype html><meta charset='utf-8'><meta name='viewport' content='width=device-width'><title>"<<htmlEscape(title)<<"</title><style>body{margin:0;background:#071116;color:#e7f2ef;font:16px system-ui,Segoe UI,sans-serif}main{max-width:980px;margin:auto;padding:28px}h1{margin:.1em 0}.sub{color:#9eb8b5}.banner{padding:18px;border-radius:12px;font-size:30px;font-weight:800;background:#14323a;border-left:12px solid #54d6a6}.banner.FAIL{border-color:#ff6363}.banner.CAUTION{border-color:#ffd166}.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:12px;margin:18px 0}.card,.chart{background:#0d1d24;border:1px solid #24404a;border-radius:10px;padding:14px}.card p{margin:.45em 0;color:#bed0ce}.card.FAIL{border-color:#ff6363}.card.CAUTION{border-color:#ffd166}.chart{margin:14px 0}.chart h2{margin:0 0 7px;font-size:18px}.chart svg{width:100%;background:#08151a;border-radius:6px}.plot{fill:#08151a;stroke:#38535a}.grid{stroke:#183039;stroke-width:1}.chart text{fill:#91aaa8;font-size:11px}.legend{display:none}.key span{margin-right:18px;font-weight:700}.note{padding:14px;background:#13262d;border-radius:8px;color:#c8d8d5}code{color:#8be0ca}</style><main>";
  html<<"<p class='sub'>VIRTUAL RC TEST PILOT / deterministic telemetry</p><h1>"<<htmlEscape(title)<<"</h1><p>Source: <b>"<<htmlEscape(source)<<"</b> | "<<f(float(samples.back().timeSeconds),1)<<" seconds | "<<samples.size()<<" samples at 120 Hz</p><div class='banner "<<result.verdict<<"'>"<<result.verdict<<"</div>";
  html<<"<div class='cards'>"<<statusCard(finite?"PASS":"FAIL","Finite state",finite?"All sampled positions and attitudes are finite.":"The simulation produced a non-finite position or attitude.")<<statusCard(crashed?"FAIL":"PASS","Crash state",crashed?"A crash occurred during the run.":"No crash occurred during the run.")<<statusCard(climbed?"PASS":"CAUTION","Takeoff / climb","Peak Y altitude "+f(peakAltitude)+" m; peak airspeed "+f(peakSpeed)+" m/s.")<<statusCard(maxAbsRoll<35?"PASS":"CAUTION","Lateral stability","Maximum absolute bank "+f(maxAbsRoll)+" degrees.")<<"</div>";
  html<<"<div class='cards'>"<<statusCard(result.elevatorFinding.find("REVERSED")!=std::string::npos?"FAIL":"INFO","Elevator sign",result.elevatorFinding)<<statusCard(result.aileronFinding.find("REVERSED")!=std::string::npos?"FAIL":"INFO","Aileron sign",result.aileronFinding)<<statusCard(result.rudderFinding.find("REVERSED")!=std::string::npos?"FAIL":"INFO","Rudder sign",result.rudderFinding)<<"</div>";
  if(scriptedScenario)html<<"<div class='cards'>"<<statusCard(pitchUp?"PASS":"FAIL","Pull response",pitchUp?"Positive elevator increased the nose-up angular response within 0.5 seconds.":"No adequate nose-up angular response followed the scripted pull.")<<statusCard(pitchDown?"PASS":"FAIL","Push response",pitchDown?"Negative elevator reduced the nose-up angular response within 0.5 seconds.":"No adequate nose-down angular response followed the scripted push.")<<"</div>";
  if(!envelopeChecks.empty()){
    html<<"<h2>Expected trajectory corridors</h2><div class='cards'>";
    for(const auto& check:envelopeChecks){
      std::string detail="Expected: "+check.expected+" Observed: "+check.observed;
      if(!check.diagnosis.empty())detail+=" Diagnosis: "+check.diagnosis;
      html<<statusCard(check.passed?"PASS":(check.mandatory?"FAIL":"CAUTION"),check.name,detail);
    }
    html<<"</div>";
  }
  html<<chart("Controls: what the simulator actually received",samples,{{"aileron","#ff8a65",ail,-1,1,true},{"elevator","#64b5f6",ele,-1,1,true},{"throttle","#81c784",thr,0,1,true},{"rudder","#ffd54f",rud,-1,1,true}});
  html<<chart("Nose and vertical response",samples,{{"pitch deg","#64b5f6",pitch},{"Y altitude m","#81c784",y},{"vertical speed m/s","#ffd54f",vy}});
  html<<chart("World position over time (X/Y/Z individually scaled)",samples,{{"X right m","#ff8a65",x},{"Y up m","#81c784",y},{"Z forward m","#64b5f6",z}});
  html<<chart("Attitude over time",samples,{{"pitch deg","#64b5f6",pitch},{"bank deg","#ff8a65",roll},{"heading deg","#ffd54f",heading}});
  html<<chart("Energy, angle of attack, and stall",samples,{{"airspeed m/s","#81c784",airspeed},{"AoA deg","#64b5f6",aoa},{"stall percent","#ff8a65",stall}});
  html<<"<p class='note'><b>How to read this:</b> positive elevator means stick-back / nose-up. X is right, Y is up, and Z is forward. Each coloured trace is scaled to the numeric range printed in its key. The expected corridors compare isolated manoeuvres against a neutral aeroplane started from the identical state, so inertia cannot masquerade as a reversed response.</p><p class='note'><b>Evidence boundary:</b> this automated engineering oracle catches repeatable sign, coupling, stability, energy and trajectory regressions. It replaces a large amount of repetitive regression flying, but transmitter feel and final realism still require human RC-pilot judgement.</p></main>";
  result.wrote=bool(html);return result;
}
