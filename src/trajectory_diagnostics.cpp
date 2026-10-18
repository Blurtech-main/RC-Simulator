#include "trajectory_diagnostics.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>

namespace {
constexpr COLORREF Cyan=RGB(42,220,238),Green=RGB(60,230,105),Amber=RGB(255,196,45),Red=RGB(255,70,70),Purple=RGB(178,105,255),Grid=RGB(43,65,72),Ink=RGB(224,240,242);
V3 localPoint(V3 origin,V3 right,V3 forward,float side,float up,float along){return origin+right*side+V3{0,up,0}+forward*along;}
float horizontalDistance(V3 a,V3 b){float x=a.x-b.x,z=a.z-b.z;return std::sqrt(x*x+z*z);}
COLORREF errorColour(float e){return e<=2.f?Green:e<=5.f?Amber:Red;}
void polyline(HDC dc,const std::vector<POINT>& points,COLORREF colour,int width,int style=PS_SOLID){
  if(points.size()<2)return;HPEN pen=CreatePen(style,width,colour);auto old=SelectObject(dc,pen);Polyline(dc,points.data(),static_cast<int>(points.size()));SelectObject(dc,old);DeleteObject(pen);
}
void label(HDC dc,int x,int y,const std::wstring& text,COLORREF colour=Ink){SetBkMode(dc,TRANSPARENT);SetTextColor(dc,colour);TextOutW(dc,x,y,text.c_str(),static_cast<int>(text.size()));}
const char* routeId(TrajectoryRoute route){
  switch(route){case TrajectoryRoute::StraightLevel:return "straight_level";case TrajectoryRoute::ConstantRadiusTurn:return "constant_radius_turn";case TrajectoryRoute::FigureEight:return "figure_eight";case TrajectoryRoute::ClimbLevelDescend:return "climb_level_descend";case TrajectoryRoute::PrecisionCircuit:return "precision_circuit_01";default:return "held_controls_1s";}
}
RECT rect(int l,int t,int r,int b){return {l,t,r,b};}
bool inside(const RECT& r,int x,int y){return x>=r.left&&x<r.right&&y>=r.top&&y<r.bottom;}
struct DashboardLayout {
  RECT header{},map{},rail{},plots{};
  std::array<RECT,6> routes{};
  RECT flight{},top{},side{},wind{},follow{},layer{},exportRun{},record{},stop{};
  RECT markRaising{},markBumpy{},markSluggish{},markOther{};
};
DashboardLayout dashboardLayout(int width,int height){
  DashboardLayout l{};int pad=12,headerH=66,plotH=std::clamp(int(height*.24f),178,226),railW=std::clamp(int(width*.29f),330,420);int contentTop=headerH+10,plotsTop=height-plotH-pad,railLeft=width-railW-pad;
  l.header=rect(0,0,width,headerH);l.map=rect(pad,contentTop,railLeft-10,plotsTop-10);l.rail=rect(railLeft,contentTop,width-pad,plotsTop-10);l.plots=rect(pad,plotsTop,width-pad,height-pad);
  int bx=340,by=15,bh=36;l.flight=rect(bx,by,bx+86,by+bh);l.top=rect(bx+92,by,bx+154,by+bh);l.side=rect(bx+160,by,bx+226,by+bh);l.wind=rect(bx+232,by,bx+354,by+bh);l.follow=rect(bx+360,by,bx+472,by+bh);l.layer=rect(bx+478,by,bx+570,by+bh);l.exportRun=rect(width-142,by,width-14,by+bh);
  int cardY=contentTop+46,cardH=48;for(int i=0;i<6;i++)l.routes[i]=rect(railLeft+10,cardY+i*(cardH+6),width-pad-10,cardY+i*(cardH+6)+cardH);
  int buttonsY=cardY+6*(cardH+6)+8,half=(railW-32)/2;l.record=rect(railLeft+10,buttonsY,railLeft+10+half,buttonsY+34);l.stop=rect(railLeft+16+half,buttonsY,width-pad-10,buttonsY+34);
  int issueY=buttonsY+44;l.markRaising=rect(railLeft+10,issueY,railLeft+10+half,issueY+34);l.markBumpy=rect(railLeft+16+half,issueY,width-pad-10,issueY+34);l.markSluggish=rect(railLeft+10,issueY+40,railLeft+10+half,issueY+74);l.markOther=rect(railLeft+16+half,issueY+40,width-pad-10,issueY+74);return l;
}
void fill(HDC dc,const RECT& r,COLORREF colour){HBRUSH brush=CreateSolidBrush(colour);FillRect(dc,&r,brush);DeleteObject(brush);}
void strokeRect(HDC dc,const RECT& r,COLORREF colour,int width=1){HPEN pen=CreatePen(PS_SOLID,width,colour);auto oldPen=SelectObject(dc,pen);auto oldBrush=SelectObject(dc,GetStockObject(HOLLOW_BRUSH));RoundRect(dc,r.left,r.top,r.right,r.bottom,8,8);SelectObject(dc,oldBrush);SelectObject(dc,oldPen);DeleteObject(pen);}
void dashboardButton(HDC dc,const RECT& r,const std::wstring& text,bool active=false,COLORREF accent=Cyan){fill(dc,r,active?RGB(15,58,69):RGB(13,31,39));strokeRect(dc,r,active?accent:RGB(40,67,77),active?2:1);SIZE sz{};GetTextExtentPoint32W(dc,text.c_str(),static_cast<int>(text.size()),&sz);label(dc,(r.left+r.right-sz.cx)/2,(r.top+r.bottom-sz.cy)/2,text,active?RGB(245,252,253):RGB(190,210,216));}
std::wstring oneDecimal(float value){wchar_t b[32]{};swprintf_s(b,L"%.1f",value);return b;}
void metricCard(HDC dc,const RECT& r,const std::wstring& title,const std::wstring& value,COLORREF valueColour){fill(dc,r,RGB(10,28,35));strokeRect(dc,r,RGB(35,65,75));label(dc,r.left+9,r.top+7,title,RGB(145,172,180));label(dc,r.left+9,r.top+27,value,valueColour);}
HBITMAP landscapeBitmap(){static HBITMAP bitmap=[](){wchar_t module[MAX_PATH]{};GetModuleFileNameW(nullptr,module,MAX_PATH);std::filesystem::path p(module);p=p.parent_path()/L"assets"/L"grass-field-cheap.bmp";return (HBITMAP)LoadImageW(nullptr,p.c_str(),IMAGE_BITMAP,0,0,LR_LOADFROMFILE|LR_CREATEDIBSECTION);}();return bitmap;}
void landscape(HDC dc,const RECT& r){HBITMAP bitmap=landscapeBitmap();if(!bitmap){fill(dc,r,RGB(18,46,36));return;}BITMAP info{};GetObject(bitmap,sizeof(info),&info);HDC source=CreateCompatibleDC(dc);auto old=SelectObject(source,bitmap);SetStretchBltMode(dc,HALFTONE);StretchBlt(dc,r.left,r.top,r.right-r.left,r.bottom-r.top,source,0,0,info.bmWidth,info.bmHeight,SRCCOPY);SelectObject(source,old);DeleteDC(source);HBRUSH shade=CreateSolidBrush(RGB(8,30,24));BLENDFUNCTION blend{AC_SRC_OVER,0,85,0};HDC tint=CreateCompatibleDC(dc);HBITMAP layer=CreateCompatibleBitmap(dc,r.right-r.left,r.bottom-r.top);auto oldLayer=SelectObject(tint,layer);RECT local{0,0,r.right-r.left,r.bottom-r.top};FillRect(tint,&local,shade);AlphaBlend(dc,r.left,r.top,local.right,local.bottom,tint,0,0,local.right,local.bottom,blend);SelectObject(tint,oldLayer);DeleteObject(layer);DeleteDC(tint);DeleteObject(shade);}
void routeIcon(HDC dc,const RECT& r,int route,COLORREF colour){std::vector<POINT> p;int cx=(r.left+r.right)/2,cy=(r.top+r.bottom)/2,w=(r.right-r.left)/2-2,h=(r.bottom-r.top)/2-2;if(route==0)p={{r.left+2,cy},{r.right-5,cy},{r.right-10,cy-4},{r.right-5,cy},{r.right-10,cy+4}};else if(route==1)p={{r.left+2,cy},{r.right-4,cy},{r.right-9,cy-4},{r.right-4,cy},{r.right-9,cy+4}};else if(route==2){for(int i=0;i<=24;i++){float t=2*PI*i/24.f;p.push_back({LONG(cx+std::cos(t)*w*.72f),LONG(cy+std::sin(t)*h*.82f)});}}else if(route==3){for(int i=0;i<=40;i++){float t=2*PI*i/40.f;p.push_back({LONG(cx+std::sin(t)*w),LONG(cy+std::sin(2*t)*h*.72f)});}}else if(route==4)p={{r.left+2,r.bottom-3},{cx-4,r.top+5},{cx+7,r.top+5},{r.right-3,r.bottom-3}};else p={{r.left+5,r.bottom-5},{r.left+5,r.top+5},{r.right-5,r.top+5},{r.right-5,r.bottom-5},{r.left+5,r.bottom-5}};polyline(dc,p,colour,2,route<=1?PS_DASH:PS_SOLID);}
}

const wchar_t* TrajectoryDiagnostics::routeName() const{
  switch(route_){case TrajectoryRoute::StraightLevel:return L"STRAIGHT & LEVEL";case TrajectoryRoute::ConstantRadiusTurn:return L"CONSTANT-RADIUS TURN";case TrajectoryRoute::FigureEight:return L"FIGURE EIGHT";case TrajectoryRoute::ClimbLevelDescend:return L"CLIMB - LEVEL - DESCEND";case TrajectoryRoute::PrecisionCircuit:return L"PRECISION CIRCUIT 01";default:return L"HELD-CONTROL PREDICTION";}
}
const wchar_t* TrajectoryDiagnostics::viewName() const{
  switch(view_){case TrajectoryView::Top:return L"TOP";case TrajectoryView::Side:return L"SIDE";case TrajectoryView::WindTunnel:return L"WIND TUNNEL";default:return L"MAIN";}
}
const char* TrajectoryDiagnostics::routeKey() const{return routeId(route_);}

void TrajectoryDiagnostics::reset(V3 p,float heading){
  routeOrigin_=p;routeForward_={std::sin(heading),0,std::cos(heading)};routeRight_={routeForward_.z,0,-routeForward_.x};
  trace_.clear();forecasts_.clear();previewLine_.clear();rows_.clear();lastTraceFrame_=UINT64_MAX;rebuildRoute();
}
void TrajectoryDiagnostics::setRoute(TrajectoryRoute route,V3 p,float heading){route_=route;reset(p,heading);}
void TrajectoryDiagnostics::cycleRoute(V3 p,float heading){setRoute(static_cast<TrajectoryRoute>((static_cast<int>(route_)+1)%6),p,heading);}
void TrajectoryDiagnostics::cycleView(){view_=static_cast<TrajectoryView>((static_cast<int>(view_)+1)%4);}

void TrajectoryDiagnostics::rebuildRoute(){
  routeLine_.clear();if(route_==TrajectoryRoute::None)return;constexpr int N=240;routeLine_.reserve(N+1);
  for(int i=0;i<=N;i++){
    float u=float(i)/N,side=0,along=0,up=0;
    if(route_==TrajectoryRoute::StraightLevel){along=300.f*u;}
    else if(route_==TrajectoryRoute::ConstantRadiusTurn){float t=2.f*PI*u;side=55.f*(1.f-std::cos(t));along=55.f*std::sin(t);}
    else if(route_==TrajectoryRoute::FigureEight){float t=2.f*PI*u;along=95.f*std::sin(t);side=48.f*std::sin(2.f*t);}
    else if(route_==TrajectoryRoute::ClimbLevelDescend){along=300.f*u;if(u<.30f)up=20.f*u/.30f;else if(u<.70f)up=20.f;else up=20.f*(1.f-(u-.70f)/.30f);}
    else {float t=2.f*PI*u;along=95.f*std::sin(t);side=52.f*(1.f-std::cos(t));up=5.f*std::sin(2.f*t);}
    routeLine_.push_back(localPoint(routeOrigin_,routeRight_,routeForward_,side,up,along));
  }
}

void TrajectoryDiagnostics::setPrediction(uint64_t issued,uint64_t stepFrames,const std::vector<V3>& points,Controls c){
  previewLine_=points;if(points.size()<2||route_!=TrajectoryRoute::None)return;
  const size_t oneSecond=std::min(points.size()-1,size_t(std::max<uint64_t>(1,120/stepFrames)));
  forecasts_.push_back({issued,issued+oneSecond*stepFrames,points[oneSecond],c});
  if(forecasts_.size()>64)forecasts_.erase(forecasts_.begin(),forecasts_.begin()+(forecasts_.size()-64));
}

void TrajectoryDiagnostics::sample(uint64_t frame,V3 actual,Controls controls){
  if(lastTraceFrame_!=UINT64_MAX&&frame-lastTraceFrame_<12)return;lastTraceFrame_=frame;
  if(route_!=TrajectoryRoute::None&&!routeLine_.empty()){
    auto nearest=std::min_element(routeLine_.begin(),routeLine_.end(),[&](V3 a,V3 b){return len2(a-actual)<len2(b-actual);});
    float horizontal=horizontalDistance(actual,*nearest),vertical=actual.y-nearest->y,total=len(actual-*nearest);
    trace_.push_back({frame,actual,*nearest,horizontal,vertical,total});rows_.push_back({"route",routeId(route_),frame,frame,actual,*nearest,controls,horizontal,vertical,total});
  }else{
    trace_.push_back({frame,actual,actual,0,0,0});
    for(auto it=forecasts_.begin();it!=forecasts_.end();){
      if(it->targetFrame>frame){++it;continue;}float horizontal=horizontalDistance(actual,it->expected),vertical=actual.y-it->expected.y,total=len(actual-it->expected);
      rows_.push_back({"prediction","held_controls_1s",it->issuedFrame,it->targetFrame,actual,it->expected,it->controls,horizontal,vertical,total});it=forecasts_.erase(it);
    }
  }
  if(trace_.size()>1800)trace_.erase(trace_.begin(),trace_.begin()+300);if(rows_.size()>12000)rows_.erase(rows_.begin(),rows_.begin()+2000);
}

void TrajectoryDiagnostics::exportCsv(const std::string& path) const{
  std::filesystem::path p(path);if(p.has_parent_path())std::filesystem::create_directories(p.parent_path());std::ofstream out(path,std::ios::trunc);
  out<<"kind,scenario,issued_frame,target_frame,issued_time_s,target_time_s,actual_x,actual_y,actual_z,reference_x,reference_y,reference_z,horizontal_error_m,vertical_error_m,total_error_m,aileron,elevator,throttle,rudder\n"<<std::setprecision(9);
  for(const auto&r:rows_)out<<r.kind<<','<<r.scenario<<','<<r.issuedFrame<<','<<r.targetFrame<<','<<double(r.issuedFrame)/120.0<<','<<double(r.targetFrame)/120.0<<','<<r.actual.x<<','<<r.actual.y<<','<<r.actual.z<<','<<r.reference.x<<','<<r.reference.y<<','<<r.reference.z<<','<<r.horizontalError<<','<<r.verticalError<<','<<r.totalError<<','<<r.controls.ail<<','<<r.controls.ele<<','<<r.controls.thr<<','<<r.controls.rud<<'\n';
}
float TrajectoryDiagnostics::recentRms() const{if(rows_.empty())return 0;size_t first=rows_.size()>300?rows_.size()-300:0;double sum=0;for(size_t i=first;i<rows_.size();i++)sum+=rows_[i].totalError*rows_[i].totalError;return float(std::sqrt(sum/double(rows_.size()-first)));}
float TrajectoryDiagnostics::recentMaximum() const{float m=0;size_t first=rows_.size()>300?rows_.size()-300:0;for(size_t i=first;i<rows_.size();i++)m=std::max(m,rows_[i].totalError);return m;}

void TrajectoryDiagnostics::drawMain(HDC dc,int width,int horizon,int bottom,V3 aircraft,float heading) const{
  if(!visible_||view_!=TrajectoryView::Main)return;float forwardX=std::sin(heading),forwardZ=std::cos(heading),rightX=forwardZ,rightZ=-forwardX;
  float focal=width/(2.f*std::tan(65.f*PI/360.f)),cameraY=aircraft.y+1.8f;
  auto project=[&](V3 p,POINT& out){float dx=p.x-aircraft.x,dz=p.z-aircraft.z,depth=dx*forwardX+dz*forwardZ;if(depth<1.f)return false;float lateral=dx*rightX+dz*rightZ;out={LONG(width*.5f+lateral*focal/depth),LONG(horizon+(cameraY-p.y)*focal/depth)};return out.x>-width&&out.x<width*2&&out.y>-bottom&&out.y<bottom*2;};
  const auto& reference=route_==TrajectoryRoute::None?previewLine_:routeLine_;if(route_!=TrajectoryRoute::None){for(float side:{-7.f,7.f}){std::vector<POINT> lane;for(size_t i=0;i<reference.size();i++){size_t a=i?i-1:i,b=std::min(reference.size()-1,i+1);V3 t=reference[b]-reference[a];float n=std::max(.01f,std::sqrt(t.x*t.x+t.z*t.z));V3 q=reference[i]+V3{t.z/n*side,0,-t.x/n*side};POINT s;if(project(q,s))lane.push_back(s);}polyline(dc,lane,RGB(70,190,205),1,PS_DOT);}}std::vector<POINT> intended;for(V3 p:reference){POINT q;if(project(p,q))intended.push_back(q);}polyline(dc,intended,Cyan,3,PS_DASH);
  for(size_t i=1;i<trace_.size();i++){POINT a{},b{};if(project(trace_[i-1].actual,a)&&project(trace_[i].actual,b)){HPEN pen=CreatePen(PS_SOLID,3,route_==TrajectoryRoute::None?Purple:errorColour(trace_[i].totalError));auto old=SelectObject(dc,pen);MoveToEx(dc,a.x,a.y,nullptr);LineTo(dc,b.x,b.y);SelectObject(dc,old);DeleteObject(pen);}}
  if(route_!=TrajectoryRoute::None){size_t stride=std::max<size_t>(1,trace_.size()/70);for(size_t i=0;i<trace_.size();i+=stride){if(trace_[i].totalError<.25f)continue;POINT a{},b{};if(project(trace_[i].actual,a)&&project(trace_[i].reference,b)){HPEN pen=CreatePen(PS_SOLID,2,errorColour(trace_[i].totalError));auto old=SelectObject(dc,pen);MoveToEx(dc,b.x,b.y,nullptr);LineTo(dc,a.x,a.y);SelectObject(dc,old);DeleteObject(pen);}}}
  // Always-visible padded overview: the chase camera can no longer hide a
  // deviation merely because it is behind or outside the perspective frustum.
  if(route_!=TrajectoryRoute::None&&!reference.empty()){RECT inset{std::max(12,width-352),std::max(horizon+18,bottom-240),width-18,bottom-18};landscape(dc,inset);strokeRect(dc,inset,Cyan,2);float minX=1e9f,maxX=-1e9f,minZ=1e9f,maxZ=-1e9f;for(V3 p:reference){minX=std::min(minX,p.x);maxX=std::max(maxX,p.x);minZ=std::min(minZ,p.z);maxZ=std::max(maxZ,p.z);}for(const auto&t:trace_){minX=std::min(minX,t.actual.x);maxX=std::max(maxX,t.actual.x);minZ=std::min(minZ,t.actual.z);maxZ=std::max(maxZ,t.actual.z);}float scale=.84f*std::min(float(inset.right-inset.left-20)/std::max(20.f,maxX-minX),float(inset.bottom-inset.top-30)/std::max(20.f,maxZ-minZ));float cx=(minX+maxX)*.5f,cz=(minZ+maxZ)*.5f;auto top=[&](V3 p){return POINT{LONG((inset.left+inset.right)*.5f+(p.x-cx)*scale),LONG((inset.top+inset.bottom)*.5f-(p.z-cz)*scale)};};std::vector<POINT> ref;for(V3 p:reference)ref.push_back(top(p));polyline(dc,ref,Cyan,2,PS_DASH);for(size_t i=1;i<trace_.size();i++){POINT a=top(trace_[i-1].actual),b=top(trace_[i].actual);HPEN pen=CreatePen(PS_SOLID,2,errorColour(trace_[i].totalError));auto old=SelectObject(dc,pen);MoveToEx(dc,a.x,a.y,nullptr);LineTo(dc,b.x,b.y);SelectObject(dc,old);DeleteObject(pen);}size_t stride=std::max<size_t>(1,trace_.size()/40);for(size_t i=0;i<trace_.size();i+=stride){POINT a=top(trace_[i].actual),b=top(trace_[i].reference);HPEN pen=CreatePen(PS_SOLID,1,errorColour(trace_[i].totalError));auto old=SelectObject(dc,pen);MoveToEx(dc,b.x,b.y,nullptr);LineTo(dc,a.x,a.y);SelectObject(dc,old);DeleteObject(pen);}label(dc,inset.left+9,inset.top+7,L"ROUTE OVERVIEW  |  deviation whiskers",Ink);}
}

void TrajectoryDiagnostics::drawDiagnosticView(HDC dc,const RECT& r,V3 aircraft,float aircraftHeading) const{
  if(view_==TrajectoryView::Main)return;HBRUSH bg=CreateSolidBrush(RGB(8,20,26));FillRect(dc,&r,bg);DeleteObject(bg);
  HPEN grid=CreatePen(PS_SOLID,1,Grid);auto oldPen=SelectObject(dc,grid);for(int x=r.left;x<r.right;x+=50){MoveToEx(dc,x,r.top,nullptr);LineTo(dc,x,r.bottom);}for(int y=r.top;y<r.bottom;y+=50){MoveToEx(dc,r.left,y,nullptr);LineTo(dc,r.right,y);}SelectObject(dc,oldPen);DeleteObject(grid);
  if(!visible_){label(dc,r.left+18,r.top+16,std::wstring(L"TRAJECTORY ")+viewName()+L" VIEW  |  LAYER OFF (H)",RGB(130,145,148));return;}
  const auto& reference=route_==TrajectoryRoute::None?previewLine_:routeLine_;std::vector<V3> all=reference;for(const auto&t:trace_)all.push_back(t.actual);if(all.empty())all.push_back(aircraft);
  V3 centre=aircraft;float scale=5.f;
  if(!closeFollow_){float minA=std::numeric_limits<float>::max(),maxA=-minA,minB=minA,maxB=-minA;for(V3 p:all){float a=view_==TrajectoryView::Top?p.x:p.z,b=view_==TrajectoryView::Top?p.z:p.y;minA=std::min(minA,a);maxA=std::max(maxA,a);minB=std::min(minB,b);maxB=std::max(maxB,b);}if(view_==TrajectoryView::Top){centre.x=(minA+maxA)*.5f;centre.z=(minB+maxB)*.5f;}else{centre.z=(minA+maxA)*.5f;centre.y=(minB+maxB)*.5f;}float sx=(r.right-r.left-80)/std::max(20.f,maxA-minA),sy=(r.bottom-r.top-100)/std::max(10.f,maxB-minB);scale=std::max(.5f,std::min(sx,sy));}
  else scale=view_==TrajectoryView::Top?7.f:9.f;
  auto project=[&](V3 p){float a{},b{};if(view_==TrajectoryView::Top){if(closeFollow_){float dx=p.x-centre.x,dz=p.z-centre.z;a=dx*std::cos(aircraftHeading)-dz*std::sin(aircraftHeading);b=dx*std::sin(aircraftHeading)+dz*std::cos(aircraftHeading);}else{a=p.x-centre.x;b=p.z-centre.z;}}else{float forwardX=std::sin(aircraftHeading),forwardZ=std::cos(aircraftHeading);a=closeFollow_?(p.x-centre.x)*forwardX+(p.z-centre.z)*forwardZ:p.z-centre.z;b=p.y-centre.y;}return POINT{LONG((r.left+r.right)*.5f+a*scale),LONG((r.top+r.bottom)*.5f-b*scale)};};
  std::vector<POINT> intended;for(V3 p:reference)intended.push_back(project(p));polyline(dc,intended,Cyan,3,PS_DASH);
  for(size_t i=1;i<trace_.size();i++){POINT a=project(trace_[i-1].actual),b=project(trace_[i].actual);HPEN pen=CreatePen(PS_SOLID,3,route_==TrajectoryRoute::None?Purple:errorColour(trace_[i].totalError));auto old=SelectObject(dc,pen);MoveToEx(dc,a.x,a.y,nullptr);LineTo(dc,b.x,b.y);SelectObject(dc,old);DeleteObject(pen);}
  if(route_!=TrajectoryRoute::None)for(size_t i=0;i<trace_.size();i+=8){POINT a=project(trace_[i].actual),b=project(trace_[i].reference);HPEN pen=CreatePen(PS_SOLID,1,errorColour(trace_[i].totalError));auto old=SelectObject(dc,pen);MoveToEx(dc,a.x,a.y,nullptr);LineTo(dc,b.x,b.y);SelectObject(dc,old);DeleteObject(pen);}
  POINT plane=project(aircraft);POINT tri[4]={{plane.x,plane.y-12},{plane.x-8,plane.y+10},{plane.x+8,plane.y+10},{plane.x,plane.y-12}};polyline(dc,std::vector<POINT>(tri,tri+4),Ink,2);
  label(dc,r.left+18,r.top+16,std::wstring(L"TRAJECTORY ")+viewName()+L" VIEW  |  "+(closeFollow_?L"CLOSE FOLLOW":L"ROUTE OVERVIEW"),Cyan);
  label(dc,r.left+18,r.top+42,std::wstring(routeName())+L"  |  cyan: intended/predicted  green/amber/red: actual deviation");
}

void TrajectoryDiagnostics::drawDashboard(HDC dc,const RECT& bounds,V3 aircraft,float aircraftHeading,const TroubleshootingFeed& feed,const DashboardReadout& readout) const{
  DashboardLayout l=dashboardLayout(bounds.right-bounds.left,bounds.bottom-bounds.top);fill(dc,bounds,RGB(5,14,19));
  HFONT regular=CreateFontW(16,0,0,0,FW_NORMAL,FALSE,FALSE,FALSE,DEFAULT_CHARSET,OUT_DEFAULT_PRECIS,CLIP_DEFAULT_PRECIS,CLEARTYPE_QUALITY,FIXED_PITCH|FF_MODERN,L"B612 Mono");HFONT bold=CreateFontW(19,0,0,0,FW_BOLD,FALSE,FALSE,FALSE,DEFAULT_CHARSET,OUT_DEFAULT_PRECIS,CLIP_DEFAULT_PRECIS,CLEARTYPE_QUALITY,FIXED_PITCH|FF_MODERN,L"B612 Mono");auto oldFont=SelectObject(dc,regular);
  fill(dc,l.header,RGB(8,22,30));RECT headerLine{0,l.header.bottom-2,l.header.right,l.header.bottom};fill(dc,headerLine,RGB(24,68,80));SelectObject(dc,bold);label(dc,16,11,L"TRAJECTORY VALIDATION",RGB(220,236,255));SelectObject(dc,regular);label(dc,16,39,readout.aircraft+L"  |  CALM AIR  |  "+readout.controller,RGB(125,160,170));
  dashboardButton(dc,l.flight,L"FLIGHT",false);dashboardButton(dc,l.top,L"TOP",view_==TrajectoryView::Top);dashboardButton(dc,l.side,L"SIDE",view_==TrajectoryView::Side);dashboardButton(dc,l.wind,L"WIND TUNNEL",view_==TrajectoryView::WindTunnel);dashboardButton(dc,l.follow,closeFollow_?L"CLOSE FOLLOW":L"OVERVIEW",closeFollow_);dashboardButton(dc,l.layer,visible_?L"LAYER ON":L"LAYER OFF",visible_);dashboardButton(dc,l.exportRun,L"EXPORT RUN",false,RGB(66,150,255));
  int statusX=l.layer.right+12;label(dc,statusX,13,L"CODEX LIVE FEED",RGB(145,172,180));label(dc,statusX,34,L"● CONNECTED",Green);

  fill(dc,l.map,RGB(9,25,29));strokeRect(dc,l.map,RGB(35,70,79));RECT mapPlot=l.map;mapPlot.top+=43;mapPlot.bottom-=78;if(view_==TrajectoryView::WindTunnel)fill(dc,mapPlot,RGB(7,20,28));else landscape(dc,mapPlot);
  HPEN gridPen=CreatePen(PS_SOLID,1,view_==TrajectoryView::WindTunnel?RGB(21,52,66):RGB(35,67,56));auto previousPen=SelectObject(dc,gridPen);for(int x=mapPlot.left;x<mapPlot.right;x+=55){MoveToEx(dc,x,mapPlot.top,nullptr);LineTo(dc,x,mapPlot.bottom);}for(int y=mapPlot.top;y<mapPlot.bottom;y+=55){MoveToEx(dc,mapPlot.left,y,nullptr);LineTo(dc,mapPlot.right,y);}SelectObject(dc,previousPen);DeleteObject(gridPen);
  label(dc,l.map.left+14,l.map.top+12,std::wstring(viewName())+L" VIEW  |  "+(closeFollow_?L"PLANE FOLLOW":L"WHOLE ROUTE"),Cyan);label(dc,l.map.right-320,l.map.top+12,routeName(),RGB(190,213,220));
  const auto& current=feed.current();
  if(view_==TrajectoryView::WindTunnel){
    int cx=(mapPlot.left+mapPlot.right)/2,cy=(mapPlot.top+mapPlot.bottom)/2;float angle=current.aoaDeg*PI/180.f;
    for(int i=-3;i<=3;i++){std::vector<POINT> flow;for(int s=0;s<=24;s++){float u=float(s)/24.f,x=float(mapPlot.left+20)+u*(mapPlot.right-mapPlot.left-40),influence=std::exp(-std::pow((x-cx)/150.f,2.f));float y=cy+i*34+std::sin(angle)*120.f*(u-.5f)-influence*i*2.f;flow.push_back({LONG(x),LONG(y)});}polyline(dc,flow,i==0?Cyan:RGB(70,139,160),i==0?3:1,PS_SOLID);}
    HPEN aircraftPen=CreatePen(PS_SOLID,4,RGB(235,241,239));auto old=SelectObject(dc,aircraftPen);MoveToEx(dc,cx-115,cy,nullptr);LineTo(dc,cx+105,cy);MoveToEx(dc,cx-25,cy,nullptr);LineTo(dc,cx+12,cy-28);LineTo(dc,cx+55,cy);MoveToEx(dc,cx-76,cy,nullptr);LineTo(dc,cx-98,cy-20);SelectObject(dc,old);DeleteObject(aircraftPen);
    auto arrow=[&](int x1,int y1,int x2,int y2,COLORREF colour,const wchar_t* name){HPEN p=CreatePen(PS_SOLID,4,colour);auto op=SelectObject(dc,p);MoveToEx(dc,x1,y1,nullptr);LineTo(dc,x2,y2);POINT h[3]={{x2,y2},{x2+(x1-x2)/5-6,y2+(y1-y2)/5},{x2+(x1-x2)/5+6,y2+(y1-y2)/5}};Polyline(dc,h,3);SelectObject(dc,op);DeleteObject(p);label(dc,x2+7,y2-9,name,colour);};
    arrow(cx,cy,cx,cy-std::clamp(int(current.lift*2.f),35,120),Cyan,L"LIFT");arrow(cx,cy,cx-std::clamp(int(current.drag*5.f),30,105),cy,Amber,L"DRAG");arrow(cx,cy,cx+std::clamp(int(current.thrust*5.f),30,105),cy,Green,L"THRUST");arrow(cx,cy,cx,cy+70,RGB(170,185,190),L"WEIGHT");
    label(dc,mapPlot.left+18,mapPlot.top+14,L"SIMPLIFIED SOLVER VIEW - NOT CFD",Amber);label(dc,mapPlot.left+18,mapPlot.top+39,L"Relative airflow and force vectors from the live aerodynamic model",RGB(170,197,204));label(dc,mapPlot.right-280,mapPlot.top+18,L"AoA "+oneDecimal(current.aoaDeg)+L" deg   TAS "+oneDecimal(current.airspeed)+L" m/s",Ink);label(dc,mapPlot.right-280,mapPlot.top+43,L"Stall "+std::to_wstring(int(current.stallFraction*100))+L"%   Pitch "+oneDecimal(current.pitchDeg)+L" deg",current.stallFraction>.4f?Red:Ink);
  }else if(visible_){
    const auto& reference=route_==TrajectoryRoute::None?previewLine_:routeLine_;std::vector<V3> all=reference;for(const auto&t:trace_)all.push_back(t.actual);if(all.empty())all.push_back(aircraft);V3 centre=aircraft;float scale=view_==TrajectoryView::Top?5.f:7.f;
    if(!closeFollow_){float minA=1e9f,maxA=-1e9f,minB=1e9f,maxB=-1e9f;all.push_back(aircraft);for(V3 p:all){float a=view_==TrajectoryView::Top?p.x:p.z,b=view_==TrajectoryView::Top?p.z:p.y;minA=std::min(minA,a);maxA=std::max(maxA,a);minB=std::min(minB,b);maxB=std::max(maxB,b);}if(view_==TrajectoryView::Top){centre.x=(minA+maxA)*.5f;centre.z=(minB+maxB)*.5f;}else{centre.z=(minA+maxA)*.5f;centre.y=(minB+maxB)*.5f;}scale=std::max(.4f,.84f*std::min(float(mapPlot.right-mapPlot.left-80)/std::max(20.f,maxA-minA+14.f),float(mapPlot.bottom-mapPlot.top-65)/std::max(10.f,maxB-minB+(view_==TrajectoryView::Top?14.f:4.f))));}
    auto project=[&](V3 p){float a{},b{};if(view_==TrajectoryView::Top){if(closeFollow_){float dx=p.x-centre.x,dz=p.z-centre.z;a=dx*std::cos(aircraftHeading)-dz*std::sin(aircraftHeading);b=dx*std::sin(aircraftHeading)+dz*std::cos(aircraftHeading);}else{a=p.x-centre.x;b=p.z-centre.z;}}else{float fx=std::sin(aircraftHeading),fz=std::cos(aircraftHeading);a=closeFollow_?(p.x-centre.x)*fx+(p.z-centre.z)*fz:p.z-centre.z;b=p.y-centre.y;}return POINT{LONG((mapPlot.left+mapPlot.right)*.5f+a*scale),LONG((mapPlot.top+mapPlot.bottom)*.5f-b*scale)};};
    if(route_!=TrajectoryRoute::None){for(float offset:{-7.f,7.f}){std::vector<POINT> lane;for(size_t i=0;i<reference.size();i++){V3 q=reference[i];if(view_==TrajectoryView::Side)q.y+=offset*(2.f/7.f);else{size_t a=i?i-1:i,b=std::min(reference.size()-1,i+1);V3 t=reference[b]-reference[a];float n=std::max(.01f,std::sqrt(t.x*t.x+t.z*t.z));q+=V3{t.z/n*offset,0,-t.x/n*offset};}lane.push_back(project(q));}polyline(dc,lane,RGB(92,195,207),2,PS_DOT);}}
    std::vector<POINT> intended;for(V3 p:reference)intended.push_back(project(p));polyline(dc,intended,Cyan,4,PS_DASH);if(route_!=TrajectoryRoute::None){size_t stride=std::max<size_t>(1,trace_.size()/80);for(size_t i=0;i<trace_.size();i+=stride){if(trace_[i].totalError<.25f)continue;POINT a=project(trace_[i].actual),b=project(trace_[i].reference);HPEN p=CreatePen(PS_SOLID,3,errorColour(trace_[i].totalError));auto op=SelectObject(dc,p);MoveToEx(dc,b.x,b.y,nullptr);LineTo(dc,a.x,a.y);SelectObject(dc,op);DeleteObject(p);}}
    for(size_t i=1;i<trace_.size();i++){POINT a=project(trace_[i-1].actual),b=project(trace_[i].actual);HPEN p=CreatePen(PS_SOLID,4,route_==TrajectoryRoute::None?Purple:errorColour(trace_[i].totalError));auto op=SelectObject(dc,p);MoveToEx(dc,a.x,a.y,nullptr);LineTo(dc,b.x,b.y);SelectObject(dc,op);DeleteObject(p);}POINT planePoint=project(aircraft);POINT icon[4]={{planePoint.x,planePoint.y-13},{planePoint.x-9,planePoint.y+11},{planePoint.x+9,planePoint.y+11},{planePoint.x,planePoint.y-13}};polyline(dc,std::vector<POINT>(icon,icon+4),RGB(245,250,250),2);
    label(dc,mapPlot.left+14,mapPlot.top+12,L"- - Intended / prediction",Cyan);label(dc,mapPlot.left+220,mapPlot.top+12,L"Actual: green / amber / red",Green);
  }else label(dc,mapPlot.left+20,mapPlot.top+20,L"TRAJECTORY LAYER OFF - click LAYER OFF to restore",RGB(145,172,180));

  int cardGap=8,cardW=(l.map.right-l.map.left-cardGap*5)/4,cardY=l.map.bottom-68;RECT c1=rect(l.map.left,cardY,l.map.left+cardW, l.map.bottom);RECT c2=rect(c1.right+cardGap,cardY,c1.right+cardGap+cardW,l.map.bottom);RECT c3=rect(c2.right+cardGap,cardY,c2.right+cardGap+cardW,l.map.bottom);RECT c4=rect(c3.right+cardGap,cardY,l.map.right,l.map.bottom);metricCard(dc,c1,L"CROSS-TRACK RMS",oneDecimal(recentRms())+L" m",recentRms()>5?Red:recentRms()>2?Amber:Green);metricCard(dc,c2,L"MAX DEVIATION",oneDecimal(recentMaximum())+L" m",recentMaximum()>8?Red:recentMaximum()>3?Amber:Green);metricCard(dc,c3,L"RAISING / BUMPY",oneDecimal(current.raisingScore*100)+L"% / "+oneDecimal(current.bumpinessScore*100)+L"%",current.raising||current.bumpy?Red:Green);metricCard(dc,c4,L"SLUGGISH / OSCILLATION",oneDecimal(current.sluggishScore*100)+L"% / "+oneDecimal(current.oscillationScore*100)+L"%",current.sluggish||current.oscillating?Red:Green);

  fill(dc,l.rail,RGB(8,22,29));strokeRect(dc,l.rail,RGB(35,70,79));SelectObject(dc,bold);label(dc,l.rail.left+12,l.rail.top+12,L"TEST ROUTES",RGB(220,236,242));SelectObject(dc,regular);const wchar_t* routeNames[6]={L"HELD-CONTROL PREDICTION",L"STRAIGHT & LEVEL",L"CONSTANT-RADIUS TURN",L"FIGURE EIGHT",L"CLIMB - LEVEL - DESCEND",L"PRECISION CIRCUIT 01"};for(int i=0;i<6;i++){bool active=static_cast<int>(route_)==i;fill(dc,l.routes[i],active?RGB(12,54,65):RGB(12,30,38));strokeRect(dc,l.routes[i],active?Cyan:RGB(38,66,75),active?2:1);RECT iconBox{l.routes[i].left+8,l.routes[i].top+8,l.routes[i].left+52,l.routes[i].bottom-8};routeIcon(dc,iconBox,i,active?Cyan:RGB(130,170,180));label(dc,l.routes[i].left+60,l.routes[i].top+7,routeNames[i],active?RGB(238,250,252):RGB(194,214,220));label(dc,l.routes[i].left+60,l.routes[i].top+27,i==0?L"Live 2 s solver forecast":i==1?L"Heading and altitude":i==2?L"360 degree level turn":i==3?L"Linked turns":i==4?L"Climb, hold, descend":L"Mixed turns and height",RGB(125,154,164));}
  dashboardButton(dc,l.record,L"RECORD",false,Green);dashboardButton(dc,l.stop,L"STOP",false,Amber);dashboardButton(dc,l.markRaising,L"MARK RAISING",false,Amber);dashboardButton(dc,l.markBumpy,L"MARK BUMPY",false,Amber);dashboardButton(dc,l.markSluggish,L"MARK SLUGGISH",false,Amber);dashboardButton(dc,l.markOther,L"MARK OTHER",false,Amber);

  fill(dc,l.plots,RGB(7,18,24));strokeRect(dc,l.plots,RGB(35,70,79));const auto& h=feed.history();auto drawChart=[&](RECT r,const std::wstring& title,COLORREF aColour,COLORREF bColour,auto getA,auto getB,float range){fill(dc,r,RGB(8,24,31));strokeRect(dc,r,RGB(30,58,68));label(dc,r.left+9,r.top+7,title,RGB(185,208,215));int top=r.top+29,bottom=r.bottom-12,mid=(top+bottom)/2;HPEN gp=CreatePen(PS_SOLID,1,RGB(28,51,59));auto op=SelectObject(dc,gp);MoveToEx(dc,r.left+6,mid,nullptr);LineTo(dc,r.right-6,mid);SelectObject(dc,op);DeleteObject(gp);if(h.size()<2)return;size_t first=h.size()>300?h.size()-300:0;std::vector<POINT> pa,pb;for(size_t i=first;i<h.size();i++){float u=float(i-first)/float(std::max<size_t>(1,h.size()-first-1));int x=int(r.left+7+u*(r.right-r.left-14));pa.push_back({x,LONG(mid-clampf(getA(h[i])/range,-1.f,1.f)*(bottom-top)*.47f)});pb.push_back({x,LONG(mid-clampf(getB(h[i])/range,-1.f,1.f)*(bottom-top)*.47f)});}polyline(dc,pa,aColour,2);polyline(dc,pb,bColour,2);};int pw=(l.plots.right-l.plots.left-24)/3;RECT p1=rect(l.plots.left+7,l.plots.top+7,l.plots.left+7+pw,l.plots.bottom-7),p2=rect(p1.right+5,l.plots.top+7,p1.right+5+pw,l.plots.bottom-7),p3=rect(p2.right+5,l.plots.top+7,l.plots.right-7,l.plots.bottom-7);drawChart(p1,L"ALTITUDE / VERTICAL SPEED",Cyan,Green,[](const TroubleshootingSample&s){return s.position.y;},[](const TroubleshootingSample&s){return s.velocity.y*5.f;},30.f);drawChart(p2,L"PITCH RATE / ROLL RATE",Purple,Amber,[](const TroubleshootingSample&s){return -s.omegaBody.x*180.f/PI;},[](const TroubleshootingSample&s){return -s.omegaBody.z*180.f/PI;},80.f);drawChart(p3,L"ELEVATOR / AILERON",Cyan,Green,[](const TroubleshootingSample&s){return s.controls.ele;},[](const TroubleshootingSample&s){return s.controls.ail;},1.f);
  label(dc,18,bounds.bottom-8,L"Session "+std::wstring(feed.sessionId().begin(),feed.sessionId().end())+L"  |  aircraft "+readout.aircraft+L"  |  controller "+readout.controller+L"  |  "+readout.mode,RGB(105,136,146));
  SelectObject(dc,oldFont);DeleteObject(regular);DeleteObject(bold);
}

DashboardAction TrajectoryDiagnostics::dashboardHitTest(int x,int y,int width,int height) const{
  auto l=dashboardLayout(width,height);if(inside(l.flight,x,y))return DashboardAction::FlightView;if(inside(l.top,x,y))return DashboardAction::ViewTop;if(inside(l.side,x,y))return DashboardAction::ViewSide;if(inside(l.wind,x,y))return DashboardAction::ViewWindTunnel;if(inside(l.follow,x,y))return DashboardAction::ToggleFollow;if(inside(l.layer,x,y))return DashboardAction::ToggleLayer;if(inside(l.exportRun,x,y))return DashboardAction::ExportRun;if(inside(l.record,x,y))return DashboardAction::Record;if(inside(l.stop,x,y))return DashboardAction::Stop;if(inside(l.markRaising,x,y))return DashboardAction::MarkRaising;if(inside(l.markBumpy,x,y))return DashboardAction::MarkBumpy;if(inside(l.markSluggish,x,y))return DashboardAction::MarkSluggish;if(inside(l.markOther,x,y))return DashboardAction::MarkOther;for(int i=0;i<6;i++)if(inside(l.routes[i],x,y))return static_cast<DashboardAction>(static_cast<int>(DashboardAction::RouteNone)+i);return DashboardAction::None;
}
