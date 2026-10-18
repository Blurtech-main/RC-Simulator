#pragma once

#include <windows.h>
#include <cstdint>
#include <string>
#include <vector>
#include "flight_math.h"
#include "troubleshooting_feed.h"

enum class TrajectoryRoute { None, StraightLevel, ConstantRadiusTurn, FigureEight, ClimbLevelDescend, PrecisionCircuit };
enum class TrajectoryView { Main, Top, Side, WindTunnel };
enum class DashboardAction { None,FlightView,ViewTop,ViewSide,ViewWindTunnel,ToggleFollow,ToggleLayer,ExportRun,Record,Stop,RouteNone,RouteStraight,RouteTurn,RouteEight,RouteClimb,RouteCircuit,MarkRaising,MarkBumpy,MarkSluggish,MarkOther };

struct DashboardReadout {
  std::wstring aircraft{L"C172 ELECTRIC"},controller{L"KEYBOARD"},mode{L"LIVE"};
  float batteryVoltage{},motorCurrent{},propRpm{},groundEffect{};
  bool trainerAssist{};
};

struct TrajectoryTracePoint {
  uint64_t frame{};
  V3 actual{},reference{};
  float horizontalError{},verticalError{},totalError{};
};

class TrajectoryDiagnostics {
  struct ForecastCheck { uint64_t issuedFrame{},targetFrame{}; V3 expected{}; Controls controls{}; };
  struct DataRow {
    std::string kind,scenario;
    uint64_t issuedFrame{},targetFrame{};
    V3 actual{},reference{};
    Controls controls{};
    float horizontalError{},verticalError{},totalError{};
  };
  TrajectoryRoute route_{TrajectoryRoute::None};
  TrajectoryView view_{TrajectoryView::Main};
  bool visible_{true},closeFollow_{true};
  V3 routeOrigin_{};
  V3 routeForward_{0,0,1},routeRight_{1,0,0};
  std::vector<V3> routeLine_,previewLine_;
  std::vector<TrajectoryTracePoint> trace_;
  std::vector<ForecastCheck> forecasts_;
  std::vector<DataRow> rows_;
  uint64_t lastTraceFrame_{UINT64_MAX};

  void rebuildRoute();
public:
  void reset(V3 aircraftPosition,float headingRadians);
  void cycleRoute(V3 aircraftPosition,float headingRadians);
  void setRoute(TrajectoryRoute route,V3 aircraftPosition,float headingRadians);
  void cycleView();
  void setView(TrajectoryView view){view_=view;}
  void toggleVisible(){visible_=!visible_;}
  void toggleCloseFollow(){closeFollow_=!closeFollow_;}
  void setCloseFollow(bool value){closeFollow_=value;}
  bool visible() const{return visible_;}
  bool closeFollow() const{return closeFollow_;}
  TrajectoryRoute route() const{return route_;}
  TrajectoryView view() const{return view_;}
  const wchar_t* routeName() const;
  const wchar_t* viewName() const;
  const char* routeKey() const;
  const std::vector<V3>& routeLine() const{return routeLine_;}

  void setPrediction(uint64_t issuedFrame,uint64_t stepFrames,const std::vector<V3>& points,Controls controls);
  void sample(uint64_t frame,V3 actual,Controls controls);
  void exportCsv(const std::string& path) const;
  float recentRms() const;
  float recentMaximum() const;

  void drawMain(HDC dc,int width,int horizon,int bottom,V3 aircraftPosition,float cameraHeading) const;
  void drawDiagnosticView(HDC dc,const RECT& bounds,V3 aircraftPosition,float aircraftHeading) const;
  void drawDashboard(HDC dc,const RECT& bounds,V3 aircraftPosition,float aircraftHeading,const TroubleshootingFeed& feed,const DashboardReadout& readout) const;
  DashboardAction dashboardHitTest(int x,int y,int width,int height) const;
};
