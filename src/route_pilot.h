#pragma once

#include <cstddef>
#include <vector>
#include "flight_math.h"

struct RoutePilotState {
  V3 position{},velocity{},omegaBody{};
  Quat attitude{};
  float airspeed{},beta{};
};

// A deliberately limited virtual RC pilot. It only moves the same four sticks
// available to a human pilot; it never overwrites the simulated flight state.
class RoutePilot {
  bool active_{};
  size_t progress_{};
  float elapsed_{};
public:
  void start(){active_=true;progress_=0;elapsed_=0;}
  void stop(){active_=false;}
  bool active() const{return active_;}
  size_t progress() const{return progress_;}
  float completion(size_t points) const{return points>1?float(progress_)/float(points-1):0.f;}
  Controls update(const RoutePilotState&,const std::vector<V3>& route,float dt);
};
