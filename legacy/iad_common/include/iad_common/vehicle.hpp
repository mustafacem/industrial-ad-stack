// Vehicle parameters and footprint checks shared by simulation, planning and control.
// base_link = rear-axle centre at ground level (Autoware convention), x forward, y left, z up.
#pragma once
#include <string>

#include "iad_common/grid.hpp"
#include "iad_common/math.hpp"

namespace iad {

struct VehicleParams {
  std::string name = "haul_truck";
  double wheelbase = 4.3;      // [m]
  double track = 3.3;          // [m] wheel track (for pitch/roll from terrain)
  double width = 4.2;          // [m] body width
  double front = 6.6;          // [m] base_link -> front bumper
  double rear = 2.8;           // [m] base_link -> rear bumper
  double height = 4.4;         // [m]
  double max_steer = 0.6;      // [rad] road wheel angle
  double max_steer_rate = 0.35;  // [rad/s] hydraulic steering
  double steer_tau = 0.25;     // [s] steering first-order lag
  double delay = 0.4;          // [s] actuator dead time (CAN + hydraulics)
  double accel_tau = 0.6;      // [s] drivetrain / brake lag
  double max_acc = 2.0;        // [m/s^2] rimpull/traction limit at low speed, empty (power limits it above)
  double max_dec = 2.5;        // [m/s^2] service brake
  double mass_empty = 32000;   // [kg]
  double payload = 40000;      // [kg] when loaded
  double power = 400e3;        // [W] at the wheels (limits acceleration when loaded / uphill)
  double rolling_resistance = 0.03;  // fraction of g on haul roads

  double length() const { return front + rear; }
};

inline VehicleParams haulTruckParams() { return VehicleParams{}; }

inline VehicleParams containerAgvParams() {
  VehicleParams v;
  v.name = "container_agv";
  v.wheelbase = 7.0;
  v.track = 2.6;
  v.width = 3.0;
  v.front = 10.0;
  v.rear = 3.0;
  v.height = 4.2;  // with a container on top
  v.max_steer = 0.55;
  v.max_steer_rate = 0.4;
  v.steer_tau = 0.2;
  v.delay = 0.25;
  v.accel_tau = 0.4;
  v.max_acc = 1.5;
  v.max_dec = 2.0;
  v.mass_empty = 25000;
  v.payload = 30000;
  v.power = 250e3;
  v.rolling_resistance = 0.015;
  return v;
}

// Rectangle footprint (+ margin) vs cost grid: true if any sample lands on a lethal cell or off-map.
inline bool footprintCollides(const CostGrid& g, double x, double y, double yaw, const VehicleParams& v,
                              double margin) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double len = v.front + v.rear + 2 * margin, wid = v.width + 2 * margin;
  const int na = int(std::ceil(len / (0.9 * g.res))) + 1, nb = int(std::ceil(wid / (0.9 * g.res))) + 1;
  for (int ia = 0; ia < na; ++ia) {
    const double a = -v.rear - margin + len * ia / (na - 1);
    for (int ib = 0; ib < nb; ++ib) {
      const double b = -wid / 2 + wid * ib / (nb - 1);
      int i, j;
      if (!g.toCell(x + a * c - b * s, y + a * s + b * c, i, j) || g.at(i, j) >= kLethal) return true;
    }
  }
  return false;
}

}  // namespace iad
