// gnss_effects: turns Gazebo's ideal NavSat fix into a realistic receiver output. The antenna position is
// ray-tested against the same 3D site geometry Gazebo renders: open sky -> RTK fix (2 cm), near steel
// stacks and cranes -> float with Gauss-Markov multipath bias, under structure -> single / no fix.
// Reported covariance is deliberately optimistic, as real receivers are.
//   in : /sim/gnss/raw        sensor_msgs/NavSatFix (Gazebo navsat, WGS84)
//   out: /sensing/gnss/fix    sensor_msgs/NavSatFix (status: GBAS = RTK fix, SBAS = float, FIX = single)
//        /sim/gnss/debug      std_msgs/String JSON (true error, sky blocked) - evaluation only
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>

#include "iad_gz_sim/geodesy.hpp"
#include "iad_gz_sim/gnss_model.hpp"

using namespace iad::sim;

class GnssEffects : public rclcpp::Node {
 public:
  GnssEffects() : Node("gnss_effects") {
    const std::string scenario = declare_parameter("scenario", "port");
    const int seed = int(declare_parameter("seed", 7));
    origin_ = iad::geo::Origin(declare_parameter("origin_latitude", 35.6206), declare_parameter("origin_longitude", 139.79),
                               declare_parameter("origin_altitude", 0.0));
    enabled_ = declare_parameter("enabled", true);
    world_ = makeWorld(scenario, uint32_t(seed));
    rng_ = FastRng(uint64_t(seed) * 31u + 5u);
    pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/sensing/gnss/fix", 10);
    dbg_ = create_publisher<std_msgs::msg::String>("/sim/gnss/debug", 10);
    sub_ = create_subscription<sensor_msgs::msg::NavSatFix>("/sim/gnss/raw", 10, [this](sensor_msgs::msg::NavSatFix::SharedPtr m) { onFix(*m); });
    RCLCPP_INFO(get_logger(), "GNSS effects on %s world (%s)", scenario.c_str(), enabled_ ? "enabled" : "pass-through");
  }

 private:
  void onFix(sensor_msgs::msg::NavSatFix m) {
    const double t = rclcpp::Time(m.header.stamp).seconds();
    const double dt = last_t_ < 0 ? 0.1 : std::clamp(t - last_t_, 0.0, 1.0);
    last_t_ = t;
    const Eigen::Vector3d p = origin_.toEnu(m.latitude, m.longitude, m.altitude);
    m.header.frame_id = "gnss_link";
    if (!enabled_) {
      m.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      setCov(m, 0.02);
      pub_->publish(m);
      return;
    }
    const GnssSample s = model_.sample(world_, p, dt, rng_);
    char buf[200];
    std::snprintf(buf, sizeof(buf), "{\"t\":%.2f,\"status\":\"%s\",\"error\":%.3f,\"sky_blocked\":%.3f}", t,
                  gnssStatusName(s.status), s.error, s.sky_blocked);
    std_msgs::msg::String d;
    d.data = buf;
    dbg_->publish(d);
    if (!s.valid) return;  // no fix: receivers stop reporting positions
    origin_.toGeodetic(s.pos, m.latitude, m.longitude, m.altitude);
    using St = sensor_msgs::msg::NavSatStatus;
    m.status.status = s.status == GnssStatus::kFix ? St::STATUS_GBAS_FIX : s.status == GnssStatus::kFloat ? St::STATUS_SBAS_FIX : St::STATUS_FIX;
    m.status.service = St::SERVICE_GPS;
    setCov(m, s.sigma_reported);
    pub_->publish(m);
  }
  static void setCov(sensor_msgs::msg::NavSatFix& m, double sigma) {
    m.position_covariance = {sigma * sigma, 0, 0, 0, sigma * sigma, 0, 0, 0, 4 * sigma * sigma};
    m.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  }

  World world_;
  GnssModel model_;
  FastRng rng_;
  iad::geo::Origin origin_;
  bool enabled_ = true;
  double last_t_ = -1;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dbg_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssEffects>());
  rclcpp::shutdown();
  return 0;
}
