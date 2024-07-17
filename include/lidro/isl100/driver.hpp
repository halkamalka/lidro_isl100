#ifndef __LIDRO_ISL100_DRIVER_H__
#define __LIDRO_ISL100_DRIVER_H__

#include <cstdint>
#include <vector>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <future>
#include "lidro/isl100/config.hpp"
#include <sensor_msgs/msg/point_cloud.hpp>
#include "lidro/isl100/interface.hpp"
#include <sensor_msgs/msg/image.hpp>
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>

namespace lidro::isl100 {

class Driver final : public rclcpp::Node {
public:
  explicit Driver(const rclcpp::NodeOptions &options);
  ~Driver() override;
  Driver(Driver &&c) = delete;
  Driver &operator=(Driver &&c) = delete;
  Driver(const Driver &c) = delete;
  Driver &operator=(const Driver &c) = delete;

  int updateFrame(std::unique_ptr<Frame> frame);

private:
  int publishImage(Frame &frame);
  int publish(Frame &frame);

  rcl_interfaces::msg::SetParametersResult onParamUpdated(const std::vector<rclcpp::Parameter> &parameters);
  void initialize();


private:
  lidro::isl100::Config config_;
  std::unique_ptr<lidro::isl100::Interface> interface_;
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rclcpp::Time last_time_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ambi_pub_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr intensity_pub_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_{nullptr};
};

} // end namespace lidro

#endif // __LIDRO_ISL100_DRIVER_H__
