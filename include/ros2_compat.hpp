#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <cfloat>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>

namespace livox_ros_driver { using CustomMsg = livox_ros_driver2::msg::CustomMsg; }
namespace sensor_msgs { using Imu = msg::Imu; using PointCloud2 = msg::PointCloud2; using ImuConstPtr = msg::Imu::ConstSharedPtr; }
namespace nav_msgs { using Odometry = msg::Odometry; using Path = msg::Path; }
namespace geometry_msgs { using Quaternion = msg::Quaternion; using PoseStamped = msg::PoseStamped; using Vector3 = msg::Vector3; }

inline double stamp_seconds(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}
inline builtin_interfaces::msg::Time stamp_from_seconds(double seconds) {
  return static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(static_cast<int64_t>(std::llround(seconds * 1e9)), RCL_ROS_TIME));
}

namespace ros {
inline rclcpp::Node::SharedPtr &node_instance() { static rclcpp::Node::SharedPtr node; return node; }
inline void init(int argc, char **argv, const char *) { rclcpp::init(argc, argv); }
inline bool ok() { return rclcpp::ok(); }
inline void spinOnce() { if (rclcpp::ok()) rclcpp::spin_some(node_instance()); }
class Rate { public: explicit Rate(double hz): rate_(hz) {} void sleep() { rate_.sleep(); } private: rclcpp::Rate rate_; };
class Publisher {
public:
  Publisher() = default;
  template<class T> static Publisher make(const typename rclcpp::Publisher<T>::SharedPtr &pub) {
    Publisher result;
    result.publish_ = [pub](const void *msg) { pub->publish(*static_cast<const T *>(msg)); };
    return result;
  }
  template<class T> void publish(const T &msg) const { publish_(&msg); }
private:
  std::function<void(const void *)> publish_;
};
using Subscriber = rclcpp::SubscriptionBase::SharedPtr;
class NodeHandle {
public:
  NodeHandle() { node_instance() = std::make_shared<rclcpp::Node>("li_init"); }
  template<class T> void param(const std::string &name, T &out, const T &fallback) {
    std::string key = name;
    for (auto &c: key) if (c == '/') c = '.';
    if constexpr (std::is_same_v<T, float>) {
      out = static_cast<float>(node_instance()->declare_parameter<double>(key, static_cast<double>(fallback)));
    } else { out = node_instance()->declare_parameter<T>(key, fallback); }
  }
  template<class T> Publisher advertise(const std::string &topic, size_t depth) {
    return Publisher::make<T>(node_instance()->create_publisher<T>(topic, rclcpp::QoS(std::min(depth, size_t(1000)))));
  }
  template<class T, class F> Subscriber subscribe(const std::string &topic, size_t depth, F callback) {
    return node_instance()->create_subscription<T>(topic, rclcpp::SensorDataQoS().keep_last(std::min(depth, size_t(1000))), callback);
  }
};
}
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("li_init"), __VA_ARGS__)
#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("li_init"), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("li_init"), __VA_ARGS__)
#define ROS_ASSERT(expr) assert(expr)

namespace tf {
inline geometry_msgs::Quaternion createQuaternionMsgFromRollPitchYaw(double roll, double pitch, double yaw) {
  double cr=std::cos(roll/2), sr=std::sin(roll/2);
  double cp=std::cos(pitch/2), sp=std::sin(pitch/2);
  double cy=std::cos(yaw/2), sy=std::sin(yaw/2);
  geometry_msgs::Quaternion q;
  q.x=sr*cp*cy-cr*sp*sy; q.y=cr*sp*cy+sr*cp*sy;
  q.z=cr*cp*sy-sr*sp*cy; q.w=cr*cp*cy+sr*sp*sy;
  return q;
}
}
