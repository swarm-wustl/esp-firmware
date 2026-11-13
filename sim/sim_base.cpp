#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <tf2/exceptions.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "encoder.h"
#include "robot.h"
#include "sdkconfig.h"
#include "watchdog.h"
#include "world.h"

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistWithCovarianceStamped;
using sensor_msgs::msg::Range;
using tf2_msgs::msg::TFMessage;

// HW::POLL_PERIOD_MS, which main.cpp keeps with the ranging task
constexpr std::chrono::milliseconds RANGE_PERIOD{100};

// the one frame every robot shares, and the one the anchors are placed in. The
// world is simulated in it; each robot's odom starts wherever it was put down
constexpr char MAP_FRAME[] = "map";

World::Pose pose_from(const std::vector<double> &xyt) {
  return {xyt.at(0), xyt.at(1), xyt.at(2)};
}

class SimBase : public rclcpp::Node {
public:
  SimBase()
      : Node(Robot::NODE, CONFIG_SWARM_ROS_NAMESPACE),
        slip_(declare_parameter("slip", 0.05)),
        time_constant_(declare_parameter("time_constant", 0.08)),
        gyro_noise_(declare_parameter("gyro_noise", 0.005)),
        gyro_bias_(declare_parameter("gyro_bias", 0.002)),
        imu_(declare_parameter("imu", true)),
        anchors_(declare_parameter("anchors", std::vector<std::string>{})),
        range_noise_(declare_parameter("range_noise", 0.05)),
        world_(World::initial(
            declare_parameter("motor_mismatch", 0.05),
            pose_from(declare_parameter("start",
                                        std::vector<double>{0.0, 0.0, 0.0})))),
        odom_frame_(frame(Robot::ODOM_FRAME)),
        base_frame_(frame(Robot::BASE_FRAME)),
        truth_frame_(frame("truth")) {
    cmd_vel_ = create_subscription<Twist>(
        Robot::CMD_VEL_TOPIC, rclcpp::SensorDataQoS(),
        [this](const Twist &msg) { newest_ = Robot::read_twist(msg); });

    twist_pub_ = create_publisher<TwistWithCovarianceStamped>(
        Robot::ODOM_TOPIC, rclcpp::SensorDataQoS());
    range_pub_ =
        create_publisher<Range>(Robot::UWB_TOPIC, rclcpp::SensorDataQoS());
    truth_pub_ = create_publisher<PoseStamped>("sim/truth", 10);
    odom_pub_ = create_publisher<PoseStamped>("sim/odom", 10);
    tf_pub_ = create_publisher<TFMessage>("/tf", 100);

    control_timer_ = create_wall_timer(
        std::chrono::milliseconds(Robot::DRIVE_PERIOD_MS), [this] { step(); });
    pose_timer_ =
        create_wall_timer(std::chrono::milliseconds(Robot::ODOMETRY_PERIOD_MS),
                          [this] { publish_odometry(); });
    range_timer_ = create_wall_timer(RANGE_PERIOD, [this] { publish_range(); });
  }

private:
  // the motion task's pipeline from main.cpp, against World instead of GPIO
  // and PCNT: watched cmd_vel, inverse kinematics, Robot::drive, motors.run
  void step() {
    feed_ = Watchdog::observe(feed_, std::exchange(newest_, std::nullopt),
                              {.safe = Drive::Twist{},
                               .timeout_ms = Robot::CMD_VEL_TIMEOUT_MS,
                               .period_ms = Robot::DRIVE_PERIOD_MS});

    const auto targets = Drive::inverse_kinematics<Robot::STYLE>(feed_.value);
    const auto [next, frame] = Robot::drive<World::ENCODER>(
        base_, targets, advance(World::counts(world_)), gyro());
    base_ = next;

    world_ = World::step(world_, frame, {gauss(slip_), gauss(slip_)},
                         time_constant_, Robot::CONTROL.period);

    publish_truth();
  }

  std::array<Encoder::Reading, World::N>
  advance(std::array<Encoder::Reading, World::N> counts) {
    for (size_t i = 0; i < counts.size(); ++i) {
      counts[i].counts = trackers_[i].advance(counts[i].counts);
    }

    return counts;
  }

  // an empty mailbox -- no IMU, bus down -- is the firmware's encoder fallback
  std::optional<double> gyro() {
    if (!imu_) {
      return std::nullopt;
    }

    return world_.yaw_rate + gyro_bias_ + gauss(gyro_noise_);
  }

  double gauss(double sigma) {
    return sigma > 0.0 ? std::normal_distribution<double>{0.0, sigma}(rng_)
                       : 0.0;
  }

  // the odometry task from main.cpp. sim/odom is extra: the pose the board
  // integrates but no longer publishes, to set against sim/truth
  void publish_odometry() {
    const auto pose = base_.pose;
    const auto previous = std::exchange(last_pose_, pose);
    const auto stamp = now();

    if (previous) {
      TwistWithCovarianceStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = base_frame_;
      Robot::fill_twist(
          msg, Odometry::twist_between(*previous, pose,
                                       Robot::ODOMETRY_PERIOD_MS / 1000.0));
      twist_pub_->publish(msg);
    }

    odom_pub_->publish(
        pose_stamped(odom_frame_, pose.x, pose.y, pose.theta, stamp));
  }

  // one anchor per period, in turn, the way one radio has to range them.
  // frame_id names the anchor rather than the robot's antenna: Range has no
  // other field to say which peer answered, and the anchor's frame is where
  // the fix node looks up its position
  void publish_range() {
    if (anchors_.empty()) {
      return;
    }

    const auto &anchor = anchors_[next_anchor_++ % anchors_.size()];
    geometry_msgs::msg::Vector3 at;

    try {
      at = tf_.lookupTransform(MAP_FRAME, anchor, tf2::TimePointZero)
               .transform.translation;
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "no %s: %s",
                           anchor.c_str(), e.what());
      return;
    }

    const double distance =
        std::hypot(world_.pose.x - at.x, world_.pose.y - at.y);

    Range msg;
    msg.header.stamp = now();
    msg.header.frame_id = anchor;
    Robot::fill_range(msg, std::max(0.0, distance + gauss(range_noise_)));
    range_pub_->publish(msg);
  }

  // sim-only, like sim/truth: the EKF will own odom -> base_link, so this
  // frame must not be base_link
  void publish_truth() {
    const auto truth = pose_stamped(MAP_FRAME, world_.pose.x, world_.pose.y,
                                    world_.pose.theta, now());
    truth_pub_->publish(truth);

    TransformStamped transform;
    transform.header = truth.header;
    transform.child_frame_id = truth_frame_;
    transform.transform.translation.x = world_.pose.x;
    transform.transform.translation.y = world_.pose.y;
    transform.transform.rotation = truth.pose.orientation;

    TFMessage tf;
    tf.transforms.push_back(transform);
    tf_pub_->publish(tf);
  }

  // Robot::frame_in at runtime: one binary serves every namespace, so the
  // sim can't fix its frames at compile time the way the board does
  std::string frame(const char *name) const {
    const std::string ns = std::string(get_namespace()).substr(1);

    return ns.empty() ? name : ns + "/" + name;
  }

  static PoseStamped pose_stamped(const std::string &frame, double x, double y,
                                  double theta, const rclcpp::Time &stamp) {
    const auto [z, w] = Odometry::heading(theta);

    PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame;
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.orientation.z = z;
    msg.pose.orientation.w = w;

    return msg;
  }

  double slip_;
  double time_constant_;
  double gyro_noise_;
  double gyro_bias_;
  bool imu_;
  std::vector<std::string> anchors_;
  size_t next_anchor_ = 0;
  double range_noise_;

  World::State world_;
  Base::State<Robot::STYLE> base_{};
  std::array<Encoder::Tracker, World::N> trackers_{};
  Watchdog::Feed<Drive::Twist> feed_{Drive::Twist{}, 0};
  std::optional<Drive::Twist> newest_;
  std::optional<Odometry::Pose> last_pose_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string truth_frame_;
  std::mt19937 rng_{std::random_device{}()};
  tf2_ros::Buffer tf_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_};

  rclcpp::Subscription<Twist>::SharedPtr cmd_vel_;
  rclcpp::Publisher<TwistWithCovarianceStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<Range>::SharedPtr range_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr truth_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr odom_pub_;
  rclcpp::Publisher<TFMessage>::SharedPtr tf_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
  rclcpp::TimerBase::SharedPtr range_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimBase>());
  rclcpp::shutdown();
}
