#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <mutex>

class ClickToMove {
public:
  explicit ClickToMove(const rclcpp::Node::SharedPtr& node)
  : node_(node),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_) {

    planning_group_ = node_->declare_parameter<std::string>("planning_group", "abb_irb6700");
    base_frame_     = node_->declare_parameter<std::string>("base_frame", "base");
    actuator_link_  = node_->declare_parameter<std::string>("eef_link", "");
    clicked_topic_  = node_->declare_parameter<std::string>("clicked_point_topic", "/clicked_point");
    above_offset_   = node_->declare_parameter<double>("above_offset", 0.15);
    plan_only_      = node_->declare_parameter<bool>("plan_only", false);
    vel_scale_      = node_->declare_parameter<double>("velocity_scaling", 0.3);
    acc_scale_      = node_->declare_parameter<double>("acceleration_scaling", 0.3);
    use_current_orientation_ = node_->declare_parameter<bool>("use_current_orientation", true);
    std::vector<double> default_rpy = {M_PI, 0.0, 0.0};
    rpy_ = node_->declare_parameter<std::vector<double>>("fallback_rpy", default_rpy);

    // 用 main 里创建的 node_ 来构造 MoveGroupInterface（不会再触发 bad_weak_ptr）
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, planning_group_);
    move_group_->setPoseReferenceFrame(base_frame_);
    move_group_->setMaxVelocityScalingFactor(vel_scale_);
    move_group_->setMaxAccelerationScalingFactor(acc_scale_);
    move_group_->setPlanningTime(5.0);

    if (!actuator_link_.empty()) {
      move_group_->setEndEffectorLink(actuator_link_);
      RCLCPP_INFO(node_->get_logger(), "Using eef_link: %s", actuator_link_.c_str());
    } else {
      actuator_link_ = move_group_->getEndEffectorLink();
      RCLCPP_INFO(node_->get_logger(), "Using group's default eef_link: %s", actuator_link_.c_str());
    }

    sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
      clicked_topic_, rclcpp::QoS(10),
      std::bind(&ClickToMove::onPoint, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "click_to_move ready. group=%s, base_frame=%s, topic=%s",
                planning_group_.c_str(), base_frame_.c_str(), clicked_topic_.c_str());
  }

private:
  static geometry_msgs::msg::Point applyTransform(
      const geometry_msgs::msg::TransformStamped& T,
      const geometry_msgs::msg::Point& p_src)
  {
    tf2::Quaternion q(T.transform.rotation.x,
                      T.transform.rotation.y,
                      T.transform.rotation.z,
                      T.transform.rotation.w);
    tf2::Matrix3x3 R(q);

    tf2::Vector3 v(p_src.x, p_src.y, p_src.z);
    tf2::Vector3 vr = R * v;

    geometry_msgs::msg::Point p_out;
    p_out.x = vr.x() + T.transform.translation.x;
    p_out.y = vr.y() + T.transform.translation.y;
    p_out.z = vr.z() + T.transform.translation.z;
    return p_out;
  }

  void onPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    std::scoped_lock<std::mutex> lk(mutex_);

    geometry_msgs::msg::PointStamped pin = *msg;
    if (pin.header.frame_id.empty()) pin.header.frame_id = base_frame_;

    geometry_msgs::msg::TransformStamped T_base_src;
    try {
      T_base_src = tf_buffer_.lookupTransform(
          base_frame_, pin.header.frame_id, pin.header.stamp, tf2::durationFromSec(0.5));
    } catch (const std::exception& e) {
      RCLCPP_WARN(node_->get_logger(), "TF lookup failed %s -> %s: %s",
                  pin.header.frame_id.c_str(), base_frame_.c_str(), e.what());
      return;
    }

    geometry_msgs::msg::Point p_in_base = applyTransform(T_base_src, pin.point);

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = p_in_base.x;
    target_pose.position.y = p_in_base.y;
    target_pose.position.z = p_in_base.z + above_offset_;

    if (use_current_orientation_) {
      auto cur = move_group_->getCurrentPose(actuator_link_);
      target_pose.orientation = cur.pose.orientation;
    } else {
      tf2::Quaternion q;
      double rr = rpy_.size() > 0 ? rpy_[0] : M_PI;
      double rp = rpy_.size() > 1 ? rpy_[1] : 0.0;
      double ry = rpy_.size() > 2 ? rpy_[2] : 0.0;
      q.setRPY(rr, rp, ry);
      geometry_msgs::msg::Quaternion qmsg;
      qmsg.x = q.x(); qmsg.y = q.y(); qmsg.z = q.z(); qmsg.w = q.w();
      target_pose.orientation = qmsg;
    }

    geometry_msgs::msg::PoseStamped target_pose_st;
    target_pose_st.header.frame_id = base_frame_;
    target_pose_st.header.stamp = node_->now();
    target_pose_st.pose = target_pose;

    RCLCPP_INFO(node_->get_logger(),
      "Clicked @ [%.3f, %.3f, %.3f] (%s) -> goal [%.3f, %.3f, %.3f] (%s), offset=%.3f",
      pin.point.x, pin.point.y, pin.point.z, pin.header.frame_id.c_str(),
      target_pose.position.x, target_pose.position.y, target_pose.position.z, base_frame_.c_str(),
      above_offset_);

    move_group_->setStartStateToCurrentState();
    move_group_->setPoseReferenceFrame(base_frame_);
    move_group_->setPoseTarget(target_pose_st, actuator_link_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_->plan(plan);

    if (result) {
      RCLCPP_INFO(node_->get_logger(), "Plan SUCCESS (%.3fs, %zu pts).",
                  plan.planning_time_, plan.trajectory_.joint_trajectory.points.size());
      if (!plan_only_) {
        auto exe = move_group_->execute(plan);
        if (exe) {
          RCLCPP_INFO(node_->get_logger(), "Execute SUCCESS.");
        } else {
          RCLCPP_WARN(node_->get_logger(), "Execute FAILED. (Check controllers/collisions)");
        }
      }
    } else {
      RCLCPP_WARN(node_->get_logger(), "Plan FAILED.");
    }

    move_group_->clearPoseTargets();
  }

  rclcpp::Node::SharedPtr node_;
  std::mutex mutex_;
  std::string planning_group_, base_frame_, actuator_link_, clicked_topic_;
  double above_offset_{0.15};
  bool plan_only_{false}, use_current_orientation_{true};
  double vel_scale_{0.3}, acc_scale_{0.3};
  std::vector<double> rpy_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("click_to_move");
  // 业务对象拿到已共享的 node，内部再去创建 MoveGroupInterface
  auto app = std::make_shared<ClickToMove>(node);
  (void)app; // 防止未使用警告
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
