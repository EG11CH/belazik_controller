#include "rack_pinion_controller/ackermann_steering_controller.hpp"
#include "visualization_msgs/msg/marker.hpp" 
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/joint_state.hpp> 
#include <geometry_msgs/msg/transform_stamped.hpp> 

namespace ackermann_steering_controller {

AckermannSteeringController::AckermannSteeringController() 
: steering_controllers_library::SteeringControllersLibrary() {}

void AckermannSteeringController::initialize_implementation_parameter_listener() {
  auto node = this->get_node();
  ackermann_param_listener_ = std::make_shared<ackermann_steering_controller::ParamListener>(node);
  
  node->declare_parameter<double>("radius_rotation", 1.0);

  marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("controller_markers", rclcpp::SystemDefaultsQoS());
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  joint_state_publisher_ = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
}

controller_interface::CallbackReturn AckermannSteeringController::configure_odometry() {
  ackermann_params_ = ackermann_param_listener_->get_params();
  const double front_wheels_radius = ackermann_params_.front_wheels_radius;
  const double rear_wheels_radius = ackermann_params_.rear_wheels_radius;
  const double front_wheel_track = ackermann_params_.front_wheel_track;
  const double rear_wheel_track = ackermann_params_.rear_wheel_track;
  const double wheelbase = ackermann_params_.wheelbase;

  if (params_.front_steering) {
    odometry_.set_wheel_params(rear_wheels_radius, wheelbase, rear_wheel_track);
  } else {
    odometry_.set_wheel_params(front_wheels_radius, wheelbase, front_wheel_track);
  }

  odometry_.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);
  set_interface_numbers(NR_STATE_ITFS, NR_CMD_ITFS, NR_REF_ITFS);
  
  auto node = this->get_node();
  RCLCPP_INFO(node->get_logger(), "Ackermann odometry configuration successful");

  return controller_interface::CallbackReturn::SUCCESS;
}

bool AckermannSteeringController::update_odometry(const rclcpp::Duration & period) {
  auto node = this->get_node();
  double radius_rotation = node->get_parameter("radius_rotation").as_double();

  if (params_.open_loop) {
    odometry_.update_open_loop(last_linear_velocity_, last_angular_velocity_, period.seconds());
  } else {
    const double traction_right_wheel_value = state_interfaces_[STATE_TRACTION_RIGHT_WHEEL].get_value();
    const double traction_left_wheel_value = state_interfaces_[STATE_TRACTION_LEFT_WHEEL].get_value();
    const double steering_right_position = state_interfaces_[STATE_STEER_RIGHT_WHEEL].get_value();
    const double steering_left_position = state_interfaces_[STATE_STEER_LEFT_WHEEL].get_value();

    if (std::isfinite(traction_right_wheel_value) && std::isfinite(traction_left_wheel_value) && 
        std::isfinite(steering_right_position) && std::isfinite(steering_left_position)) {

      double rack_position = this->calculate_rack_position(steering_right_position, steering_left_position, radius_rotation);

      if (params_.position_feedback) {
        odometry_.update_from_position(
          traction_right_wheel_value, traction_left_wheel_value, rack_position, period.seconds());
      } else {
        odometry_.update_from_velocity(
          traction_right_wheel_value, traction_left_wheel_value, rack_position, period.seconds());
      }

      sensor_msgs::msg::JointState joint_state_msg;
      joint_state_msg.header.stamp = node->get_clock()->now();
      joint_state_msg.name = {"rack_pinion_joint", "wheel_front_left", "wheel_front_right", "wheel_rear_left", "wheel_rear_right"};
      joint_state_msg.position = {rack_position, steering_left_position, steering_right_position, traction_left_wheel_value, traction_right_wheel_value};
      joint_state_publisher_->publish(joint_state_msg);

      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header.stamp = node->get_clock()->now();
      transform_stamped.header.frame_id = "odom";
      transform_stamped.child_frame_id = "base_link";
      transform_stamped.transform.translation.x = odometry_.get_x();
      transform_stamped.transform.translation.y = odometry_.get_y();
      transform_stamped.transform.translation.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, odometry_.get_heading());
      transform_stamped.transform.rotation.x = q.x();
      transform_stamped.transform.rotation.y = q.y();
      transform_stamped.transform.rotation.z = q.z();
      transform_stamped.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(transform_stamped);

      publishMarker(rack_position);
    }
  }
  return true;
}

// Функция для вычисления позиции рейки
double AckermannSteeringController::calculate_rack_position(double right_pos, double left_pos, double radius_rotation) {
  return 0.0;
}

void AckermannSteeringController::publishMarker(double rack_position) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = this->get_node()->get_clock()->now();
  marker.ns = "ackermann_controller";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = rack_position; 
  marker.pose.position.y = 0.0; 
  marker.pose.position.z = 0.5; 
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.lifetime = rclcpp::Duration(1s); 

  marker_pub_->publish(marker); 
}

}  // namespace ackermann_steering_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ackermann_steering_controller::AckermannSteeringController, controller_interface::ChainableControllerInterface)
