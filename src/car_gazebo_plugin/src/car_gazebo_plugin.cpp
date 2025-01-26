#include "car_gazebo_plugin.hpp"

#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <iostream>
#include <gazebo/common/Time.hh>

namespace car_gazebo_plugin {

CarGazeboPlugin::CarGazeboPlugin()
    : robot_namespace_{""},
      last_sim_time_{0},
      last_update_time_{0},
      update_period_ms_{8} {}

void CarGazeboPlugin::Load(gazebo::physics::ModelPtr model,
                           sdf::ElementPtr sdf) {
  // Get model and world references
  model_ = model;
  world_ = model_->GetWorld();
  auto physicsEngine = world_->Physics();
  physicsEngine->SetParam("friction_model", std::string{"cone_model"});

  if (sdf->HasElement("robotNamespace")) {
    robot_namespace_ =  sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  // Set up ROS node and subscribers and publishers
  ros_node_ = gazebo_ros::Node::Get(sdf);
  RCLCPP_INFO(ros_node_->get_logger(), "Loading Car Gazebo Plugin");

  joint_state_pub_ = ros_node_->create_publisher<JointState>("/joint_states", rclcpp::SensorDataQoS());

  // Find joints
  auto allJoints = model_->GetJoints();
  for (auto const& j : allJoints) {
    if (j->GetType() == gazebo::physics::Joint::FIXED_JOINT) {
      continue;
    }

    auto pid = gazebo::common::PID{};
    pid.SetPGain(200.0);
    pid.SetIGain(0.0);
    pid.SetDGain(0.0);

    auto const& name = j->GetName();
    joints_[name] = std::make_pair(j, pid);
    joint_targets_[name] = 0.0;
  }

  RCLCPP_DEBUG(ros_node_->get_logger(), "Got joints:");
  for (auto const& j : joints_) {
    RCLCPP_DEBUG(ros_node_->get_logger(), j.first.c_str());
  }

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*ros_node_);

  // fake_car code here
  {
    RCLCPP_DEBUG(ros_node_->get_logger(), "Connected to model %s", model_->GetName().c_str());

    jc = model_->GetJointController();

    // front left str
    fl_pid = gazebo::common::PID(0.5, 0, 0);
    fl_str_joint = get_joint("front_left_wheel_steer_joint");

    jc->SetPositionPID(fl_str_joint->GetScopedName(), fl_pid);

    // front right str
    fr_pid = gazebo::common::PID(0.5, 0, 0);
    fr_str_joint = get_joint("front_right_wheel_steer_joint");
    jc->SetPositionPID(fr_str_joint->GetScopedName(), fr_pid);

    fl_axle_joint = get_joint("front_left_wheel_joint");
    fr_axle_joint = get_joint("front_right_wheel_joint");

    // back left speed
    bl_pid = gazebo::common::PID(0.1, 0.01, 0.0);
    bl_axle_joint = get_joint("back_left_wheel_joint");
    jc->SetVelocityPID(bl_axle_joint->GetScopedName(), bl_pid);

    br_pid = gazebo::common::PID(0.1, 0.01, 0.0);
    br_axle_joint = get_joint("back_right_wheel_joint");
    jc->SetVelocityPID(br_axle_joint->GetScopedName(), br_pid);

    // publish
    odo_fl_pub = ros_node_->create_publisher<std_msgs::msg::Int32>("/" + model_->GetName() + "/odo_fl", 10);
    status_pub = ros_node_->create_publisher<smartcar_msgs::msg::Status>("/" + model_->GetName() + "/vehicle_status", 10);
    odo_fr_pub = ros_node_->create_publisher<std_msgs::msg::Int32>("/" + model_->GetName() + "/odo_fr", 10);
    ackermann_pub = ros_node_->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/" + model_->GetName() + "/cmd_ackermann", 10);

    pose_pub = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>("/" + model_->GetName() + "/pose", 10);
    odom_pub = ros_node_->create_publisher<nav_msgs::msg::Odometry>("/" + model_->GetName() + "/odom", 10);

    ackermann_sub = ros_node_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "/" + model_->GetName() + "/cmd_ackermann", 2, std::bind(&CarGazeboPlugin::ackermann_callback, this, std::placeholders::_1));

    cmd_vel_sub = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 2, std::bind(&CarGazeboPlugin::twist_callback, this, std::placeholders::_1));
  }

  // Hook into simulation update loop
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&CarGazeboPlugin::Update, this));
}

void CarGazeboPlugin::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Extract linear and angular velocities from the Twist message
    double linear_velocity = msg->linear.x;  // Forward/backward speed
    double angular_velocity = msg->angular.z; // Rotation speed (steering)

    // Apply linear velocity to wheel joints (forward/backward movement)
    jc->SetVelocityTarget(fl_axle_joint->GetScopedName(), linear_velocity);  // Left wheel
    jc->SetVelocityTarget(fr_axle_joint->GetScopedName(), linear_velocity);  // Right wheel

    // Apply angular velocity to steering joints (turning)
    jc->SetPositionTarget(fl_str_joint->GetScopedName(), angular_velocity);  // Left steering
    jc->SetPositionTarget(fr_str_joint->GetScopedName(), angular_velocity);  // Right steering

    RCLCPP_INFO(ros_node_->get_logger(), "Linear velocity: %f, Angular velocity: %f", linear_velocity, angular_velocity);
}

void CarGazeboPlugin::Update() {
  auto cur_time = world_->SimTime();
  if (last_sim_time_ == 0) {
    last_sim_time_ = cur_time;
    last_update_time_ = cur_time;
    return;
  }

  // publish to ros every update_period_ms
  auto update_dt = (cur_time - last_update_time_).Double();
  if (update_dt * 1000 >= update_period_ms_) {

    auto pose = model_->WorldPose();
    // RCLCPP_INFO_STREAM( ros_node_->get_logger(), "pose.x" << pose.X() << "pose.y" << pose.Y() << "pose.x" << pose.Z());

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = ros_node_->now();
    pose_msg.header.frame_id = "odom";
    pose_msg.pose.position.x = pose.X();
    pose_msg.pose.position.y = pose.Y();
    pose_msg.pose.position.z = pose.Z();
    pose_msg.pose.orientation.x = pose.Rot().X();
    pose_msg.pose.orientation.y = pose.Rot().Y();
    pose_msg.pose.orientation.z = pose.Rot().Z();
    pose_msg.pose.orientation.w = pose.Rot().W();
    pose_pub->publish(pose_msg);

    nav_msgs::msg::Odometry odom_msg;
    auto linear_vel = model_->WorldLinearVel();
    auto angular_vel = model_->WorldAngularVel();
    odom_msg.header.stamp = ros_node_->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";
    odom_msg.pose.pose.position.x = pose.X();
    odom_msg.pose.pose.position.y = pose.Y();
    odom_msg.pose.pose.position.z = pose.Z();
    odom_msg.pose.pose.orientation.x = pose.Rot().X();
    odom_msg.pose.pose.orientation.y = pose.Rot().Y();
    odom_msg.pose.pose.orientation.z = pose.Rot().Z();
    odom_msg.pose.pose.orientation.w = pose.Rot().W();

    // Get velocity in odom frame
    auto linear = model_->WorldLinearVel();
    odom_msg.twist.twist.angular.z = model_->WorldAngularVel().Z();

    // Convert velocity to child_frame_id(aka base_footprint)
    auto yaw = static_cast<float>(pose.Rot().Yaw());
    odom_msg.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
    odom_msg.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
    odom_pub->publish(odom_msg);

    // Publish joint states
    auto msg = JointState{};
    msg.header.stamp.sec = cur_time.sec;
    msg.header.stamp.nanosec = cur_time.nsec;

    for (auto& j : joints_) {
      auto const& name = j.first;
      auto joint = j.second.first;
      auto const& joint_pos = joint->Position(0);
      auto joint_vel = joint->GetVelocity(0);
      msg.name.push_back(name);
      msg.position.push_back(joint_pos);
      msg.velocity.push_back(joint_vel);
    }

    joint_state_pub_->publish(msg);

    // Update last time
    last_update_time_ = cur_time;
  }
}

}  // namespace car_gazebo_plugin

