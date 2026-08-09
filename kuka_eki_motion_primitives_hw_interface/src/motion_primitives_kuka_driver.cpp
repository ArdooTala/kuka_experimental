// Copyright (c) 2025, b»robotized
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Mathias Fuhrer

#include <limits>
#include <vector>

#include "kuka_eki_motion_primitives_hw_interface/motion_primitives_kuka_driver.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace kuka_eki_motion_primitives_hw_interface
{
MotionPrimitivesKukaDriver::~MotionPrimitivesKukaDriver()
{
  if (async_execute_motion_thread_ && async_execute_motion_thread_->joinable()) 
  {
    async_execute_motion_thread_->join();
    async_execute_motion_thread_.reset();
  }
}
hardware_interface::CallbackReturn MotionPrimitivesKukaDriver::on_init(
  const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Initializing Hardware Interface");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Failed to initialize SystemInterface");
    return CallbackReturn::ERROR;
  }

  info_ = info;

  async_thread_shutdown_ = false;

  // Joint states for RViz, ...
  hw_joint_pos_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_joint_vel_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_joint_eff_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // State interfaces for the motion_primitive_forward_controller
  hw_mo_prim_states_.resize(2, std::numeric_limits<double>::quiet_NaN());     // execution_status, ready_for_new_primitive
  
  // Command Interfaces
  hw_mo_prim_commands_motion_.resize(5, std::numeric_limits<double>::quiet_NaN());  // motion_type + blend_radius + velocity + acceleration + move_time
  hw_mo_prim_commands_joints_.resize(6, std::numeric_limits<double>::quiet_NaN());  // 6 joints
  hw_mo_prim_commands_pos_.resize(7, std::numeric_limits<double>::quiet_NaN());     // 7 positions
  hw_mo_prim_commands_via_pos_.resize(7, std::numeric_limits<double>::quiet_NaN()); // 7 via positions

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotionPrimitivesKukaDriver::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Configuring Hardware Interface ...");
  async_execute_motion_thread_ = std::make_unique<std::thread>(&MotionPrimitivesKukaDriver::asyncExecuteMotionThread, this);
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MotionPrimitivesKukaDriver::export_state_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Exporting State Interfaces ...");

  std::vector<hardware_interface::StateInterface> state_interfaces;
  // State interfaces with joint states for RViz, ...
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joint_pos_states_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_vel_states_[i]));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_joint_eff_states_[i]));
  }

  // State interfaces for the motion_primitive_forward_controller
  state_interfaces.emplace_back(hardware_interface::StateInterface("motion_primitive", "execution_status", &hw_mo_prim_states_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("motion_primitive", "ready_for_new_primitive", &hw_mo_prim_states_[1]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MotionPrimitivesKukaDriver::export_command_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Exporting Command Interfaces");

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Joint position commands (q1, q2, ..., q6)
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "q1", &hw_mo_prim_commands_joints_[0]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "q2", &hw_mo_prim_commands_joints_[1]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "q3", &hw_mo_prim_commands_joints_[2]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "q4", &hw_mo_prim_commands_joints_[3]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "q5", &hw_mo_prim_commands_joints_[4]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "q6", &hw_mo_prim_commands_joints_[5]));
  // Position commands (pos_x, pos_y, pos_z, pos_qx, pos_qy, pos_qz, pos_qz)
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_x", &hw_mo_prim_commands_pos_[0]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_y", &hw_mo_prim_commands_pos_[1]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_z", &hw_mo_prim_commands_pos_[2]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_qx", &hw_mo_prim_commands_pos_[3]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_qy", &hw_mo_prim_commands_pos_[4]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_qz", &hw_mo_prim_commands_pos_[5]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_qw", &hw_mo_prim_commands_pos_[6]));
  // Via Position commands for circula motion
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_via_x", &hw_mo_prim_commands_via_pos_[0]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_via_y", &hw_mo_prim_commands_via_pos_[1]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_via_z", &hw_mo_prim_commands_via_pos_[2]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_via_qx", &hw_mo_prim_commands_via_pos_[3]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_via_qy", &hw_mo_prim_commands_via_pos_[4]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_via_qz", &hw_mo_prim_commands_via_pos_[5]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "pos_via_qw", &hw_mo_prim_commands_via_pos_[6]));
  // Command for motion type (motion_type)
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "motion_type", &hw_mo_prim_commands_motion_[0]));
  // Other command parameters (blend_radius, velocity, acceleration, move_time)
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "blend_radius", &hw_mo_prim_commands_motion_[1]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "velocity", &hw_mo_prim_commands_motion_[2]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "acceleration", &hw_mo_prim_commands_motion_[3]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("motion_primitive", "move_time", &hw_mo_prim_commands_motion_[4]));

  return command_interfaces;
}

hardware_interface::CallbackReturn MotionPrimitivesKukaDriver::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Activating Hardware Interface");
  ready_for_new_primitive_ = true; // set to true to allow sending new commands

  robot_ip_ = info_.hardware_parameters["robot_ip"];
  eki_robot_port_ = std::stoi(info_.hardware_parameters["eki_robot_port"]);
  eki_robot_meta_port_ = std::stoi(info_.hardware_parameters["eki_robot_meta_port"]);
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Trying to connect to the host: [%s], port: [%d], meta_port: [%d]", robot_ip_.c_str(), eki_robot_port_, eki_robot_meta_port_);
  if (robot_ip_.empty() || eki_robot_port_ == 0)
  {
      RCLCPP_ERROR(
          rclcpp::get_logger("MotionPrimitivesKukaDriver"), "robot_ip and eki_robot_port cannot be empty");
      return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Connecting to the robot ...");
  robot_.connect_async(robot_ip_, eki_robot_port_, eki_robot_meta_port_);
  robot_.await_connection();
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Connected to the robot.");
    RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "System Successfully activated!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotionPrimitivesKukaDriver::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Deactivating Hardware Interface ...");
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Abort all commands ...");
  robot_.abort_commands();
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Disconnecting from the robot ...");
  robot_.disconnect();
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "System Successfully deactivated!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MotionPrimitivesKukaDriver::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  rbt::RobotState robot_state = robot_.get_state();
  const rbt::PoseJoints& joints = robot_state.position_joints;
  constexpr double deg_to_rad = M_PI / 180.0;
  hw_joint_pos_states_[0] = joints.a1 * deg_to_rad;
  hw_joint_pos_states_[1] = joints.a2 * deg_to_rad;
  hw_joint_pos_states_[2] = joints.a3 * deg_to_rad;
  hw_joint_pos_states_[3] = joints.a4 * deg_to_rad;
  hw_joint_pos_states_[4] = joints.a5 * deg_to_rad;
  hw_joint_pos_states_[5] = joints.a6 * deg_to_rad;

  const rbt::PoseJoints& velocity = robot_state.velocity;
  hw_joint_vel_states_[0] = velocity.a1;
  hw_joint_vel_states_[1] = velocity.a2;
  hw_joint_vel_states_[2] = velocity.a3;
  hw_joint_vel_states_[3] = velocity.a4;
  hw_joint_vel_states_[4] = velocity.a5;
  hw_joint_vel_states_[5] = velocity.a6;

  const rbt::PoseJoints& torque = robot_state.torque;
  hw_joint_eff_states_[0] = torque.a1;
  hw_joint_eff_states_[1] = torque.a2;
  hw_joint_eff_states_[2] = torque.a3;
  hw_joint_eff_states_[3] = torque.a4;
  hw_joint_eff_states_[4] = torque.a5;
  hw_joint_eff_states_[5] = torque.a6;
  
  if(!checkCommandIdDoneQueue.empty() && checkCommandIdDoneQueue.front() == robot_.last_finished_command_id()) // Motion Primitive or Sequence done
  {
    RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Robot finished command with ID: %d", robot_.last_finished_command_id());
    checkCommandIdDoneQueue.pop();
    current_execution_status_ = MoprimExecutionState::SUCCESS;
  } 
  else if (robot_error_) 
  {
    current_execution_status_ = MoprimExecutionState::ERROR;
  } 
  else if(robot_stopped_) 
  {
    current_execution_status_ = MoprimExecutionState::STOPPED;
  } 
  else if (robot_.robot_in_movement()) 
  {
    current_execution_status_ = MoprimExecutionState::EXECUTING;
  } 
  else 
  {
    current_execution_status_ = MoprimExecutionState::IDLE;
  }

  hw_mo_prim_states_[0] = static_cast<uint8_t>(current_execution_status_);
  hw_mo_prim_states_[1] = static_cast<double>(ready_for_new_primitive_);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MotionPrimitivesKukaDriver::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Check if we have a new command
  if (!std::isnan(hw_mo_prim_commands_motion_[0])) {
    ready_for_new_primitive_ = false; // set to false to indicate that the driver is busy handeling a command
    double motion_type = hw_mo_prim_commands_motion_[0];
    switch (static_cast<uint8_t>(motion_type))
    {
      case static_cast<uint8_t>(MoprimMotionHelperType::STOP_MOTION): {
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "STOP_MOTION command received");
        std::lock_guard<std::mutex> guard(stop_mutex_);
        if (!new_stop_available_) {
          new_stop_available_ = true;
          reset_command_interfaces();
        }
        break;
      }
      case static_cast<uint8_t>(MoprimMotionHelperType::RESET_STOP): {
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "RESET_STOP command received");
        std::lock_guard<std::mutex> guard(stop_mutex_);
        if (!new_reset_available_) {
          new_reset_available_ = true;
          reset_command_interfaces();
        }
        break;
      }
      case static_cast<uint8_t>(MoprimMotionHelperType::MOTION_SEQUENCE_START): {
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Received MOTION_SEQUENCE_START: add all following commands to the motion sequence.");
        build_motion_sequence_ = true;  // set flag to put all following commands into the motion sequence
        reset_command_interfaces();
        ready_for_new_primitive_ = true; // set to true to allow sending new commands
        break;
      }
      case static_cast<uint8_t>(MoprimMotionHelperType::MOTION_SEQUENCE_END): {
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Received MOTION_SEQUENCE_END: executing motion sequence ...");
        build_motion_sequence_ = false;
        std::lock_guard<std::mutex> guard(execution_mutex_);
        if (!new_execution_available_) {
          new_execution_available_ = true;  // set flag for async thread to send command to robot
        }
        reset_command_interfaces();
        break;
      }
      case MoprimMotionType::LINEAR_JOINT: { // MoveJ/ PTP
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "LINEAR_JOINT command received");
        if(!add_linear_joint_cmd()) {
          RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Failed to add LINEAR_JOINT command");
          robot_error_ = true;
          return hardware_interface::return_type::ERROR;
        }
        reset_command_interfaces();
        if(!build_motion_sequence_) { // send single command imimediately
          std::lock_guard<std::mutex> guard(execution_mutex_);
          if (!new_execution_available_) {
            new_execution_available_ = true;
          }
        } else {
          ready_for_new_primitive_ = true;
        }
        break;
      }
      case MoprimMotionType::LINEAR_CARTESIAN: { // MoveL/ LIN
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "LINEAR_CARTESIAN command received");
        if(!add_linear_cartesian_cmd()) {
          RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Failed to add LINEAR_CARTESIAN command");
          robot_error_ = true;
          return hardware_interface::return_type::ERROR;
        }
        reset_command_interfaces();
        if(!build_motion_sequence_) {
          std::lock_guard<std::mutex> guard(execution_mutex_);
          if (!new_execution_available_) {
            new_execution_available_ = true;
          }
        } else {
          ready_for_new_primitive_ = true;
        }
        break;
      }
      case MoprimMotionType::CIRCULAR_CARTESIAN: {  // MoveC/ CIRC
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "CIRCULAR_CARTESIAN command received");
        if(!add_circular_cartesian_cmd()) {
          RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Failed to add CIRCULAR_CARTESIAN command");
          robot_error_ = true;
          return hardware_interface::return_type::ERROR;
        }
        reset_command_interfaces();
        if(!build_motion_sequence_) {
          std::lock_guard<std::mutex> guard(execution_mutex_);
          if (!new_execution_available_) {
            new_execution_available_ = true;
          }
        } else {
          ready_for_new_primitive_ = true;
        }
        break;
      }
      default: {
        RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Invalid motion command: motion type %f is not supported", motion_type);
        robot_error_ = true;
        return hardware_interface::return_type::ERROR;
      }
    }
  } 
  return hardware_interface::return_type::OK;
}

bool MotionPrimitivesKukaDriver::add_linear_joint_cmd()
{
  // Check if joint positions are valid
  for (int i = 0; i < 6; ++i) {
    if (std::isnan(hw_mo_prim_commands_joints_[i])) {
        RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Invalid motion command: joint positions contain NaN values");
        return false;
    }
  }
  constexpr double rad_to_deg = 180.0 / M_PI;
  std::vector<double> joints = {     // get joint positions in degrees
      hw_mo_prim_commands_joints_[0] * rad_to_deg,
      hw_mo_prim_commands_joints_[1] * rad_to_deg,
      hw_mo_prim_commands_joints_[2] * rad_to_deg,
      hw_mo_prim_commands_joints_[3] * rad_to_deg,
      hw_mo_prim_commands_joints_[4] * rad_to_deg,
      hw_mo_prim_commands_joints_[5] * rad_to_deg};
  rbt::MoveCommand command;
  command = rbt::MoveCommand(rbt::PoseJoints(joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]));
  add_vel_and_acc_to_command(command);
  add_blending_to_command(command);
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), 
        "Added LINEAR_JOINT with joint positions: [%f, %f, %f, %f, %f, %f]"
        ", velocity: %f, acceleration: %f, blending: %f",
        joints[0], joints[1], joints[2], joints[3], joints[4], joints[5],
        command.velocity, command.acceleration, command.blending);
  robot_.perform(command);
  return true;
}

bool MotionPrimitivesKukaDriver::add_linear_cartesian_cmd()
{
  // Check if pose values (position and quaternion) are valid
  for (int i = 0; i < 7; ++i) {
    if (std::isnan(hw_mo_prim_commands_pos_[i])) {
        RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Invalid motion command: pose contains NaN values");
        return false;
    }
  }
  double a, b, c;
  quaternionToKukaABC(hw_mo_prim_commands_pos_[3], hw_mo_prim_commands_pos_[4], hw_mo_prim_commands_pos_[5], hw_mo_prim_commands_pos_[6], a, b, c);


  std::vector<double> pose = {
    hw_mo_prim_commands_pos_[0] * 1000.0, // from m to mm
    hw_mo_prim_commands_pos_[1] * 1000.0,
    hw_mo_prim_commands_pos_[2] * 1000.0,
    a, b, c};

  rbt::MoveCommand command;
  command = rbt::MoveCommand(rbt::PoseCartesian(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]), true);
  add_vel_and_acc_to_command(command);
  add_blending_to_command(command);
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), 
        "Adding LINEAR_CARTESIAN with pose: [%f, %f, %f, %f, %f, %f]"
        ", velocity: %f, acceleration: %f, blending: %f",
        pose[0], pose[1], pose[2], pose[3], pose[4], pose[5],
        command.velocity, command.acceleration, command.blending);
  robot_.perform(command);
  return true;
}

bool MotionPrimitivesKukaDriver::add_circular_cartesian_cmd()
{
  // Check if pose values (position and quaternion) are valid
  for (int i = 0; i < 7; ++i) {
    if (std::isnan(hw_mo_prim_commands_pos_[i])) {
        RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Invalid motion command: pose contains NaN values");
        return false;
    }
  }
  // Check if via pose values (position and quaternion) are valid
  for (int i = 0; i < 7; ++i) {
    if (std::isnan(hw_mo_prim_commands_via_pos_[i])) {
        RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Invalid motion command: via pose contains NaN values");
        return false;
    }
  }
  double goal_a, goal_b, goal_c;
  quaternionToKukaABC(hw_mo_prim_commands_pos_[3], hw_mo_prim_commands_pos_[4], hw_mo_prim_commands_pos_[5], hw_mo_prim_commands_pos_[6], goal_a, goal_b, goal_c);
  
  double via_a, via_b, via_c;
  quaternionToKukaABC(hw_mo_prim_commands_via_pos_[3], hw_mo_prim_commands_via_pos_[4], hw_mo_prim_commands_via_pos_[5], hw_mo_prim_commands_via_pos_[6], via_a, via_b, via_c);

  std::vector<double> goal_pose = {
    hw_mo_prim_commands_pos_[0] * 1000.0, // from m to mm
    hw_mo_prim_commands_pos_[1] * 1000.0,
    hw_mo_prim_commands_pos_[2] * 1000.0,
    goal_a, goal_b, goal_c};
  std::vector<double> via_pose = {
    hw_mo_prim_commands_via_pos_[0] * 1000.0, // from m to mm
    hw_mo_prim_commands_via_pos_[1] * 1000.0,
    hw_mo_prim_commands_via_pos_[2] * 1000.0,
    via_a, via_b, via_c};

  rbt::MoveCommand command;
  command = rbt::MoveCommand(rbt::PoseCartesian(via_pose[0], via_pose[1], via_pose[2], via_pose[3], via_pose[4], via_pose[5]),
                             rbt::PoseCartesian(goal_pose[0], goal_pose[1], goal_pose[2], goal_pose[3], goal_pose[4], goal_pose[5]));
  add_vel_and_acc_to_command(command);
  add_blending_to_command(command);
  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), 
        "Adding CIRCULAR_CARTESIAN with goal_pose: [%f, %f, %f, %f, %f, %f] and via_pose: [%f, %f, %f, %f, %f, %f]"
        ", velocity: %f, acceleration: %f, blending: %f",
        goal_pose[0], goal_pose[1], goal_pose[2], goal_pose[3], goal_pose[4], goal_pose[5],
        via_pose[0], via_pose[1], via_pose[2], via_pose[3], via_pose[4], via_pose[5],
        command.velocity, command.acceleration, command.blending);
  robot_.perform(command);

  return true;
}

void MotionPrimitivesKukaDriver::add_vel_and_acc_to_command(rbt::MoveCommand &command)
{
  if (std::isnan(hw_mo_prim_commands_motion_[2])) {
    command.velocity = 0.0;
  } else {
    command.velocity = hw_mo_prim_commands_motion_[2];
  }
  
  if (std::isnan(hw_mo_prim_commands_motion_[3])) {
    command.acceleration = 0.0;
  } else {
    command.acceleration = hw_mo_prim_commands_motion_[3];
  }
}

void MotionPrimitivesKukaDriver::add_blending_to_command(rbt::MoveCommand &command)
{
  // (Blending only alowed in sequence)
  if (std::isnan(hw_mo_prim_commands_motion_[1]) || !build_motion_sequence_) {
    command.blending = 0.0;
  } else {
    command.blending = hw_mo_prim_commands_motion_[1];
  }
}

void MotionPrimitivesKukaDriver::reset_command_interfaces()
{
  std::fill(hw_mo_prim_commands_motion_.begin(), hw_mo_prim_commands_motion_.end(), std::numeric_limits<double>::quiet_NaN());
  std::fill(hw_mo_prim_commands_joints_.begin(), hw_mo_prim_commands_joints_.end(), std::numeric_limits<double>::quiet_NaN());
  std::fill(hw_mo_prim_commands_pos_.begin(), hw_mo_prim_commands_pos_.end(), std::numeric_limits<double>::quiet_NaN());
  std::fill(hw_mo_prim_commands_via_pos_.begin(), hw_mo_prim_commands_via_pos_.end(), std::numeric_limits<double>::quiet_NaN());
}

void MotionPrimitivesKukaDriver::asyncExecuteMotionThread()
{
  const auto TIMEOUT_DURATION = std::chrono::seconds(5);
  std::chrono::time_point start_time = std::chrono::steady_clock::now();
  bool pending_execution = false;
  while (!async_thread_shutdown_) 
  {
    if (new_stop_available_) {
      robot_.abort_commands();
      std::lock_guard<std::mutex> guard(stop_mutex_);
      robot_stopped_ = true;
      pending_execution = false;
      while(!checkCommandIdDoneQueue.empty()){
        // Remove all command IDs from the queue --> dont wait for them to get finished since they are discarded
        checkCommandIdDoneQueue.pop();    
      }
      if (!robot_.robot_stopped()) {
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Waiting for Robot to stop ...");
        continue;
      }
      RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Robot stopped");
      new_stop_available_ = false;
      continue;
    } else if (new_reset_available_) {
      robot_.reset_abort_commands();
      if (robot_.robot_stopped()) {
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Waiting for Robot to reset stop ...");
        continue;
      }
      RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Robot reset stop done");
      std::lock_guard<std::mutex> guard(stop_mutex_);
      robot_stopped_ = false;
      new_reset_available_ = false;
      continue;
    }
    if (robot_stopped_)
        continue;
    if (pending_execution) {
      if (robot_.run())
      {
        RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Sent command to robot successfully.");
        pending_execution = false;
        continue;
      }
      // Check if the timeout has been reached
      if (std::chrono::steady_clock::now() - start_time > TIMEOUT_DURATION) 
      {
        RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Timeout reached: Failed to send command to robot.");
        return;
      }
      continue;
    }
    if (new_execution_available_) {
      std::lock_guard<std::mutex> guard(execution_mutex_);
      new_execution_available_ = false;
      RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Sending command to robot ...");
      start_time = std::chrono::steady_clock::now();
      pending_execution = true;
      // Store last command ID to check if it was executed
      RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Last command ID: %d", robot_.last_command_id_of_sequence());
      checkCommandIdDoneQueue.push(robot_.last_command_id_of_sequence());
      continue;
    }

    ready_for_new_primitive_ = true;

    // Small sleep to prevent busy waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  RCLCPP_INFO(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "[asyncExecuteMotionThread] Exiting");
}


void MotionPrimitivesKukaDriver::quaternionToKukaABC(double qx, double qy, double qz, double qw,
                                                     double& A, double& B, double& C)
{
  double norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
  if (std::abs(norm - 1.0) > 0.001) {
    RCLCPP_ERROR(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Quaternion is not normalized! Norm is %f (expected ~1.0)", norm);
  }
  // Calculation according to https://doc.rc-visard.com/v21.07/en/pose_format_kuka.html
  const double rad2deg = 180.0 / M_PI;
  A = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)) * rad2deg;
  B = std::asin (2.0 * (qw * qy - qz * qx)) * rad2deg;
  C = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)) * rad2deg;

  RCLCPP_DEBUG(rclcpp::get_logger("MotionPrimitivesKukaDriver"), "Converted quaternion [%f, %f, %f, %f] to Kuka ABC angles: A=%f, B=%f, C=%f",
              qx, qy, qz, qw, A, B, C);
}

}  // namespace kuka_eki_motion_primitives_hw_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kuka_eki_motion_primitives_hw_interface::MotionPrimitivesKukaDriver, hardware_interface::SystemInterface)
