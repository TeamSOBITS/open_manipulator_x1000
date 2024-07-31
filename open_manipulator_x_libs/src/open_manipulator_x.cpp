// Copyright 2019 ROBOTIS CO., LTD.
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
// Author: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na

#include "../include/open_manipulator_x_libs/open_manipulator_x.hpp"

OpenManipulatorX::OpenManipulatorX() {}

OpenManipulatorX::~OpenManipulatorX()
{
  delete kinematics_;
  delete actuator_;
  delete tool_;
  for(uint8_t index = 0; index < CUSTOM_TRAJECTORY_SIZE; index++)
    delete custom_trajectory_[index];
}

void OpenManipulatorX::init_open_manipulator_x(bool sim, STRING usb_port, STRING baud_rate, float control_loop_time, std::vector<uint8_t> dxl_id)
{
  /*****************************************************************************
    ** Initialize Manipulator Parameter
    *****************************************************************************/
    addWorld("world",   // world name
             "omx1_arm1_joint"); // child name

    addJoint("omx1_arm1_joint",  // my name
             "world",   // parent name
             "omx1_arm2_joint",  // child name
             math::vector3(-0.0295, 0.0, 0.0525),             // relative position
             math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
             Z_AXIS,    // axis of rotation
             dxl_id[0], // actuator id
             M_PI,      // max joint limit (3.14 rad)
             -M_PI,     // min joint limit (-3.14 rad)
             1.0,       // coefficient
             0.1        // TODO: mass
            //  math::inertiaMatrix(1.0, 0.0, 0.0,
            //                           1.0, 1.0,
            //                                1.0),  // TODO: inertial tensor
            //  math::vector3(0.0, 0.0, 0.0)         // TODO: COM
             );

    addJoint("omx1_arm2_joint",  // my name
             "omx1_arm1_joint",  // parent name
             "omx1_arm3_joint",  // child name
             math::vector3(0.0, 0.0, 0.05095),                // relative position
             math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
             Y_AXIS,    // axis of rotation
             dxl_id[1], // actuator id
             M_PI_2,    // max joint limit (1.67 rad)
             -M_PI_2,   // min joint limit (-1.67 rad)
             1.0,       // coefficient
             0.1        // TODO: mass
            //  math::inertiaMatrix(1.0, 0.0, 0.0,
            //                           1.0, 1.0,
            //                                1.0),  // TODO: inertial tensor
            //  math::vector3(0.0, 0.0, 0.0)         // TODO: COM
             );

    addJoint("omx1_arm3_joint",  // my name
             "omx1_arm2_joint",  // parent name
             "omx1_arm4_joint",  // child name
             math::vector3(0.022, 0.0, 0.128),                // relative position
             math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
             Y_AXIS,    // axis of rotation
            //  dxl_id[3], // actuator id
             dxl_id[2], // actuator id
             M_PI_2,    // max joint limit (1.67 rad)
             -M_PI_2,   // min joint limit (-1.67 rad)
             1.0,       // coefficient
             0.1        // TODO: mass
            //  math::inertiaMatrix(1.0, 0.0, 0.0,
            //                           1.0, 1.0,
            //                                1.0),  // TODO: inertial tensor
            //  math::vector3(0.0, 0.0, 0.0)         // TODO: COM
             );

    addJoint("omx1_arm4_joint",  // my name
             "omx1_arm3_joint",  // parent name
             "omx1_arm5_joint", // child name
             math::vector3(0.0835, 0.0, 0.0),                 // relative position
             math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
             Y_AXIS,    // axis of rotation
            //  dxl_id[4], // actuator id
             dxl_id[3], // actuator id
             M_PI,      // max joint limit (3.14 rad)
             -M_PI,     // min joint limit (-3.14 rad)
             1.0,       // coefficient
             0.1        // TODO: mass
            //  math::inertiaMatrix(1.0, 0.0, 0.0,
            //                           1.0, 1.0,
            //                                1.0),  // TODO: inertial tensor
            //  math::vector3(0.0, 0.0, 0.0)         // TODO: COM
             );

    addJoint("omx1_arm5_joint",  // my name
             "omx1_arm4_joint",  // parent name
             "omx1_arm6_joint", // child name
             math::vector3(0.0405, 0.0, 0.0),                 // relative position
             math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
             Y_AXIS,    // axis of rotation
            //  dxl_id[5], // actuator id
             dxl_id[4], // actuator id
             M_PI_2,    // max joint limit (1.67 rad)
             -M_PI_2,   // min joint limit (-1.67 rad)
             1.0,       // coefficient
             0.1        // TODO: mass
            //  math::inertiaMatrix(1.0, 0.0, 0.0,
            //                           1.0, 1.0,
            //                                1.0),  // TODO: inertial tensor
            //  math::vector3(0.0, 0.0, 0.0)         // TODO: COM
             );

    addJoint("omx1_arm6_joint",  // my name
             "omx1_arm5_joint",  // parent name
             "omx1_gripper_l_finger_joint", // child name
             math::vector3(0.064, 0.0, 0.0),                  // relative position
             math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
             Y_AXIS,    // axis of rotation
            //  dxl_id[6], // actuator id
             dxl_id[5], // actuator id
             M_PI,      // max joint limit (3.14 rad)
             -M_PI,     // min joint limit (-3.14 rad)
             1.0,       // coefficient
             0.1        // TODO: mass
            //  math::inertiaMatrix(1.0, 0.0, 0.0,
            //                           1.0, 1.0,
            //                                1.0),  // TODO: inertial tensor
            //  math::vector3(0.0, 0.0, 0.0)         // TODO: COM
             );

    addTool("omx1_gripper_l_finger_joint",  // my name
            "omx1_arm6_joint",   // parent name
            math::vector3(0.11225, 0.0, 0.0),                 // relative position
            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
            // dxl_id[7],  // actuator id
            dxl_id[6],  // actuator id
            0.02825,    // max gripper limit (0.01 m)
            -0.02825,   // min gripper limit (-0.01 m)
            -0.015,     // Change unit from `meter` to `radian`
            0.1         // TODO: mass
            //  math::inertiaMatrix(1.0, 0.0, 0.0,
            //                           1.0, 1.0,
            //                                1.0),  // TODO: inertial tensor
            //  math::vector3(0.0, 0.0, 0.0)         // TODO: COM
            );
          
  /*****************************************************************************
  ** Initialize Kinematics 
  *****************************************************************************/
  kinematics_ = new kinematics::SolverCustomizedforOMChain();
//  kinematics_ = new kinematics::SolverUsingCRAndSRPositionOnlyJacobian();
  addKinematics(kinematics_);

  if(!sim)
  {
    /*****************************************************************************
    ** Initialize Joint Actuator
    *****************************************************************************/
    // actuator_ = new dynamixel::JointDynamixel();
    actuator_ = new dynamixel::JointDynamixelProfileControl(control_loop_time);
    
    // Set communication arguments
    STRING dxl_comm_arg[2] = {usb_port, baud_rate};
    void *p_dxl_comm_arg = &dxl_comm_arg;

    // Set joint actuator id
    std::vector<uint8_t> jointDxlId;
    jointDxlId.push_back(dxl_id[0]);
    jointDxlId.push_back(dxl_id[1]);
    jointDxlId.push_back(dxl_id[2]);
    jointDxlId.push_back(dxl_id[3]);
    jointDxlId.push_back(dxl_id[4]);
    jointDxlId.push_back(dxl_id[5]);
    // jointDxlId.push_back(dxl_id[6]);
    addJointActuator(JOINT_DYNAMIXEL, actuator_, jointDxlId, p_dxl_comm_arg);

    // Set joint actuator control mode
    STRING joint_dxl_mode_arg = "position_mode";
    void *p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
    setJointActuatorMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_mode_arg);

    /*****************************************************************************
    ** Initialize Tool Actuator
    *****************************************************************************/
    tool_ = new dynamixel::GripperDynamixel();

    // uint8_t gripperDxlId = dxl_id[7];
    uint8_t gripperDxlId = dxl_id[6];
    addToolActuator(TOOL_DYNAMIXEL, tool_, gripperDxlId, p_dxl_comm_arg);

    // Set gripper actuator control mode
    STRING gripper_dxl_mode_arg = "current_based_position_mode";
    void *p_gripper_dxl_mode_arg = &gripper_dxl_mode_arg;
    setToolActuatorMode(TOOL_DYNAMIXEL, p_gripper_dxl_mode_arg);

    // Set gripper actuator parameter
    STRING gripper_dxl_opt_arg[2];
    void *p_gripper_dxl_opt_arg = &gripper_dxl_opt_arg;
    gripper_dxl_opt_arg[0] = "Profile_Acceleration";
    gripper_dxl_opt_arg[1] = "20";
    setToolActuatorMode(TOOL_DYNAMIXEL, p_gripper_dxl_opt_arg);

    gripper_dxl_opt_arg[0] = "Profile_Velocity";
    gripper_dxl_opt_arg[1] = "200";
    setToolActuatorMode(TOOL_DYNAMIXEL, p_gripper_dxl_opt_arg);

    // Enable All Actuators 
    enableAllActuator();

    // Receive current angles from all actuators 
    receiveAllJointActuatorValue();
    receiveAllToolActuatorValue();
  }

  /*****************************************************************************
  ** Initialize Custom Trajectory
  *****************************************************************************/
  custom_trajectory_[0] = new custom_trajectory::Line();
  custom_trajectory_[1] = new custom_trajectory::Circle();
  custom_trajectory_[2] = new custom_trajectory::Rhombus();
  custom_trajectory_[3] = new custom_trajectory::Heart();

  addCustomTrajectory(CUSTOM_TRAJECTORY_LINE, custom_trajectory_[0]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_CIRCLE, custom_trajectory_[1]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_RHOMBUS, custom_trajectory_[2]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_HEART, custom_trajectory_[3]);
}

void OpenManipulatorX::process_open_manipulator_x(double present_time)
{
  // Planning (ik)
  JointWaypoint goal_joint_value = getJointGoalValueFromTrajectory(present_time);
  JointWaypoint goal_tool_value  = getToolGoalValue();

  // Control (motor)
  receiveAllJointActuatorValue();
  receiveAllToolActuatorValue();
  if(goal_joint_value.size() != 0) sendAllJointActuatorValue(goal_joint_value);
  if(goal_tool_value.size() != 0) sendAllToolActuatorValue(goal_tool_value);

  // Perception (fk)
  solveForwardKinematics();
}
