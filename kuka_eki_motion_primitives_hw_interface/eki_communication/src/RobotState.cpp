// Copyright (c) 2025, H-KA Hochschule Karlsruhe - University of Applied Sciences
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
// Authors: Students of the Insitute for Robotics and Autonomous Systems (IRAS) 
//          - (Supervisor: Prof. Dr.-Ing. Christian Wurll), 
//          Moritz Weisenböhler,
//          Mathias Fuhrer

#include <eki_communication/RobotState.h>

int rbt::RobotState::max_id_ = 0;

void rbt::RobotState::from_xml(XmlReader &reader)
{
    // The meta channel only sends Position/Velocity/Torque
    if (auto element = reader.get_element("Position/Joint"))
    {
        position_joints = PoseJoints{element->FloatAttribute("A1", position_joints.a1),
                                     element->FloatAttribute("A2", position_joints.a2),
                                     element->FloatAttribute("A3", position_joints.a3),
                                     element->FloatAttribute("A4", position_joints.a4),
                                     element->FloatAttribute("A5", position_joints.a5),
                                     element->FloatAttribute("A6", position_joints.a6)};
    }

    if (auto element = reader.get_element("Position/Cartesian"))
    {
        position_cartesian = PoseCartesian{element->FloatAttribute("X", position_cartesian.x),
                                           element->FloatAttribute("Y", position_cartesian.y),
                                           element->FloatAttribute("Z", position_cartesian.z),
                                           element->FloatAttribute("A", position_cartesian.a),
                                           element->FloatAttribute("B", position_cartesian.b),
                                           element->FloatAttribute("C", position_cartesian.c)};
    }

    if (auto element = reader.get_element("Velocity"))
    {
        velocity = PoseJoints{element->FloatAttribute("A1", velocity.a1),
                              element->FloatAttribute("A2", velocity.a2),
                              element->FloatAttribute("A3", velocity.a3),
                              element->FloatAttribute("A4", velocity.a4),
                              element->FloatAttribute("A5", velocity.a5),
                              element->FloatAttribute("A6", velocity.a6)};
    }

    if (auto element = reader.get_element("Position/ExtAxis"))
    {
        position_ext_joints = PoseExtJoints{element->FloatAttribute("E1", position_ext_joints.e1),
                                            element->FloatAttribute("E2", position_ext_joints.e2),
                                            element->FloatAttribute("E3", position_ext_joints.e3),
                                            element->FloatAttribute("E4", position_ext_joints.e4),
                                            element->FloatAttribute("E5", position_ext_joints.e5),
                                            element->FloatAttribute("E6", position_ext_joints.e6)};
    }

    if (auto element = reader.get_element("Torque"))
    {
        torque = PoseJoints{element->FloatAttribute("A1", torque.a1),
                            element->FloatAttribute("A2", torque.a2),
                            element->FloatAttribute("A3", torque.a3),
                            element->FloatAttribute("A4", torque.a4),
                            element->FloatAttribute("A5", torque.a5),
                            element->FloatAttribute("A6", torque.a6)};
    }
}
