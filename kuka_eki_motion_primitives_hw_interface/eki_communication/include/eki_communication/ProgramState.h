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

#pragma once

#include <string>

#include <eki_communication/xml/XmlReader.h>

namespace rbt
{
    class ProgramState
    {
    public:
        int status = 0;              // ProgramState/State/@Status: 0 idle, 1 executing, 5 stopped
        bool stopped = false;        // ProgramState/State/@Stopped
        int command_id = 0;          // ProgramState/Command/@Id
        int command_status = 0;      // ProgramState/Command/State/@Status: 1 started, 2 finished, 3 error
        int error_code = 0;          // ProgramState/Error/@Code
        std::string error_message;   // ProgramState/Error/@Message

        void from_xml(XmlReader &reader);
    };
} // namespace rbt
