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

#include <eki_communication/ProgramState.h>

void rbt::ProgramState::from_xml(XmlReader &reader)
{
    if (auto element = reader.get_element("State"))
    {
        status = element->IntAttribute("Status", status);
        stopped = element->BoolAttribute("Stopped", stopped);
    }

    if (auto element = reader.get_element("Command"))
    {
        command_id = element->IntAttribute("Id", command_id);

        if (auto state_element = element->FirstChildElement("State"))
        {
            command_status = state_element->IntAttribute("Status", command_status);
        }

        if (auto error_element = element->FirstChildElement("Error"))
        {
            error_code = error_element->IntAttribute("Code", error_code);

            const char *message = error_element->Attribute("Message");
            if (message != nullptr)
            {
                error_message = message;
            }
        }
    }
}
