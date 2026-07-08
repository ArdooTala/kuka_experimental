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

#include <array>
#include <cstdint>

#include <eki_communication/xml/XmlWriter.h>
#include <eki_communication/core/Types.h>

namespace rbt
{
class CustomCommand
{
public:
    CustomCommand() {}
    CustomCommand(int cmd_index, const std::array<uint8_t, 256> &input_params)
        : cmd_index_(cmd_index), input_params_(input_params)
    {
    }
    ~CustomCommand() {}

    int cmd_index_ = 0;
    std::array<uint8_t, 256> input_params_ = {};

    void to_xml(XmlWriter &writer) const;
};
} // namespace rbt
