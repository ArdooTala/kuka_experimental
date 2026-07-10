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

#include <vector>
#include <string>
#include <cstdint>
#include <cstring>

#include <eki_communication/xml/XmlWriter.h>
#include <eki_communication/core/Types.h>

namespace rbt
{
struct CmdParam
{
    int type;
    int index;
    std::vector<uint8_t> value;

    CmdParam(int idx, const std::vector<uint8_t> &val, int type_code)
        : type(type_code), index(idx), value(val) {}

    CmdParam(int idx, int32_t val)
        : type(3), index(idx)
    {
        value.resize(4);
        std::memcpy(value.data(), &val, 4);
    }

    CmdParam(int idx, double val)
        : type(2), index(idx)
    {
        value.resize(8);
        std::memcpy(value.data(), &val, 8);
    }

    CmdParam(int idx, bool val)
        : type(4), index(idx)
    {
        value.resize(1);
        value[0] = val ? 1 : 0;
    }

    CmdParam(int idx, const std::string &val)
        : type(1), index(idx)
    {
        value.assign(val.begin(), val.end());
    }
};

class CustomCommand
{
public:
    CustomCommand() {}
    CustomCommand(int id, int cmd_index, const std::vector<CmdParam> &params = {})
        : id_(id), cmd_index_(cmd_index), params_(params)
    {
    }
    ~CustomCommand() {}

    int id() const { return id_; }
    int cmd_index() const { return cmd_index_; }
    int params_count() const { return params_.size(); }
    const std::vector<CmdParam> &params() const { return params_; }

    void to_xml(XmlWriter &writer) const;
    static void param_to_xml(XmlWriter &writer, const CmdParam &param, int batch_id);

private:
    int id_ = 0;
    int cmd_index_ = 0;
    std::vector<CmdParam> params_;
};
} // namespace rbt
