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

#include <eki_communication/CustomCommand.h>

void rbt::CustomCommand::to_xml(XmlWriter &writer) const
{
    writer.add_element("CustomCmd", {
        {"id", std::to_string(id_)},
        {"CmdIndex", std::to_string(cmd_index_)},
        {"ParamsCount", std::to_string(params_count())}
    });
}

void rbt::CustomCommand::param_to_xml(XmlWriter &writer, const CmdParam &param, int batch_id)
{
    writer.open_element("CustomParam", {
        {"Id", std::to_string(batch_id)},
        {"Type", std::to_string(param.type)},
        {"Index", std::to_string(param.index)}
    });
    writer.open_element("Value");
    std::string raw(reinterpret_cast<const char *>(param.value.data()), param.value.size());
    writer.add_content(raw);
    writer.close_element();
    writer.close_element();
}
