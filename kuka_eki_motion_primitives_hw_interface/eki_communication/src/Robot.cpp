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
//          Moritz Weisenböhler, Mathias Fuhrer

#include <eki_communication/Robot.h>

#include <thread>
#include <iostream>

bool rbt::Robot::is_connected()
{
    return (!interface_used_ || interface_.is_connected()) && (!meta_interface_used_ || meta_interface_.is_connected());
}

bool rbt::Robot::connect(const std::string &host, int port, int meta_port)
{
    std::cout << "[Robot] Trying to connect to the host: [" << host << "], port: [" << port << "], meta_port: [" << meta_port << "]" << std::endl;
    interface_used_ = port > 0;
    meta_interface_used_ = meta_port > 0;
    
    if (meta_interface_used_)
    {
        connect_to(meta_interface_, host, meta_port, true);
    }

    if (interface_used_)
    {
        connect_to(interface_, host, port, false);
    }
    std::cout << "[Robot] is_connected: " << is_connected() << std::endl;
    return is_connected();
}

void rbt::Robot::disconnect()
{
    if (interface_used_)
    {
        std::cout << "[Robot] Disconnecting EKI Interface ..." << std::endl;
        interface_.disconnect();
    }

    if (meta_interface_used_)
    {
        std::cout << "[Robot] Disconnecting Meta EKI Interface ..." << std::endl;
        meta_interface_.disconnect();
    }
}

void rbt::Robot::perform(const MoveCommand &move)
{
    perform(Command(move));
}

void rbt::Robot::perform(const GripCommand &grip)
{
    perform(Command(grip));
}

void rbt::Robot::perform(const MoveCommand &move, const GripCommand &grip)
{
    perform(Command(move, grip));
}

void rbt::Robot::perform(const CustomCommand &custom)
{
    perform(Command(custom));
}

void rbt::Robot::perform(const Command &command)
{
    waiting_sequence_.add(command);

    if (auto_run)
    {
        run();
    }
}

void rbt::Robot::pause_commands()
{
    if (!commands_paused_)
    {
        commands_paused_ = true;
    }
}

void rbt::Robot::continue_commands()
{
    if (commands_paused_)
    {
        commands_paused_ = false;
    }
}

void rbt::Robot::abort_commands()
{
    send_abort(true);
}

void rbt::Robot::reset_abort_commands()
{
    send_abort(false);
}

void rbt::Robot::set_velocity(float value)
{
    if (velocity_override_ != value)
    {
        velocity_override_ = value;

        if (!commands_paused_)
        {
        }
    }
}

bool rbt::Robot::run()
{
    if (!is_active() && waiting_sequence_.size() > 0)
    {
        active_sequence_.clear();

        active_sequence_.add(waiting_sequence_);
        waiting_sequence_.clear();

        send_sequence();

        call_listener(RobotEvent::RUN);

        return true;
    }

    return false;
}


void rbt::Robot::send_sequence()
{
    XmlWriter writer;

    writer.line_break = "\n";

    writer.add_prolog();

    active_sequence_.to_xml(writer);

    int size = interface_.send(writer.get_string());
}

void rbt::Robot::send_abort(bool abort)
{
    rbt::AbortCommand command{abort};

    XmlWriter writer;
    writer.add_prolog();

    command.to_xml(writer);

    int size = interface_.send(writer.get_string());
}

void rbt::Robot::connect_to(rbt::EKInterface &interface, const std::string &host, int port, bool udp)
{
    while (!interface.is_connected())
    {
        bool connected = interface.connect_to(host, port, udp);

        if (connected && udp)
        {
            // EKI's UDP server may only start sending datagrams to the client after
            // receiving a first packet (client address registration). For UDP the
            // connect() call never fails, so a failed ping (e.g. ICMP port unreachable
            // while the server is not up yet) is the only way to detect that the
            // server did not register us.
            interface.send(";");
            connected = interface.is_connected();
        }

        if (connected)
        {
            call_listener(RobotEvent::CONNECT);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_));
        }
    }
}

void rbt::Robot::poll_state()
{
    if (interface_used_)
    {
        std::string xml = collect_state_xml(interface_, buffer_, "ProgramState");
        update_state(xml, false);
    }

    if (meta_interface_used_)
    {
        std::string meta_xml = collect_state_xml(meta_interface_, meta_buffer_, "RobotState");
        update_state(meta_xml, true);
    }
}

std::string rbt::Robot::collect_state_xml(EKInterface &interface, std::string &buffer, const std::string &tag)
{
    buffer += interface.receive();

    int end_index = buffer.find("</" + tag + ">");

    if (end_index > -1)
    {
        end_index += 3 + tag.size();

        std::string xml_message = buffer.substr(0, end_index);
        buffer = buffer.substr(end_index);

        return xml_message;
    }

    // Guard against unbounded growth if the closing tag never arrives
    // (e.g. tag mismatch or a malformed stream).
    if (buffer.size() > 64 * 1024)
    {
        buffer.clear();
    }

    return "";
}

void rbt::Robot::update_state(std::string &xml_message, bool is_meta)
{
    // std::cout << "[Robot update_state] " << (is_meta ? "Meta_XML message: " : "XML message: ") << xml_message << std::endl;
    if (xml_message.size() > 0)
    {
        XmlReader reader(xml_message);

        if (!reader.has_error())
        {
            if (is_meta)
            {
                state_.from_xml(reader);
            }
            else
            {
                program_state_.from_xml(reader);
                active_sequence_.update(program_state_.command_id, program_state_.command_status);

                if (program_state_.status == 0)
                {
                    // EKI command buffer is empty -> the whole sequence is done.
                    active_sequence_.finish();
                }
            }

            call_listener(RobotEvent::STATE);

            if (!is_meta && auto_run)
            {
                run();
            }
        }
        else if (reader.error_id() != tinyxml2::XMLError::XML_ERROR_EMPTY_DOCUMENT)
        {
            std::cout << reader.error() << std::endl;
            std::cout << "-> " << xml_message << std::endl;
        }
    }
}

void rbt::Robot::clear_waiting_commands()
{
    waiting_sequence_.clear();
}

void rbt::Robot::call_listener(RobotEvent event)
{
    if (listener != nullptr)
    {
        listener(event, this);
    }
}
