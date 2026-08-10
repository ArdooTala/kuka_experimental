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

#pragma once

#include <functional>

#include <eki_common/Component.h>

#include <eki_communication/core/EKInterface.h>
#include <eki_communication/core/CommandSequence.h>
#include <eki_communication/AbortCommand.h>
#include <eki_communication/RobotState.h>
#include <eki_communication/RobotMetaState.h>
#include <eki_communication/ProgramState.h>

namespace rbt
{
    class Robot : public Component
    {
    private:
        EKInterface interface_;
        EKInterface meta_interface_;

        int reconnect_delay_ = 1000;
        int loop_delay_ = 20;
        bool interface_used_ = false;
        bool meta_interface_used_ = false;

        RobotState state_;
        ProgramState program_state_;
        CommandSequence active_sequence_;
        CommandSequence waiting_sequence_;

        std::string buffer_;
        std::string meta_buffer_;

        float velocity_override_ = 1.f;
        bool commands_paused_ = false;

        void connect_to(rbt::EKInterface &interface, const std::string &host, int port, bool udp = false);
        void spin();
        std::string collect_state_xml(EKInterface &interface, std::string &buffer, const std::string &tag);
        void update_state(std::string &xml_message, bool is_meta);
        void call_listener(RobotEvent event);

        ChronoEntry chrono_;

        void perform(const Command &command);
        void send_sequence();
        void send_abort(bool abort = true);

    public:
        Robot() : Component("rbt::Robot") {}
        ~Robot() {}

        bool is_connected();
        bool connect(const std::string &host, int port, int meta_port = 0);
        void disconnect();

        void perform(const MoveCommand &move);
        void perform(const GripCommand &grip);
        void perform(const MoveCommand &move, const GripCommand &grip);

        bool auto_run = false;

        void pause_commands();
        void continue_commands();
        void abort_commands();
        void reset_abort_commands();
        void set_velocity(float value);

        bool run();
        bool is_active() { return !active_sequence_.is_finished(); }
        bool robot_in_movement();
        RobotState get_state() { return state_; }
        ProgramState get_program_state() { return program_state_; }
        float get_velocity_override() { return velocity_override_; }
        bool commands_paused() { return commands_paused_; }

        int last_command_id_of_sequence() const { return waiting_sequence_.last_command_id(); };
        int last_finished_command_id() const { return state_.last_finished_command_id; }
        int robot_stopped() const { return state_.robot_stopped; }

        std::function<void(RobotEvent event, Robot *robot)> listener = nullptr;
        // CommandSequence get_active_sequence() { return active_sequence_; }
        // CommandSequence get_waiting_sequence() { return waiting_sequence_; }
    };
} // namespace rbt
