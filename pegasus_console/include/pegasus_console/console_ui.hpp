/*****************************************************************************
 * 
 *   Author: Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
 *   Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright 
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in 
 * the documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this 
 * software must display the following acknowledgement: This product 
 * includes software developed by Project Pegasus.
 * 4. Neither the name of the copyright holder nor the names of its 
 * contributors may be used to endorse or promote products derived 
 * from this software without specific prior written permission.
 *
 * Additional Restrictions:
 * 4. The Software shall be used for non-commercial purposes only. 
 * This includes, but is not limited to, academic research, personal 
 * projects, and non-profit organizations. Any commercial use of the 
 * Software is strictly prohibited without prior written permission 
 * from the copyright holders.
 * 5. The Software shall not be used, directly or indirectly, for 
 * military purposes, including but not limited to the development 
 * of weapons, military simulations, or any other military applications. 
 * Any military use of the Software is strictly prohibited without 
 * prior written permission from the copyright holders.
 * 6. The Software may be utilized for academic research purposes, 
 * with the condition that proper acknowledgment is given in all 
 * corresponding publications.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/
#pragma once

#include <memory>
#include <utility>
#include <sstream>
#include <functional>

#include "vehicle_status.hpp"
#include "ftxui/component/component.hpp"  // for Button, Horizontal, Renderer
#include "ftxui/component/screen_interactive.hpp"  // for ScreenInteractive

class ConsoleUI {

public:    

    /**
     * @brief An alias for a shared, unique and weak pointers of a ConsoleUI
     */
    using SharedPtr = std::shared_ptr<ConsoleUI>;
    using UniquePtr = std::unique_ptr<ConsoleUI>;
    using WeakPtr = std::weak_ptr<ConsoleUI>;

    struct Config{
        // Basic low level operation of the vehicle
        std::function<void(bool)> on_arm_disarm_click;
        std::function<void()> on_offboard_click;
        std::function<void()> on_land_click;
        std::function<void()> on_hold_click;
        std::function<void()> on_kill_switch_click;

        // Thrust curve control of the vehicle
        std::function<void()> on_thrust_curve_click;
        std::function<void()> on_thrust_curve_stop;
        std::function<bool()> is_thrust_curve_running;

        // Offboard position control of the vehicle
        std::function<void()> on_setpoint_click;
        std::function<void()> on_setpoint_stop;
        std::function<bool()> is_setpoint_running;

        // Autopulot of the vehicle
        std::function<void(std::string&)> on_set_autopilot_mode;

        // Add trajectory segments to the autopilot to follow
        std::function<void()> on_add_waypoint_click;
        std::function<void()> on_add_arc_click;
        std::function<void()> on_add_line_click;
        std::function<void()> on_add_circle_click;
        std::function<void()> on_add_lemniscate_click;
        std::function<void()> on_reset_path_click;
    };
    
    ConsoleUI(const Config & config);
    ~ConsoleUI();

    void clear_terminal();
    void loop();

    // Get the thrust curve value selected from the thrust curve widget
    float get_throtle();

    // Get the setpoint selected from the setpoint widget
    std::pair<Eigen::Vector3d, float> get_setpoint();

    // Get the autopilot data
    inline AutopilotwidgetData & get_autopilot_data() { return autopilot_data_; }

    // The latest status and state of the vehicle
    FmuStatus status_;
    State state_;
    std::string autopilot_mode_;

protected:

    // Auxiliar function to convert a float to string
    std::string float_to_string(float value, int precision=2) const;

    // Auxiliar function to validate the input of a string
    float validate_input(std::string & input, float default_value=0.0) const;
   
    // Individual Components that make up the UI
    ftxui::Component control_buttons();
    ftxui::Element state_display();
    ftxui::Component thrust_curve();
    ftxui::Component onboard_position_control();
    ftxui::Component autopilot_control();

    // Screen where the UI will be placed
    ftxui::ScreenInteractive screen_;

    // UI Config
    Config config_;

    // The selected tab in the UI
    int tab_selected_{2};
    std::vector<std::string> tab_values_{"Thrust Curve", "Position Control", "Autopilot"};

    // The structure that holds the data for the Position Control tab
    PositionControlWidgetData position_control_data_;

    // The float slider value for the thrust curve
    ThrotleWidgetData throtle_data_;

    // The structure that holds the data for the Autopilot tab
    AutopilotwidgetData autopilot_data_;
    
    int slider_value_;
};