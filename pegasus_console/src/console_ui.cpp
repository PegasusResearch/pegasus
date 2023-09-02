#include <cstdio>
#include "console_ui.hpp"
#include "ftxui/component/captured_mouse.hpp"  // for ftxui
#include "ftxui/component/component.hpp"  // for Button, Horizontal, Renderer
#include "ftxui/component/component_base.hpp"      // for ComponentBase
#include "ftxui/component/component_options.hpp"   // for ButtonOption
#include "ftxui/component/screen_interactive.hpp"  // for ScreenInteractive
#include "ftxui/dom/elements.hpp"  // for gauge, separator, text, vbox, operator|, Element, border
#include "ftxui/screen/color.hpp"  // for Color, Color::Blue, Color::Green, Color::Red

ConsoleUI::ConsoleUI(const Config & config) : 
    screen_(ftxui::ScreenInteractive::Fullscreen()), config_(config) {

    // Clear the terminal
    clear_terminal();
}

ConsoleUI::~ConsoleUI() {
    // Clear the terminal
    clear_terminal();
}

void ConsoleUI::clear_terminal() {
    printf("\033[2J\033[1;1H");
}

std::string ConsoleUI::float_to_string(float value, int precision) const {

    // String stream to set the precision of the floats to represent on the UI
    std::stringstream stream;
    
    // Set the precision of the floats to represent in the UI to 2 decimal places
    stream.str("");
    stream.precision(precision);
    stream << value;
    return stream.str();
}

// Auxiliar function to validate the input of a string
float ConsoleUI::validate_input(std::string & input, float default_value) const {
    
    float output = default_value;

    if (!input.empty()) {
        // Check if the input is a valid float
        try {
            output = std::stof(input);
        } catch (std::invalid_argument const & e) {
            // do nothing if the input is not a valid float
        }
    }
    return output;
}

// Auxiliar function to return the current setpoint selected in the onboard setpoint widget
std::pair<Eigen::Vector3d, float> ConsoleUI::get_setpoint() {
    return std::make_pair(position_control_data_.position, position_control_data_.yaw);
}

float ConsoleUI::get_throtle() {
    return throtle_data_.throtle;
}

void ConsoleUI::loop() {

    // Create the top bar
    auto top = ftxui::Renderer([] { return ftxui::text("Drone Console") | ftxui::center; });
    
    // Create the left bar
    auto left = ftxui::Container::Vertical({
        control_buttons(),
        ftxui::Renderer([this] { return this->state_display(); }),
    });

    // Create the tabs for the right screen
    auto tab_toggle = ftxui::Toggle(&tab_values_, &tab_selected_);
    
    auto tab_container = ftxui::Container::Tab({
        thrust_curve(),
        onboard_position_control(),
        autopilot_control()
    }, &tab_selected_);
 
    auto container = ftxui::Container::Vertical({
        tab_toggle,
        tab_container,
    });

    // Create the Right screen
    auto middle = ftxui::Renderer(container, [&] {
        return ftxui::vbox({
            tab_toggle->Render(),
            ftxui::separator(),
            tab_container->Render(),
        });
    });

    // Create a split screen
    int left_size = 43;
    int top_size = 1;
    auto container_aux = ftxui::ResizableSplitLeft(left, middle, &left_size);
    auto document = ftxui::ResizableSplitTop(top, container_aux, &top_size);

    // Create a thread that will create events to refresh the terminal UI
    std::atomic<bool> refresh_ui_continue = true;
    
    std::thread refresh_ui([&] {
        while (refresh_ui_continue) {
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(0.1s);
            // After updating the state, request a new frame to be drawn. This is done
            // by simulating a new "custom" event to be handled.
            screen_.Post(ftxui::Event::Custom);
        }
    });

    // Loop the renderer
    screen_.Loop(document);

    // After exiting the loop, kill the event thread and join it
    refresh_ui_continue = false;
    refresh_ui.join();
}

ftxui::Component ConsoleUI::control_buttons() {
    
    // Basic control buttons
    auto control_buttons = ftxui::Container::Vertical({
        ftxui::Renderer([] { return 
            ftxui::vbox({ 
                ftxui::text("Low Level Control") | ftxui::center, 
                ftxui::separator() 
            });
        }),
        ftxui::Container::Horizontal({
            ftxui::Button("Arm", std::bind(config_.on_arm_disarm_click, true), ftxui::ButtonOption::Animated(ftxui::Color::Green)),
            ftxui::Button("Disarm", std::bind(config_.on_arm_disarm_click, false), ftxui::ButtonOption::Animated(ftxui::Color::Blue)),
            ftxui::Button("Offboard", config_.on_offboard_click, ftxui::ButtonOption::Animated(ftxui::Color::Green)),
            ftxui::Button("Land", config_.on_land_click, ftxui::ButtonOption::Animated(ftxui::Color::Green)),
            ftxui::Button("Hold", config_.on_hold_click, ftxui::ButtonOption::Animated(ftxui::Color::Green)),
            ftxui::Button("Kill", config_.on_kill_switch_click, ftxui::ButtonOption::Animated(ftxui::Color::Red)),
        }) | ftxui::center,
        ftxui::Renderer([] { return ftxui::separator(); })
    });

    return control_buttons;
}

ftxui::Element ConsoleUI::state_display() {
    
    return ftxui::vbox({
        ftxui::hbox({
            ftxui::vbox({
                ftxui::text("Status") | ftxui::center,
                ftxui::separator(),
                ftxui::text("ID: " + std::to_string(this->status_.system_id)),
                ftxui::text("Armed: " + std::string(this->status_.armed == true ? "True" : "False")),
                ftxui::text("Mode: " + fmu_flight_mode_map.at(this->status_.flight_mode)),
                ftxui::text("State: " + fmu_landed_state_map.at(this->status_.landed_state)),
                ftxui::text("RC Status: " + std::string(this->status_.rc_status.available == true ? "True" : "False")),
                ftxui::text("RC Strength: " + std::to_string(this->status_.rc_status.signal_strength))
            }),
            ftxui::separator(),
            ftxui::vbox({
                ftxui::text("Battery") | ftxui::center,
                ftxui::separator(),
                ftxui::hbox({
                    ftxui::text(std::to_string(int(this->status_.battery.percentage)) + "% "),
                    ftxui::gauge(this->status_.battery.percentage / 100.0) | ftxui::color(ftxui::Color::Red),
                }),
                ftxui::text(std::to_string(this->status_.battery.voltage) + "V"),
                ftxui::text(std::to_string(this->status_.battery.current) + "A"),
                ftxui::text("Consumed: "),
                ftxui::text(std::to_string(this->status_.battery.amps_hour_consumed) + "mAh")
            })
        }),
        ftxui::separator(),
        ftxui::text("Autopilot: " + autopilot_mode_),
        ftxui::separator(),
        ftxui::text("State") | ftxui::center,
        ftxui::separator(),
        ftxui::text("Position: [" +  float_to_string(this->state_.position[0]) + ", " + float_to_string(this->state_.position[1]) + ", " + float_to_string(this->state_.position[2]) + "]"),
        ftxui::text("Orientation: [" + float_to_string(this->state_.attitude_euler[0]) + ", " + float_to_string(this->state_.attitude_euler[1]) + ", " + float_to_string(this->state_.attitude_euler[2]) + "]"),
        ftxui::text("Inertial Velocity: [" + float_to_string(this->state_.velocity_inertial[0]) + ", " + float_to_string(this->state_.velocity_inertial[1]) + ", " + float_to_string(this->state_.velocity_inertial[2]) + "]"),
        ftxui::text("Angular Velocity: [" + float_to_string(this->state_.angular_velocity[0]) + ", " + float_to_string(this->state_.angular_velocity[1]) + ", " + float_to_string(this->state_.angular_velocity[2]) + "]"),
        ftxui::separator(),
        ftxui::text("Health") | ftxui::center,
        ftxui::separator(),
        ftxui::text("Armable: " + std::string(this->status_.health.is_armable ? "True" : "False")),
        ftxui::text("Global Position: " + std::string(this->status_.health.global_position_ok ? "Ok" : "Not Ok")),
        ftxui::text("Home Position: " + std::string(this->status_.health.home_position_ok ? "Ok" : "Not Ok")),
        ftxui::text("Local Position: " + std::string(this->status_.health.local_position_ok ? "Ok" : "Not Ok")),
        ftxui::text("Acc. Calibration: " + std::string(this->status_.health.acc_calibrated ? "Ok" : "Not Ok")),
        ftxui::text("Mag. Calibration: " + std::string(this->status_.health.mag_calibrated ? "Ok" : "Not Ok")),
    });    
}

ftxui::Component ConsoleUI::thrust_curve() {

    // Set a lambda function to be called when the user presses enter
    auto throtle_validator = [this]() { 

        // Get the updated value
        float new_throtle = validate_input(throtle_data_.input, throtle_data_.throtle);

        // Make sure that the new throtle is within the range [0, 100]
        new_throtle = std::max(0.0f, std::min(new_throtle, 100.0f));

        // Update the text and the value
        throtle_data_.input = float_to_string(new_throtle);
        throtle_data_.throtle = new_throtle;
    };

    ftxui::InputOption throtle_option = ftxui::InputOption();
    throtle_option.on_enter = throtle_validator;

    auto thrust_curve_input = ftxui::Container::Vertical({
        ftxui::Renderer([] { return 
            ftxui::vbox({ 
                ftxui::text("Thrust Curve") | ftxui::center, 
                ftxui::separator() 
            });
        }),
        ftxui::Container::Vertical({
            ftxui::Container::Horizontal({
                ftxui::Renderer([] { return ftxui::text("Throtle: "); }),
                ftxui::Input(&this->throtle_data_.input, "0.0", throtle_option),
            }),
            ftxui::Container::Horizontal({
                ftxui::Button("Go", config_.on_thrust_curve_click, ftxui::ButtonOption::Animated(ftxui::Color::Green)),
                ftxui::Button("Stop", config_.on_thrust_curve_stop, ftxui::ButtonOption::Animated(ftxui::Color::Red)),
            })
        }),
        ftxui::Renderer([this] { 
            return ftxui::vbox({
                ftxui::separator(),
                ftxui::text("Throtle test state: " + std::string(config_.is_thrust_curve_running() ? "Running" : "Stopped")) | ftxui::color((config_.is_thrust_curve_running() ? ftxui::Color::Red : ftxui::Color::Blue)),
                ftxui::text("Throtle value: " + std::to_string(this->throtle_data_.throtle)),
                ftxui::text("Perform this operation without the autopilot node running") | ftxui::color(ftxui::Color::Red),
            });
        })
    });
    
    return thrust_curve_input;
}

ftxui::Component ConsoleUI::onboard_position_control() {

    // Set a lambda function to be called when the user presses enter
    auto position_validator = [this](int index) { 

        // Get the updated value
        float new_value = validate_input(position_control_data_.inputs[index], position_control_data_.position[index]);

        // Update the text and the value
        position_control_data_.inputs[index] = float_to_string(new_value);
        position_control_data_.position[index] = new_value;
    };

    // Set a lambda function to be called when the user presses enter
    auto yaw_validator = [this]() { 

        // Get the updated value
        float new_yaw = validate_input(position_control_data_.inputs[3], position_control_data_.yaw);

        // Validate that the new is between 0 and 360
        new_yaw = std::max(std::min(new_yaw, 360.0f), 0.0f);

        // Update the text and the value
        position_control_data_.inputs[3] = float_to_string(new_yaw);
        position_control_data_.yaw = new_yaw;
    };

    // Set the input options for the position inputs and yaw input
    std::array<ftxui::InputOption, 3> position_input_option{ftxui::InputOption(), ftxui::InputOption(), ftxui::InputOption()};
    ftxui::InputOption yaw_input_option = ftxui::InputOption();

    for (int i = 0; i < 3; i++) {
        position_input_option[i].on_enter = std::bind(position_validator, i);
    }

    yaw_input_option.on_enter = yaw_validator;

    // Basic control buttons
    auto position_input = ftxui::Container::Vertical({
        ftxui::Renderer([] { return 
            ftxui::vbox({ 
                ftxui::text("Onboard Position Control") | ftxui::center, 
                ftxui::separator() 
            });
        }),
        ftxui::Container::Vertical({
            ftxui::Container::Horizontal({
                ftxui::Renderer([] { return ftxui::text("X: "); }),
                ftxui::Input(&this->position_control_data_.inputs[0], "0.0", position_input_option[0]),
            }),
            ftxui::Container::Horizontal({
                ftxui::Renderer([] { return ftxui::text("Y: "); }),
                ftxui::Input(&this->position_control_data_.inputs[1], "0.0", position_input_option[1]),
            }),
            ftxui::Container::Horizontal({
                ftxui::Renderer([] { return ftxui::text("Z: "); }),
                ftxui::Input(&this->position_control_data_.inputs[2], "0.0", position_input_option[2]),
            }),
            ftxui::Container::Horizontal({
                ftxui::Renderer([] { return ftxui::text("Yaw: "); }),
                ftxui::Input(&this->position_control_data_.inputs[3], "0.0", yaw_input_option),
            }),
            ftxui::Container::Horizontal({
                ftxui::Button("Go", config_.on_setpoint_click, ftxui::ButtonOption::Animated(ftxui::Color::Green)),
                ftxui::Button("Stop", config_.on_setpoint_stop, ftxui::ButtonOption::Animated(ftxui::Color::Red)),
            })
        }),
        ftxui::Renderer([this] { 
            return ftxui::vbox({
                ftxui::separator(),
                ftxui::text("Setpoint state: " + std::string(config_.is_setpoint_running() ? "Running" : "Stopped")) | ftxui::color((config_.is_setpoint_running() ? ftxui::Color::Red : ftxui::Color::Blue)),
                ftxui::text("Setpoint: [" + float_to_string(this->position_control_data_.position[0]) + ", " + float_to_string(this->position_control_data_.position[1]) + ", " + float_to_string(this->position_control_data_.position[2]) + "]"),
                ftxui::text("Yaw: " + float_to_string(this->position_control_data_.yaw)),
            });
        })
    });

    return position_input;
}

ftxui::Component ConsoleUI::autopilot_control() {

    // --------------------------------------------
    // Set the inputs for the waypoint
    // --------------------------------------------
    std::array<ftxui::InputOption, 3> waypoint_pos_opt{ftxui::InputOption(), ftxui::InputOption(), ftxui::InputOption()};
    ftxui::InputOption waypoint_yaw_opt = ftxui::InputOption();

    for (int i = 0; i < 3; i++) {
        waypoint_pos_opt[i].on_enter = [this, i] () {
            // Get the updated value from the string of text and validate it
            float new_value = validate_input(autopilot_data_.waypoint_pos_input[i], autopilot_data_.waypoint[i]);

            // Update the text and the value
            autopilot_data_.waypoint_pos_input[i] = float_to_string(new_value);
            autopilot_data_.waypoint[i] = new_value;
        };
    }

    waypoint_yaw_opt.on_enter = [this] () {
        // Get the updated value from the string of text and validate it
        float new_value = validate_input(autopilot_data_.waypoint_yaw_input, autopilot_data_.waypoint_yaw);

        // Validate that the new is between 0 and 360
        new_value = std::max( 0.0f, std::min(new_value, 360.0f));

        // Update the text and the value
        autopilot_data_.waypoint_yaw_input = float_to_string(new_value);
        autopilot_data_.waypoint_yaw = new_value;
    };

    // --------------------------------------------
    // Set the inputs for the arc path section
    // --------------------------------------------
    std::array<ftxui::InputOption, 5> arc_pos_opt{ftxui::InputOption(), ftxui::InputOption(), ftxui::InputOption(), ftxui::InputOption(), ftxui::InputOption()};
    ftxui::InputOption arc_speed_opt = ftxui::InputOption();

    for (int i = 0; i < 5; i++) {
        arc_pos_opt[i].on_enter = [this, i] () {
            // Get the updated value from the string of text and validate it
            float new_value = validate_input(autopilot_data_.arc_pos_input[i], autopilot_data_.arc[i]);

            // Update the text and the value
            autopilot_data_.arc_pos_input[i] = float_to_string(new_value);
            autopilot_data_.arc[i] = new_value;
        };
    }

    arc_speed_opt.on_enter = [this] () {

        // Get the updated value from the string of text and validate it
        float new_value = validate_input(autopilot_data_.arc_speed_input, autopilot_data_.arc_speed);

        // Validate that the new is between 0 and 50 m/s
        new_value = std::max( 0.0f, std::min(new_value, 50.0f));

        // Update the text and the value
        autopilot_data_.arc_speed_input = float_to_string(new_value);
        autopilot_data_.arc_speed = new_value;
    };

    // --------------------------------------------
    // Set the inputs for the line path section
    // --------------------------------------------
    std::array<ftxui::InputOption, 6> line_pos_opt{ftxui::InputOption(), ftxui::InputOption(), ftxui::InputOption(), ftxui::InputOption(), ftxui::InputOption(), ftxui::InputOption()};
    ftxui::InputOption line_speed_opt = ftxui::InputOption();

    for (int i = 0; i < 6; i++) {
        line_pos_opt[i].on_enter = [this, i] () {
            // Get the updated value from the string of text and validate it
            float new_value = validate_input(autopilot_data_.line_pos_input[i], autopilot_data_.line[i]);

            // Update the text and the value
            autopilot_data_.line_pos_input[i] = float_to_string(new_value);
            autopilot_data_.line[i] = new_value;
        };
    }

    line_speed_opt.on_enter = [this] () {

        // Get the updated value from the string of text and validate it
        float new_value = validate_input(autopilot_data_.line_speed_input, autopilot_data_.line_speed);

        // Validate that the new is between 0 and 50 m/s
        new_value = std::max( 0.0f, std::min(new_value, 50.0f));

        // Update the text and the value
        autopilot_data_.line_speed_input = float_to_string(new_value);
        autopilot_data_.line_speed = new_value;
    };

    // --------------------------------------------
    // Set the inputs for the circle path section
    // --------------------------------------------
    std::array<ftxui::InputOption, 4> circle_pos_opt{ftxui::InputOption(), ftxui::InputOption(), ftxui::InputOption(), ftxui::InputOption()};
    ftxui::InputOption circle_speed_opt = ftxui::InputOption();

    for (int i = 0; i < 4; i++) {
        circle_pos_opt[i].on_enter = [this, i] () {
            // Get the updated value from the string of text and validate it
            float new_value = validate_input(autopilot_data_.circle_pos_input[i], autopilot_data_.circle[i]);

            // Update the text and the value
            autopilot_data_.circle_pos_input[i] = float_to_string(new_value);
            autopilot_data_.circle[i] = new_value;
        };
    }

    circle_speed_opt.on_enter = [this] () {

        // Get the updated value from the string of text and validate it
        float new_value = validate_input(autopilot_data_.circle_speed_input, autopilot_data_.circle_speed);

        // Validate that the new is between 0 and 50 m/s
        new_value = std::max( 0.0f, std::min(new_value, 50.0f));

        // Update the text and the value
        autopilot_data_.circle_speed_input = float_to_string(new_value);
        autopilot_data_.circle_speed = new_value;
    };

    // --------------------------------------------
    // Set the inputs for the lemniscate path section
    // --------------------------------------------
    std::array<ftxui::InputOption, 4> lemniscate_pos_opt{ftxui::InputOption(), ftxui::InputOption(), ftxui::InputOption(), ftxui::InputOption()};
    ftxui::InputOption lemniscate_speed_opt = ftxui::InputOption();

    for (int i = 0; i < 4; i++) {
        lemniscate_pos_opt[i].on_enter = [this, i] () {
            // Get the updated value from the string of text and validate it
            float new_value = validate_input(autopilot_data_.lemniscate_pos_input[i], autopilot_data_.lemniscate[i]);

            // Update the text and the value
            autopilot_data_.lemniscate_pos_input[i] = float_to_string(new_value);
            autopilot_data_.lemniscate[i] = new_value;
        };
    }

    lemniscate_speed_opt.on_enter = [this] () {

        // Get the updated value from the string of text and validate it
        float new_value = validate_input(autopilot_data_.lemniscate_speed_input, autopilot_data_.lemniscate_speed);

        // Validate that the new is between 0 and 50 m/s
        new_value = std::max( 0.0f, std::min(new_value, 50.0f));

        // Update the text and the value
        autopilot_data_.lemniscate_speed_input = float_to_string(new_value);
        autopilot_data_.lemniscate_speed = new_value;
    };

    // Basic control buttons
    auto autopilot_control = ftxui::Container::Vertical({
        ftxui::Renderer([] { return 
            ftxui::vbox({ 
                ftxui::text("Autopilot Control") | ftxui::center, 
                ftxui::separator() 
            });
        }),
        ftxui::Container::Horizontal({
            ftxui::Button("Arm", std::bind(config_.on_set_autopilot_mode, std::string("ArmMode")), ftxui::ButtonOption::Animated(ftxui::Color::Green)),
            ftxui::Button("Disarm", std::bind(config_.on_set_autopilot_mode, std::string("DisarmMode")), ftxui::ButtonOption::Animated(ftxui::Color::Red)),
            ftxui::Button("Takeoff", std::bind(config_.on_set_autopilot_mode, std::string("TakeoffMode")), ftxui::ButtonOption::Animated(ftxui::Color::Green)),
            ftxui::Button("Hold", std::bind(config_.on_set_autopilot_mode, std::string("HoldMode")), ftxui::ButtonOption::Animated(ftxui::Color::Green)),
            ftxui::Button("Land", std::bind(config_.on_set_autopilot_mode, std::string("LandMode")), ftxui::ButtonOption::Animated(ftxui::Color::Green)),
        }) | ftxui::center,
        ftxui::Renderer([] { return ftxui::separator(); }),
        ftxui::Container::Horizontal({
            // Create a waypoint box
            ftxui::Container::Vertical({
                ftxui::Renderer([] { return ftxui::text("Set Waypoint"); }),
                ftxui::Container::Horizontal({
                    ftxui::Renderer([] { return ftxui::text("X: "); }),
                    ftxui::Input(&this->autopilot_data_.waypoint_pos_input[0], "0.0", waypoint_pos_opt[0]),
                }),
                ftxui::Container::Horizontal({
                    ftxui::Renderer([] { return ftxui::text("Y: "); }),
                    ftxui::Input(&this->autopilot_data_.waypoint_pos_input[1], "0.0", waypoint_pos_opt[1]),
                }),
                ftxui::Container::Horizontal({
                    ftxui::Renderer([] { return ftxui::text("Z: "); }),
                    ftxui::Input(&this->autopilot_data_.waypoint_pos_input[2], "0.0", waypoint_pos_opt[2]),
                }),
                ftxui::Container::Horizontal({
                    ftxui::Renderer([] { return ftxui::text("Yaw: "); }),
                    ftxui::Input(&this->autopilot_data_.waypoint_yaw_input, "0.0", waypoint_yaw_opt),
                }),
                ftxui::Container::Horizontal({
                    ftxui::Button("Set Waypoint", config_.on_add_waypoint_click, ftxui::ButtonOption::Animated(ftxui::Color::Green)),
                }),
                ftxui::Container::Horizontal({
                    ftxui::Button("Go to Waypoint", std::bind(config_.on_set_autopilot_mode, std::string("WaypointMode")), ftxui::ButtonOption::Animated(ftxui::Color::Green)),
                })
            }),
            ftxui::Container::Vertical({
                ftxui::Container::Horizontal({
                    ftxui::Renderer([] { return ftxui::separator(); }),
                    // Create an arc box
                    ftxui::Container::Vertical({
                        ftxui::Renderer([] { return ftxui::text("Add Arc"); }),
                        ftxui::Renderer([] { return ftxui::text("Start: "); }),
                        ftxui::Input(&this->autopilot_data_.arc_pos_input[0], "0.0", arc_pos_opt[0]),
                        ftxui::Input(&this->autopilot_data_.arc_pos_input[1], "0.0", arc_pos_opt[1]),
                        ftxui::Renderer([] { return ftxui::text("Center: "); }),
                        ftxui::Input(&this->autopilot_data_.arc_pos_input[2], "0.0", arc_pos_opt[2]),
                        ftxui::Input(&this->autopilot_data_.arc_pos_input[3], "0.0", arc_pos_opt[3]),
                        ftxui::Input(&this->autopilot_data_.arc_pos_input[4], "0.0", arc_pos_opt[4]),
                        ftxui::Renderer([] { return ftxui::text("Speed: "); }),
                        ftxui::Input(&this->autopilot_data_.arc_speed_input, "0.0", arc_speed_opt),
                        ftxui::Button("Add Arc", config_.on_add_arc_click, ftxui::ButtonOption::Animated(ftxui::Color::Green)),
                    }),
                    ftxui::Renderer([] { return ftxui::separator(); }),
                    // Create an line box
                    ftxui::Container::Vertical({
                        ftxui::Renderer([] { return ftxui::text("Add Line"); }),
                        ftxui::Renderer([] { return ftxui::text("Start: "); }),
                        ftxui::Input(&this->autopilot_data_.line_pos_input[0], "0.0", line_pos_opt[0]),
                        ftxui::Input(&this->autopilot_data_.line_pos_input[1], "0.0", line_pos_opt[1]),
                        ftxui::Input(&this->autopilot_data_.line_pos_input[2], "0.0", line_pos_opt[2]),
                        ftxui::Renderer([] { return ftxui::text("End: "); }),
                        ftxui::Input(&this->autopilot_data_.line_pos_input[3], "0.0", line_pos_opt[3]),
                        ftxui::Input(&this->autopilot_data_.line_pos_input[4], "0.0", line_pos_opt[4]),
                        ftxui::Input(&this->autopilot_data_.line_pos_input[5], "0.0", line_pos_opt[5]),
                        ftxui::Renderer([] { return ftxui::text("Speed: "); }),
                        ftxui::Input(&this->autopilot_data_.line_speed_input, "0.0", line_speed_opt),
                        ftxui::Button("Add Line", config_.on_add_line_click, ftxui::ButtonOption::Animated(ftxui::Color::Green)),
                    }),
                    ftxui::Renderer([] { return ftxui::separator(); }),
                    // Create a circle box
                    ftxui::Container::Vertical({
                        ftxui::Renderer([] { return ftxui::text("Add Circle"); }),
                        ftxui::Renderer([] { return ftxui::text("Center: "); }),
                        ftxui::Input(&this->autopilot_data_.circle_pos_input[0], "0.0", circle_pos_opt[0]),
                        ftxui::Input(&this->autopilot_data_.circle_pos_input[1], "0.0", circle_pos_opt[1]),
                        ftxui::Input(&this->autopilot_data_.circle_pos_input[2], "0.0", circle_pos_opt[2]),
                        ftxui::Renderer([] { return ftxui::text("Radius: "); }),
                        ftxui::Input(&this->autopilot_data_.circle_pos_input[3], "0.0", circle_pos_opt[3]),
                        ftxui::Renderer([] { return ftxui::text("Speed: "); }),
                        ftxui::Input(&this->autopilot_data_.circle_speed_input, "0.0", circle_speed_opt),
                        ftxui::Button("Add Circle", config_.on_add_circle_click, ftxui::ButtonOption::Animated(ftxui::Color::Green)),
                    }),
                    ftxui::Renderer([] { return ftxui::separator(); }),
                    // Create lemniscate box
                    ftxui::Container::Vertical({
                        ftxui::Renderer([] { return ftxui::text("Add Lemniscate"); }),
                        ftxui::Renderer([] { return ftxui::text("Center: "); }),
                        ftxui::Input(&this->autopilot_data_.lemniscate_pos_input[0], "0.0", lemniscate_pos_opt[0]),
                        ftxui::Input(&this->autopilot_data_.lemniscate_pos_input[1], "0.0", lemniscate_pos_opt[1]),
                        ftxui::Input(&this->autopilot_data_.lemniscate_pos_input[2], "0.0", lemniscate_pos_opt[2]),
                        ftxui::Renderer([] { return ftxui::text("Radius: "); }),
                        ftxui::Input(&this->autopilot_data_.lemniscate_pos_input[3], "0.0", lemniscate_pos_opt[3]),
                        ftxui::Renderer([] { return ftxui::text("Speed: "); }),
                        ftxui::Input(&this->autopilot_data_.lemniscate_speed_input, "0.0", lemniscate_speed_opt),
                        ftxui::Button("Add Lemniscate", config_.on_add_lemniscate_click, ftxui::ButtonOption::Animated(ftxui::Color::Green)),
                    }),
                }),
            ftxui::Button("Reset Trajectory", config_.on_reset_path_click, ftxui::ButtonOption::Animated(ftxui::Color::Green)),
            ftxui::Button("Follow Trajectory", std::bind(config_.on_set_autopilot_mode, std::string("FollowTrajectoryMode")), ftxui::ButtonOption::Animated(ftxui::Color::Green)),
            }),
        })
        ,
    });

    return autopilot_control;
}