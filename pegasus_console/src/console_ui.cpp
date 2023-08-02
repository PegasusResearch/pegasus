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
        onboard_position_control()
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
            ftxui::Button("Land", config_.on_land_click, ftxui::ButtonOption::Animated(ftxui::Color::Green)),
            ftxui::Button("Hold", config_.on_hold_click, ftxui::ButtonOption::Animated(ftxui::Color::Green)),
            ftxui::Button("Kill Switch", config_.on_kill_switch_click, ftxui::ButtonOption::Animated(ftxui::Color::Red)),
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

    auto thrust_curve_input = ftxui::Container::Vertical({
        ftxui::Renderer([] { return 
            ftxui::vbox({ 
                ftxui::text("Thrust Curve") | ftxui::center, 
                ftxui::separator() 
            });
        }),

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
                ftxui::text("Setpoint state: " + std::string(config_.is_setpoint_running() ? "Running" : "Stopped")),
                ftxui::text("Setpoint: [" + float_to_string(this->position_control_data_.position[0]) + ", " + float_to_string(this->position_control_data_.position[1]) + ", " + float_to_string(this->position_control_data_.position[2]) + "]"),
                ftxui::text("Yaw: " + float_to_string(this->position_control_data_.yaw)),
            });
        })
    });

    return position_input;
}