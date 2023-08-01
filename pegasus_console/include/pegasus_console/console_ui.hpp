#pragma once

#include <memory>
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
        std::function<void(bool)> on_arm_disarm_click;
        std::function<void()> on_land_click;
        std::function<void()> on_hold_click;
        std::function<void()> on_kill_switch_click;
    };
    
    ConsoleUI(const Config & config);
    ~ConsoleUI();

    void clear_terminal();
    void loop();

    // The latest status and state of the vehicle
    FmuStatus status_;
    State state_;

protected:

    // Auxiliar function to convert a float to string
    std::string float_to_string(float value);

    // Individual Components that make up the UI
    ftxui::Component control_buttons();
    ftxui::Element state_display();
    ftxui::Component thrust_curve();
    ftxui::Component onboard_position_control();

    // Screen where the UI will be placed
    ftxui::ScreenInteractive screen_;

    // UI Config
    Config config_;
};