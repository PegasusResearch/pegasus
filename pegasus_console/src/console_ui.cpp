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

void ConsoleUI::loop() {

    // Create the top bar
    auto top = ftxui::Renderer([] { return ftxui::text("Drone Console") | ftxui::center; });
    
    // Create the left bar
    auto left = ftxui::Container::Vertical( {
        control_buttons(),
        state_display(),
    } );

    // Create the Right spot
    auto middle = ftxui::Renderer([&] { return ftxui::text("Middle") | ftxui::center; });

    int left_size = 40;
    int top_size = 1;

    // Create a split screen
    auto container = ftxui::ResizableSplitLeft(left, middle, &left_size);
    container = ftxui::ResizableSplitTop(top, container, &top_size);

    // Renderer for the entire UI
    auto renderer = ftxui::Renderer(container, [&] {
        return container->Render() | ftxui::border;
    });

    // Loop the renderer
    screen_.Loop(renderer);
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
        }),
        ftxui::Renderer([] { return ftxui::separator(); })
    });

    return control_buttons;
}

ftxui::Component ConsoleUI::state_display() {
    
    auto status = ftxui::Container::Vertical({
        ftxui::Renderer([] { return
            ftxui::vbox({
                ftxui::text("Status") | ftxui::center,
                ftxui::separator(),
                ftxui::text("ID: "),
                ftxui::text("Armed: "),
                ftxui::text("Mode: "),
                ftxui::separator(),
                ftxui::text("State") | ftxui::center,
                ftxui::separator(),
                ftxui::text("Position: "),
                ftxui::text("Orientation: "),
                ftxui::text("Inertial Velocity: "),
                ftxui::text("Angular Velocity: "),
            });
        })
    });

    return status;
    
}