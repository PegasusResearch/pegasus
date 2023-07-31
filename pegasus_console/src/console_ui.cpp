#include <cstdio>
#include "console_ui.hpp"
#include "ftxui/component/captured_mouse.hpp"  // for ftxui
#include "ftxui/component/component.hpp"  // for Button, Horizontal, Renderer
#include "ftxui/component/component_base.hpp"      // for ComponentBase
#include "ftxui/component/component_options.hpp"   // for ButtonOption
#include "ftxui/component/screen_interactive.hpp"  // for ScreenInteractive
#include "ftxui/dom/elements.hpp"  // for gauge, separator, text, vbox, operator|, Element, border
#include "ftxui/screen/color.hpp"  // for Color, Color::Blue, Color::Green, Color::Red

using namespace ftxui;

ConsoleUI::ConsoleUI() : screen_(ScreenInteractive::Fullscreen()) {

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

    // Create the arm menu
    auto arm_menu = arm_buttons();

    // Renderer for the entire UI
    auto renderer = Renderer(arm_menu, [&] {
        return vbox({
            arm_menu->Render() | border,
            separator(),
        });
    });

    // Loop the renderer
    screen_.Loop(renderer);
}

Component ConsoleUI::arm_buttons() {
    // Basic control buttons
    auto control_buttons = Container::Horizontal({
        Button("Arm", [&] { printf("Arming\n"); }, ButtonOption::Animated(Color::Green)),
        Button("Disarm", [&] { printf("Disarming\n"); }, ButtonOption::Animated(Color::Green)),
        Button("Takeoff", [&] { printf("Taking off\n"); }, ButtonOption::Animated(Color::Blue)),
        Button("Kill Switch", [&] { printf("Killing\n"); }, ButtonOption::Animated(Color::Red)),
    });

    return control_buttons;
}