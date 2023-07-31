#pragma once
#include "ftxui/component/component.hpp"  // for Button, Horizontal, Renderer
#include "ftxui/component/screen_interactive.hpp"  // for ScreenInteractive


using namespace ftxui;

class ConsoleUI {

public:    
    
    ConsoleUI();
    ~ConsoleUI();

    void clear_terminal();
    void loop();

    // Individual Components that make up the UI
    Component arm_buttons();

protected:

    int value = 50;

    // Screen where the UI will be placed
    ScreenInteractive screen_;
};