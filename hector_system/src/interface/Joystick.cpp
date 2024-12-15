#include "../../include/interface/Joystick.h"
#include <iostream>

Joystick::Joystick() {
    if (SDL_Init(SDL_INIT_JOYSTICK) != 0) {
        std::cerr << "SDL_INit error: " << SDL_GetError() << std::endl;
        joystick = nullptr;
        return;
    }

    if (SDL_NumJoysticks() < 1) {
        std::cerr << "No joysticks connected." << std::endl;
        joystick = nullptr;
        return;
    }

    joystick = SDL_JoystickOpen(0);
    if (joystick == nullptr) {
        std::cerr << "Unable to open joystick: " << SDL_GetError() << std::endl;
        return;
    }
}

Joystick::~Joystick() {
    if (joystick) {
        SDL_JoystickClose(joystick);
    }

    SDL_Quit();
}

double Joystick::scaleAxisValue(Sint16 value) {
    return value / 32768.0;
}

void Joystick::pollEvents() {
    if (!joystick) {
        return;
    }

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_QUIT:
                exit(0);
                return;
            case SDL_JOYBUTTONDOWN:
                state.buttons[event.jbutton.button] = true;
                break;
            case SDL_JOYBUTTONUP:
                state.buttons[event.jbutton.button] = false;
                break;
            case SDL_JOYAXISMOTION:
                state.axes[event.jaxis.axis] = scaleAxisValue(event.jaxis.value);
                break;
        }
    }
}

JoystickState Joystick::getState() const {
    return state;
}