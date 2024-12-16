#include <SDL2/SDL.h>
#include <map>
#include "CmdPanel.h"

struct JoystickState{
    std::map<int, bool> buttons;
    std::map<int, double> axes;
};

class Joystick : public CmdPanel{
    private:
        SDL_Joystick* joystick;
        JoystickState state;

    public:
        Joystick();
        ~Joystick();

        double scaleAxisValue(Sint16 value);
        void pollEvents();
        JoystickState getState() const;
};