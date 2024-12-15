#include <SDL2/SDL.h>
#include <map>

struct JoystickState{
    std::map<int, bool> buttons;
    std::map<int, double> axes;
};

class Joystick{
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