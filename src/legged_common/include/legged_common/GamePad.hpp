#ifndef GAMEPAD_HPP
#define GAMEPAD_HPP

#include "Eigen/Dense"
#include "legged_common/parameters.hpp"
// #include "functions.hpp"
#include <iostream>
// #include <allegro5/allegro.h>
// #include <allegro5/allegro_native_dialog.h>
#include <SDL2/SDL.h>
#include <cmath> 

class GamePad {
    private:
        double Kv = 0.1;
        double MIN_BODY_HEIGHT = 0.2;
        double MAX_BODY_HEIGHT = 0.4;
        double Vx_mean = 0.0, Vy_mean = 0.0;
        double cmdZc = initZc;
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        double Vyaw;
        double decreaseHeight = 0.0, increaseHeight = 0.0;

        bool standUp = false;
        int selectedGait = 1;

        SDL_Event event;
        SDL_GameController* controller = nullptr;

        double mapVal(double inVal, double inMax, double inMin, double outMax, double outMin);

    public:
        double joyCmd[22] = {0.0, 0.0, 0.0, 0.0, 0.0, initZc, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        bool close = false;
        bool walkEnable = false;
        int gait = 0;
        bool latsag_corr = false;

        ~GamePad(){
            if (controller) {
                SDL_GameControllerClose(controller);
                std::cout << "Gamepad disconnected." << std::endl;
            }
        };
        void intiSDL2();
        void gamepad();
        void callGamePad();
};

#endif // GAMEPAD_HPP