#include "legged_common/GamePad.hpp"

GamePad::GamePad() {
    vBody.setZero();
    wBody.setZero();
    zCom = initZc;

    for(int i=0; i<14; i++) prev_button_state[i] = 0;
    for(int i=0; i<8; i++) prev_axes_state[i] = 0.0;
    
    joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, &GamePad::joyCallback, this);
}

void GamePad::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    vX = LPF(msg->axes[1], vX, 2*M_PI*0.2, 0.01);
    vY = LPF(msg->axes[0], vY, 2*M_PI*0.2, 0.01);
    wZ = LPF(msg->axes[3], wZ, 2*M_PI*0.2, 0.01);
    
    vBody << vX, 0.5*vY, 0.0;
    wBody << 0.0, 0.0, wZ;
    zCom = initZc;

    if(msg->buttons[9] == 1 && prev_button_state[9] == 0) { 
        walkEnable = !walkEnable;
        std::cout << (walkEnable ? "Walking mode enabled!" : "Walking mode disabled!") << std::endl;
    }

    if(fabs(msg->axes[6]) == 1 && prev_axes_state[6] == 0) {
        selectedGait -= msg->axes[6];
        if(selectedGait > 3) selectedGait = 1;
        else if(selectedGait < 1) selectedGait = 3;
        std::cout << "Selected gait: " << selectedGait << std::endl;
    }
    

    if(walkEnable) gait = selectedGait;
    else gait = 0;

    prev_button_state[9] = msg->buttons[9];
    prev_axes_state[6] = msg->axes[6];
}

void GamePad::reset() {
    walkEnable = false;
    gait = 0;
    selectedGait = 1;
    vBody.setZero();
    wBody.setZero();
    cmdJoy[0] = 0.0; cmdJoy[1] = 0.0; cmdJoy[2] = 0.0; cmdJoy[3] = initZc;
    
    for(int i=0; i<14; i++) prev_button_state[i] = 0;
    for(int i=0; i<8; i++) prev_axes_state[i] = 0.0;
}

double GamePad::mapVal(double inVal, double inMax, double inMin, double outMax, double outMin) { 
    double mappedVal = (inVal - inMin)*((outMax-outMin)/(inMax - inMin)) + outMin;
    return mappedVal;
}