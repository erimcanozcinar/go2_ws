#ifndef GAMEPAD_HPP
#define GAMEPAD_HPP

#include "Eigen/Dense"
#include "legged_common/parameters.hpp"
#include "legged_common/matTools.hpp"
#include <iostream>
#include <cmath> 

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

class GamePad {
    private:
        ros::NodeHandle nh;
        ros::Subscriber joy_sub;

        int prev_button_state[14];
        double prev_axes_state[8];

        double vX, vY, wZ;

        bool walkEnable = false;
        int selectedGait = 1;

        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
        double mapVal(double inVal, double inMax, double inMin, double outMax, double outMin);

        GamePad();
        GamePad(const GamePad&) = delete;
        GamePad& operator=(const GamePad&) = delete;

    public:
        double cmdJoy[4] = {0.0, 0.0, 0.0, initZc};
        Eigen::Vector3d vBody, wBody;
        double zCom;
        int gait = 0;

        ~GamePad(){}

        static GamePad& getInstance() {
            // Sadece ilk çağrılışta oluşturulur, diğerlerinde aynı adresi döndürür.
            static GamePad instance; 
            return instance;
        }

        void reset();
};

#endif // GAMEPAD_HPP