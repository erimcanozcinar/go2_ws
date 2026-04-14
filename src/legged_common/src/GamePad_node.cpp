#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <SDL2/SDL.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "sdl2_joy_node");
    ros::NodeHandle nh;
    
    // Publisher for the standard /joy topic
    ros::Publisher joy_pub = nh.advertise<sensor_msgs::Joy>("/joy", 1);

    // Initialize the SDL Joystick subsystem
    if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
        ROS_ERROR("SDL could not initialize! SDL Error: %s", SDL_GetError());
        return -1;
    }

    // Check for connected joysticks
    if (SDL_NumJoysticks() < 1) {
        ROS_ERROR("No joysticks connected! Make sure usbipd is attached.");
        SDL_Quit();
        return -1;
    }

    // Open the first connected joystick (index 0)
    SDL_Joystick* joystick = SDL_JoystickOpen(0);
    if (joystick == NULL) {
        ROS_ERROR("Unable to open game controller! SDL Error: %s", SDL_GetError());
        SDL_Quit();
        return -1;
    }

    ROS_INFO("Successfully connected to gamepad: %s", SDL_JoystickName(joystick));

    // Get the hardware capabilities
    int num_axes = SDL_JoystickNumAxes(joystick);
    int num_buttons = SDL_JoystickNumButtons(joystick);
    int num_hats = SDL_JoystickNumHats(joystick);
    
    sensor_msgs::Joy joy_msg;
    joy_msg.axes.resize(num_axes + (num_hats * 2));
    joy_msg.buttons.resize(num_buttons);

    // Set the loop rate to 50 Hz
    ros::Rate loop_rate(100); 

    while (ros::ok()) {
        // Tell SDL to poll the hardware for the latest state
        SDL_JoystickUpdate();

        // Read and normalize axes (-1.0 to 1.0)
        // SDL raw axes range from -32768 to 32767
        for (int i = 0; i < num_axes; ++i) {
            // We use 32767.0 to avoid dividing by zero or getting values > 1.0
            float normalized_axis = SDL_JoystickGetAxis(joystick, i) / 32767.0;
            
            // Cap at -1.0 to 1.0 just in case of hardware quirks
            if (normalized_axis > 1.0) normalized_axis = 1.0;
            if (normalized_axis < -1.0) normalized_axis = -1.0;
            
            joy_msg.axes[i] = normalized_axis;
        }

        // Read hats (0-3)
        for (int i = 0; i < num_hats; ++i) {
            Uint8 hat = SDL_JoystickGetHat(joystick, i);
            float hat_x = 0.0;
            float hat_y = 0.0;
            if (hat & SDL_HAT_LEFT)  hat_x = -1.0;
            if (hat & SDL_HAT_RIGHT) hat_x =  1.0;
            if (hat & SDL_HAT_UP)    hat_y =  1.0; 
            if (hat & SDL_HAT_DOWN)  hat_y = -1.0;
            joy_msg.axes[num_axes + (i * 2)] = hat_x;       // D-Pad Yatay (X)
            joy_msg.axes[num_axes + (i * 2) + 1] = hat_y;   // D-Pad Dikey (Y)
        }

        // Read buttons (0 or 1)
        for (int i = 0; i < num_buttons; ++i) {
            joy_msg.buttons[i] = SDL_JoystickGetButton(joystick, i);
        }

        // Add timestamp and publish
        joy_msg.header.stamp = ros::Time::now();
        joy_pub.publish(joy_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Clean up on exit
    SDL_JoystickClose(joystick);
    SDL_Quit();
    
    return 0;
}