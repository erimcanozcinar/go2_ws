#include "simulation/main_sim.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    _sim.init();
    _sim.run();
    
    return 0;
}