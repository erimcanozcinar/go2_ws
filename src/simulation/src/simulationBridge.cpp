#include "simulation/simulationBridge.hpp"

raisim::World world;
raisim::RaisimServer server(&world);


simulation::simulation(){
    initialConditions.resize(19); jointTorques.resize(18);
    initialConditions.setZero(); jointTorques.setZero();
    Fcon_LF.setZero(); Fcon_RF.setZero(); Fcon_LB.setZero(); Fcon_RB.setZero();
    Pcon_LF.setZero(); Pcon_RF.setZero(); Pcon_LB.setZero(); Pcon_RB.setZero();
}

simulation::~simulation(){
    server.killServer();
    rclcpp::shutdown();    
}

void simulation::init(){
    world.setTimeStep(period);
    world.setMaterialPairProp("steel", "steel", 0.95, 0.95, 0.001, 0.95, 0.001);
    world.setMaterialPairProp("steel", "rubber", 0.95, 0.15, 0.001, 0.95, 0.001);
    auto ground = world.addGround(0, "steel");
    // world.setGravity({0,0,0});

    robot = world.addArticulatedSystem("/home/erim/go2_ws/src/simulation/rsc/urdf/go2_2.urdf");
    robot->getCollisionBody("FL_calf/0").setMaterial("rubber");
    robot->getCollisionBody("FR_calf/0").setMaterial("rubber");
    robot->getCollisionBody("RL_calf/0").setMaterial("rubber");
    robot->getCollisionBody("RR_calf/0").setMaterial("rubber");

    initialConditions << 0, 0, 0.39, 1, 0, 0, 0, 0, 30*M_PI/180, -60*M_PI/180, 
                                                 0, 30*M_PI/180, -60*M_PI/180, 
                                                 0, 30*M_PI/180, -60*M_PI/180, 
                                                 0, 30*M_PI/180, -60*M_PI/180;
    robot->setGeneralizedCoordinate(initialConditions);   
        
    server.setMap("default");
    server.focusOn(robot);
    server.launchServer();
    raisim::MSLEEP(5000);
}

void simulation::run(){
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    auto node = rclcpp::Node::make_shared("raisim_joint_state_publisher");
    auto pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);
    std::vector<std::string> joint_names;
    for(int i = 0; i < 12; i++) {
        joint_names.push_back("joint_" + std::to_string(i));
    }

    while(rclcpp::ok()){
        RS_TIMED_LOOP(int(world.getTimeStep()*1e6));        
        t = world.getWorldTime();        
        dt = world.getTimeStep();

        contactDefinition();

        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = node->get_clock()->now();
        msg.name = joint_names;

        Eigen::VectorXd q = robot->getGeneralizedCoordinate().e().tail(12);
        Eigen::VectorXd dq = robot->getGeneralizedVelocity().e().tail(12);

        msg.position.assign(q.begin(), q.end());
        msg.velocity.assign(dq.begin(), dq.end());

        pub->publish(msg);
        rclcpp::spin_some(node);

        server.integrateWorldThreadSafe();
    }  
    std::cout << "end of simulation" << std::endl;
}

void simulation::contactDefinition(){
    for (auto& contact : robot->getContacts()) // LF:3, RF:2, LB:1, RB:0
        {
            if (contact.skip()) continue;
            if (robot->getBodyIdx("FL_calf") == contact.getlocalBodyIndex())
            {
                Fcon_LF = -contact.getContactFrame().e().transpose() * contact.getImpulse().e() / dt;
                Pcon_LF = contact.getPosition().e().transpose();
            }
            else if (robot->getBodyIdx("FR_calf") == contact.getlocalBodyIndex())
            {
                Fcon_RF = -contact.getContactFrame().e().transpose() * contact.getImpulse().e() / dt;
                Pcon_RF = contact.getPosition().e().transpose();
            }
            else if (robot->getBodyIdx("RL_calf") == contact.getlocalBodyIndex())
            {
                Fcon_LB = -contact.getContactFrame().e().transpose() * contact.getImpulse().e() / dt;
                Pcon_LB = contact.getPosition().e().transpose();
            }
            else if (robot->getBodyIdx("RR_calf") == contact.getlocalBodyIndex())
            {
                Fcon_RB = -contact.getContactFrame().e().transpose() * contact.getImpulse().e() / dt;
                Pcon_RB = contact.getPosition().e().transpose();
            }
        }
}


