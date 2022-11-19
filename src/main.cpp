//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           Sensor Simulator                                       //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    The main Sensor Simulator executable. This top level executable parses the          //
//              sensor simulation configuration set by the users creates the ROS2 publisher that    //
//              output the simulated sensor data.                                                   //                                                                         //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Standard Includes
#include <iostream>
#include <fstream>
#include <thread>
#include <queue>

// ROS2 Include Headers
#include "rclcpp/rclcpp.hpp"

// Main Sensor Simulation Function
int main(int argc, char **argv) {

    // Initialize ROS2 Node
    rclcpp::init(argc, argv);

    // Unpack Inputs Arguments
    if (argc != 2) {
        std::cout << "[main] Invalid arguments - expected ./SensorSim <master config>" << std::endl;
        return 1;
    } else {
        std::cout << "[main] Master Configuration File: " << argv[1] << std::endl;
    }
    const std::string masterConfig = argv[1];

    // Verify Master Configuration File Exists
    std::ifstream mc;
    mc.open(masterConfig);
    if (!mc) {
        std::cout << "[main] Master Configuration File Specified does not exist" << std::endl;
        return 1;
    } 

    // Create ROS2 IMU Sensor Publisher

    // Create ROS2 GPS Sensor Publisher

    // Shutdown ROS2 Node
    rclcpp::shutdown();
    
    // Successful Return
    return 0;

}