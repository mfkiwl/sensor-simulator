//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           Sensor Simulator                                       //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    The main Sensor Simulator executable. This top level executable parses the          //
//              sensor simulation configuration set by the users and creates the ROS2 publishers    //
//              that outputs the simulated sensor data.                                             //                                                                         //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Standard Includes
#include <iostream>
#include <fstream>
#include <thread>
#include <queue>

// Sensor Sim Include Headers
#include "imuSensor.hpp"
#include "trajectory.hpp"
#include "jsonUtilities.hpp"

// ROS2 Include Headers
#include "rclcpp/rclcpp.hpp"

// Main Sensor Simulation Function
int main(int argc, char **argv) {

    // Initialize ROS2 Node
    rclcpp::init(argc, argv);

    // Unpack Inputs Arguments
    if (argc != 2) {
        std::cout << "[main] Invalid arguments - expected ./SensorSim <sensor config>" << std::endl;
        return 1;
    } else {
        std::cout << "[main] Sensor Configuration File: " << argv[1] << std::endl;
    }
    const std::string sensorConfig = argv[1];

    // Verify Sensor Configuration File Exists
    std::ifstream sc;
    sc.open(sensorConfig);
    if (!sc) {
        std::cout << "[main] Sensor Configuration File Specified does not exist" << std::endl;
        return 1;
    }

    // Parse Sensor Configuration File
    jsonUtilities ju;
    sensorSimData_t config;
    if (!ju.parseSensorConfig(sensorConfig, config)) {
        std::cout << "[main] Failed to parse Sensor Configuration File" << std::endl;
        return 1;
    }

    // Parse NED Trajectory CSV
    trajectory traj;
    if (!traj.parseNedTrajectory(config.trajectoryFile)) {
        std::cout << "[main] Failed to parse Trajectory File" << std::endl;
        return 1;
    }

    // Generate IMU Sensor Measurement History
    imuSensor imu_;
    if (config.imu.useImu) {
        std::cout << "[main] Adding IMU Sensor..." << std::endl;

    }

    // Generate Loosely-Coupled GPS Sensor Measurement History

    // Create ROS2 IMU Sensor Publisher

    // Create ROS2 GPS Sensor Publisher

    // Shutdown ROS2 Node
    rclcpp::shutdown();
    
    // Successful Return
    return 0;

}