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
#include "looselyCoupledGpsSensor.hpp"
#include "util/trajectory.hpp"
#include "util/jsonUtilities.hpp"

// ROS2 Include Headers
#include "rclcpp/rclcpp.hpp"

// NavROS Include Headers
#include "nav_interfaces/msg/looselycoupledgps.hpp"

// Timing
using namespace std::chrono_literals;

// IMU Sensor Publisher

// Loosely Coupled GPS Sensor Publisher
class GpsPublisher : public rclcpp::Node {

    // Public 
    public:

	// GPS Publisher Constructor
        GpsPublisher(looselyCoupledGpsSensorSimData_t sensorData, nedTrajSensorSimData_t nedTraj) : 
		Node("loosely_coupled_gps"), 
		count_(0), 
		sensorData_(sensorData),
                nedTraj_(nedTraj) {
	    
	    // Create GPS Trajectory
	    lcGps_.init(sensorData_);
	    lcGps_.generateGpsMeasurements(nedTraj_, gpsTov_, gpsData_);

	    // Create GPS Publisher
            publisher_ = this->create_publisher<nav_interfaces::msg::Looselycoupledgps>("loosely_coupled_gps", 10);
	    timer_ = this->create_wall_timer(1000ms, std::bind(&GpsPublisher::timer_callback, this));
        }

    // Private
    private:

	// Timer Callback
	void timer_callback() {

	    // Create Message
	    auto message = nav_interfaces::msg::Looselycoupledgps();

	    // Fill Time of Validity
            message.tov = gpsTov_.front();
	    gpsTov_.pop();

	    // Fill LLA
	    Eigen::Vector3d lla = gpsData_.front();
	    message.latitude = lla[0];
	    message.longitude = lla[1];
	    message.altitude = lla[2];
	    gpsData_.pop();

	    // Publish Message
	    std::cout << "[GpsPublisher] Publishing GPS Message: tov =  " << message.tov << std::endl;
	    publisher_->publish(message);
	}

	// Class Variables
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<nav_interfaces::msg::Looselycoupledgps>::SharedPtr publisher_;
	size_t count_;
	looselyCoupledGpsSensor lcGps_;
	looselyCoupledGpsSensorSimData_t sensorData_;
	nedTrajSensorSimData_t nedTraj_;
	std::queue<int64_t> gpsTov_;
	std::queue<Eigen::Vector3d> gpsData_;

};

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

    // Generate Loosely-Coupled GPS Publisher
    looselyCoupledGpsSensor lcGps_;
    if (config.looselyCoupledGps.useLooselyCoupledGps) {
        std::cout << "[main] Adding Loosely Coupled GPS Sensor..." << std::endl;
        rclcpp::spin(std::make_shared<GpsPublisher>(config.looselyCoupledGps, traj.nedTraj_));
    }

    // Create ROS2 IMU Sensor Publisher

    // Shutdown ROS2 Node
    rclcpp::shutdown();
    
    // Successful Return
    return 0;

}
