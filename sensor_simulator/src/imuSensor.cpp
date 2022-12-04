//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                          IMU Sensor Model Class                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    The IMU Sensor Model class implementation. Intializes the IMU model and then runs   //
//              an offline stochastic simulation of the outputs of the IMU to be published in the   //
//              main executable.                                                                    //
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include <iostream>
#include <thread>
#include "imuSensor.hpp"

// Initialize IMU Model
bool imuSensor::init(imuSensorSimData_t imuConfig) {

    //

    // Successful Return
    return true;

}

// Run Generate IMU Sensor Measurements
bool imuSensor::generateImuMeasurements(imuSensorSimData_t imuConfig) {
    
    // Initialize IMU Thread
    if (!init(imuConfig)) {
        std::cout << "[imuSensor::generateImuMeasurements] Failed to initialize IMU sensor model" << std::endl;
        return false;
    }

    // Main Loop to Generate IMU Measurements

    // Successful Return
    std::cout << "[imuSensor::generateImuMeasurements] Completed IMU Measurement Generation" << std::endl;
    return true;

}