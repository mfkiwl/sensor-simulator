//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Loosely Coupled GPS Sensor Model Class                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    The Loosely Coupled GPS Sensor Model class implementation. Intializes the GPS model //
//              and then runs an offline stochastic simulation of the outputs of the GPS to be      //
//              published in the main executable.                                                   //
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include <iostream>
#include <thread>
#include "looselyCoupledGpsSensor.hpp"

// Initialize GPS Sensor Model
bool looselyCoupledGpsSensor::init(looselyCoupledGpsSensorSimData_t gpsConfig) {

    // Set Class Variables
    rate_ = gpsConfig.rate;
    level_pos_sigma_ = gpsConfig.level_pos_sigma;
    vertical_pos_sigma_ = gpsConfig.vertical_pos_sigma;

    // Successful Return
    return true;

}

// Generate GPS Sensor Measurements
bool looselyCoupledGpsSensor::generateGpsMeasurements(nedTrajSensorSimData_t nedTraj,
		                                      std::vector<int64_t> gpsTov,
                                                      std::vector<Eigen::Vector3d> gpsData) {

    // Get GPS Time History
    gpsTov = nedTraj.tov;

    // Get GPS Measurement History
    gpsData = nedTraj.lla;

    // Successful Return
    std::cout << "[looselyCoupledGpsSensor::generateGpsMeasurements] Completed Loosely Coupled GPS Measurement Generation" << std::endl;
    return true;

}
