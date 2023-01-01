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
		                                      std::queue<int64_t> &gpsTov,
                                                      std::queue<Eigen::Vector3d> &gpsData) {

    // Get GPS Measurement Time History
    for (unsigned int ii = 0; ii < nedTraj.tov.size(); ii++) {
        gpsTov.push(nedTraj.tov[ii]);
	gpsData.push(nedTraj.lla[ii]);
    }

    // Successful Return
    std::cout << "[looselyCoupledGpsSensor::generateGpsMeasurements] Completed Loosely Coupled GPS Measurement Generation" << std::endl;
    return true;

}
