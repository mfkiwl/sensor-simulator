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

    //

    // Successful Return
    return true;

}

// Generate GPS Sensor Measurements
bool looselyCoupledGpsSensor::generateGpsMeasurements(looselyCoupledGpsSensorSimData_t gpsConfig) {
    
    // Initialize GPS Model
    if (!init(gpsConfig)) {
        std::cout << "[looselyCoupledGpsSensor::generateGpsMeasurements] Failed to initialize Loosely Coupled GPS sensor model" << std::endl;
        return false;
    }

    // Main Loop to Generate GPS Measurements

    // Successful Return
    std::cout << "[looselyCoupledGpsSensor::generateGpsMeasurements] Completed Loosely Coupled GPS Measurement Generation" << std::endl;
    return true;

}