//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                       IMU Sensor Model Header                                    //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for IMU Sensor Model class                                              //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <dataTypes.hpp>

// IMU Sensor Model Class
class imuSensor {

    // Public Class Members/Functions
    public:
        
        /* @generateImuMeasurements
            Inputs:
                imuConfig: imuSensorSimData_t struct containing IMU model parameters
            Outputs:

            Description:
                Function which takes in the IMU model parameters and trajectory and generates
                a time history of measurements.
        */
        bool generateImuMeasurements(imuSensorSimData_t imuConfig);

    // Private Class Members/Function
    private:

        /* @init
            Inputs:
                imuConfig: imuSensorSimData_t struct containing IMU model parameters
            Outputs:
            Description:
                Function which initializes the IMU sensor model.
        */
        bool init(imuSensorSimData_t imuConfig);

        // IMU Sensor Sim Data
        imuSensorSimData_t imuConfig_;
        

};
