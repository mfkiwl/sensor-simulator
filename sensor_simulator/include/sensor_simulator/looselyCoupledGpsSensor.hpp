//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     Loosely Coupled GPS Model Header                             //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for the Loosely Coupled GPS Model class                                 //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <dataTypes.hpp>

// Loosely Coupled GPS Sensor Model Class
class looselyCoupledGpsSensor {

    // Public Class Members/Functions
    public:
        
        /* @generateGpsMeasurements
            Inputs:
                gpsConfig: looselyCoupledGpsSensorSimData_t struct containing GPS model parameters
            Outputs:
                gpsTov: std::vector<int64_t> Vector containing GPS measurement UTC timestamps
                gpsData: std::vector<Eigen::Vector3d> Vector containing GPS LLA measurements
            Description:
                Function which takes in the GPS model parameters and trajectory and generates
                a time history of measurements at the specified rate.
        */
        bool generateGpsMeasurements(looselyCoupledGpsSensorSimData_t gpsConfig);

    // Private Class Members/Function
    private:

        /* @init
            Inputs:
                gpsConfig: looselyCoupledGpsSensorSimData_t struct containing GPS model parameters
            Outputs:
            Description:
                Function which initializes the GPS sensor model.
        */
        bool init(looselyCoupledGpsSensorSimData_t gpsConfig_);

        // GPS Sensor Sim Data
        looselyCoupledGpsSensorSimData_t gpsConfig_;
        

};
