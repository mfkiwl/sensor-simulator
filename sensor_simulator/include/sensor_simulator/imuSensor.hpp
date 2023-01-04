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
#include <queue>
#include <random>
#include <Eigen/Dense>
#include <Rotations.hpp>
#include <dataTypes.hpp>

// IMU Sensor Model Class
class imuSensor {

    // Public Class Members/Functions
    public:
        
        /* @init
            Inputs:
                imuConfig: imuSensorSimData_t struct containing IMU model parameters
            Outputs:
            Description:
                Function which initializes the IMU sensor model.
        */
        bool init(imuSensorSimData_t imuConfig);
        
        /* @generateImuMeasurements
            Inputs:
	            nedTraj: nedTrajSensorSimData_t containing NED truth trajectory
            Outputs:
                imuTov: std::queue<int64_t> FIFO queue containing IMU measurement UTC timestamps
                dVData: std::queue<Eigen::Vector3d> FIFO queue containing IMU delta velocity measurements
                dThData: std::queue<Eigen::Vector3d> FIFO queue containing IMU delta theta measurements
        */
        bool generateImuMeasurements(nedTrajSensorSimData_t nedTraj,
			                         std::queue<int64_t> &imuTov,
				                     std::queue<Eigen::Vector3d> &dvData,
                                     std::queue<Eigen::Vector3d> &dThData);

    // Private Class Members/Function
    private:

        // IMU Sensor Sim Model Parameters
        double rate_;
	    double accel_bias_repeatability_rms_mg_;
		double accel_bias_instability_rms_mg_;
		double accel_sf_sigma_ppm_;
		double accel_mis_sigma_urad_;
		double accel_vrw_m_s_sqrtHr_;
		double gyro_bias_repeatability_rms_deg_hr_;
		double gyro_bias_instability_rms_deg_hr_;
		double gyro_sf_sigma_ppm_;
		double gyro_mis_sigma_urad_;
		double gyro_arw_deg_sqrtHr_;

	    // NavUtils Rotations
	    Rotations rot_;
        
};
