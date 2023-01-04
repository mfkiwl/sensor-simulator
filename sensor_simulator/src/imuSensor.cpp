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

    // Set Class Variables
    rate_ = imuConfig.rate;
    accel_bias_repeatability_rms_mg_ = imuConfig.accel_bias_repeatability;
	accel_bias_instability_rms_mg_ = imuConfig.accel_bias_instability;
	accel_sf_sigma_ppm_ = imuConfig.accel_sf;
	accel_mis_sigma_urad_ = imuConfig.accel_mis;
	accel_vrw_m_s_sqrtHr_ = imuConfig.accel_vrw;
	gyro_bias_repeatability_rms_deg_hr_ = imuConfig.gyro_bias_repeatability;
	gyro_bias_instability_rms_deg_hr_ = imuConfig.gyro_bias_instability;
	gyro_sf_sigma_ppm_ = imuConfig.gyro_sf;
	gyro_mis_sigma_urad_ = imuConfig.gyro_mis;
	gyro_arw_deg_sqrtHr_ = imuConfig.gyro_arw;

    // Successful Return
    return true;

}

// Run Generate IMU Sensor Measurements
bool imuSensor::generateImuMeasurements(nedTrajSensorSimData_t nedTraj,
			                            std::queue<int64_t> &imuTov,
				                        std::queue<Eigen::Vector3d> &dvData,
                                        std::queue<Eigen::Vector3d> &dThData) {
    
    // Random Number Generators
    std::default_random_engine generator;
    std::normal_distribution<double> sigAccelBiasRepeatability(0.0, accel_bias_repeatability_rms_mg_);
    std::normal_distribution<double> sigAccelBiasInstability(0.0, accel_bias_instability_rms_mg_);
    std::normal_distribution<double> sigAccelSf(0.0, accel_sf_sigma_ppm_);
    std::normal_distribution<double> sigAccelMis(0.0, accel_mis_sigma_urad_);
    std::normal_distribution<double> sigGyroBiasRepeatability(0.0, gyro_bias_repeatability_rms_deg_hr_);
    std::normal_distribution<double> sigGyroBiasInstability(0.0, gyro_bias_instability_rms_deg_hr_);
    std::normal_distribution<double> sigGyroSf(0.0, gyro_sf_sigma_ppm_);
    std::normal_distribution<double> sigGyroMis(0.0, gyro_mis_sigma_urad_);




    // Main Loop to Generate IMU Measurements

    // Successful Return
    std::cout << "[imuSensor::generateImuMeasurements] Completed IMU Measurement Generation" << std::endl;
    return true;

}