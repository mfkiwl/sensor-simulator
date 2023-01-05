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

    // Initialization
    int64_t tCurr = nedTraj.tov[0];
    int64_t dt = (int64_t) ((1.0 / rate_) * 1.0e6);
    int idxLower = 0;
    int idxUpper = 1;

    // Main Loop to Generate IMU Measurements
    while (tCurr < nedTraj.tov[nedTraj.tov.size()-1]) {
        
	// Find Indices of Truth Trajectory around Current Time
	while (tCurr >= nedTraj.tov[idxUpper]) {
	    idxLower++;
	    idxUpper++;
	}

	// Get Upper and Lower Index Position/Velocity/Attitude and Time
	Eigen::Vector3d lowerLla = nedTraj.lla[idxLower];
	Eigen::Vector3d upperLla = nedTraj.lla[idxUpper];
	Eigen::Vector3d lowerVelNed = nedTraj.vNed[idxLower];
	Eigen::Vector3d upperVelNed = nedTraj.vNed[idxUpper];
	Eigen::Vector3d lowerRph = nedTraj.rph[idxLower];
	Eigen::Vector3d upperRph = nedTraj.rph[idxUpper];
	int64_t tovLower = nedTraj.tov[idxLower];
	int64_t tovUpper = nedTraj.tov[idxUpper];

        // Compute Lower Bound Rotations
	Eigen::Matrix3d lowerRN2E, lowerRE2J = Eigen::Matrix3d::Identity(3,3);
        if (!rot_.computeRNed2Ecef(lowerLla[0], lowerLla[1], lowerRN2E)) {
            std::cout << "[imuSensor::generateImuMeasurements] Unable to compute rotation from NED to ECEF from for lower bound" << std::endl;
	    return false;
	}
	std::vector<double> dateVecLower;
        if (!unixTimestampToDateVec(tovLower, dateVecLower)) {
            std::cout << "[imuSensor::generateImuMeasurements] Unable to convert timestamp to date vector" << std::endl;
	    return false;
	}
        std::string eopPath = "_deps/navfuse-src/test/testData/EOP-Last5Years.csv";
        if (!rot_.computeREcef2J2k(dateVecLower, eopPath, lowerRE2J)) {
            std::cout << "[imuSensor::generateImuMeasurements] Failed to compute rotation from ECEF to J2K inertial frame" << std::endl;
	    return false;
	}	

	// Compute Upper Bound Rotations
	

	// Rotate PVA from NED to J2K Inertial Frame

        // Interpolate Inertial Frame Data
	
	// Compute Gravity
	
	// Compute Truth Delta Velocity and Delta Thetas
	
	// Propagate IMU Errors Forward

	// Compute Delta Velocity and Delta Theta Noisy Measurements

	// Push Measurements to Output Queues

	// Update Current Time
	tCurr += dt;

    }

    // Successful Return
    std::cout << "[imuSensor::generateImuMeasurements] Completed IMU Measurement Generation" << std::endl;
    return true;

}
