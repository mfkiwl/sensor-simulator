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

    // Random Number Generators
    std::default_random_engine generator;
    std::normal_distribution<double> noiseLevel(0.0, level_pos_sigma_);
    std::normal_distribution<double> noiseVert(0.0, vertical_pos_sigma_);

    // Initialize Noise and Measurements Variables
    double noiseN, noiseE, noiseAlt;
    double measLat, measLon, measAlt;
    double truthLat, truthLon, truthAlt;
    Eigen::Vector3d truthLla, measLla, rE;
    Eigen::Vector3d noiseNed, noiserE;

    // Get GPS Measurement Time History
    for (unsigned int ii = 0; ii < nedTraj.tov.size(); ii++) {

	// Generate Measurement Noise
	noiseN = noiseLevel(generator);
       	noiseE = noiseLevel(generator);
	noiseAlt = noiseVert(generator);

	// Get Truth LLA
	truthLla = nedTraj.lla[ii];
	truthLat = truthLla[0]; 
	truthLon = truthLla[1];
	truthAlt = truthLla[2];

	// Get Rotation from NED to ECEF
	Eigen::Matrix3d RN2E = Eigen::Matrix3d::Zero(3,3);
	if (!rot_.computeRNed2Ecef(truthLat, truthLon, RN2E)) {
            std::cout << "[looselyCoupledGpsSensor::generateGpsMeasurements] Failed to compute rotation from NED to ECEF" << std::endl;
	    return false;
	}

	// Get Truth Position in ECEF
	if (!rot_.lla2Ecef(truthLat, truthLon, truthAlt, rE)) {
            std::cout << "[looselyCoupledGpsSensor::generateGpsMeasurements] Failed to rotate truth position to ECEF" << std::endl;
            return false;
        }

	// Rotate Noise to ECEF
        noiseNed << noiseN, noiseE, -noiseAlt;
        noiserE = RN2E * noiseNed;	

	// Create Measurement in ECEF Frame
	rE += noiserE;
	
	// Compute LLA from ECEF Frame Measurement
	if (!rot_.ecef2Lla(rE, measLat, measLon, measAlt)) {
            std::cout << "[looselyCoupledGpsSensor::generateGpsMeasurements] Failed to compute LLA measurement from ECEF" << std::endl;
            return false;
        }
	measLla << measLat, measLon, measAlt;
	
	// Push Measurement to Queue
        gpsTov.push(nedTraj.tov[ii]);
	gpsData.push(measLla);
    }

    // Successful Return
    std::cout << "[looselyCoupledGpsSensor::generateGpsMeasurements] Completed Loosely Coupled GPS Measurement Generation" << std::endl;
    return true;

}
