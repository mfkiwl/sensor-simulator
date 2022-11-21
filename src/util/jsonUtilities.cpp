//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                         JSON Utilities Class                                     //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    The JSON Utilities class. This class implementation contains useful functions for   //
//              parsing the JSON configuration files used by the system.                            //                                                          //                                                                         //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include <iostream>
#include <thread>
#include "jsonUtilities.hpp"

// Parse Master Configuration File
bool jsonUtilities::parseSensorConfig(const std::string fileName,
                                      sensorSimData_t &sensorConfig) {

    // Read in File
    json config;
    if (!parseJson(fileName, config)) {
        std::cout << "[jsonUtilities::parseSensorConfig] Failed to parse Sensor Config" << std::endl;
        return false;
    }

    // Get IMU Data
    json imuData = config["imu"];
    imuSensorSimData_t imu;
    int useImuVal = imuData.value("use_imu", 0);
    if (useImuVal == 1) {
        imu.useImu = true;
    } else {
        imu.useImu = false;
    }
    imu.rate = imuData.value("rate", 1.0);
    json imuErrors = imuData["errors"];
    imu.accel_bias_repeatability = imuErrors.value["accel_bias_repeatability_rms_mg", 0.0];
    imu.accel_bias_instability = imuErrors.value["accel_bias_instability_rms_mg", 0.0]; 
    imu.accel_sf = imuErrors.value["accel_sf_sigma_ppm", 0.0];      
    imu.accel_mis = imuErrors.value["accel_mis_sigma_urad", 0.0];       
    imu.accel_vrw = imuErrors.value["accel_vrw_m_s_sqrtHr", 0.0];           
    imu.gyro_bias_repeatability = imuErrors.value["gyro_bias_repeatability_rms_mg", 0.0];
    imu.gyro_bias_instability = imuErrors.value["gyro_bias_instability_rms_mg", 0.0]; 
    imu.gyro_sf = imuErrors.value["gyro_sf_sigma_ppm", 0.0];     
    imu.gyro_mis = imuErrors.value["gyro_mis_sigma_urad", 0.0];         
    imu.gyro_arw = imuErrors.value["gyro_arw_deg_sqrtHr", 0.0];

    // Get Loosely Coupled GPS
    json lcGps = config["loosely-coupled-gps"];
    looselyCoupledGpsSensorSimData_t looselyCoupledGps;
    int useLCGpsVal = lcGps.value("use_gps", 0);
    if (useLCGpsVal == 1) {
        looselyCoupledGps.useLooselyCoupledGps = true;
    } else {
        looselyCoupledGps.useLooselyCoupledGps = false;
    }
    looselyCoupledGps.rate = lcGps.value("rate", 1.0);
    json lcGpsErrors = lcGps["errors"];
    looselyCoupledGps.level_pos_sigma = lcGpsErrors.value("level_pos_sigma_m", 0.0); 
    looselyCoupledGps.vertical_pos_sigma = lcGpsErrors.value("vertical_pos_sigma_m", 0.0);

    // Assemble Sensor Simulation Data
    sensorConfig.imu = imu;
    sensorConfig.looselyCoupledGps = looselyCoupledGps;
    json traj = config["trajectory"];
    sensorConfig.TrajectoryFile = traj.value("file","");

    // Successful Return
    return true;

}

// Parse JSON File
bool jsonUtilities::parseJson(const std::string fileName,
                              json &data) {

    // Read in File
    std::ifstream file(fileName);
    if (!file.good()) {
        std::cout << "[jsonUtilities::parseJson] Invalid JSON file path" << std::endl;
        return false;
    }

    // Parse File
    data = json::parse(file);

    // Successful Return
    return true;

}