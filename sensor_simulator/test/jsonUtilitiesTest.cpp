//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                      JSON Utilities Unit Testing                                 //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    JSON Utilities Class Unit Tests.                                                    //
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Statements
#include "gtest/gtest.h"
#include "util/jsonUtilities.hpp"
#include <Eigen/Dense>
#include <string>

// Parse Sensor Config Unit Test
TEST(ParseConfig, Master)
{

    // Create JSON Utilities Object
    jsonUtilities ju;

    // Create IMU Calibration Data
    sensorSimData_t config;

    // Parse Master Config
    const std::string fileName = "/home/parkerb7/nav_ws/src/sensor-simulator/sensor_simulator/test/testData/sensor_config.json";

    // Successfully Parse Sensor Config 
    EXPECT_TRUE(ju.parseSensorConfig(fileName,config));

    // Define Expected Results
    bool useImuTruth = true;
    double rateImuTruth = 100; 
    double accel_bias_repeatabilityTruth = 0.7; 
    double accel_bias_instabilityTruth = 0.04;
    double accel_sfTruth = 500; 
    double accel_misTruth = 0.2;
    double accel_vrwTruth = 0.1; 
    double gyro_bias_repeatabilityTruth = 15; 
    double gyro_bias_instabilityTruth = 5.0;
    double gyro_sfTruth = 1000;
    double gyro_misTruth = 0.2; 
    double gyro_arwTruth = 0.3;
    bool useLooselyCoupledGpsTruth = true;
    double rateLCGpsTruth = 1.0; 
    double level_pos_sigmaTruth = 3.0;
    double vertical_pos_sigmaTruth = 5.0;
    std::string trajectoryFileTruth = "/home/parkerb7/sensor-simulator/config/trajectories/sample_trajectory.csv"; 

    // Check Results - IMU
    EXPECT_EQ(useImuTruth, config.imu.useImu);
    EXPECT_EQ(rateImuTruth, config.imu.rate);
    EXPECT_EQ(accel_bias_repeatabilityTruth, config.imu.accel_bias_repeatability);
    EXPECT_EQ(accel_bias_instabilityTruth, config.imu.accel_bias_instability);
    EXPECT_EQ(accel_sfTruth, config.imu.accel_sf);
    EXPECT_EQ(accel_misTruth, config.imu.accel_mis);
    EXPECT_EQ(accel_vrwTruth, config.imu.accel_vrw);
    EXPECT_EQ(gyro_bias_repeatabilityTruth, config.imu.gyro_bias_repeatability);
    EXPECT_EQ(gyro_bias_instabilityTruth, config.imu.gyro_bias_instability);
    EXPECT_EQ(gyro_sfTruth, config.imu.gyro_sf);
    EXPECT_EQ(gyro_misTruth, config.imu.gyro_mis);
    EXPECT_EQ(gyro_arwTruth, config.imu.gyro_arw);
    
    // Check Results - Loosely Coupled GPS
    EXPECT_EQ(useLooselyCoupledGpsTruth, config.looselyCoupledGps.useLooselyCoupledGps);
    EXPECT_EQ(rateLCGpsTruth, config.looselyCoupledGps.rate);
    EXPECT_EQ(level_pos_sigmaTruth, config.looselyCoupledGps.level_pos_sigma);
    EXPECT_EQ(vertical_pos_sigmaTruth, config.looselyCoupledGps.vertical_pos_sigma);

    // Check Results - Trajectory
    EXPECT_EQ(trajectoryFileTruth, config.trajectoryFile);

}
