//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     Trajectory Class Unit Testing                                //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Trajectory Class Unit Tests.                                                        //
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Statements
#include "gtest/gtest.h"
#include "util/trajectory.hpp"
#include "util/jsonUtilities.hpp"
#include <Eigen/Dense>
#include <string>

// Trajectory File Does Not Exist Unit Test
TEST(Trajectory, FileDne)
{

    // Create Trajectory Object
    trajectory traj;

    // Fail to Load Nonexistent File
    EXPECT_FALSE(traj.parseNedTrajectory("NonexistentFile.csv"));

}

// Load NED Trajectory File Unit Test
TEST(Trajectory, LoadNed)
{

    // Create JSON Utilities Object
    jsonUtilities ju;

    // Create IMU Calibration Data
    sensorSimData_t config;

    // Parse Master Config
    const std::string fileName = "../test/testData/master_config_test.json";

    // Successfully Parse Sensor Config 
    EXPECT_TRUE(ju.parseSensorConfig(fileName,config));

    // Create Trajectory Object
    trajectory traj;

    EXPECT_TRUE(traj.parseNedTrajectory(config.trajectoryFile));

}
