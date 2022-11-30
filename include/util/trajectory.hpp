//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                       Trajectory Class Header                                    //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for the Trajectory class                                                //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <sstream>
#include <deque>
#include <memory>
#include <algorithm>
#include <unistd.h>
#include <sys/stat.h>
#include <Eigen/Dense>
#include <dataTypes.hpp>

// Trajectory Class
class trajectory {

    // Public Class Members/Functions
    public:
        
        /* @parseNedTrajectory
            Inputs:
                nedTrajFile: const std::string of NED trajectory csv 
            Outputs:
            Description:
                Function which takes in a string of the NED trajectory CSV and parses
                the file to extract the trajectory
        */
        bool parseNedTrajectory(const std::string nedTrajFile);

    // Private Class Members/Function
    private:

        // NED Trajectory Data
        nedTrajSensorSimData_t nedTraj_;

};
