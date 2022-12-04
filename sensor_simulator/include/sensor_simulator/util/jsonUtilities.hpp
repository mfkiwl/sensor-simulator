//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                         JSON Utility Header                                      //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for JSON Utilities class                                                //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <dataTypes.hpp>
#include "nlohmann/json.hpp"
using json = nlohmann::json;

// JSON Utilities Class
class jsonUtilities {

    // Public Class Members/Functions
    public:
        
        /* @parseSensorConfig
            Inputs:
                fileName: std::string of config file name
            Outputs:
                sensorConfig: sensorSimData_t struct containing sensor simulation configuration data
            Description:
                Function which parses the sensor simulation configuration file
        */
        bool parseSensorConfig(const std::string fileName,
                               sensorSimData_t &sensorConfig);

    // Private Class Members/Function
    private:

        /* @parseJson
            Inputs:
                fileName: std::string of config file name
            Outputs:
                data: json object containing contents of file
            Description:
                Function which parses the JSON configuration file
        */
        bool parseJson(const std::string fileName,
                       json &data);

};
