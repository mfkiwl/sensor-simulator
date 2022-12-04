//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                             Trajectory Class                                     //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    The Trajectory class implementation. Contains function which take in the user       //
//              defined trajectories and parses and extracts the data for the sensor simulator.     //
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include <iostream>
#include <thread>
#include "util/trajectory.hpp"

// Initialize IMU Model
bool trajectory::parseNedTrajectory(const std::string nedTrajFile) {

    // Open CSV File
    std::ifstream infile(nedTrajFile);
    if (!infile.good()) {
        std::cout << "[trajectory::parseNedTrajectory] Invalid Trajectory file path" << std::endl;
        return false;
    }
    std::fstream file;
    file.open(nedTrajFile);
    std::string line, data;

    // Get Rows
    int64_t tov;
    double lat, lon, alt;
    double roll, pitch, hdg;
    double vN, vE, vD;
    std::getline(file, line, '\n');
    while (std::getline(file, line, '\n')) {

        // Set String Stream
        int temp = 0;
        std::stringstream str(line);

        // Get Columns
        while (std::getline(str, data, ',')) {

            // Get Data
            if (temp == 0) {
                tov = std::stoi(data);
                nedTraj_.tov.push_back(tov); 
            } else if (temp == 1) {
                lat = std::stod(data);
            } else if (temp == 2) {
                lon = std::stod(data);
            } else if (temp == 3) {
                alt = std::stod(data);
                Eigen::Vector3d lla;
                lla << lat, lon, alt;
                nedTraj_.lla.push_back(lla);
            } else if (temp == 4) {
                roll = std::stod(data);
            } else if (temp == 5) {
                pitch = std::stod(data);
            } else if (temp == 6) {
                hdg = std::stod(data);
                Eigen::Vector3d rph;
                rph << roll, pitch, hdg;
                nedTraj_.rph.push_back(rph);
            } else if (temp == 7) {
                vN = std::stod(data);
            } else if (temp == 8) {
                vE = std::stod(data);
            } else if (temp == 9) {
                vD = std::stod(data);
                Eigen::Vector3d vNed;
                vNed << vN, vE, vD;
                nedTraj_.vNed.push_back(vNed);
            } 

            // Increment Temporary Iterator
            temp++;
        }
    }

    // Close File
    file.close();

    // Successful Return
    return true;

}
