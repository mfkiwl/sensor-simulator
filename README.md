![image](sensor_simulator/docs/SensorSimLogo.PNG)

# A Flexible Software Based ROS2 Sensor Simulation Package
The Sensor-Simulator is a ROS2 based sensor simulation publisher capable of taking user-defined trajectories in the csv format and simulating a variety of sensor measurements. Currently, the package support inertial navigation and loosely-coupled GPS sensors. The Sensor-Simulator is intended to be used in a software-in-the-loop architecture for testing navigation systems. The package was designed to support the GPS-INS (https://github.com/ParkerBarrett959/GPS-INS) simulation and testing capabilities, but can be easily extended for use in other ROS2 based navigation software projects.

# Dependencies
* C++ 11 (or greater) <br />
* ROS2 <br />
* CMake (3.22.0 or greater) <br />
* Eigen (3.3 or greater) <br />
* GoogleTest <br />
* NavFuse: https://github.com/ParkerBarrett959/NavFuse <br />
* NavROS: https://github.com/ParkerBarrett959/NavROS <br />
* nlohmann JSON

# Build
```
# Create ROS2 Workspace
cd ~
mkdir ws/src/

# Clone Relevant ROS2 Packages - Sensor Simulator and NavROS
cd ws/src/
git clone https://github.com/ParkerBarrett959/sensor-simulator.git
git clone https://github.com/ParkerBarrett959/NavROS.git

# Build using Colcon Build Tools
cd ~/ws/
colcon build
```

# Configuration File Setup
Prior to running the Sensor Simulator executable, you must edit the system configuration files to match paths appropriately, select user-defined trajectories, and to alter sensor noise characteristics. The top level configuration file sets the sensors you would like to actively use, their noise characteristics and specifies the file location of the trajectory from which measurements are generated. To edit:
* Change directory into the sensor-simulator/sensor_simulator/config/ directory
* Open the file sensor_config.json and edit as desired
* It is recommended to add an absolute path to the trajectory csv file - for example, the file version controlled contains the path to the file (inside the ROS2 workspace) on the author's local machine. 

After the configuration file is setup, the trajectory can be altered. Currently, csv files in the format given in the example trajectory included with the repository are supported. The sample trajectory file is in the sensor-simulation/sensor_simulator/config/trajectories/ directory.

# Run Executable

To run the executable, you must first change directories into the ROS2 workspace build location. This is the location where the binaries are output after running colcon build, and can be found in the "build" directory immediately inside the top level ROS2 workspace.
```
# Move to Sensor Simulator Build Directory
cd ~/ws/build/sensor_simulator/

# Run the executable with the command line argument being the absolute path to the sensor_config.json file on your local machine
./SensorSim </path/to>/ws/src/sensor-simulator/sensor_simulator/config/sensor_config.json
```

Note that the software contains checks to make sure the specified configuration files exist. If the file does not exist, the program will display the error to the console and terinate prematurely. If this happens, be sure the file path is exactly correct and you are using absolute, not relative file paths.

# Run Unit Tests
Similarly to running the executable, running the unit tests must be done from the ROS2 workspace build directory. There is also a special unit test sensor_config.json file which should be used when running the tests. This file resides in the test/testData/ directory and should not be edited EXCEPT you must edit the trajectory file absolute path to point to the file on your local machine. The current filepath matches that of the author but will throw an error if not adjusted prior to running. 
```
# Move to Sensor Simulator Build Directory
cd ~/ws/build/sensor_simulator/

# Run the executable with the command line argument being the absolute path to the sensor_config.json test file on your local machine
./SensorSim </path/to>/ws/src/sensor-simulator/sensor_simulator/test/testData/sensor_config.json
```
