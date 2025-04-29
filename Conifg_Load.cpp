#define _CRT_SECURE_NO_WARNINGS
#include"my_rtk.hpp"
#include"json.hpp"
#include<iostream>
#include<fstream>


using namespace std;
using namespace nlohmann;



/* Load Configuration File ------------------------------------------------------
* Load and parse RTK configuration parameters from a JSON file
* args   :
*          const string& filePath    I   Path to the configuration JSON file
*          config_&      config      O   Configuration structure to be populated
*
* return : bool                      Success (true) or failure (false)
*
* notes  : This function parses the following configuration parameters:
*          - Data source type (file or socket)
*          - File paths for base and rover data
*          - IP addresses and ports for network connections
*          - Output and log file paths
*          - RTK processing mode and rover mode
*          - GNSS settings (BDS GEO satellite usage, troposphere model)
*          - Processing parameters (elevation mask, ratio threshold)
*
*          Elevation mask is converted from degrees to radians
*-----------------------------------------------------------------------------*/
bool LoadConfig(const string& filePath, config_& config)
{
	// Open the configuration file
	ifstream config_file(filePath);
	if (!config_file.is_open()) {
		cerr << "Failed to open configuration File: " << filePath << endl;
		return false;
	}

	// Parse the JSON Format
	json configJson;
	try {
		config_file >> configJson;
	}
	catch (const json::parse_error& e) {
		cerr << "JSON Parse Error: " << e.what() << " at byte " << e.byte << endl;
		return false;
	}

	// Fill the configuration structure
	try {
		config.IsFileData = configJson["SOURCE_TYPE"];

		config.Base_File = configJson["Files"]["BASE_FILE"];
		config.RovFile = configJson["Files"]["ROVER_FILE"];


		config.OutputFile = configJson["Files"]["OUTPUT_FILE"];
		config.LogFile = configJson["Files"]["LOG_FILE"];

		config.BasNetIP = configJson["Base_SOCKET"]["IP"];
		config.RovNetIP = configJson["Rover_SOCKET"]["IP"];

		config.BasPort = configJson["Base_SOCKET"]["PORT"];
		config.RovPort = configJson["Rover_SOCKET"]["PORT"];


		config.RTKProcMode = configJson["SOULTION_TYPE"];
		config.RoverMode = configJson["ROVER_MODE"];

		config.Ban_GEO = configJson["BAN_GEO"];
		config.Trop_Model = configJson["TROPOSPHERE_MODEL"];
		config.Elevation_Mask = configJson["ELEVATION_MASK"];
		config.Ratio_Thres = configJson["RATIO_THRESHOLD"];

		if (configJson.contains("Visual_TCP")) {
			config.Visual_IP = configJson["Visual_TCP"]["VISUAL_IP"];
			config.Visual_Port = configJson["Visual_TCP"]["VISUAL_PORT"];
			config.Enable_Visual = true;
		}
		else {
			config.Enable_Visual = false;
		}

		// Convert to radians
		config.Elevation_Mask = config.Elevation_Mask * PI / 180.0;

	}
	catch (const json::exception& e) {
		cerr << "Failed to Parse the Configuration File: " << e.what() << endl;
		return false;
	}

	return true;
}