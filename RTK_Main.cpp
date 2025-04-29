#define _CRT_SECURE_NO_WARNINGS
#include"my_rtk.hpp"
#include"json.hpp"
#include"RTKVisualServer.h"
#include<iostream>
#include <sstream>
#include<fstream>
#include<iomanip>

#include<Eigen/Core>
#include<Eigen/Dense>

using namespace std;




int RTK_LS(config_& config)
{
	// Create Data Objects
	RTK_RAW Raw = RTK_RAW();
	XYZ_Coord Bas_pos(0, 0, 0);
	XYZ_Coord Rov_pos(0, 0, 0);

	// Create Input Handle
	//InputHandle base_src(SOURCE_FILE_1);
	//InputHandle rover_src(SOURCE_FILE_2);

	//InputHandle base_src("47.114.134.129", 7190);
	//InputHandle rover_src("8.148.22.229", 4002);

	std::unique_ptr<InputHandle> base_src;
	std::unique_ptr<InputHandle> rover_src;

	if (config.IsFileData == 1) {
		// From File
		base_src = std::make_unique<InputHandle>(config.Base_File.c_str());
		rover_src = std::make_unique<InputHandle>(config.RovFile.c_str());
	}
	else {
		// FromSocket
		base_src = std::make_unique<InputHandle>(config.BasNetIP.c_str(), config.BasPort);
		rover_src = std::make_unique<InputHandle>(config.RovNetIP.c_str(), config.RovPort);
	}


	// Create Output FileStream
	ofstream output_file(config.OutputFile);
	ofstream log_file(config.LogFile);


	// Create Terminal MessageLog
	ostringstream log_msg;
	log_msg << "POSITIONING TYPE : LEAST SQUARE RELATIVE POSITONING\n\n\n";


	// Counter for calulating Fix Ratio
	int all_count = 0;
	int fix_count = 0;

	// Loop to Fetch Data
	while (true)
	{
		// Sleep for Waiting Data In Realtime Positioning
		if (config.IsFileData == 0)
		{
			Sleep(1000);
		}

		// Show the Terminal Message If Need
		cout << log_msg.str() << endl;
		log_file << log_msg.str() << endl;
		log_msg.str("");


		// Synchronous Status
		int syn_status = GetSynObs(*rover_src, *base_src, Raw);

		// For Differnt Status
		if (syn_status == -1)
		{
			// Data Source Already Readed
			break;
		}

		if (syn_status == 0)
		{
			// Base_Obs much more Later than Rover
			continue;
		}


		/* ------------- Syn_states == 1 ---> Start RTK Progress ------------- */
		log_msg << "----------------------------------------------------\n";
		log_msg << "Rover Time: " << Raw.RovEpk.Time.Week << "  " << Raw.RovEpk.Time.SecOfWeek << endl;
		log_msg << "Base Time:  " << Raw.BasEpk.Time.Week << "  " << Raw.BasEpk.Time.SecOfWeek << endl;



		Check_Raw_Obs(Raw.BasEpk, Raw.RovEpk);

		if (!SPP(Raw.BasEpk, Raw.GpsEph, Raw.BdsEph, Bas_pos, config))
		{
			log_msg << "Not Enough Observation For SPP of BaseStation!" << endl;
			continue;
		}

		if (!SPP(Raw.RovEpk, Raw.GpsEph, Raw.BdsEph, Rov_pos, config))
		{
			log_msg << "Not Enough Observation For SPP of Rover!" << endl;
			continue;
		}


		Construct_SD_Obs(Raw.BasEpk, Raw.RovEpk, Raw.SdObs);
		Check_SD_Obs(Raw.SdObs);


		all_count++;

		if (!Seletct_Ref_Sat(Raw.BasEpk, Raw.RovEpk, Raw.SdObs, Raw.DDObs))
		{
			// No Reference Satellite Selected --> Output SPP Solution
			output_file << Form_Pos_String(Raw, 5);

			log_msg << "No Reference Satellite Selected!" << endl;
			log_msg << "Rover SPP Position: " << fixed << setprecision(4) <<
				Raw.RovEpk.my_result.Pos.x << "  " << Raw.RovEpk.my_result.Pos.y << "  " << Raw.RovEpk.my_result.Pos.z << endl;
			continue;
		}


		Eigen::VectorXd FloatAmb = Eigen::VectorXd::Zero(2 * Raw.DDObs.Sats);
		Eigen::MatrixXd Qxx = Eigen::MatrixXd::Zero(3 + 2 * Raw.DDObs.Sats, 3 + 2 * Raw.DDObs.Sats);


		if (!RTK_Float(Raw, FloatAmb, Qxx))
		{
			// Fail to have RTK Soultion --> Output SPP Solution
			output_file << Form_Pos_String(Raw, 5);

			log_msg << "No Enough Observation for RTK_Float Solution!" << endl;
			log_msg << "Rover SPP Position: " << fixed << setprecision(4) <<
				Raw.RovEpk.my_result.Pos.x << "  " << Raw.RovEpk.my_result.Pos.y << "  " << Raw.RovEpk.my_result.Pos.z << endl;
			continue;
		}


		// Check if RTK Solution converge into an odd place --- Rov_pos is the SPP Solution
		if (Calc_XYZ_dist(Raw.RovEpk.my_result.Pos, Rov_pos) > 1e3)
		{
			// Fail to have RTK Soultion --> Output SPP Solution
			output_file << Form_Pos_String(Raw, 5);

			log_msg << "RTK Solution is Unvaild!" << endl;
			log_msg << "Rover SPP Position: " << fixed << setprecision(4) <<
				Raw.RovEpk.my_result.Pos.x << "  " << Raw.RovEpk.my_result.Pos.y << "  " << Raw.RovEpk.my_result.Pos.z << endl;
			continue;
		}


		if (!RTK_FIX(Raw, FloatAmb, Qxx, config.Ratio_Thres))
		{
			// Failed to have RTK_FIX Solution --> Output RTK Float Solution
			output_file << Form_Pos_String(Raw, 2);

			log_msg << "WARNING: Failed to Fix the Ambiguity! Ratio = " << Raw.DDObs.Ratio << endl;
			log_msg << "Float Solution of BaseLine: " << Raw.DDObs.dPos.x << "  " << Raw.DDObs.dPos.y << "  " << Raw.DDObs.dPos.z << endl;
			log_msg << "Rover RTK Float Position  : " << fixed << setprecision(4) << Raw.RovEpk.my_result.Pos.x << "  " << Raw.RovEpk.my_result.Pos.y << "  " << Raw.RovEpk.my_result.Pos.z << endl;
			log_msg << "The Satellite Information : " << "Obs->" << Raw.RovEpk.SatNum << "   DDObs->" <<
				Raw.DDObs.Sats << "  GPS->" << Raw.DDObs.DDSatNum[0] << "  BDS->" << Raw.DDObs.DDSatNum[1] << endl;
			continue;
		}


		fix_count++;
		output_file << Form_Pos_String(Raw, 1);


		log_msg << "Fixed Solution of BaseLine: " << Raw.DDObs.dPos.x << "  " << Raw.DDObs.dPos.y << "  " << Raw.DDObs.dPos.z << endl;
		log_msg << "Fixed Solution of Rover   : " << fixed << setprecision(4) << Raw.RovEpk.my_result.Pos.x << "  " << Raw.RovEpk.my_result.Pos.y << "  " << Raw.RovEpk.my_result.Pos.z << endl;
		log_msg << "The Satellite Information : " << "Obs->" << Raw.RovEpk.SatNum << "   DDObs->" <<
			Raw.DDObs.Sats << "  GPS->" << Raw.DDObs.DDSatNum[0] << "  BDS->" << Raw.DDObs.DDSatNum[1] << endl;

		log_msg << "Solution Count: " << all_count << "  Fix Ratio: " << (double)fix_count / all_count * 100 << "%" << endl;
	}


	return 1;

}




int RTK_KF(config_& config)
{
	// Create Data Objects
	RTK_RAW Raw = RTK_RAW();
	RTK_EKF EKF = RTK_EKF();

	XYZ_Coord Bas_pos(0, 0, 0);
	XYZ_Coord Rov_pos(0, 0, 0);

	// Create Input Handle
	//InputHandle base_src(SOURCE_FILE_1);
	//InputHandle rover_src(SOURCE_FILE_2);

	//InputHandle base_src("47.114.134.129", 7190);
	//InputHandle rover_src("8.148.22.229", 4002);

	std::unique_ptr<InputHandle> base_src;
	std::unique_ptr<InputHandle> rover_src;

	if (config.IsFileData == 1) {
		// From File
		base_src = std::make_unique<InputHandle>(config.Base_File.c_str());
		rover_src = std::make_unique<InputHandle>(config.RovFile.c_str());
	}
	else {
		// FromSocket
		base_src = std::make_unique<InputHandle>(config.BasNetIP.c_str(), config.BasPort);
		rover_src = std::make_unique<InputHandle>(config.RovNetIP.c_str(), config.RovPort);
	}


	// Create Output FileStream
	ofstream output_file(config.OutputFile);
	ofstream log_file(config.LogFile);


	// Create Rtkplot.exe's TCP Server
	std::unique_ptr<RTKVisualServer> visualServer;
	if (config.Enable_Visual) {
		visualServer = std::make_unique<RTKVisualServer>(config.Visual_IP.c_str(), config.Visual_Port);
		if (!visualServer->start()) {
			std::cerr << "Failed to start RTK Visual TCP Server" << std::endl;
		}
	}


	// Create Terminal MessageLog
	ostringstream log_msg;
	log_msg << "POSITIONING TYPE : EXTENT KALMAN FILTER RELATIVE POSITONING\n\n\n";

	// Counter for calulating Fix Ratio
	int all_count = 0;
	int fixed_count = 0;


	// Loop to Fetch Data
	while (true)
	{
		// Sleep for Waiting Data In Realtime Positioning
		if (config.IsFileData == 0)
		{
			Sleep(1000);
		}



		// Show the Terminal Message If Need
		cout << log_msg.str() << endl;
		log_file << log_msg.str() << endl;
		log_msg.str("");


		// Synchronous Status
		int syn_status = GetSynObs(*rover_src, *base_src, Raw);


		// For Differnt Status
		if (syn_status == -1)
		{
			// Data Source Already Readed
			break;
		}

		if (syn_status == 0)
		{
			// Base_Obs much more Later than Rover
			continue;
		}

		// Set Rover Mode: 0: Kinematic, 1: Static
		if (config.RoverMode == 0)
		{
			EKF.state = 0; // Kinematic
		}
		else
		{
			EKF.state = 1; // Static
		}


		/* -------- Syn_states == 1 ---> Start RTK Process -------- */
		log_msg << "----------------------------------------------------\n";
		log_msg << "Rover Time: " << Raw.RovEpk.Time.Week << "  " << Raw.RovEpk.Time.SecOfWeek << endl;
		log_msg << "Base Time:  " << Raw.BasEpk.Time.Week << "  " << Raw.BasEpk.Time.SecOfWeek << endl;


		Check_Raw_Obs(Raw.BasEpk, Raw.RovEpk);

		if (!SPP(Raw.BasEpk, Raw.GpsEph, Raw.BdsEph, Bas_pos, config))
		{
			log_msg << "Not Enough Observation For SPP of BaseStation!" << endl;
			continue;
		}

		if (!SPP(Raw.RovEpk, Raw.GpsEph, Raw.BdsEph, Rov_pos, config))
		{
			log_msg << "Not Enough Observation For SPP of BaseStation!" << endl;
			continue;
		}

		Construct_SD_Obs(Raw.BasEpk, Raw.RovEpk, Raw.SdObs);
		Check_SD_Obs(Raw.SdObs);

		if (!Seletct_Ref_Sat(Raw.BasEpk, Raw.RovEpk, Raw.SdObs, Raw.DDObs))
		{
			// No Reference Satellite Selected --> Output SPP Solution
			string posString = Form_Pos_String(Raw, 5);

			output_file << posString;

			if (config.Enable_Visual && visualServer) {
				visualServer->addData(posString);
			}

			log_msg << "No Reference Satellite Selected!" << endl;
			log_msg << "Rover SPP Position: " << fixed << setprecision(4) <<
				Raw.RovEpk.my_result.Pos.x << "  " << Raw.RovEpk.my_result.Pos.y << "  " << Raw.RovEpk.my_result.Pos.z << endl;
			continue;
		}



		/* -------- RTK Raw Data Process Finish and Start EKF Update -------- */

		// Initialize EKF When Get the First Fixed Solution
		if (EKF.isInit == false)
		{
			// For Interger Ambiguity Resolution
			Eigen::VectorXd FloatAmb = Eigen::VectorXd::Zero(2 * Raw.DDObs.Sats);
			Eigen::MatrixXd Qxx = Eigen::MatrixXd::Zero(3 + 2 * Raw.DDObs.Sats, 3 + 2 * Raw.DDObs.Sats);


			log_msg << "Here is the Initializtion of EKF Positioning" << endl;

			if (!RTK_Float(Raw, FloatAmb, Qxx))
			{
				log_msg << "No Enough Observation for RTK_Float Solution!" << endl;
				log_msg << "Rover SPP Position: " << fixed << setprecision(4) <<
					Raw.RovEpk.my_result.Pos.x << "  " << Raw.RovEpk.my_result.Pos.y << "  " << Raw.RovEpk.my_result.Pos.z << endl;

				continue;
			}

			if (Calc_XYZ_dist(Raw.RovEpk.my_result.Pos, Rov_pos) > 1e3)
			{
				// Fail to have RTK Soultion --> Output SPP Solution
				string posString = Form_Pos_String(Raw, 5);

				output_file << posString;

				if (config.Enable_Visual && visualServer) {
					visualServer->addData(posString);
				}

				log_msg << "RTK Solution is Unvaild!" << endl;
				log_msg << "Rover SPP Position: " << fixed << setprecision(4) <<
					Raw.RovEpk.my_result.Pos.x << "  " << Raw.RovEpk.my_result.Pos.y << "  " << Raw.RovEpk.my_result.Pos.z << endl;

				continue;
			}


			if (!RTK_FIX(Raw, FloatAmb, Qxx, config.Ratio_Thres))
			{
				// Failed to have RTK_FIX Solution --> Output RTK Float Solution
				string posString = Form_Pos_String(Raw, 2);

				output_file << posString;

				if (config.Enable_Visual && visualServer) {
					visualServer->addData(posString);
				}

				log_msg << "WARNING: Failed to Fix the Ambiguity! Ratio = " << Raw.DDObs.Ratio << endl;
				log_msg << "Float Solution of BaseLine: " << Raw.DDObs.dPos.x << "  " << Raw.DDObs.dPos.y << "  " << Raw.DDObs.dPos.z << endl;
				log_msg << "Rover RTK Float Position  : " << fixed << setprecision(4) <<
					Raw.RovEpk.my_result.Pos.x << "  " << Raw.RovEpk.my_result.Pos.y << "  " << Raw.RovEpk.my_result.Pos.z << endl;

				continue;
			}

			string posString = Form_Pos_String(Raw, 1);

			output_file << posString;

			if (config.Enable_Visual && visualServer) {
				visualServer->addData(posString);
			}

			log_msg << "Fixed Solution of BaseLine: " << Raw.DDObs.dPos.x << "  " << Raw.DDObs.dPos.y << "  " << Raw.DDObs.dPos.z << endl;
			log_msg << "Fixed Solution of Rover   : " << fixed << setprecision(4) <<
				Raw.RovEpk.my_result.Pos.x << "  " << Raw.RovEpk.my_result.Pos.y << "  " << Raw.RovEpk.my_result.Pos.z << endl;


			// Initialize EKF, If Get the Fixed Solution
			RTK_EKF_INIT(Raw, EKF);
			continue;
		}


		/* ------------------- Start EKF Process ------------------- */
		all_count++;


		/* ------------ Time Estimate ------------- */
		Eigen::MatrixXd X_k_k_1 = Eigen::MatrixXd::Zero(3 + 2 * Raw.DDObs.Sats, 1);
		Eigen::MatrixXd P_k_k_1 = Eigen::MatrixXd::Zero(3 + 2 * Raw.DDObs.Sats, 3 + 2 * Raw.DDObs.Sats);

		if (!TimeEstimate(Raw, EKF, X_k_k_1, P_k_k_1))
		{
			// Need to Re-Initialize all the State
			EKF.isInit = false;

			// Output SPP Solution
			string posString = Form_Pos_String(Raw, 5);

			output_file << posString;

			if (config.Enable_Visual && visualServer) {
				visualServer->addData(posString);
			}

			log_msg << "Failed to Have Time Estimate Process!" << endl;
			log_msg << "Rover SPP Position: " << fixed << setprecision(4)
				<< Raw.RovEpk.my_result.Pos.x << "  " << Raw.RovEpk.my_result.Pos.y << "  " << Raw.RovEpk.my_result.Pos.z << endl;
			continue;
		}


		/* --------------- Measurement Update --------------- */
		Eigen::MatrixXd X_k_k = Eigen::MatrixXd::Zero(3 + 2 * Raw.DDObs.Sats, 1);
		Eigen::MatrixXd P_k_k = Eigen::MatrixXd::Zero(3 + 2 * Raw.DDObs.Sats, 3 + 2 * Raw.DDObs.Sats);

		if (!MeasurementUpdate(Raw, X_k_k_1, P_k_k_1, X_k_k, P_k_k))
		{
			// Need to Re-Initialize all the State
			EKF.isInit = false;

			// Output SPP Solution
			string posString = Form_Pos_String(Raw, 5);

			output_file << posString;

			if (config.Enable_Visual && visualServer) {
				visualServer->addData(posString);
			}

			log_msg << "Failed to Update Measurement!" << endl;
			log_msg << "Rover SPP Position: " << fixed << setprecision(4)
				<< Raw.RovEpk.my_result.Pos.x << "  " << Raw.RovEpk.my_result.Pos.y << "  " << Raw.RovEpk.my_result.Pos.z << endl;;
			continue;
		}


		// Update BaseLine and Point Position Info
		Raw.RovEpk.my_result.Pos = XYZ_Coord(X_k_k(0, 0), X_k_k(1, 0), X_k_k(2, 0));

		Raw.DDObs.dPos.x = Raw.RovEpk.my_result.Pos.x - Raw.BasEpk.rcv_result.Pos.x;
		Raw.DDObs.dPos.y = Raw.RovEpk.my_result.Pos.y - Raw.BasEpk.rcv_result.Pos.y;
		Raw.DDObs.dPos.z = Raw.RovEpk.my_result.Pos.z - Raw.BasEpk.rcv_result.Pos.z;



		if (Calc_XYZ_dist(Raw.RovEpk.my_result.Pos, Rov_pos) > 1e3)
		{
			// Need to Re-Initialize all the State
			EKF.isInit = false;
			Raw.RovEpk.my_result.Pos = Rov_pos;

			// Output SPP Solution
			string posString = Form_Pos_String(Raw, 5);

			output_file << posString;

			if (config.Enable_Visual && visualServer) {
				visualServer->addData(posString);
			}

			log_msg << "Failed to Update Measurement!" << endl;
			log_msg << "Rover SPP Position: " << fixed << setprecision(4)
				<< Raw.RovEpk.my_result.Pos.x << "  " << Raw.RovEpk.my_result.Pos.y << "  " << Raw.RovEpk.my_result.Pos.z << endl;;
			continue;
		}

		/* ------------ EKF State Update ------------- */
		EKF.Time = Raw.RovEpk.Time;
		EKF.Last_DDObs = Raw.DDObs;
		EKF.Pos = XYZ_Coord(X_k_k(0, 0), X_k_k(1, 0), X_k_k(2, 0));
		EKF.P.resize(3 + 2 * Raw.DDObs.Sats, 3 + 2 * Raw.DDObs.Sats);
		EKF.P = P_k_k;



		/* --------- Try to Have a Fix Solution --------- */
		Eigen::VectorXd FloatAmb(X_k_k.block(3, 0, 2 * Raw.DDObs.Sats, 1));

		if (!RTK_FIX(Raw, FloatAmb, P_k_k, config.Ratio_Thres))
		{
			// Failed to have RTK_FIX Solution --> Output RTK Float Solution
			string posString = Form_Pos_String(Raw, 2);

			output_file << posString;

			if (config.Enable_Visual && visualServer) {
				visualServer->addData(posString);
			}

			log_msg << "WARNING: Failed to Fix the Ambiguity! Ratio = " << Raw.DDObs.Ratio << endl;
			log_msg << "Float Solution of BaseLine: " << Raw.DDObs.dPos.x << "  " << Raw.DDObs.dPos.y << "  " << Raw.DDObs.dPos.z << endl;
			log_msg << "Rover RTK Float Position  : " << fixed << setprecision(4) << Raw.RovEpk.my_result.Pos.x << "  " << Raw.RovEpk.my_result.Pos.y << "  " << Raw.RovEpk.my_result.Pos.z << endl;
			log_msg << "The Satellite Information : " << "Obs->" << Raw.RovEpk.SatNum << "   DDObs->" <<
				Raw.DDObs.Sats << "  GPS->" << Raw.DDObs.DDSatNum[0] << "  BDS->" << Raw.DDObs.DDSatNum[1] << endl;

			UpdateAmbiguityInfo(Raw, EKF, X_k_k);

			continue;
		}

		/* ------------ EKF State Update ------------- */
		EKF.Pos = Raw.RovEpk.my_result.Pos;


		fixed_count++;
		string posString = Form_Pos_String(Raw, 1);

		output_file << posString;

		if (config.Enable_Visual && visualServer) {
			visualServer->addData(posString);
		}


		log_msg << "Fixed Solution of BaseLine: " << Raw.DDObs.dPos.x << "  " << Raw.DDObs.dPos.y << "  " << Raw.DDObs.dPos.z << endl;
		log_msg << "Fixed Solution of Rover   : " << fixed << setprecision(4) << Raw.RovEpk.my_result.Pos.x << "  " << Raw.RovEpk.my_result.Pos.y << "  " << Raw.RovEpk.my_result.Pos.z << endl;
		log_msg << "The Satellite Information : " << "Obs->" << Raw.RovEpk.SatNum << "   DDObs->" <<
			Raw.DDObs.Sats << "  GPS->" << Raw.DDObs.DDSatNum[0] << "  BDS->" << Raw.DDObs.DDSatNum[1] << endl;

		log_msg << "Solution Count: " << all_count << "  Fix Ratio: " << (double)fixed_count / all_count * 100 << "%" << endl;

		UpdateAmbiguityInfo(Raw, EKF, X_k_k);


	}
	return 1;
}





int main()
{
	config_ config;

	// Load the configuration file
	if (!LoadConfig("config.json", config)) {
		return 0;
	}

	if (config.RTKProcMode == 0)
	{
		RTK_LS(config);
	}
	else
	{
		RTK_KF(config);
	}

	return 0;
}
