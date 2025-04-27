#define _CRT_SECURE_NO_WARNINGS
#include"my_rtk.hpp"
#include<iostream>
#include<fstream>
#include<iomanip>

#include<Eigen/Core>
#include<Eigen/Dense>

using namespace std;



/* -----------------Configuration----------------- */

// Define network configuration parameters
const char* IP = "47.114.134.129";
const unsigned short PORT = 7190;


const char* SOURCE_FILE_1 = ".//Data//oem719-202404110900-2.bin";
const char* SOURCE_FILE_2 = ".//Data//oem719-202404110900-1.bin";


const char* SOURCE_FILE_01 = ".//Data//oem719-202202021500-base.bin";
const char* SOURCE_FILE_02 = ".//Data//oem719-202202021500-rover.bin";


const char* OUTPUT_FILE = "./Solution/Rtk_result.txt";
const char* RCVRES_FILE = "./Solution/rcv_result.txt";
const char* MIXOUT_FILE = "./Solution/mix_result.txt";

const double delta_pseudo = 0.3;
const double delta_phase  = 0.01;




/* Get Synchronous Observation -------------------------------------
* args   :
*          InputHandle    base_src      I    file/socket 
*          InputHandle    rover_src     I	 file/socket 
*          RTK_RAW	      raw           O	 restore all data for rtk
*
* return : int
*		    1 ------>  Get synchronous Data Successfully
*		    0 ------>  Base_Obs much more Later than Rover -- Get Out and then back to Loop again
*		   -1 ------>  One of the Data Sources Already Readed
*
*----------------------------------------------------------------*/
int GetSynObs(InputHandle& rover_src,InputHandle& base_src, RTK_RAW& raw)
{
	// Status Record
	double time_diff = 0.0;

	double threshold = 1;

	if (rover_src.type_ == InputType::FILE_INPUT)
	{
		threshold = 0.5;
	}
	else
	{
		threshold = 0.5;
	}

	// Loop to Extract the Next Obs of Rover
	while (true)
	{
		// decode message buff until an Obs Extract
		// decode_NovOem7_Buff return 1 ---> Decode Obs Successfully
		try
		{
			if (decode_NovOem7_Buff(rover_src.buff_ptr, rover_src.valid_size, rover_src.buff_len, raw.Message, raw.RovEpk, raw.GpsEph, raw.BdsEph))
			{
				time_diff = GPST_DIFF(raw.RovEpk.Time, raw.BasEpk.Time);

				if (fabs(time_diff) <= threshold && rover_src.type_ == InputType::FILE_INPUT)
					return 1;  // -----> Old Base_Obs Synchronous with Rover
				else if (time_diff < -threshold)
					return 0;  // -----> Old Base_Obs much more Later than Rover
				else
					break;	  //  -----> Old Base_Obs much more Early than Rover 
			}
		}
		catch (exception& e)
		{
			cout << e.what() << endl;
		}
	


		// decode_NovOem7_Buff return 0 ---> Get out of the function and add data into buffer again
		if (rover_src.read())
		{
			continue;
		}
		else
		{
			return -1;	// -----> Data Source Already Readed
		}
			

			
	}

	// Loop to Extract the Next Obs of Base
	while (true)
	{

		// decode message buff until an Obs Extract 
		try
		{
			if (decode_NovOem7_Buff(base_src.buff_ptr, base_src.valid_size, base_src.buff_len, raw.Message, raw.BasEpk, raw.GpsEph, raw.BdsEph))
			{

				time_diff = GPST_DIFF(raw.RovEpk.Time, raw.BasEpk.Time);

				if (fabs(time_diff) <= threshold)
					return 1;  // ----->  New Base_Obs Synchronous with Rover

				if (time_diff < -threshold)
					return 0;  // ----->  New Base_Obs much more Later than Rover

				// Base_Obs much more Early than Rover ---> Loop and Extract Obs Again
				continue;
			}
		}
		catch (exception& e)
		{
			cout << e.what() << endl;
		}


		// decode_NovOem7_Buff return 0 ---> Get out of the function and add data into buffer again
		if (base_src.read())
		{
			continue;
		}
		else
		{
			return -1;	// -----> Data Source Already Readed
		}
	}

}



/*  Check If Single-Freq has Cycle_Slip by LOCKTIME  */
void Check_Raw_Obs(EPOCH_OBS& base_obs, EPOCH_OBS& rover_obs)
{
	
	// Check Base Station Obs
	for (int i = 0; i < base_obs.SatNum; i++)
	{

		for (int j = 0; j < MAX_CHANNEL_NUM; j++)
		{
			if (base_obs.ComObs[j].Prn != base_obs.SatObs[i].Prn || base_obs.ComObs[j].Sys != base_obs.SatObs[i].System)
			{
				continue;
			}
			else /* find the same sat */
			{
				
				// check if single-freq has cycle_slip by LOCKTIME 
				for (int k = 0; k < FREQ_NUM; k++)
				{
					if (base_obs.SatObs[i].LockTime[0] < base_obs.ComObs[j].LockTime_Last[0])
					{
						base_obs.SatObs[i].no_cycle_slip[0] = false;
					}
				}

				break;
			}
		}
		// if no the old LOCKTIME ---> No_Cycle_Slip = True By Default	
	}



	// Check Rover Obs
	for (int i = 0; i < base_obs.SatNum; i++)
	{

		for (int j = 0; j < MAX_CHANNEL_NUM; j++)
		{
			if (base_obs.ComObs[j].Prn != base_obs.SatObs[i].Prn || base_obs.ComObs[j].Sys != base_obs.SatObs[i].System)
			{
				continue;
			}
			else /* find the same sat */
			{

				// check if single-freq has cycle_slip by LOCKTIME 
				for (int k = 0; k < FREQ_NUM; k++)
				{
					if (base_obs.SatObs[i].LockTime[0] < base_obs.ComObs[j].LockTime_Last[0])
					{
						// just add flag , don't exclude observation
						base_obs.SatObs[i].no_cycle_slip[0] = false;
					}
				}

				break;
			}
		}
		// if no the old LOCKTIME ---> No_Cycle_Slip = True By Default	
	}

}


/* Construct Single Difference Observation -------------------------------------
* args   :
*          EPOCH_OBS    base_obs      I
*          EPOCH_OBS    rover_obs     I
*          SD_EPOCH_OBS SD_obs        O
*
* return : void
*-----------------------------------------------------------------------------*/
void Construct_SD_Obs(const EPOCH_OBS& base_obs, const EPOCH_OBS& rover_obs, SD_EPOCH_OBS& SD_obs)
{
	// Reset memory ---> except COMB_OBS 
	SD_obs.reset();

	// Epoch time
	SD_obs.Time = rover_obs.Time;

	// Usable SD_obs Counter
	int sd_num = 0;
	

	/* Perform the inter-station differencing for the same satellite, frequency, and observation type */
	// Loop to Find the same satellites
	for (int i = 0; i < MAX_CHANNEL_NUM; i++)
	{
		// Loop in Base Staion
		for (int j = 0; j < MAX_CHANNEL_NUM; j++)
		{

			/* Matching satellites */
			if (rover_obs.SatObs[i].System == base_obs.SatObs[j].System && rover_obs.SatObs[i].Prn == base_obs.SatObs[j].Prn)
			{
				// An Counter for SD observation in each freq
				int obs_num = 0;	 

				// Parity (Half Phase Added) 
				bool parity_add = true;


				/* Matching frequencies */
				for (int k = 0; k < 2; k++)
				{
					// check if obs is valid
					if (rover_obs.SatObs[i].p[k] < 1e-9 ||			// pesudure
						rover_obs.SatObs[i].l[k] < 1e-9 ||			// phase
						!rover_obs.SatObs[i].no_cycle_slip[k])	    // cycle Slip
					{
						SD_obs.SdSatObs[sd_num].dP[k] = 0.0;
						SD_obs.SdSatObs[sd_num].dL[k] = 0.0;
						continue;
					}
					

					// Construct SD in Freq_k
					SD_obs.SdSatObs[sd_num].dP[k] = rover_obs.SatObs[i].p[k] - base_obs.SatObs[j].p[k];   // m 
					SD_obs.SdSatObs[sd_num].dL[k] = rover_obs.SatObs[i].l[k] - base_obs.SatObs[j].l[k];   // phase

					
					if (!rover_obs.SatObs[i].half_Added[k] || !base_obs.SatObs[j].half_Added[k])
					{
						parity_add = false;
					}

					obs_num++;
				}

				if (obs_num > 0)
				{
					// Store satellite info
					SD_obs.SdSatObs[sd_num].System = rover_obs.SatObs[i].System;
					SD_obs.SdSatObs[sd_num].Prn = rover_obs.SatObs[i].Prn;


					// Store the indices of the rover and base station observations
					SD_obs.SdSatObs[sd_num].Rov_idx = i;
					SD_obs.SdSatObs[sd_num].Bas_idx = j;


					// Lable SD Valid_status
					if (obs_num == 2 && parity_add == true)
					{
						SD_obs.SdSatObs[sd_num].Valid_status = 2;
					}
					else
					{
						SD_obs.SdSatObs[sd_num].Valid_status = 1;
					}
					
					sd_num++;
				}


				// Satellite Matched and Processed
				break;
			}
		}
	}

	SD_obs.SatNum = sd_num;

}



/* Validate Single Difference Observation Data and Compute Combine Obs */
void Check_SD_Obs(SD_EPOCH_OBS& sd_obs)
{
	// Prepare a new combine obs array
	COMB_OBS new_combine[MAX_CHANNEL_NUM];
	double lambda1 = 0.0, lambda2 = 0.0, freq1 = 0.0, freq2 = 0.0;


	// Loop to check out all Single-Differnce Obs
	for (int i = 0; i < sd_obs.SatNum; i++)
	{

		// half_cycle haven't added OR no dual-obs
		if (sd_obs.SdSatObs[i].Valid_status < 2)
		{
			// add necessary info 
			new_combine[i].Prn = sd_obs.SdSatObs[i].Prn;
			new_combine[i].Sys = sd_obs.SdSatObs[i].System;

			new_combine[i].valid_epo_num = 0;

			continue;
		}


		/* ------------calculate epoch combine observation------------ */
		// get frequence and wave-length
		if (sd_obs.SdSatObs[i].System == GPS)
		{
			freq1 = FREQ_GPS_L1;
			freq2 = FREQ_GPS_L2;
			lambda1 = CLIGHT / FREQ_GPS_L1;
			lambda2 = CLIGHT / FREQ_GPS_L2;
		}
		else if (sd_obs.SdSatObs[i].System == BDS)
		{
			freq1 = FREQ_BDS_B1;
			freq2 = FREQ_BDS_B3;
			lambda1 = CLIGHT / FREQ_BDS_B1;
			lambda2 = CLIGHT / FREQ_BDS_B3;
		}

		// MW and GF in this Epoch
		double L1 = lambda1 * sd_obs.SdSatObs[i].dL[0];   // m
		double L2 = lambda2 * sd_obs.SdSatObs[i].dL[1];	  // m
		double GF = L1 - L2;							  // m

		double MW = (freq1 * L1 - freq2 * L2) / (freq1 - freq2) - (freq1 * sd_obs.SdSatObs[i].dP[0] + freq2 * sd_obs.SdSatObs[i].dP[1]) / (freq1 + freq2);

		// Extract MW and GF in Last Epoch
		double GF_0, MW_bar;
		long k_0 = 0;
		bool find_flag = false;


		// Loop to find the Cooresponding Combine Observation in the Last Epoch
		for (int j = 0; j < MAX_CHANNEL_NUM; j++)
		{
			if (sd_obs.SdCombObs[j].Prn != sd_obs.SdSatObs[i].Prn || sd_obs.SdCombObs[j].Sys != sd_obs.SdSatObs[i].System)
			{
				continue;
			}
			else /* find the same sat */
			{
				find_flag = true;
				k_0 = sd_obs.SdCombObs[j].valid_epo_num;
				GF_0 = sd_obs.SdCombObs[j].GF;
				MW_bar = sd_obs.SdCombObs[j].MW;
				break;
			}
		}

		// add info into new_combine
		new_combine[i].Prn = sd_obs.SdSatObs[i].Prn;
		new_combine[i].Sys = sd_obs.SdSatObs[i].System;

		// if is new valid epoch
		if (find_flag == false || k_0 == 0)
		{
			sd_obs.SdSatObs[i].Valid_status = 2;
			new_combine[i].GF = GF;
			new_combine[i].MW = MW;
			new_combine[i].PIF = (freq1 * freq1 * sd_obs.SdSatObs[i].dP[0] - freq2 * freq2 * sd_obs.SdSatObs[i].dP[1]) / (freq1 * freq1 - freq2 * freq2);
			new_combine[i].valid_epo_num = 1;


		} // check MW and GF difference with last epoch
		else if (fabs(GF - GF_0) > 0.05 || fabs(MW - MW_bar) > 3)
		{
			sd_obs.SdSatObs[i].Valid_status = 0;  // ---> have cycle slip
			new_combine[i].GF = GF;
			new_combine[i].MW = MW;
			new_combine[i].valid_epo_num = 0;
			

			cout << "Cycle Slip Detected:  " << sd_obs.SdSatObs[i].System << " " << sd_obs.SdSatObs[i].Prn << endl;

		}
		else
		{
			sd_obs.SdSatObs[i].Valid_status = 2;  // ---> Dual-Obs with no cycle-slip or half-cycle
			MW_bar = (k_0 * MW_bar + MW) / (k_0 + 1);
			new_combine[i].GF = GF;
			new_combine[i].MW = MW_bar;
			new_combine[i].PIF = (freq1 * freq1 * sd_obs.SdSatObs[i].dP[0] - freq2 * freq2 * sd_obs.SdSatObs[i].dP[1]) / (freq1 * freq1 - freq2 * freq2);
			new_combine[i].valid_epo_num = k_0 + 1;

		}

	}

	
	// Add new_combine epoch obs data
	for (int i = 0; i < MAX_CHANNEL_NUM; i++)
	{
		if (i < sd_obs.SatNum)
		{
			sd_obs.SdCombObs[i] = new_combine[i];
		}
		else
		{
			sd_obs.SdCombObs[i] = COMB_OBS();
		}

	}
}


/* Select Reference Satellite for Double Difference Observation -------------------------------------
* args   :
*          EPOCH_OBS       base_obs      I
*          EPOCH_OBS       rover_obs     I
*          SD_EPOCH_OBS    SD_obs        I
*		   DD_OBS		   DD_obs		 O
*
* return : bool  1 ---> Select Reference Satellite Successfully
*			     0 ---> No Reference Satellite Found
*-----------------------------------------------------------------------------*/
bool Seletct_Ref_Sat(const EPOCH_OBS& base_obs, const EPOCH_OBS& rover_obs, SD_EPOCH_OBS& SD_obs, DD_OBS& DD_obs)
{
	// Dual-Obs and No parity cycle
	// 0 --> GPS   1--> BDS
	int good_sat_num[2] = { 0,0 };

	// Maximum satellite elevation angle for each system
	double max_elve[2] = { 0,0 };
	int    maxE_idx[2] = { 0,0 };


	// Clear(Reset) previous epoch's double-difference data
	DD_obs.reset();


	/* For each navigation system, select one satellite as the reference */
	// Loop to cheack all the Single-Difference Obs
	for (int i = 0; i < SD_obs.SatNum; i++)
	{
		// check if have cycle slips, single-obs or half-cycle ambiguities
		if (SD_obs.SdSatObs[i].Valid_status != 2)
		{
			// If use single-obs ---> Add Here
			continue;
		}


		/* Select the reference satellite for different systems separately */

		// Ensure that the ephemeris is valid and the satellite position is successfully computed (by SPP)
		if (rover_obs.SatPVT[SD_obs.SdSatObs[i].Rov_idx].Valid == false
			|| base_obs.SatPVT[SD_obs.SdSatObs[i].Bas_idx].Valid == false)
		{
			SD_obs.SdSatObs[i].Valid_status = 0;
			continue; 
		}
			

		/* Count the available satellites */
		int sat_sys = 0;

		switch (SD_obs.SdSatObs[i].System)
		{
			case GPS: sat_sys =  0; break;
			case BDS: sat_sys =  1; break;
			default:   break;
		}

		// For Matrix Construction Later
		good_sat_num[sat_sys]++;


		/* Use rover¡¯s L1 frequency observations as the reference */
		if (base_obs.SatObs[SD_obs.SdSatObs[i].Bas_idx].cn0[0] > 45 &&
			rover_obs.SatObs[SD_obs.SdSatObs[i].Rov_idx].cn0[0] > 45 &&
			base_obs.SatObs[SD_obs.SdSatObs[i].Bas_idx].LockTime[0] > 10 &&
			rover_obs.SatObs[SD_obs.SdSatObs[i].Rov_idx].LockTime[0] > 10 && 
			rover_obs.SatPVT[SD_obs.SdSatObs[i].Rov_idx].Elevation > max_elve[sat_sys])		
		{
			max_elve[sat_sys] = rover_obs.SatPVT[SD_obs.SdSatObs[i].Rov_idx].Elevation;
			maxE_idx[sat_sys] = i;   // ---> Index in the single-difference observation array
		}
	}

	/* Store the reference satellite's PRN and its index */
	for (int i = 0; i < 2; i++)
	{
		if (max_elve[i] > 1e-9) // A valid reference satellite was found
		{
			DD_obs.ref_idx[i]  = maxE_idx[i];
			DD_obs.ref_prn[i]  = SD_obs.SdSatObs[maxE_idx[i]].Prn;
			DD_obs.DDSatNum[i] = good_sat_num[i] - 1;
		}
		else
		{
			DD_obs.ref_idx[i]  = -1;
			DD_obs.ref_prn[i]  =  0;
			DD_obs.DDSatNum[i] =  0;
		}
		
	}

	
	DD_obs.Sats = DD_obs.DDSatNum[0] + DD_obs.DDSatNum[1];


	if (DD_obs.Sats == 0)
	{
		return 0;
	}

	return 1;
}





bool RTK_Float(RTK_RAW& Raw, Eigen::VectorXd& FloatAmb, Eigen::MatrixXd& Qxx)
{

	// Get Initial Position of ROVER from SPP
	XYZ_Coord rover_pos(Raw.RovEpk.my_result.Pos);
	XYZ_Coord base_pos(Raw.BasEpk.rcv_result.Pos);

	// Get Initial Position of BASE from Reference Position
	if (Raw.BasEpk.rcv_result.Pos.x == 0.0)
	{
		// Get Initial Position of BASE from SPP
		std::cout << "No Base Position Reference Solution!" << std::endl;
		base_pos = Raw.BasEpk.my_result.Pos;
	}
	

	// For-Loop to Calulate Base-Satellite Distance
	double rou_B0[MAX_CHANNEL_NUM]{};

	for (int i = 0; i < Raw.BasEpk.SatNum; i++)
	{
		rou_B0[i] = Calc_XYZ_dist(Raw.BasEpk.SatPVT[i].SatPos, base_pos);
	}


	// Find the Index of Refernce Satellite in RawObs/SatPVT
	int RefIdx_Rov_GPS, RefIdx_Bas_GPS, RefIdx_Rov_BDS, RefIdx_Bas_BDS;
	RefIdx_Rov_GPS = RefIdx_Bas_GPS = RefIdx_Rov_BDS = RefIdx_Bas_BDS = -1;
	
	if (Raw.DDObs.DDSatNum[0] > 0)
	{
		RefIdx_Rov_GPS = Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[0]].Rov_idx;
		RefIdx_Bas_GPS = Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[0]].Bas_idx;
	}

	if (Raw.DDObs.DDSatNum[1] > 0)
	{
		RefIdx_Rov_BDS = Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[1]].Rov_idx;
		RefIdx_Bas_BDS = Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[1]].Bas_idx;
	}

	


	// Iteration Counter
	int iter_count = 1;
	int max_iter_count = 10;
	bool new_solution_circle = true;		// For Ambiguity Initialization

	// Save the Ambiguity Info
	Eigen::MatrixXd X = Eigen::MatrixXd::Zero(3 + 2 * Raw.DDObs.Sats, 1);

	while (1)
	{
		// Initialize Estimate Parameters
		X(0, 0) = rover_pos.x;
		X(1, 0) = rover_pos.y;
		X(2, 0) = rover_pos.z;


		// For-Loop to Calulate Rover-Satellite Distance
		double rou_R0[MAX_CHANNEL_NUM]{};
		for (int i = 0; i < Raw.RovEpk.SatNum; i++)
		{
			rou_R0[i] = Calc_XYZ_dist(Raw.RovEpk.SatPVT[i].SatPos, rover_pos);
		}


		// Construct Matrix for Least-Square
		Eigen::MatrixXd B = Eigen::MatrixXd::Zero(4 * Raw.DDObs.Sats, 3 + 2 * Raw.DDObs.Sats);
		Eigen::MatrixXd L = Eigen::MatrixXd::Zero(4 * Raw.DDObs.Sats, 1);
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(4 * Raw.DDObs.Sats, 4 * Raw.DDObs.Sats);

	

		// Prepare P-Matrix Weight for pseudo
		double W_GPS_ii_P = Raw.DDObs.DDSatNum[0] / (2.0 * delta_pseudo * delta_pseudo * (Raw.DDObs.DDSatNum[0] + 1.0));
		double W_GPS_ij_P = -1.0 / (2.0 * delta_pseudo * delta_pseudo * (Raw.DDObs.DDSatNum[0] + 1.0));

		double W_BDS_ii_P = Raw.DDObs.DDSatNum[1] / (2.0 * delta_pseudo * delta_pseudo * (Raw.DDObs.DDSatNum[1] + 1.0));
		double W_BDS_ij_P = -1.0 / (2.0 * delta_pseudo * delta_pseudo * (Raw.DDObs.DDSatNum[1] + 1.0));

		// Prepare P-Matrix Weight for carrier phase
		double W_GPS_ii_L = Raw.DDObs.DDSatNum[0] / (2.0 * delta_phase * delta_phase * (Raw.DDObs.DDSatNum[0] + 1.0));
		double W_GPS_ij_L = -1.0 / (2.0 * delta_phase * delta_phase * (Raw.DDObs.DDSatNum[0] + 1.0));

		double W_BDS_ii_L = Raw.DDObs.DDSatNum[1] / (2.0 * delta_phase * delta_phase * (Raw.DDObs.DDSatNum[1] + 1.0));
		double W_BDS_ij_L = -1.0 / (2.0 * delta_phase * delta_phase * (Raw.DDObs.DDSatNum[1] + 1.0));


		// Prepare Indeies for Matrix Filling
		int GPS_idx = 0;
		int BDS_idx = 4 * Raw.DDObs.DDSatNum[0];
		int valid_SD_num = 0;
		int SD_index[MAX_CHANNEL_NUM]{};
		memset(SD_index, 0, sizeof(SD_index));

		// For-Loop to Construct Observation Function in SD_EPOCH_OBS
		for (int j = 0; j < Raw.SdObs.SatNum; j++)
		{
			// Check if is Ref Satellite
			if (Raw.DDObs.ref_idx[0] == j || Raw.DDObs.ref_idx[1] == j)
			{
				continue;
			}

			// Check if has cycle slip or single-obs or half-cycle ambiguities
			if (Raw.SdObs.SdSatObs[j].Valid_status != 2)
			{
				continue;
			}


			// Add the Index of SD Observation -- For OutLiyer Detection
			SD_index[valid_SD_num] = j;
			valid_SD_num++;


			if (Raw.SdObs.SdSatObs[j].System == GPS)
			{

				/* ------- Set Frequency and Wave Length -------*/
				double freq1 = FREQ_GPS_L1;
				double lambda1 = CLIGHT / FREQ_GPS_L1;

				double freq2 = FREQ_GPS_L2;
				double lambda2 = CLIGHT / FREQ_GPS_L2;


				/* ------- Construct Double Difference Distance Constants-------*/ 
				// Distance of Rover-Satellite(Reference Sat)
				double rou_R0_i = rou_R0[RefIdx_Rov_GPS];

				// Distance of Rover-Satellite(This SD Sat)
				double rou_R0_j = rou_R0[Raw.SdObs.SdSatObs[j].Rov_idx];

				double rou_BR_i = rou_R0_i - rou_B0[RefIdx_Bas_GPS];
				double rou_BR_j = rou_R0_j - rou_B0[Raw.SdObs.SdSatObs[j].Bas_idx];

				// Double Difference Distance Constants
				double rou_BR_ij = rou_BR_j - rou_BR_i;

				//cout << rou_R0_j << " - " << rou_B0[Raw.SdObs.SdSatObs[j].Bas_idx] << " - " << rou_R0_i << " +" << rou_B0[RefIdx_Bas_GPS] << endl;


				/* ------- Construct Double Difference Obeservation ---------- */
				double P_f1 = Raw.SdObs.SdSatObs[j].dP[0] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[0]].dP[0];
				double P_f2 = Raw.SdObs.SdSatObs[j].dP[1] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[0]].dP[1];

				double L_f1 = Raw.SdObs.SdSatObs[j].dL[0] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[0]].dL[0];
				L_f1 *= lambda1;   // Carrier Phase --- Cycle to Metres
				double L_f2 = Raw.SdObs.SdSatObs[j].dL[1] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[0]].dL[1];
				L_f2 *= lambda2;   // Carrier Phase --- Cycle to Metres



				/* ------- Initialize Ambiguity Parameters if is the first iteration ------ */ 
				// The idx of DD is Proccessing
				int obs_count = GPS_idx / 4;
				if (new_solution_circle == true)
				{
					X(3 + obs_count * 2, 0)     = (L_f1 - P_f1) / lambda1;	 // Ambiguity of L1
					X(3 + obs_count * 2 + 1, 0) = (L_f2 - P_f2) / lambda2;   // Ambiguity of L2
				}



				/* ---------------- Fill the Matrix B --------------*/ 
				
				for (int i = 0; i < 4; i++)
				{
					// P1 P2 L1 L2 --- Direction cosine
					B(GPS_idx + i, 0) = (rover_pos.x - Raw.RovEpk.SatPVT[Raw.SdObs.SdSatObs[j].Rov_idx].SatPos.x) / rou_R0_j
										- (rover_pos.x - Raw.RovEpk.SatPVT[RefIdx_Rov_GPS].SatPos.x) / rou_R0_i;

					B(GPS_idx + i, 1) = (rover_pos.y - Raw.RovEpk.SatPVT[Raw.SdObs.SdSatObs[j].Rov_idx].SatPos.y) / rou_R0_j
										- (rover_pos.y - Raw.RovEpk.SatPVT[RefIdx_Rov_GPS].SatPos.y) / rou_R0_i;

					B(GPS_idx + i, 2) = (rover_pos.z - Raw.RovEpk.SatPVT[Raw.SdObs.SdSatObs[j].Rov_idx].SatPos.z) / rou_R0_j
										- (rover_pos.z - Raw.RovEpk.SatPVT[RefIdx_Rov_GPS].SatPos.z) / rou_R0_i;
				}

				// Carrier Phase Ambiguity
				B(GPS_idx + 2, 3 + 2 * obs_count) = lambda1;
				B(GPS_idx + 3, 4 + 2 * obs_count) = lambda2;



				/* ---------------- Fill the Matrix L --------------*/
				L(GPS_idx, 0) = P_f1 - rou_BR_ij;
				L(GPS_idx + 1, 0) = P_f2 - rou_BR_ij;
				L(GPS_idx + 2, 0) = L_f1 - rou_BR_ij - lambda1 * X(3 + obs_count * 2, 0);
				L(GPS_idx + 3, 0) = L_f2 - rou_BR_ij - lambda2 * X(3 + obs_count * 2 + 1, 0);


				

				/* ---------------- Fill the Matrix P --------------*/
				// If we use The-Same-Weight , We can Fill P outside the Loop


				/* ---------- Shift GPS_idx for the next GPS DD_OBS ----------*/
				GPS_idx += 4;

			}
			else if (Raw.SdObs.SdSatObs[j].System == BDS)
			{

				/* ------- Set Frequency and Wave Length -------*/
				double freq1 = FREQ_BDS_B1;
				double lambda1 = CLIGHT / FREQ_BDS_B1;

				double freq2 = FREQ_BDS_B3;
				double lambda2 = CLIGHT / FREQ_BDS_B3;


				/* ------- Construct Double Difference Distance Constants-------*/
				// Distance of Rover-Satellite(Reference Sat)
				double rou_R0_i = rou_R0[RefIdx_Rov_BDS];

				// Distance of Rover-Satellite(This SD Sat)
				double rou_R0_j = rou_R0[Raw.SdObs.SdSatObs[j].Rov_idx];

				double rou_BR_i = rou_R0_i - rou_B0[RefIdx_Bas_BDS];
				double rou_BR_j = rou_R0_j - rou_B0[Raw.SdObs.SdSatObs[j].Bas_idx];

				// Double Difference Distance Constants
				double rou_BR_ij = rou_BR_j - rou_BR_i;



				/* ------- Construct Double Difference Obeservation ---------- */
				double P_f1 = Raw.SdObs.SdSatObs[j].dP[0] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[1]].dP[0];
				double P_f2 = Raw.SdObs.SdSatObs[j].dP[1] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[1]].dP[1];

				double L_f1 = Raw.SdObs.SdSatObs[j].dL[0] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[1]].dL[0];
				L_f1 *= lambda1;   // Carrier Phase --- Cycle to Metres
				double L_f2 = Raw.SdObs.SdSatObs[j].dL[1] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[1]].dL[1];
				L_f2 *= lambda2;   // Carrier Phase --- Cycle to Metres



				/* ------- Initialize Ambiguity Parameters if is the first iteration ------ */

				// The idx of DD is Proccessing
				int obs_count = BDS_idx / 4;
				if (new_solution_circle == true)
				{
					X(3 + obs_count * 2, 0) = (L_f1 - P_f1) / lambda1;	 // Ambiguity of L1
					X(4 + obs_count * 2, 0) = (L_f2 - P_f2) / lambda2;   // Ambiguity of L2
				}



				/* ---------------- Fill the Matrix B --------------*/

				for (int i = 0; i < 4; i++)
				{
					// P1 P2 L1 L2 --- Direction cosine
					B(BDS_idx + i, 0) = (rover_pos.x - Raw.RovEpk.SatPVT[Raw.SdObs.SdSatObs[j].Rov_idx].SatPos.x) / rou_R0_j
										- (rover_pos.x - Raw.RovEpk.SatPVT[RefIdx_Rov_BDS].SatPos.x) / rou_R0_i;

					B(BDS_idx + i, 1) = (rover_pos.y - Raw.RovEpk.SatPVT[Raw.SdObs.SdSatObs[j].Rov_idx].SatPos.y) / rou_R0_j
										- (rover_pos.y - Raw.RovEpk.SatPVT[RefIdx_Rov_BDS].SatPos.y) / rou_R0_i;

					B(BDS_idx + i, 2) = (rover_pos.z - Raw.RovEpk.SatPVT[Raw.SdObs.SdSatObs[j].Rov_idx].SatPos.z) / rou_R0_j
										- (rover_pos.z - Raw.RovEpk.SatPVT[RefIdx_Rov_BDS].SatPos.z) / rou_R0_i;
				}

				// Carrier Phase Ambiguity
				B(BDS_idx + 2, 3 + 2 * obs_count) = lambda1;
				B(BDS_idx + 3, 4 + 2 * obs_count) = lambda2;



				/* ---------------- Fill the Matrix L --------------*/
				L(BDS_idx, 0)	  = P_f1 - rou_BR_ij;
				L(BDS_idx + 1, 0) = P_f2 - rou_BR_ij;
				L(BDS_idx + 2, 0) = L_f1 - rou_BR_ij - lambda1 * X(3 + BDS_idx / 4 * 2, 0);
				L(BDS_idx + 3, 0) = L_f2 - rou_BR_ij - lambda2 * X(3 + BDS_idx / 4 * 2 + 1, 0);

				//cout << L(BDS_idx, 0) << "  " << L(BDS_idx, 0) << " " << L(BDS_idx, 0) << " " << L(BDS_idx, 0) << endl;

				/* ---------------- Fill the Matrix P --------------*/
				// If we use The-Same-Weight , We can Fill P outside the Loop		


				/* ---------- Shift GPS_idx for the next GPS DD_OBS ----------*/
				BDS_idx += 4;

			}

			
		}

		
		/* ---------------- Fill the Matrix P --------------*/
		Eigen::MatrixXd P_GPS;
		// GPS-Part of P
		if (Raw.DDObs.DDSatNum[0] > 0)
		{
			P_GPS = Eigen::MatrixXd::Zero(4 * Raw.DDObs.DDSatNum[0], 4 * Raw.DDObs.DDSatNum[0]);

			for (int i = 0; i < 4 * Raw.DDObs.DDSatNum[0]; i++) // for rows
			{
				for (int j = i % 4; j < 4 * Raw.DDObs.DDSatNum[0]; j += 4)  // for coloums
				{
					if (i == j)
					{
						if (i % 4 < 2)
						{
							P_GPS(i, j) = W_GPS_ii_P;
						}
						else
						{
							P_GPS(i, j) = W_GPS_ii_L;
						}

					}
					else
					{
						if (i % 4 < 2)
						{
							P_GPS(i, j) = W_GPS_ij_P;
						}
						else
						{
							P_GPS(i, j) = W_GPS_ij_L;
						}
					}
				}
			}

			// Fuse GPS and BDS Parts of P
			P.topLeftCorner(4 * Raw.DDObs.DDSatNum[0], 4 * Raw.DDObs.DDSatNum[0]) = P_GPS;
		}
		
		
		// BDS-Part of P
		Eigen::MatrixXd P_BDS;
		if (Raw.DDObs.DDSatNum[1] > 0)
		{
			Eigen::MatrixXd P_BDS = Eigen::MatrixXd::Zero(4 * Raw.DDObs.DDSatNum[1], 4 * Raw.DDObs.DDSatNum[1]);
			for (int i = 0; i < 4 * Raw.DDObs.DDSatNum[1]; i++) // for rows
			{
				for (int j = i % 4; j < 4 * Raw.DDObs.DDSatNum[1]; j += 4)  // for coloums
				{
					if (i == j)
					{
						if (i % 4 < 2)
						{
							P_BDS(i, j) = W_BDS_ii_P;
						}
						else
						{
							P_BDS(i, j) = W_BDS_ii_L;
						}
					}
					else
					{
						if (i % 4 < 2)
						{
							P_BDS(i, j) = W_BDS_ij_P;
						}
						else
						{
							P_BDS(i, j) = W_BDS_ij_L;
						}
					}
				}
			}

			// Fuse GPS and BDS Parts of P
			P.bottomRightCorner(4 * Raw.DDObs.DDSatNum[1], 4 * Raw.DDObs.DDSatNum[1]) = P_BDS;
		}
		
		

	
		
		



		/* ---------------- Solve the Least-Square Problem ---------------- */

		// Check if Observation is enough to Compute Unkonws
		if (B.rows() < B.cols())
		{
			cout << "Observation is not enough to Compute Unkonws!" << endl;
			return 0;
		}

		// LS Positioning
		Eigen::MatrixXd x;
		Eigen::MatrixXd v;

		Qxx = (B.transpose() * P * B).inverse();
		x = Qxx * B.transpose() * P * L;
		v = B * x - L;

		

		/* Print Matrix for Debug */
		//cout << "X: \n" << X << endl;
		//cout << "x: \n" << x << endl;
		//cout << "v: \n" << v.transpose() << endl;
		//cout << "B: \n" << B << endl << endl;
		//cout << "L: \n" << L << endl;
		//cout << "P: \n" << P.transpose() << endl;
		
		 

		// Update LS Result and Output	
		X += x;

		//cout << "X: \n" << X << endl;
		


		rover_pos.x = X(0, 0);
		rover_pos.y = X(1, 0);
		rover_pos.z = X(2, 0);


		// Output Iteration Result
		//cout << "Iter 1teration: " << iter_count << endl;
		//cout << "Correaction of Parameters: "<< x.norm() << endl;
		//cout << "Rover X: "<< rover_pos.x << endl;
		//cout << "Rover Y: "<< rover_pos.y << endl;
		//cout << "Rover Z: "<< rover_pos.z << endl;
		//
		//cout << "Baseline_x: " << rover_pos.x - base_pos.x << endl;
		//cout << "Baseline_y: " << rover_pos.y - base_pos.y << endl;
		//cout << "Baseline_z: " << rover_pos.z - base_pos.z << endl;

		iter_count++;
		new_solution_circle == false;


		// Check LS is Converged
		if ((x.norm() < 0.01 && iter_count > 3) || iter_count == max_iter_count)
		{
			// Error Dectection
			Eigen::MatrixXd Qvv = (P.inverse() - B * (B.transpose() * P * B).inverse() * B.transpose());

			int Biggest_Outlier = -1;
			double Biggest_Omega = 0;
			for (int i = 0; i < 4 * Raw.DDObs.Sats; i++)
			{
				double omega_i = fabs(v(i, 0) / sqrt(Qvv(i, i)));

				if (omega_i > 5 && omega_i > Biggest_Omega)
				{
					// In the Order of DD_OBS(Actually is the index in the SD_OBS, exp Refernce SD)
					Biggest_Outlier = (int)i / 4;  
					Biggest_Omega = omega_i;
				}
			}
			

			if (Biggest_Outlier != -1)
			{
				// Find the Corresponding SD OBS
				int SD_idx = SD_index[Biggest_Outlier];
				

				// Change Parameters for Solution
				Raw.SdObs.SdSatObs[SD_idx].Valid_status = 0;
				Raw.DDObs.Sats--;
				Raw.DDObs.DDSatNum[Raw.SdObs.SdSatObs[SD_idx].System-1]--;

				X.resize(3 + 2 * Raw.DDObs.Sats, 1);
				FloatAmb.resize(2 * Raw.DDObs.Sats);
				Qxx.resize(3 + 4 * Raw.DDObs.Sats, 3 + 4 * Raw.DDObs.Sats);


				//cout << "Outlier Detected!" << endl;
				cout << Raw.SdObs.SdSatObs[SD_idx].System <<  " " << Raw.SdObs.SdSatObs[SD_idx].Prn << " is Outlier!" << endl;

				// If Unconverged
				if (iter_count == max_iter_count)
				{
					max_iter_count += 5;
				}
				

				// Note the Ambiguity Initialization Flag
				new_solution_circle = true;


				// If Converge into an Odd Place
				rover_pos.x = Raw.RovEpk.my_result.Pos.x;
				rover_pos.y = Raw.RovEpk.my_result.Pos.y;
				rover_pos.z = Raw.RovEpk.my_result.Pos.z;


				iter_count++;

				continue;
			}



			// Store Info And Return
			Raw.RovEpk.my_result.Pos = rover_pos;
			Raw.DDObs.dPos.x = rover_pos.x - base_pos.x;
			Raw.DDObs.dPos.y = rover_pos.y - base_pos.y;
			Raw.DDObs.dPos.z = rover_pos.z - base_pos.z;
			
			// Retuen Float Ambiguity and Q_NN
			for (int i = 0; i < 2 * Raw.DDObs.Sats; i++)
			{
				FloatAmb(i, 0) = X(3 + i, 0);
			}

			break;

			
		}

		
	}


	return 1;
}


int RTK_FIX(RTK_RAW& Raw, Eigen::VectorXd& FloatAmb, Eigen::MatrixXd& Qxx)
{
	int n = FloatAmb.rows();	 // number of float solutions
	int m = 2;					 // number of fixed solutions to search for


	Eigen::MatrixXd FixAmb = Eigen::MatrixXd::Zero(n, 1);
	Eigen::VectorXd sq_nroms = Eigen::VectorXd::Zero(2);

	// Covariance Matrix for Extraction
	Eigen::MatrixXd Q_BL	 = Qxx.block(0, 0, 3, 3);
	Eigen::MatrixXd Q_Amb	 = Qxx.block(3, 3, 2 * Raw.DDObs.Sats, 2 * Raw.DDObs.Sats);
	Eigen::MatrixXd Q_BL_Amb = Qxx.block(0, 3, 3, 2 * Raw.DDObs.Sats);

	
	// Call Lambda Function -- return 0 if success
	int result = lambda(n, 2, FloatAmb, Q_Amb, FixAmb, sq_nroms);


	// Check if Success
	if (result != 0)
	{
		cout << "No Fix Solution Found!" << endl;
		return 0;
	}
	

	// Calculate and Check Ratio
	Raw.DDObs.ResAmb[0] = sq_nroms(0);
	Raw.DDObs.ResAmb[1] = sq_nroms(1);
	Raw.DDObs.Ratio = sq_nroms(1) / sq_nroms(0);

	if (Raw.DDObs.Ratio < 3)
	{
		cout << "Ratio is too small: Ratio = "<< Raw.DDObs.Ratio << endl;
		return 0;
	}

	// cout << "Ratio = " << Raw.DDObs.Ratio << endl;


	// Calculate Fix Solution for Baseline
	Eigen::MatrixXd BL_flt = Eigen::MatrixXd::Zero(3, 1);
	BL_flt << Raw.DDObs.dPos.x, Raw.DDObs.dPos.y, Raw.DDObs.dPos.z;

	//cout << BL_flt << endl;

	// X_BL = X_BL - Q_BL_Amb * Q_Amb \ (Amb_Flt - Amb_Fix) ---> (3, 2) - (3, 3) * (N, N) * (N, 2)
	Eigen::MatrixXd BL_fix = BL_flt.replicate(1,2) - Q_BL_Amb * Q_Amb.inverse() * (FloatAmb.replicate(1, 2) - FixAmb);


	// Calculate Fix Solution for Baseline's RMS (Cause Ambuiguity is Fixed and seem as no Varience )
	// Q_BL = Q_BL - Q_BL_Amb * Q_Amb \ Q_BL_Amb'   ---> (3, 3) - (3, N) * (N, N) * (N, 3)
	Eigen::MatrixXd  Q_BL_fix = Q_BL - Q_BL_Amb * Q_Amb.inverse() * Q_BL_Amb.transpose();
	double Sigma_BL = sqrt(Q_BL_fix(0, 0) + Q_BL_fix(1, 1) + Q_BL_fix(2, 2));


	// cout << "Fixed Solution of BaseLine: "<<Raw.SdObs.Time.SecOfWeek<<"  " << BL_fix(0, 0) << "  " << BL_fix(1, 0) << "  " << BL_fix(2, 0) << endl;
	// cout << "Fixed BaseLine Varicance  "<< Sigma_BL << endl;


	// Stroe Fixed Solution
	for (int i = 0; i < n; i++)
	{
		Raw.DDObs.FixedAmb[i] = FixAmb(i, 0);
		Raw.DDObs.FixedAmb[2 * i] = FixAmb(i, 1);
	}

	Raw.DDObs.dPos.x = BL_fix(0, 0);
	Raw.DDObs.dPos.y = BL_fix(1, 0);
	Raw.DDObs.dPos.z = BL_fix(2, 0);

	Raw.DDObs.bFixed = true;

	return 1;

}




int RTK_LS()
{
    // Create Data Objects
    RTK_RAW Raw = RTK_RAW();
	

	// Create Input Handle
	//InputHandle base_src(SOURCE_FILE_1);
	//InputHandle rover_src(SOURCE_FILE_2);

	InputHandle base_src("47.114.134.129", 7190);
	InputHandle rover_src("8.148.22.229", 4002);

	// Create Output FileStream
	ofstream output_file(OUTPUT_FILE);


	XYZ_Coord Bas_pos(0, 0, 0);
	XYZ_Coord Rov_pos(0, 0, 0);
	int all_count = 0;
	int fix_count = 0;

	// Loop to Fetch Data
	while (true)
	{
		// Synchronous Status
		//Sleep(980);
		int syn_status = GetSynObs(rover_src, base_src, Raw);

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

		
		/* -------- Syn_states == 1 ---> Start RTK Progress -------- */
		
		Check_Raw_Obs(Raw.BasEpk, Raw.RovEpk);

		if(!SPP(Raw.BasEpk, Raw.GpsEph, Raw.BdsEph, Bas_pos))
		{
			cout << "Not Enough Observation For BaseStation!" << endl;
			continue;
		}

		if (!SPP(Raw.RovEpk, Raw.GpsEph, Raw.BdsEph, Rov_pos))
		{
			cout << "Not Enough Observation For Rover!" << endl;
			continue;
		}

		all_count++;

		Construct_SD_Obs(Raw.BasEpk, Raw.RovEpk, Raw.SdObs);
		Check_SD_Obs(Raw.SdObs);

		if (!Seletct_Ref_Sat(Raw.BasEpk, Raw.RovEpk, Raw.SdObs, Raw.DDObs))
		{
			cout << "No Reference Satellite Selected!" << endl;
			continue;
		}

		Eigen::VectorXd FloatAmb = Eigen::VectorXd::Zero(2 * Raw.DDObs.Sats);
		Eigen::MatrixXd Qxx = Eigen::MatrixXd::Zero(3 + 2 * Raw.DDObs.Sats, 3 + 2 * Raw.DDObs.Sats);

	
		if (!RTK_Float(Raw, FloatAmb, Qxx))
		{
			cout << "No Enough Observation for RTK_Float Solution!" << endl;
			continue;
		}

		if (Calc_XYZ_dist(Raw.RovEpk.my_result.Pos, Rov_pos) > 1e3)
		{
			cout << "RTK_Float Failed!" << endl;
			continue;
		}


		//cout << "Float Solution of BaseLine: " << Raw.SdObs.Time.SecOfWeek << "  " << Raw.DDObs.dPos.x << "  " << Raw.DDObs.dPos.y << "  " << Raw.DDObs.dPos.z << endl;


		if (!RTK_FIX(Raw, FloatAmb, Qxx))
		{
			cout << "Failed to Fix Ambiguity!" << endl;
			continue;
		}

		cout << "Fixed Solution of BaseLine: " << Raw.SdObs.Time.SecOfWeek << "  " << Raw.DDObs.dPos.x << "  " << Raw.DDObs.dPos.y << "  " << Raw.DDObs.dPos.z << "  " << Raw.DDObs.ref_prn[0] << "  " << Raw.DDObs.ref_prn[1] << endl;

		// Update Position
		Raw.RovEpk.my_result.Pos.x = Raw.DDObs.dPos.x + Raw.BasEpk.rcv_result.Pos.x;
		Raw.RovEpk.my_result.Pos.y = Raw.DDObs.dPos.y + Raw.BasEpk.rcv_result.Pos.y;
		Raw.RovEpk.my_result.Pos.z = Raw.DDObs.dPos.z + Raw.BasEpk.rcv_result.Pos.z;

		BLH_Coord Rover_blh;
		XYZ2BLH(Raw.RovEpk.my_result.Pos, Rover_blh, WGS84);

		fix_count++;

		// cout << "Fixed Solution of Rover   : " << Raw.RovEpk.my_result.Pos.x << "  " << Raw.RovEpk.my_result.Pos.y << "  " << Raw.RovEpk.my_result.Pos.z << endl;
		// cout << "Fixed Solution of BaseLine: " << Raw.SdObs.Time.SecOfWeek << "  " << Raw.DDObs.dPos.x << "  " << Raw.DDObs.dPos.y << "  " << Raw.DDObs.dPos.z << endl;

		string fix_result;
		char fix_buffer[256];
		sprintf(fix_buffer, "%6d %13.3f %30.25f %30.25f %15.7f %2d\n",
			Raw.RovEpk.Time.Week, Raw.RovEpk.Time.SecOfWeek,
			Rover_blh.B, Rover_blh.L, Rover_blh.H, Raw.DDObs.bFixed);

		fix_result = fix_buffer;

		output_file << fix_result;
		
		cout << "Fix Ratio: " << (double)fix_count / all_count * 100 << "%" << endl;
	}
	
	cout << "All Count: " << all_count << endl;
	cout << "Fix Count: " << fix_count << endl;
	cout << "Fix Ratio: " << (double)fix_count / all_count * 100 << "%" << endl;

	return 1;
  
}



void RTK_EKF_INIT(RTK_RAW& raw, RTK_EKF& kf)
{
	/* ------- Initialize the State--Position ---------- */ 
	kf.Pos.x = raw.RovEpk.my_result.Pos.x;
	kf.Pos.y = raw.RovEpk.my_result.Pos.y;
	kf.Pos.z = raw.RovEpk.my_result.Pos.z;

	/* ------- Initialize the State--Ambiguity ---------- */ 
	int gps_amb_count = 0;
	int bds_amb_count = raw.DDObs.DDSatNum[0];

	// For Loop to Init Amb -- j as not RefSat
	for (int j = 0; j < raw.SdObs.SatNum; j++)
	{
		// Does not Estimate Ambiguity
		if (raw.SdObs.SdSatObs[j].Valid_status != 2)
		{
			continue;
		}

		// Check if is Ref Satellite
		if (raw.DDObs.ref_idx[0] == j || raw.DDObs.ref_idx[1] == j)
		{
			continue;
		}


		// For GPS
		if (raw.SdObs.SdSatObs[j].System == GPS)
		{
			Amb_Info amb_info;
			amb_info.index = gps_amb_count;  // To Find the Corresponding Sit in Matrix P
			amb_info.value[0] = raw.DDObs.FixedAmb[2 * gps_amb_count];
			amb_info.value[1] = raw.DDObs.FixedAmb[2 * gps_amb_count + 1];

			// Add Amb Info into the Map:  Prn <--> Value & Index
			kf.GPS_Amb[raw.SdObs.SdSatObs[j].Prn] = amb_info;

			gps_amb_count++;
		}
		// For BDS
		else if (raw.SdObs.SdSatObs[j].System == BDS)
		{
			Amb_Info amb_info;
			amb_info.index = bds_amb_count; // To Find the Corresponding Sit in Matrix P
			amb_info.value[0] = raw.DDObs.FixedAmb[2 * bds_amb_count];
			amb_info.value[1] = raw.DDObs.FixedAmb[2 * bds_amb_count + 1];

			// Add Amb Info into the Map:  Prn <--> Value & Index
			kf.BDS_Amb[raw.SdObs.SdSatObs[j].Prn] = amb_info;

			bds_amb_count++;
		}
	}


	/* ------- Initialize the State Covariance ---------- */
	kf.P.resize(3 + 2 * raw.DDObs.Sats, 3 + 2 * raw.DDObs.Sats);
	kf.P.setZero();
	
	// Position Covariance -- fixed
	kf.P.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * 1e-3;

	// Ambiguity Covariance -- fixed
	kf.P.block(3, 3, 2 * raw.DDObs.Sats, 2 * raw.DDObs.Sats) = Eigen::MatrixXd::Identity(2 * raw.DDObs.Sats, 2 * raw.DDObs.Sats) * 900;


	/* ------ Store "Last" Epoch Info ------ */
	kf.Last_DDObs = raw.DDObs;
	kf.isInit = true;
}




void check_RefChange(RTK_RAW& raw, RTK_EKF& kf, int* d_ref)
{
	// For GPS Ref
	if (raw.DDObs.ref_prn[0] == kf.Last_DDObs.ref_prn[0])
	{
		d_ref[0] = -1;		// Reference Satellite is not Changed
	}
	else
	{
		// Reference Satellite is Changed, Find the Corresponding Index in the Last Ambiguity Sqeuence
		Amb_Map::const_iterator gps_ref = kf.GPS_Amb.find(raw.DDObs.ref_prn[0]);

		// Check if the Reference Satellite is in the Map
		if (gps_ref == kf.GPS_Amb.end())
		{
			cout << "GPS Reference Satellite is not in the Map!" << endl;
			d_ref[0] = -2;
		}
		else
		{
			// Set New Index of Ref Satellite in Last Ambiguity Sqeuence
			d_ref[0] = gps_ref->second.index;
		}


	}

	// For BDS Ref
	if (raw.DDObs.ref_prn[1] == kf.Last_DDObs.ref_prn[1])
	{
		d_ref[1] = -1;		// Reference Satellite is not Changed
	}
	else
	{
		// Reference Satellite is Changed, Find the Corresponding Index in the Last Ambiguity Sqeuence
		Amb_Map::const_iterator bds_ref = kf.BDS_Amb.find(raw.DDObs.ref_prn[1]);
		// Check if the Reference Satellite is in the Map
		if (bds_ref == kf.BDS_Amb.end())
		{
			cout << "BDS Reference Satellite is not in the Map!" << endl;
			d_ref[1] = -2;
		}
		else
		{
			// Set New Index of Ref Satellite in Last Ambiguity Sqeuence
			d_ref[1] = bds_ref->second.index;
		}

	}

}



/* Process Ambiguity State for EKF --------------------------------------
* args   :
*          RTK_RAW&            Raw           I    raw observation data
*          RTK_EKF&            EKF           IO   EKF state and parameters
*          int*                d_ref         I    reference satellite change flags
*          Eigen::MatrixXd&    X             O    state vector
*          Eigen::MatrixXd&    Phi_Amb       O    state transition matrix for ambiguities
*          Eigen::MatrixXd&    Q             O    process noise matrix
*
* return : void
*
* note   : d_ref[0/1]: -2 for Not Found, -1 for Not Changed, >=0 for the corresponding
*          index in the last ambiguity sequence
*-----------------------------------------------------------------------------*/
void ProcessAmbiguityState(RTK_RAW& Raw, RTK_EKF& EKF, int* d_ref,
	Eigen::MatrixXd& X, Eigen::MatrixXd& Phi_Amb,
	Eigen::MatrixXd& Q)
{
	// Temporary Matrix for consider GPS/BDS Separately
	int GPS_idx = 0;
	int BDS_idx = 2 * Raw.DDObs.DDSatNum[0];

	// Initialize Phi_Amb (if not done by caller)
	if (Phi_Amb.rows() != 2 * Raw.DDObs.Sats || Phi_Amb.cols() != 2 * EKF.Last_DDObs.Sats) {
		Phi_Amb = Eigen::MatrixXd::Zero(2 * Raw.DDObs.Sats, 2 * EKF.Last_DDObs.Sats);
	}

	// Loop through all satellites in SD observations
	for (int j = 0; j < Raw.SdObs.SatNum; j++)
	{
		// Skip satellites with cycle slips or invalid observations
		if (Raw.SdObs.SdSatObs[j].Valid_status != 2) {
			continue;
		}

		// Skip reference satellites
		if (Raw.DDObs.ref_idx[0] == j || Raw.DDObs.ref_idx[1] == j) {
			continue;
		}

		// Process GPS satellites
		if (Raw.SdObs.SdSatObs[j].System == GPS)
		{
			// Find the corresponding ambiguity in the previous epoch
			Amb_Map::const_iterator last_gps = EKF.GPS_Amb.find(Raw.SdObs.SdSatObs[j].Prn);

			// Satellite found in previous epoch and valid reference
			if (last_gps != EKF.GPS_Amb.end() && d_ref[0] >= -1)
			{
				// Fill state transition matrix
				if (d_ref[0] == -1) {
					// Reference satellite unchanged
					Phi_Amb(GPS_idx, 2 * last_gps->second.index) = 1.0;
					Phi_Amb(GPS_idx + 1, 2 * last_gps->second.index + 1) = 1.0;

					// Initialize state vector with previous values
					X(3 + GPS_idx, 0) = last_gps->second.value[0];
					X(3 + GPS_idx + 1, 0) = last_gps->second.value[1];
				}
				else {
					// Reference satellite changed
					Phi_Amb(GPS_idx, 2 * d_ref[0]) = -1.0;
					Phi_Amb(GPS_idx, 2 * last_gps->second.index) = 1.0;

					Phi_Amb(GPS_idx + 1, 2 * d_ref[0] + 1) = -1.0;
					Phi_Amb(GPS_idx + 1, 2 * last_gps->second.index + 1) = 1.0;


					// Initialize state vector with previous values
					X(3 + GPS_idx, 0) = last_gps->second.value[0] - EKF.GPS_Amb[Raw.DDObs.ref_prn[0]].value[0];
					X(3 + GPS_idx + 1, 0) = last_gps->second.value[1] - EKF.GPS_Amb[Raw.DDObs.ref_prn[0]].value[1];
				}

				// Set low process noise for existing ambiguities
				if (EKF.Last_DDObs.bFixed)
				{
					Q(3 + GPS_idx, 3 + GPS_idx) = 1e-8;
					Q(3 + GPS_idx + 1, 3 + GPS_idx + 1) = 1e-8;
				}
				else
				{
					Q(3 + GPS_idx, 3 + GPS_idx) = 3;
					Q(3 + GPS_idx + 1, 3 + GPS_idx + 1) = 3;
				}

				

				// Mark ambiguity as used in this epoch
				EKF.GPS_Amb[Raw.SdObs.SdSatObs[j].Prn].isUsed = true;
			}
			else
			{
				// New satellite or invalid reference - leave Phi_Amb at zero


				// Initialize ambiguity from code-carrier difference
				double lambda1 = CLIGHT / FREQ_GPS_L1;
				double lambda2 = CLIGHT / FREQ_GPS_L2;

				double L_f1 = Raw.SdObs.SdSatObs[j].dL[0] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[0]].dL[0];
				L_f1 *= lambda1;   // Carrier phase in meters
				double L_f2 = Raw.SdObs.SdSatObs[j].dL[1] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[0]].dL[1];
				L_f2 *= lambda2;   // Carrier phase in meters

				double P_f1 = Raw.SdObs.SdSatObs[j].dP[0] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[0]].dP[0];
				double P_f2 = Raw.SdObs.SdSatObs[j].dP[1] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[0]].dP[1];
				
				// Set initial ambiguity values ---> Double Difference Ambiguity
				X(3 + GPS_idx, 0) = (L_f1 - P_f1) / lambda1;
				X(3 + GPS_idx + 1, 0) = (L_f2 - P_f2) / lambda2;

				// Set high process noise to reinitialize ambiguity
				Q(3 + GPS_idx, 3 + GPS_idx) = 36.0 / (lambda1 * lambda1);
				Q(3 + GPS_idx + 1, 3 + GPS_idx + 1) = 36.0 / (lambda2 * lambda2);

			}

			// Move to next GPS ambiguity position
			GPS_idx += 2;
		}
		// Process BDS satellites
		else if (Raw.SdObs.SdSatObs[j].System == BDS)
		{
			// Find the corresponding ambiguity in the previous epoch
			Amb_Map::const_iterator last_bds = EKF.BDS_Amb.find(Raw.SdObs.SdSatObs[j].Prn);

			// Satellite found in previous epoch and valid reference
			if (last_bds != EKF.BDS_Amb.end() && d_ref[1] >= -1)
			{
				// Fill state transition matrix
				if (d_ref[1] == -1) {
					// Reference satellite unchanged
					Phi_Amb(BDS_idx, 2 * last_bds->second.index) = 1.0;
					Phi_Amb(BDS_idx + 1, 2 * last_bds->second.index + 1) = 1.0;


					// Initialize state vector with previous values
					X(3 + BDS_idx, 0) = last_bds->second.value[0];
					X(3 + BDS_idx + 1, 0) = last_bds->second.value[1];
				}
				else {
					// Reference satellite changed
					Phi_Amb(BDS_idx, 2 * d_ref[1]) = -1.0;
					Phi_Amb(BDS_idx, 2 * last_bds->second.index) = 1.0;

					Phi_Amb(BDS_idx + 1, 2 * d_ref[1] + 1) = -1.0;
					Phi_Amb(BDS_idx + 1, 2 * last_bds->second.index + 1) = 1.0;

					// Initialize state vector with previous values
					X(3 + BDS_idx, 0) = last_bds->second.value[0] - EKF.BDS_Amb[Raw.DDObs.ref_prn[1]].value[0];
					X(3 + BDS_idx + 1, 0) = last_bds->second.value[1] - EKF.BDS_Amb[Raw.DDObs.ref_prn[1]].value[1];
				}

				// Set low process noise for existing ambiguities
				if (EKF.Last_DDObs.bFixed)
				{
					Q(3 + BDS_idx, 3 + BDS_idx) = 1e-8;
					Q(3 + BDS_idx + 1, 3 + BDS_idx + 1) = 1e-8;
				}
				else
				{
					Q(3 + BDS_idx, 3 + BDS_idx) = 3;
					Q(3 + BDS_idx + 1, 3 + BDS_idx + 1) = 3;
				}


			
			}
			else
			{
				// New satellite or invalid reference - leave Phi_Amb at zero
				
				// Initialize ambiguity from code-carrier difference
				double lambda1 = CLIGHT / FREQ_BDS_B1;
				double lambda2 = CLIGHT / FREQ_BDS_B3;

				double L_f1 = Raw.SdObs.SdSatObs[j].dL[0] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[1]].dL[0];
				L_f1 *= lambda1;   // Carrier phase in meters
				double L_f2 = Raw.SdObs.SdSatObs[j].dL[1] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[1]].dL[1];
				L_f2 *= lambda2;   // Carrier phase in meters

				double P_f1 = Raw.SdObs.SdSatObs[j].dP[0] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[1]].dP[0];
				double P_f2 = Raw.SdObs.SdSatObs[j].dP[1] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[1]].dP[1];

				// Set initial ambiguity values
				X(3 + BDS_idx, 0) = (L_f1 - P_f1) / lambda1;
				X(3 + BDS_idx + 1, 0) = (L_f2 - P_f2) / lambda2;

				// Set high process noise to reinitialize ambiguity
				Q(3 + BDS_idx, 3 + BDS_idx) = 36.0 / (lambda1 * lambda1);
				Q(3 + BDS_idx + 1, 3 + BDS_idx + 1) = 36.0 / (lambda2 * lambda2);
			}

			// Move to next BDS ambiguity position
			BDS_idx += 2;
		}
	}
}



/* Time Update for Extended Kalman Filter ------------------------------------
* args   :
*          RTK_RAW&         Raw        I    raw observation data
*          RTK_EKF&         EKF        IO   Kalman filter state and covariance
*          Eigen::MatrixXd& X_k_k_1    O    predicted state vector
*          Eigen::MatrixXd& P_k_k_1    O    predicted state covariance matrix
*
* return : int  1 ---> Time Update Successful
*              0 ---> Failed to Update (reference satellite problem)
*
* note   : Performs the time update step of the Extended Kalman Filter
*          by handling reference satellite changes and predicting the state
*          vector and covariance matrix forward in time
*-----------------------------------------------------------------------------*/
int TimeEstimate(RTK_RAW& Raw, RTK_EKF& EKF,
	Eigen::MatrixXd& X_k_k_1, Eigen::MatrixXd& P_k_k_1)
{
	// Initialize state and transition matrices
	X_k_k_1 = Eigen::MatrixXd::Zero(3 + 2 * Raw.DDObs.Sats, 1);
	Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(3 + 2 * Raw.DDObs.Sats, 3 + 2 * EKF.Last_DDObs.Sats);
	Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(3 + 2 * Raw.DDObs.Sats, 3 + 2 * Raw.DDObs.Sats);

	// Check if the Reference Satellite has changed
	// d_ref[0/1]: -2 for Not Found, -1 for Not Changed, >=0 for the corresponding index in the last ambiguity sequence
	int d_ref[2] = { -1, -1 };
	check_RefChange(Raw, EKF, d_ref);

	// Position state transition (identity matrix for position components)
	Phi.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);

	// Handle different motion models
	if (EKF.state == 0)
	{
		// Kinematic model: use SPP to initialize position
		X_k_k_1(0, 0) = Raw.RovEpk.my_result.Pos.x;
		X_k_k_1(1, 0) = Raw.RovEpk.my_result.Pos.y;
		X_k_k_1(2, 0) = Raw.RovEpk.my_result.Pos.z;

		// Higher process noise for kinematic model
		Q.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * 100.0;
	}
	else if (EKF.state == 1 && EKF.Last_DDObs.bFixed == true)
	{
		// Static model: use previous EKF solution
		X_k_k_1(0, 0) = EKF.Pos.x;
		X_k_k_1(1, 0) = EKF.Pos.y;
		X_k_k_1(2, 0) = EKF.Pos.z;

		// Lower process noise for static model
		Q.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * 0.01;
	}
	else
	{
		// Unsupported motion model
		std::cout << "EKF State is not Correct!" << std::endl;
		return 0;
	}

	// Initialize ambiguity state transition matrix
	Eigen::MatrixXd Phi_Amb = Eigen::MatrixXd::Zero(2 * Raw.DDObs.Sats, 2 * EKF.Last_DDObs.Sats);

	// Process ambiguity states, handling satellite changes and initializing new ambiguities
	ProcessAmbiguityState(Raw, EKF, d_ref, X_k_k_1, Phi_Amb, Q);

	// Add ambiguity state transition to the full transition matrix
	Phi.block(3, 3, 2 * Raw.DDObs.Sats, 2 * EKF.Last_DDObs.Sats) = Phi_Amb;

	// Perform time update calculation using state transition equation:
	// P_k_k-1 = Phi * P_k-1_k-1 * Phi^T + Q
	P_k_k_1 = Phi * EKF.P * Phi.transpose() + Q;


	// Debugging: Print all matrices
	//std::cout << "X_k_k_1 (Predicted State Vector): \n" << X_k_k_1 << std::endl;
	//std::cout << "Phi (State Transition Matrix): \n" << Phi << std::endl;
	//cout << "EKF.P = \n" << EKF.P << endl;
	//std::cout << "Q (Process Noise Matrix): \n" << Q << std::endl;
	//std::cout << "P_k_k_1 (Predicted State Covariance Matrix): \n" << P_k_k_1 << std::endl;


	return 1;
}





/* Measurement Update for Extended Kalman Filter ------------------------------
* args   :
*          RTK_RAW&         Raw        I    raw observation data
*          Eigen::MatrixXd& X_k_k_1    I    predicted state vector
*          Eigen::MatrixXd& P_k_k_1    I    predicted state covariance matrix
*          Eigen::MatrixXd& X_k_k      O    updated state vector
*          Eigen::MatrixXd& P_k_k      O    updated state covariance matrix
*
* return : int  1 ---> Measurement Update Successful
*              0 ---> Failed to Update (not enough observations)
*
* note   : Performs the measurement update step of the Extended Kalman Filter
*          for RTK positioning using double difference observations
*-----------------------------------------------------------------------------*/
int MeasurementUpdate(RTK_RAW& Raw, const Eigen::MatrixXd& X_k_k_1, 
	const Eigen::MatrixXd& P_k_k_1,Eigen::MatrixXd& X_k_k, Eigen::MatrixXd& P_k_k)
{
	// Get rover position from state vector
	XYZ_Coord rover_pos(X_k_k_1(0, 0), X_k_k_1(1, 0), X_k_k_1(2, 0));
	XYZ_Coord base_pos(Raw.BasEpk.rcv_result.Pos);

	// Get Initial Position of BASE from Reference Position
	if (Raw.BasEpk.rcv_result.Pos.x == 0.0)
	{
		// Get Initial Position of BASE from SPP
		std::cout << "No Base Position Reference Solution!" << std::endl;
		base_pos = Raw.BasEpk.my_result.Pos;
	}


	// Calculate base station to satellites distances
	double rou_B0[MAX_CHANNEL_NUM]{};
	for (int i = 0; i < Raw.BasEpk.SatNum; i++)
	{
		rou_B0[i] = Calc_XYZ_dist(Raw.BasEpk.SatPVT[i].SatPos, base_pos);
	}

	// Find reference satellite indices
	int RefIdx_Rov_GPS = 0, RefIdx_Bas_GPS = 0, RefIdx_Rov_BDS = 0, RefIdx_Bas_BDS = 0;

	if (Raw.DDObs.DDSatNum[0] > 0)
	{
		RefIdx_Rov_GPS = Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[0]].Rov_idx;
		RefIdx_Bas_GPS = Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[0]].Bas_idx;
	}

	if (Raw.DDObs.DDSatNum[1] > 0)
	{
		RefIdx_Rov_BDS = Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[1]].Rov_idx;
		RefIdx_Bas_BDS = Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[1]].Bas_idx;
	}

	// Calculate rover to satellites distances
	double rou_R0[MAX_CHANNEL_NUM]{};
	for (int i = 0; i < Raw.RovEpk.SatNum; i++)
	{
		rou_R0[i] = Calc_XYZ_dist(Raw.RovEpk.SatPVT[i].SatPos, rover_pos);
	}

	// Construct observation matrices H and measurement noise matrix R
	int obs_count = 4 * Raw.DDObs.Sats; // Each satellite has 4 observations (P1,P2,L1,L2)

	// Check if enough observations are available
	if (obs_count == 0)
	{
		return 0; // Not enough observations
	}

	Eigen::MatrixXd H = Eigen::MatrixXd::Zero(obs_count, 3 + 2 * Raw.DDObs.Sats);
	Eigen::MatrixXd R = Eigen::MatrixXd::Zero(obs_count, obs_count);
	Eigen::MatrixXd Z = Eigen::MatrixXd::Zero(obs_count, 1);      // Actual measurements
	Eigen::MatrixXd Z_pred = Eigen::MatrixXd::Zero(obs_count, 1); // Predicted measurements
	Eigen::MatrixXd V = Eigen::MatrixXd::Zero(obs_count, 1);      // Innovation (measurement residual)

	// Calculate weights for pseudorange and carrier phase measurements
	double R_GPS_ii_P = 2.0 * delta_pseudo * delta_pseudo * (Raw.DDObs.DDSatNum[0]);
	double R_GPS_ij_P = 2.0 * delta_pseudo * delta_pseudo;
	double R_BDS_ii_P = 2.0 * delta_pseudo * delta_pseudo * (Raw.DDObs.DDSatNum[1]);
	double R_BDS_ij_P = 2.0 * delta_pseudo * delta_pseudo;

	double R_GPS_ii_L = 2.0 * delta_phase * delta_phase * (Raw.DDObs.DDSatNum[0]);
	double R_GPS_ij_L = 2.0 * delta_phase * delta_phase;
	double R_BDS_ii_L = 2.0 * delta_phase * delta_phase * (Raw.DDObs.DDSatNum[1]);
	double R_BDS_ij_L = 2.0 * delta_phase * delta_phase;

	// Indices for matrix filling
	int GPS_idx = 0;
	int BDS_idx = 4 * Raw.DDObs.DDSatNum[0];

	// Loop through all single-difference observations
	for (int j = 0; j < Raw.SdObs.SatNum; j++)
	{
		// Skip reference satellites
		if (Raw.DDObs.ref_idx[0] == j || Raw.DDObs.ref_idx[1] == j)
		{
			continue;
		}

		// Skip invalid observations
		if (Raw.SdObs.SdSatObs[j].Valid_status != 2)
		{
			continue;
		}

		// Process GPS satellites
		if (Raw.SdObs.SdSatObs[j].System == GPS)
		{
			// Set frequency and wavelength
			double freq1 = FREQ_GPS_L1;
			double lambda1 = CLIGHT / FREQ_GPS_L1;
			double freq2 = FREQ_GPS_L2;
			double lambda2 = CLIGHT / FREQ_GPS_L2;

			// Calculate double difference geometric distances
			double rou_R0_i = rou_R0[RefIdx_Rov_GPS];
			double rou_R0_j = rou_R0[Raw.SdObs.SdSatObs[j].Rov_idx];
			double rou_BR_i = rou_R0_i - rou_B0[RefIdx_Bas_GPS];
			double rou_BR_j = rou_R0_j - rou_B0[Raw.SdObs.SdSatObs[j].Bas_idx];
			double rou_BR_ij = rou_BR_j - rou_BR_i;

			// Construct double difference observations
			double P_f1 = Raw.SdObs.SdSatObs[j].dP[0] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[0]].dP[0];
			double P_f2 = Raw.SdObs.SdSatObs[j].dP[1] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[0]].dP[1];
			double L_f1 = Raw.SdObs.SdSatObs[j].dL[0] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[0]].dL[0];
			L_f1 *= lambda1;   // Convert carrier phase to meters
			double L_f2 = Raw.SdObs.SdSatObs[j].dL[1] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[0]].dL[1];
			L_f2 *= lambda2;   // Convert carrier phase to meters

			// Current ambiguity index
			int amb_idx = GPS_idx / 4;

			// Fill Jacobian matrix H with partial derivatives
			for (int i = 0; i < 4; i++)
			{
				// Position partial derivatives
				H(GPS_idx + i, 0) = (rover_pos.x - Raw.RovEpk.SatPVT[Raw.SdObs.SdSatObs[j].Rov_idx].SatPos.x) / rou_R0_j
					- (rover_pos.x - Raw.RovEpk.SatPVT[RefIdx_Rov_GPS].SatPos.x) / rou_R0_i;

				H(GPS_idx + i, 1) = (rover_pos.y - Raw.RovEpk.SatPVT[Raw.SdObs.SdSatObs[j].Rov_idx].SatPos.y) / rou_R0_j
					- (rover_pos.y - Raw.RovEpk.SatPVT[RefIdx_Rov_GPS].SatPos.y) / rou_R0_i;

				H(GPS_idx + i, 2) = (rover_pos.z - Raw.RovEpk.SatPVT[Raw.SdObs.SdSatObs[j].Rov_idx].SatPos.z) / rou_R0_j
					- (rover_pos.z - Raw.RovEpk.SatPVT[RefIdx_Rov_GPS].SatPos.z) / rou_R0_i;
			}

			// Ambiguity partial derivatives
			H(GPS_idx + 2, 3 + 2 * amb_idx) = lambda1;      // L1 ambiguity
			H(GPS_idx + 3, 3 + 2 * amb_idx + 1) = lambda2;  // L2 ambiguity

			// Fill measurement vector Z
			Z(GPS_idx, 0) = P_f1;
			Z(GPS_idx + 1, 0) = P_f2;
			Z(GPS_idx + 2, 0) = L_f1;
			Z(GPS_idx + 3, 0) = L_f2;

			// Fill predicted measurement vector Z_pred
			Z_pred(GPS_idx, 0) = rou_BR_ij;
			Z_pred(GPS_idx + 1, 0) = rou_BR_ij;
			Z_pred(GPS_idx + 2, 0) = rou_BR_ij + lambda1 * X_k_k_1(3 + 2 * amb_idx, 0);
			Z_pred(GPS_idx + 3, 0) = rou_BR_ij + lambda2 * X_k_k_1(3 + 2 * amb_idx + 1, 0);

			// Move to next GPS observation index
			GPS_idx += 4;
		}
		// Process BDS satellites
		else if (Raw.SdObs.SdSatObs[j].System == BDS)
		{
			// Set frequency and wavelength
			double freq1 = FREQ_BDS_B1;
			double lambda1 = CLIGHT / FREQ_BDS_B1;
			double freq2 = FREQ_BDS_B3;
			double lambda2 = CLIGHT / FREQ_BDS_B3;

			// Calculate double difference geometric distances
			double rou_R0_i = rou_R0[RefIdx_Rov_BDS];
			double rou_R0_j = rou_R0[Raw.SdObs.SdSatObs[j].Rov_idx];
			double rou_BR_i = rou_R0_i - rou_B0[RefIdx_Bas_BDS];
			double rou_BR_j = rou_R0_j - rou_B0[Raw.SdObs.SdSatObs[j].Bas_idx];
			double rou_BR_ij = rou_BR_j - rou_BR_i;

			// Construct double difference observations
			double P_f1 = Raw.SdObs.SdSatObs[j].dP[0] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[1]].dP[0];
			double P_f2 = Raw.SdObs.SdSatObs[j].dP[1] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[1]].dP[1];
			double L_f1 = Raw.SdObs.SdSatObs[j].dL[0] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[1]].dL[0];
			L_f1 *= lambda1;   // Convert carrier phase to meters
			double L_f2 = Raw.SdObs.SdSatObs[j].dL[1] - Raw.SdObs.SdSatObs[Raw.DDObs.ref_idx[1]].dL[1];
			L_f2 *= lambda2;   // Convert carrier phase to meters

			// Current ambiguity index
			int amb_idx = BDS_idx / 4;

			// Fill Jacobian matrix H with partial derivatives
			for (int i = 0; i < 4; i++)
			{
				// Position partial derivatives
				H(BDS_idx + i, 0) = (rover_pos.x - Raw.RovEpk.SatPVT[Raw.SdObs.SdSatObs[j].Rov_idx].SatPos.x) / rou_R0_j
					- (rover_pos.x - Raw.RovEpk.SatPVT[RefIdx_Rov_BDS].SatPos.x) / rou_R0_i;

				H(BDS_idx + i, 1) = (rover_pos.y - Raw.RovEpk.SatPVT[Raw.SdObs.SdSatObs[j].Rov_idx].SatPos.y) / rou_R0_j
					- (rover_pos.y - Raw.RovEpk.SatPVT[RefIdx_Rov_BDS].SatPos.y) / rou_R0_i;

				H(BDS_idx + i, 2) = (rover_pos.z - Raw.RovEpk.SatPVT[Raw.SdObs.SdSatObs[j].Rov_idx].SatPos.z) / rou_R0_j
					- (rover_pos.z - Raw.RovEpk.SatPVT[RefIdx_Rov_BDS].SatPos.z) / rou_R0_i;
			}

			// Ambiguity partial derivatives
			H(BDS_idx + 2, 3 + 2 * amb_idx) = lambda1;      // L1 ambiguity
			H(BDS_idx + 3, 3 + 2 * amb_idx + 1) = lambda2;  // L2 ambiguity

			// Fill measurement vector Z
			Z(BDS_idx, 0) = P_f1;
			Z(BDS_idx + 1, 0) = P_f2;
			Z(BDS_idx + 2, 0) = L_f1;
			Z(BDS_idx + 3, 0) = L_f2;

			// Fill predicted measurement vector Z_pred
			Z_pred(BDS_idx, 0) = rou_BR_ij;
			Z_pred(BDS_idx + 1, 0) = rou_BR_ij;
			Z_pred(BDS_idx + 2, 0) = rou_BR_ij + lambda1 * X_k_k_1(3 + 2 * amb_idx, 0);
			Z_pred(BDS_idx + 3, 0) = rou_BR_ij + lambda2 * X_k_k_1(3 + 2 * amb_idx + 1, 0);

			// Move to next BDS observation index
			BDS_idx += 4;
		}
	}

	// Construct R matrix (measurement noise covariance)
	// GPS part
	if (Raw.DDObs.DDSatNum[0] > 0)
	{
		Eigen::MatrixXd R_GPS = Eigen::MatrixXd::Zero(4 * Raw.DDObs.DDSatNum[0], 4 * Raw.DDObs.DDSatNum[0]);

		for (int i = 0; i < 4 * Raw.DDObs.DDSatNum[0]; i++) // rows
		{
			for (int j = i % 4; j < 4 * Raw.DDObs.DDSatNum[0]; j += 4)  // columns
			{
				if (i == j)
				{
					if (i % 4 < 2) // Pseudorange observations
					{
						R_GPS(i, j) = R_GPS_ii_P;
					}
					else // Carrier phase observations
					{
						R_GPS(i, j) = R_GPS_ii_L;
					}
				}
				else
				{
					if (i % 4 < 2) // Pseudorange observations
					{
						R_GPS(i, j) = R_GPS_ij_P;
					}
					else // Carrier phase observations
					{
						R_GPS(i, j) = R_GPS_ij_L;
					}
				}
			}
		}

		// Add GPS part to R matrix
		R.topLeftCorner(4 * Raw.DDObs.DDSatNum[0], 4 * Raw.DDObs.DDSatNum[0]) = R_GPS;
	}

	// BDS part
	if (Raw.DDObs.DDSatNum[1] > 0)
	{
		Eigen::MatrixXd R_BDS = Eigen::MatrixXd::Zero(4 * Raw.DDObs.DDSatNum[1], 4 * Raw.DDObs.DDSatNum[1]);

		for (int i = 0; i < 4 * Raw.DDObs.DDSatNum[1]; i++) // rows
		{
			for (int j = i % 4; j < 4 * Raw.DDObs.DDSatNum[1]; j += 4)  // columns
			{
				if (i == j)
				{
					if (i % 4 < 2) // Pseudorange observations
					{
						R_BDS(i, j) = R_BDS_ii_P;
					}
					else // Carrier phase observations
					{
						R_BDS(i, j) = R_BDS_ii_L;
					}
				}
				else
				{
					if (i % 4 < 2) // Pseudorange observations
					{
						R_BDS(i, j) = R_BDS_ij_P;
					}
					else // Carrier phase observations
					{
						R_BDS(i, j) = R_BDS_ij_L;
					}
				}
			}
		}

		// Add BDS part to R matrix
		R.bottomRightCorner(4 * Raw.DDObs.DDSatNum[1], 4 * Raw.DDObs.DDSatNum[1]) = R_BDS;
	}

	// Perform Kalman filter measurement update

	// Calculate innovation covariance
	//Eigen::MatrixXd R = R_inv.inverse();
	Eigen::MatrixXd S = H * P_k_k_1 * H.transpose() + R;

	// Calculate Kalman gain
	Eigen::MatrixXd K = P_k_k_1 * H.transpose() * S.inverse();

	// Calculate innovation/residual vector
	V = Z - Z_pred;

	// Update state estimate
	X_k_k = X_k_k_1 + K * V;

	// Update state covariance using Joseph form for numerical stability
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3 + 2 * Raw.DDObs.Sats, 3 + 2 * Raw.DDObs.Sats);
	P_k_k = (I - K * H) * P_k_k_1 * (I - K * H).transpose() + K * R * K.transpose();


	// Debugging: Print all matrices
	//cout << "R_inv: \n" << R_inv << endl;
	//cout << "R: \n" << R << endl;
	//cout << "V: \n" << V << endl;
	//cout << "H: \n" << H << endl;
	//cout << "X_k_k: \n" << X_k_k << endl;
	//cout << "P_k_k: \n" << P_k_k << endl;
	//cout << "Kalman Gain: \n" << K << endl;
	//cout << "Innovation: \n" << V << endl;
	//cout << "K*V: \n" << K * V << endl;



	return 1;   // Successful update
}




void RemoveUnusedAmbiguities(Amb_Map& amb_map)
{
	for (auto it = amb_map.begin(); it != amb_map.end(); ) 
	{
		if (!it->second.isUsed) {
			it = amb_map.erase(it);  // Remove unused ambiguity and update iterator  
		}
		else {
			++it;  // Move to the next element  
		}
	}


	for (auto it = amb_map.begin(); it != amb_map.end(); ++it)
	{
		it->second.isUsed = false;  // Reset isUsed flag for all remaining ambiguities
	}
	

}



void UpdateAmbiguityInfo(const RTK_RAW& Raw, RTK_EKF& EKF, const Eigen::MatrixXd& X_k_k) {
	int GPS_count = 0;
	int BDS_count = Raw.DDObs.DDSatNum[0];

	EKF.Last_DDObs.bFixed = Raw.DDObs.bFixed;

	for (int j = 0; j < Raw.SdObs.SatNum; j++) {
		// Skip reference satellites
		if (Raw.DDObs.ref_idx[0] == j || Raw.DDObs.ref_idx[1] == j) {
			continue;
		}

		// Skip invalid observations
		if (Raw.SdObs.SdSatObs[j].Valid_status != 2) {
			continue;
		}

		if (Raw.SdObs.SdSatObs[j].System == GPS) {
			int prn = Raw.SdObs.SdSatObs[j].Prn;

			if (Raw.DDObs.bFixed)
			{
				EKF.GPS_Amb[prn].value[0] = Raw.DDObs.FixedAmb[2 * GPS_count];       // Update ambiguity value -- Fixed
				EKF.GPS_Amb[prn].value[1] = Raw.DDObs.FixedAmb[2 * GPS_count + 1];   // Update ambiguity value -- Fixed
			}
			else
			{
				EKF.GPS_Amb[prn].value[0] = X_k_k(3 + 2 * GPS_count, 0);       // Update ambiguity value -- Fixed
				EKF.GPS_Amb[prn].value[1] = X_k_k(3 + 2 * GPS_count + 1, 0);   // Update ambiguity value -- Fixed
			}

			

			EKF.GPS_Amb[prn].isUsed = true;  // Mark ambiguity as used
			EKF.GPS_Amb[prn].index = GPS_count; // Update ambiguity index

			GPS_count++;
		}
		else {
			int prn = Raw.SdObs.SdSatObs[j].Prn;

			if (Raw.DDObs.bFixed)
			{
				EKF.BDS_Amb[prn].value[0] = Raw.DDObs.FixedAmb[2 * BDS_count];       // Update ambiguity value
				EKF.BDS_Amb[prn].value[1] = Raw.DDObs.FixedAmb[2 * BDS_count + 1];   // Update ambiguity value
			}
			else
			{
				EKF.BDS_Amb[prn].value[0] = X_k_k(3 + 2 * BDS_count, 0);       // Update ambiguity value
				EKF.BDS_Amb[prn].value[1] = X_k_k(3 + 2 * BDS_count + 1, 0);   // Update ambiguity value
			}
			

			EKF.BDS_Amb[prn].isUsed = true;  // Mark ambiguity as used
			EKF.BDS_Amb[prn].index = BDS_count; // Update ambiguity index

			BDS_count++;
		}
	}

	// Remove unused ambiguities
	RemoveUnusedAmbiguities(EKF.GPS_Amb);
	RemoveUnusedAmbiguities(EKF.BDS_Amb);
}



int RTK_KF()
{

	// Create Data Objects
	RTK_RAW Raw = RTK_RAW();
	RTK_EKF EKF = RTK_EKF();

	// Create Input Handle
	//InputHandle base_src(SOURCE_FILE_1);
	//InputHandle rover_src(SOURCE_FILE_2);

	// For Temporary Storage of Solution
	XYZ_Coord Bas_pos(0, 0, 0);
	XYZ_Coord Rov_pos(0, 0, 0);

	InputHandle base_src("47.114.134.129", 7190);
	InputHandle rover_src("8.148.22.229", 4002);


	int all_count = 0;
	int fixed_count = 0;


	// Loop to Fetch Data
	while (true)
	{
		// Synchronous Status
		int syn_status = GetSynObs(rover_src, base_src, Raw);


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


		/* -------- Syn_states == 1 ---> Start RTK Observation Process -------- */

		Check_Raw_Obs(Raw.BasEpk, Raw.RovEpk);

		if (!SPP(Raw.BasEpk, Raw.GpsEph, Raw.BdsEph, Bas_pos))
		{
			cout << "Not Enough Observation For BaseStation!" << endl;
			continue;
		}

		if (!SPP(Raw.RovEpk, Raw.GpsEph, Raw.BdsEph, Rov_pos))
		{
			cout << "Not Enough Observation For Rover!" << endl;
			continue;
		}

		Construct_SD_Obs(Raw.BasEpk, Raw.RovEpk, Raw.SdObs);
		Check_SD_Obs(Raw.SdObs);

		if (!Seletct_Ref_Sat(Raw.BasEpk, Raw.RovEpk, Raw.SdObs, Raw.DDObs))
		{
			cout << "No Reference Satellite Selected!" << endl;
			continue;
		}



		/* -------- RTK Raw Data Process Finish and Start EKF Update -------- */
		
		// Initialize EKF When Get the First Fixed Solution
		if (EKF.isInit == false)
		{
			// For Ambiguity Resolution
			Eigen::VectorXd FloatAmb = Eigen::VectorXd::Zero(2 * Raw.DDObs.Sats);
			Eigen::MatrixXd Qxx = Eigen::MatrixXd::Zero(3 + 2 * Raw.DDObs.Sats, 3 + 2 * Raw.DDObs.Sats);


			if (!RTK_Float(Raw, FloatAmb, Qxx))
			{
				cout << "No Enough Observation for RTK_Float Solution!" << endl;
				continue;
			}

			if (Calc_XYZ_dist(Raw.RovEpk.my_result.Pos, Rov_pos) > 1e3)
			{
				cout << "RTK_Float Failed!" << endl;
				continue;
			}

			cout << "Float Solution of BaseLine: " << Raw.SdObs.Time.SecOfWeek << "  " << Raw.DDObs.dPos.x << "  " << Raw.DDObs.dPos.y << "  " << Raw.DDObs.dPos.z << endl;

			if (!RTK_FIX(Raw, FloatAmb, Qxx))
			{
				cout << "Failed to Fix Ambiguity!" << endl;
				continue;
			}

			cout<<"The First Fixed Epoch: Initialize of the EKF"<<endl;
			cout << "Fixed Solution of BaseLine: " << Raw.SdObs.Time.SecOfWeek << "  " << Raw.DDObs.dPos.x << "  " << Raw.DDObs.dPos.y << "  " << Raw.DDObs.dPos.z << endl;
			
			// Initialize EKF, If Get the Fixed Solution
			RTK_EKF_INIT(Raw, EKF);
			continue;
		}


		/* -------------- Regular EKF Process -------------- */
		all_count++;
		
		
		/* ------------ Time Estimate ------------- */
		Eigen::MatrixXd X_k_k_1 = Eigen::MatrixXd::Zero(3 + 2 * Raw.DDObs.Sats, 1);
		Eigen::MatrixXd P_k_k_1 = Eigen::MatrixXd::Zero(3 + 2 * Raw.DDObs.Sats, 3 + 2 * Raw.DDObs.Sats);

		if (!TimeEstimate(Raw, EKF, X_k_k_1, P_k_k_1))
		{
			cout << "Failed to Update Time Estimate!" << endl;
			EKF.isInit = false;
			continue;
		}

		// cout << "X_k_k_1: \n" << X_k_k_1.transpose() << endl;
		// cout << "P_k_k_1: \n" << P_k_k_1 << endl;


		/* ------------ Measurement Update ------------- */

		// NOTICE Non-Liner EKF Process , especially the Z
		Eigen::MatrixXd X_k_k = Eigen::MatrixXd::Zero(3 + 2 * Raw.DDObs.Sats, 1);
		Eigen::MatrixXd P_k_k = Eigen::MatrixXd::Zero(3 + 2 * Raw.DDObs.Sats, 3 + 2 * Raw.DDObs.Sats);

		if (!MeasurementUpdate(Raw, X_k_k_1, P_k_k_1, X_k_k, P_k_k))
		{
			cout << "Failed to Update Measurement!" << endl;
			EKF.isInit = false;
			continue;
		}

		// cout << "X_k_k: \n" << X_k_k.transpose() << endl;
		// cout << "P_k_k: \n" << P_k_k << endl;
		

		// Update BaseLine Info
		Raw.RovEpk.my_result.Pos = XYZ_Coord(X_k_k(0, 0), X_k_k(1, 0), X_k_k(2, 0));

		Raw.DDObs.dPos.x = Raw.RovEpk.my_result.Pos.x - Raw.BasEpk.rcv_result.Pos.x;
		Raw.DDObs.dPos.y = Raw.RovEpk.my_result.Pos.y - Raw.BasEpk.rcv_result.Pos.y;
		Raw.DDObs.dPos.z = Raw.RovEpk.my_result.Pos.z - Raw.BasEpk.rcv_result.Pos.z;
		


		if (Calc_XYZ_dist(Raw.RovEpk.my_result.Pos, Rov_pos) > 1e3)
		{
			cout << "Measurement Update Failed!" << endl;
			Raw.RovEpk.my_result.Pos = Rov_pos;
			EKF.isInit = false;
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


	
		//cout << "X_k_k: \n" << X_k_k << endl;
		//cout << "FloatAmb: \n" << FloatAmb << endl;
		//cout << "P_k_k: \n" << P_k_k << endl;
		//
		//
		//cout << "Float Solution of BaseLine: " << Raw.SdObs.Time.SecOfWeek << "  " << Raw.DDObs.dPos.x << "  " << Raw.DDObs.dPos.y << "  " << Raw.DDObs.dPos.z << endl;


		if (!RTK_FIX(Raw, FloatAmb, P_k_k))
		{
			//cout << "Failed to Fix Ambiguity!" << endl;
			cout << "Float time: " << Raw.SdObs.Time.SecOfWeek << " " << Raw.DDObs.DDSatNum[0] << " " << Raw.DDObs.DDSatNum[1] << endl;
			UpdateAmbiguityInfo(Raw, EKF, X_k_k);

			//cout << "X_k_k: \n" << X_k_k << endl;
		    //cout << "FloatAmb: \n" << FloatAmb << endl;
		    //cout << "P_k_k: \n" << P_k_k << endl;

			continue;
		}

		fixed_count++;
		cout << "Ratio: " << Raw.DDObs.Ratio << endl;
		cout << "Fixed Solution of BaseLine: " << Raw.SdObs.Time.SecOfWeek << "  " << Raw.DDObs.dPos.x << "  " << Raw.DDObs.dPos.y << "  " << Raw.DDObs.dPos.z << "  " << Raw.DDObs.ref_prn[0] << "  " << Raw.DDObs.ref_prn[1] << endl;

		double fixed_rate = (double)fixed_count / all_count;
		cout << "EKF_Solution Count: " << all_count << "  " << "Fixed Rate: " << fixed_rate << endl;
		
		UpdateAmbiguityInfo(Raw, EKF, X_k_k);
		
	
	}


	return 1;

}





int main()
{
	//RTK_LS();
	RTK_KF();


	return 0;
}


int SPP_FROM_BIN_FILE()
{
	// creat objects
	size_t buff_len = 0;
	size_t bytesRead = 0;
	unsigned char msg_buff[MSG_BUFF_SIZE];
	unsigned char* buff_ptr = msg_buff;
	GNSS_EPHREC gps_eph;
	GNSS_EPHREC bds_eph;
	EPOCH_OBS	eph_obs;
	OEM7_MSG	message;

	// write result
    ofstream ofs(OUTPUT_FILE);
    ofstream rcv(RCVRES_FILE);
    ofstream mix(MIXOUT_FILE);

	// open a file
	FILE* fp = fopen(SOURCE_FILE_1, "rb");

	if (fp == NULL)
	{
		cerr << "Fails in Opening File" << endl;
		return -1;
	}
	cout << "open file successfully" << endl;

	XYZ_Coord pos(-2267335.3952, 5008648.4165, 3222375.1482);
	//XYZ_Coord pos(0, 0, 0);

	// read data
	while ((bytesRead = fread(buff_ptr + buff_len, 1, MSG_BUFF_SIZE - buff_len, fp)) > 0)
	{
		buff_len += bytesRead;
		size_t size = buff_len; // buffer valid size


		while (decode_NovOem7_Buff(buff_ptr, size, buff_len, message, eph_obs, gps_eph, bds_eph) == 1)
		{
            //Sleep(980);

			// Here goes to Position Solution Function
			if (SPP(eph_obs, gps_eph, bds_eph, pos) == 0)
			{
				cout << "Not Enough Observation!" << endl;
				mix << "Not Enough Observation!" << endl;
				continue;
			}

			// Single Point Velocity
			SPV(eph_obs, pos);



			// Epoch Result Output
			char my_solution[1024];
			char rcv_solution[1024];

			string my_str;
			string rcv_str;


			// Terminal Output
			if (fabs(GPST_DIFF(eph_obs.rcv_result.time, eph_obs.my_result.time)) <= 2)
			{
				sprintf(rcv_solution, "Reciver Best Pos:\t%6d   %10.3f %15.3f %15.3f %15.3f %10.3f %10.3f %10.3f %8d %8d\n",
					eph_obs.rcv_result.time.Week, eph_obs.rcv_result.time.SecOfWeek,
					eph_obs.rcv_result.Pos.x, eph_obs.rcv_result.Pos.y, eph_obs.rcv_result.Pos.z,
					0.0, 0.0, 0.0,
					eph_obs.rcv_result.track_satnum, eph_obs.rcv_result.used_satnum);


				sprintf(my_solution, "My Solution PVT:\t%6d   %10.3f %15.3f %15.3f %15.3f %10.3f %10.3f %10.3f %8d %8d %10.3f %10.3f %10.3f\n",
					eph_obs.Time.Week, eph_obs.Time.SecOfWeek,
					eph_obs.my_result.Pos.x, eph_obs.my_result.Pos.y, eph_obs.my_result.Pos.z,
					eph_obs.my_result.Vel.x, eph_obs.my_result.Vel.y, eph_obs.my_result.Vel.z,
					eph_obs.SatNum, eph_obs.my_result.used_satnum,
					eph_obs.my_result.PDOP, eph_obs.my_result.Sigma_Pos, eph_obs.my_result.Sigma_Vel);

				rcv_str = rcv_solution;
				my_str = my_solution;


				cout << rcv_str << endl << my_str;
			}
			else
			{
				sprintf(my_solution, "My Solution PVT:\t%6d   %10.3f %15.3f %15.3f %15.3f %10.3f %10.3f %10.3f %8d %8d %10.3f %10.3f %10.3f\n",
					eph_obs.Time.Week, eph_obs.Time.SecOfWeek,
					eph_obs.my_result.Pos.x, eph_obs.my_result.Pos.y, eph_obs.my_result.Pos.z,
					eph_obs.my_result.Vel.x, eph_obs.my_result.Vel.y, eph_obs.my_result.Vel.z,
					eph_obs.SatNum, eph_obs.my_result.used_satnum,
					eph_obs.my_result.PDOP, eph_obs.my_result.Sigma_Pos, eph_obs.my_result.Sigma_Vel);

				my_str = my_solution;
				cout << my_str;
			}


			// File Output
            ofs << my_str;
            rcv << rcv_str;
			mix << rcv_str << endl << my_str;

		}

	}

	fclose(fp);
	ofs.close();
	rcv.close();
	mix.close();

	return 1;
}


