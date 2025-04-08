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


const char* OUTPUT_FILE = "./solution/spp_result.txt";
const char* RCVRES_FILE = "./solution/rcv_result.txt";
const char* MIXOUT_FILE = "./solution/mix_result.txt";

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

				if (fabs(time_diff) <= 0.5)
					return 1;  // -----> Old Base_Obs Synchronous with Rover
				else if (time_diff < -0.5)
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

				if (fabs(time_diff) <= 0.5)
					return 1;  // ----->  New Base_Obs Synchronous with Rover

				if (time_diff < -0.5)
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

		// half_cycle haven added OR no dual-obs
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
			if (sd_obs.SdCombObs[j].Prn != sd_obs.SdCombObs[i].Prn || sd_obs.SdCombObs[j].Sys != sd_obs.SdCombObs[i].Sys)
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
	int RefIdx_Rov_GPS, RefIdx_Bas_GPS, RefIdx_Rov_BDS, RefIdx_Bas_BDS = 0;
	
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


	cout << "Fixed Solution of BaseLine: "<<Raw.SdObs.Time.SecOfWeek<<"  " << BL_fix(0, 0) << "  " << BL_fix(1, 0) << "  " << BL_fix(2, 0) << endl;
	//cout << "Fixed BaseLine Varicance  "<< Sigma_BL << endl;


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
	InputHandle base_src(SOURCE_FILE_1);
	InputHandle rover_src(SOURCE_FILE_2);


	XYZ_Coord Bas_pos(0, 0, 0);
	XYZ_Coord Rov_pos(0, 0, 0);

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


		if (!RTK_FIX(Raw, FloatAmb, Qxx))
		{
			cout << "Failed to Fix Ambiguity!" << endl;
			continue;
		}




	}
	
	return 1;
   

   
}



int main()
{
	RTK_LS();



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


