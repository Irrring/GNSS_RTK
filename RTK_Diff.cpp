#include"my_rtk.hpp"
#include"json.hpp"
#include<iostream>
#include <sstream>
#include<fstream>
#include<iomanip>

#include<Eigen/Core>
#include<Eigen/Dense>

using namespace std;







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
*----------------------------------------------------------------*/
int GetSynObs(InputHandle& rover_src, InputHandle& base_src, RTK_RAW& raw)
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
		threshold = 3;
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



/* Check Raw Observation for Cycle Slips --------------------------------------
* Detect cycle slips by comparing current LOCKTIME with previous values
* args   :
*          EPOCH_OBS& base_obs     IO   Base station observation data
*          EPOCH_OBS& rover_obs    IO   Rover station observation data
*
* return : void
*
* notes  : This function checks if cycle slips occurred on L1 frequency by
*          comparing current lock time with the previous value stored in
*          ComObs.LockTime_Last. 
*
*		   If current lock time is less than previous,it indicates a cycle 
*          slip has occurred and sets the no_cycle_slipflag to false.
*
*          If no previous lock time exists for a satellite, the default
*          assumption is that no cycle slip has occurred (no_cycle_slip = true).
*-----------------------------------------------------------------------------*/
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
* Form single difference observations between base and rover stations
* args   :
*          const EPOCH_OBS& base_obs      I   Base station observation data
*          const EPOCH_OBS& rover_obs     I   Rover station observation data
*          SD_EPOCH_OBS& SD_obs           O   Single difference observation data
*
* return : void
*
* Valid_status values:
*          0: Invalid (has cycle slip or other error)
*          1: Single-frequency or half-cycle ambiguities not resolved
*          2: Dual-frequency with half-cycle ambiguities resolved
*-----------------------------------------------------------------------------*/
void Construct_SD_Obs(const EPOCH_OBS& base_obs, const EPOCH_OBS& rover_obs, SD_EPOCH_OBS& SD_obs)
{
	// Reset memory ---> except COMB_OBS 
	SD_obs.reset();

	// Epoch time
	SD_obs.Time = rover_obs.Time;

	// Usable SD_obs Counter
	int sd_num = 0;


	/* Perform the Inter-Station differencing for the same satellite, frequency, and observation type */
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



/* Validate Single Difference Observations and Compute Combined Values ---------
* Detect cycle slips and calculate combined observation values for SD measurements
* args   :
*          SD_EPOCH_OBS& sd_obs     IO   Single difference epoch observation data
*
* return : void
*
* notes  : This function performs the following tasks:
*          1. Checks validity of single difference observations
*          2. Calculates geometry-free (GF) and Melbourne-W¨¹bbena (MW) combinations
*          3. Compares current GF and MW values with previous epoch's values
*		   4. Thresholds: 0.05m for GF, 3 cycles for MW
*          5. Updates observation validity status:
*               -->  status=0: cycle slip detected
*               -->  status=2: valid dual-frequency observation
*          6. Maintains running average of MW values for continuous tracking
*
* references : Melbourne, W.G. (1985) and W¨¹bbena, G. (1985) for MW combination
*-----------------------------------------------------------------------------*/
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

			//cout << "Cycle Slip Detected:  " << sd_obs.SdSatObs[i].System << " " << sd_obs.SdSatObs[i].Prn << endl;
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



/* Select Reference Satellite for Double Difference Observation ---------------
* Select optimal reference satellites for GPS and BDS double differencing
* args   :
*          const EPOCH_OBS& base_obs      I   Base station observation data
*          const EPOCH_OBS& rover_obs     I   Rover station observation data
*          SD_EPOCH_OBS& SD_obs           IO  Single difference observation data
*          DD_OBS& DD_obs                 O   Double difference observation data
*
* return : bool  true  ---> Reference satellites successfully selected
*                false ---> No suitable reference satellites found
*
* notes  : This function selects reference satellites for GPS and BDS separately:
*          1. Only dual-frequency observations without cycle slips are considered
*          2. Satellites must have valid ephemeris and position data
*          3. Selection criteria include:
*             - Signal-to-noise ratio (CN0) > 45 dB-Hz
*             - Lock time > 10 seconds
*             - Highest elevation angle among valid satellites
*
* GPS/BDS systems are handled in array indices: 0->GPS, 1->BDS
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
			case GPS: sat_sys = 0; break;
			case BDS: sat_sys = 1; break;
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
			DD_obs.ref_idx[i] = maxE_idx[i];
			DD_obs.ref_prn[i] = SD_obs.SdSatObs[maxE_idx[i]].Prn;
			DD_obs.DDSatNum[i] = good_sat_num[i] - 1;
		}
		else
		{
			DD_obs.ref_idx[i] = -1;
			DD_obs.ref_prn[i] =  0;
			DD_obs.DDSatNum[i] = 0;
		}
	}


	// Total Numbers of DD Obs
	DD_obs.Sats = DD_obs.DDSatNum[0] + DD_obs.DDSatNum[1];

	// If no or only one reference satellite is found, return 0
	if (DD_obs.Sats == 0)
	{
		return 0;
	}

	return 1;
}
