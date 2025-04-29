#define _CRT_SECURE_NO_WARNINGS
#include"my_rtk.hpp"
#include<iostream>

#include<Eigen/Core>
#include<Eigen/Dense>

using namespace std;





/* Initialize Extended Kalman Filter for RTK Positioning ---------------------
* args   :
*          RTK_RAW&  raw       I    Raw observation data with fixed solution
*          RTK_EKF&  kf        O    EKF state structure to be initialized
*
* return : void
*
* notes  : This function initializes the EKF state vector and covariance matrix
*          using the first fixed RTK solution. The initialization includes:
*          1. Setting initial rover position from RTK solution
*          2. Storing fixed integer ambiguities for GPS and BDS satellites
*          3. Initializing state covariance matrix with appropriate values
*          4. Saving current epoch's double-difference information
*-----------------------------------------------------------------------------*/
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




/* Check Reference Satellite Changes ------------------------------------
* args   :
*          RTK_RAW&  raw      I    Current raw observation data
*          RTK_EKF&  kf       I    Current EKF state
*          int*      d_ref    O    Reference satellite change flags
*
* return : void
*
* notes  : This function detects reference satellite changes between epochs
*          by comparing current reference PRNs with previous ones.
*
*          d_ref[0] = GPS reference satellite change status
*          d_ref[1] = BDS reference satellite change status
*
*          Each element can be:
*          -2: Reference satellite not found in previous ambiguity map
*          -1: Reference satellite unchanged
*          >=0: Index of reference satellite in previous epoch's ambiguity map
*-----------------------------------------------------------------------------*/
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
			//cout << "GPS Reference Satellite is not in the Map!" << endl;
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
			//cout << "BDS Reference Satellite is not in the Map!" << endl;
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
*          RTK_RAW&            Raw           I    Raw observation data
*          RTK_EKF&            EKF           IO   EKF state and parameters
*          int*                d_ref         I    Reference satellite change flags
*          Eigen::MatrixXd&    X             O    State vector
*          Eigen::MatrixXd&    Phi_Amb       O    State transition matrix for ambiguities
*          Eigen::MatrixXd&    Q             O    Process noise matrix
*
* return : void
*
* notes  : This function manages ambiguity states in the EKF by:
*          1. Updating the state transition matrix to handle reference satellite changes
*          2. Maintaining continuous tracking of ambiguities across epochs
*          3. Initializing ambiguities for newly observed satellites
*          4. Setting appropriate process noise for ambiguities based on fix status
*
*          d_ref[0/1]: -2 for Not Found, -1 for Not Changed, >=0 for the corresponding
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
*          RTK_RAW&         Raw        I    Raw observation data
*          RTK_EKF&         EKF        IO   Kalman filter state and covariance
*          Eigen::MatrixXd& X_k_k_1    O    Predicted state vector
*          Eigen::MatrixXd& P_k_k_1    O    Predicted state covariance matrix
*
* return : int  1 ---> Time Update Successful
*               0 ---> Failed to Update (reference satellite problem)
*
* notes  : Performs the time update step of the Extended Kalman Filter by:
*          1. Detecting reference satellite changes between epochs
*          2. Selecting appropriate motion model (kinematic or static)
*          3. Building the state transition matrix considering ambiguity changes
*          4. Computing the process noise matrix with appropriate weights
*          5. Predicting the state vector and covariance matrix forward in time
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
		// std::cout << "EKF State is not Correct!" << std::endl;
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
*          RTK_RAW&         Raw        I    Raw observation data
*          Eigen::MatrixXd& X_k_k_1    I    Predicted state vector
*          Eigen::MatrixXd& P_k_k_1    I    Predicted state covariance matrix
*          Eigen::MatrixXd& X_k_k      O    Updated state vector
*          Eigen::MatrixXd& P_k_k      O    Updated state covariance matrix
*
* return : int  1 ---> Measurement Update Successful
*               0 ---> Failed to Update (not enough observations)
*
* notes  : Performs the measurement update step of the Extended Kalman Filter by:
*          1. Computing geometric distances between stations and satellites
*          2. Constructing double-difference observations (code and phase)
*          3. Building observation matrix H relating states to measurements
*          4. Forming measurement noise covariance matrix R with appropriate weights
*          5. Computing innovation vector, Kalman gain and state updates
*          6. Updating state covariance using numerically stable Joseph form
*-----------------------------------------------------------------------------*/
int MeasurementUpdate(RTK_RAW& Raw, const Eigen::MatrixXd& X_k_k_1,
	const Eigen::MatrixXd& P_k_k_1, Eigen::MatrixXd& X_k_k, Eigen::MatrixXd& P_k_k)
{
	// Get rover position from state vector
	XYZ_Coord rover_pos(X_k_k_1(0, 0), X_k_k_1(1, 0), X_k_k_1(2, 0));
	XYZ_Coord base_pos(Raw.BasEpk.rcv_result.Pos);

	// Get Initial Position of BASE from Reference Position
	if (Raw.BasEpk.rcv_result.Pos.x == 0.0)
	{
		// Get Initial Position of BASE from SPP
		// std::cout << "No Base Position Reference Solution!" << std::endl;
		// base_pos = Raw.BasEpk.my_result.Pos;
		return 0;
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




/* Remove Unused Ambiguities from Map --------------------------------------
* args   :
*          Amb_Map&  amb_map    IO   Ambiguity map to be cleaned
*
* return : void
*
* notes  : This function manages satellite ambiguity maps by:
*          1. Removing ambiguities for satellites no longer in view
*          2. Resetting the "isUsed" flag for remaining ambiguities
*
*          This ensures the ambiguity map only contains current satellites
*          and prevents memory growth or misuse over time 
*-----------------------------------------------------------------------------*/
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



/* Update Ambiguity Information in EKF -------------------------------------
* args   :
*          const RTK_RAW&       Raw     I    Raw observation data
*          RTK_EKF&             EKF     IO   EKF state and parameters
*          const Eigen::MatrixXd& X_k_k I    Updated state vector
*
* return : void
*
* notes  : This function updates the ambiguity maps in the EKF state by:
*          1. Tracking which satellites are observed in the current epoch
*          2. Extracting ambiguity values from either fixed solution or filter state
*          3. Updating ambiguity indices to match their position in the state vector
*          4. Removing ambiguities for satellites no longer in view
*
*          The function handles both GPS and BDS satellites separately
*-----------------------------------------------------------------------------*/
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
