#define _CRT_SECURE_NO_WARNINGS
#include"my_rtk.hpp"

#include<Eigen/Core>
#include<Eigen/Dense>

using namespace std;



/* RTK Float Solution Calculation -------------------------------------
* Calculate float ambiguity solution using least squares method
* args   :
*          RTK_RAW&         Raw       IO   Raw observation data
*          Eigen::VectorXd& FloatAmb  O    Float ambiguity vector
*          Eigen::MatrixXd& Qxx       O    Covariance matrix of parameters
*
* return : bool  true  ---> Float solution successfully calculated
*                false ---> Not enough observations or solution failed
*
* notes  : This function performs RTK float solution calculation by:
*          1. Using rover and base station positions
*          2. Computing geometric distances between stations and satellites
*          3. Forming double difference observations for GPS and BDS
*          4. Setting up and solving least squares equations iteratively
*          5. Detecting and removing outliers using standardized residuals
*          6. Estimating rover position and carrier phase ambiguities
*
*          The state（Parameter） vector includes:
*          - XYZ coordinates of rover position (3 parameters)
*          - Float ambiguities for each satellite pair and frequency (2×n)
*-----------------------------------------------------------------------------*/
bool RTK_Float(RTK_RAW& Raw, Eigen::VectorXd& FloatAmb, Eigen::MatrixXd& Qxx)
{

	// Get Initial Position of ROVER from SPP
	XYZ_Coord rover_pos(Raw.RovEpk.my_result.Pos);
	XYZ_Coord base_pos(Raw.BasEpk.rcv_result.Pos);

	// Get Initial Position of BASE from Reference Position
	if (Raw.BasEpk.rcv_result.Pos.x == 0.0)
	{
		// Get Initial Position of BASE from SPP
		// std::cout << "No Base Position Reference Solution!" << std::endl;
		// base_pos = Raw.BasEpk.my_result.Pos;
		return 0;
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

		
		rover_pos.x = X(0, 0);
		rover_pos.y = X(1, 0);
		rover_pos.z = X(2, 0);


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
				//cout << Raw.SdObs.SdSatObs[SD_idx].System <<  " " << Raw.SdObs.SdSatObs[SD_idx].Prn << " is Outlier!" << endl;

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




/* RTK Fixed Solution Calculation -------------------------------------
* Resolve integer ambiguities using LAMBDA method and compute fixed solution
* args   :
*          RTK_RAW&          Raw          IO   Raw observation data
*          Eigen::VectorXd&  FloatAmb     I    Float ambiguity vector
*          Eigen::MatrixXd&  Qxx          I    Covariance matrix of parameters
*          double            ratio_thres  I    Ratio test threshold
*
* return : int   1 ---> Integer ambiguities successfully fixed
*                0 ---> Failed to fix ambiguities or ratio test failed
*
* notes  : This function performs the following operations:
*          1. Applies LAMBDA method to fix float ambiguities to integers
*          2. Performs ratio test to validate the fixed solution
*          3. Updates the baseline vector based on fixed ambiguities
*          4. Computes rover position coordinates from fixed baseline
*
*          The fixed solution is validated using the ratio test:
*          ratio = squared norm of second-best / squared norm of best solution
*          If ratio > threshold, the fixed solution is accepted
*-----------------------------------------------------------------------------*/
int RTK_FIX(RTK_RAW& Raw, Eigen::VectorXd& FloatAmb, Eigen::MatrixXd& Qxx, double ratio_thres)
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
		return 0;
	}
	

	// Calculate and Check Ratio
	Raw.DDObs.ResAmb[0] = sq_nroms(0);
	Raw.DDObs.ResAmb[1] = sq_nroms(1);
	Raw.DDObs.Ratio = sq_nroms(1) / sq_nroms(0);

	if (Raw.DDObs.Ratio < ratio_thres)
	{
		return 0;
	}


	// Calculate Fix Solution for Baseline
	Eigen::MatrixXd BL_flt = Eigen::MatrixXd::Zero(3, 1);
	BL_flt << Raw.DDObs.dPos.x, Raw.DDObs.dPos.y, Raw.DDObs.dPos.z;


	// X_BL = X_BL - Q_BL_Amb * Q_Amb \ (Amb_Flt - Amb_Fix) ---> (3, 2) - (3, 3) * (N, N) * (N, 2)
	Eigen::MatrixXd BL_fix = BL_flt.replicate(1,2) - Q_BL_Amb * Q_Amb.inverse() * (FloatAmb.replicate(1, 2) - FixAmb);


	// Calculate Fix Solution for Baseline's RMS (Cause Ambuiguity is Fixed and seem as no Varience )
	// Q_BL = Q_BL - Q_BL_Amb * Q_Amb \ Q_BL_Amb'   ---> (3, 3) - (3, N) * (N, N) * (N, 3)
	Eigen::MatrixXd  Q_BL_fix = Q_BL - Q_BL_Amb * Q_Amb.inverse() * Q_BL_Amb.transpose();
	double Sigma_BL = sqrt(Q_BL_fix(0, 0) + Q_BL_fix(1, 1) + Q_BL_fix(2, 2));


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

	// Update RTK_FIX Point Position
	Raw.RovEpk.my_result.Pos.x = Raw.DDObs.dPos.x + Raw.BasEpk.rcv_result.Pos.x;
	Raw.RovEpk.my_result.Pos.y = Raw.DDObs.dPos.y + Raw.BasEpk.rcv_result.Pos.y;
	Raw.RovEpk.my_result.Pos.z = Raw.DDObs.dPos.z + Raw.BasEpk.rcv_result.Pos.z;

	return 1;

}



/* Generate Position Output String --------------------------------------
* Format rover position solution into standardized string format
* args   :
*          RTK_RAW&   raw        I    Raw observation data with solution
*          int        type       I    Position solution type:
*                                      1: RTK fixed solution
*                                      2: RTK float solution
*                                      5: Single point positioning
*
* return : string  Formatted position string in RTKLib POS format
*
* notes  : Creates a formatted string containing position information in the form:
*          "Week SOW Latitude Longitude Height Quality"
*          Where:
*          - Week is GPS week number
*          - SOW is seconds of week
*          - Latitude and longitude are in degrees
*          - Height is in meters
*          - Quality is the solution type (1=fixed, 2=float, 5=single)
*-----------------------------------------------------------------------------*/
string Form_Pos_String(RTK_RAW& raw, int type)
{
	string result;

	char buffer[256];

	BLH_Coord Rover_blh;
	XYZ2BLH(raw.RovEpk.my_result.Pos, Rover_blh, WGS84);

	// RTKLiB POS Format --- Week SOW B L H Qualitry
	sprintf(buffer, "%6d %13.3f %30.25f %30.25f %15.7f %2d\n",
		raw.RovEpk.Time.Week, raw.RovEpk.Time.SecOfWeek,
		Rover_blh.B, Rover_blh.L, Rover_blh.H, type);

	result = buffer;

	return result;
}