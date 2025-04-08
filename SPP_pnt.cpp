#include"my_rtk.hpp"
#include<iostream>

#include<Eigen/Core>
#include<Eigen/Dense>





/* Check Ephemeris Exist and In Date and Get eph_data */
int get_eph_data(const int prn, const GNSS_Sys sys, const GPS_TIME& t, const GNSS_EPHREC& gps_eph, const GNSS_EPHREC& bds_eph, SAT_MIDRES& Mid, GNSS_EPHREC_DATA& eph)
{
	// Find coorisponding ephemeris
	GNSS_EPHREC::const_iterator eph_it;

	if (sys == GPS)
	{
		// check if eph exist and get eph_data
		eph_it = gps_eph.find(prn);

		if (eph_it == gps_eph.end())
		{
			/*std::cerr << eph.PRN << std::endl;
			std::cerr << "No coorisponding ephemeris" << std::endl;*/
			Mid.Valid = false;
			return -1;
		}

		// check if eph out of date
		eph = eph_it->second;
		if ((fabs(GPST_DIFF(t, eph.TOE)) > (GPS_EPH_MAX + 100))|| eph.SVHealth != 0)
		{
			/*std::cerr << eph.PRN << std::endl;
			std::cerr << "ephemeris out of date or Satellite is UnHealthy" << std::endl;*/
			Mid.Valid = false;
			return -1;
		}
	}
	else if (sys == BDS)
	{
		// check if eph exist and get eph_data
		eph_it = bds_eph.find(prn);

		if (eph_it == bds_eph.end())
		{
			/*std::cerr << eph.PRN << std::endl;
			std::cerr << "No coorisponding ephemeris" << std::endl;*/
			Mid.Valid = false;
			return -1;
		}

		// check if eph out of date
		eph = eph_it->second;
		if ((fabs(GPST_DIFF(t, eph.TOE)) > (BDS_EPH_MAX + 100))|| eph.SVHealth != 0)
		{
			/*std::cerr << eph.PRN << "  " << fabs(GPS_DIFF(t, eph.TOE)) << std::endl;
			std::cerr << "ephemeris out of date or Satellite is UnHealthy" << std::endl;*/
			Mid.Valid = false;
			return -1;
		}
	}

	return 1;
}



/* broadcast ephemeris to satellite clock bias ---------------------------------
* compute satellite clock bias with broadcast ephemeris
* notes  : 
*		 gtime_t t :   Is  the "Transimittion Time Calculate by Tr and Percedual Code" time by satellite clock (gpst)
*        satellite clock does not include relativity correction and tdg
*-----------------------------------------------------------------------------*/
void Calc_ClkOft(const GPS_TIME& t, GNSS_EPHREC_DATA& eph, SAT_MIDRES& Mid)
{
	double dt = GPST_DIFF(t, eph.TOC);
	Mid.SatClkOft = eph.ClkBias + eph.ClkDrift * dt + eph.ClkDriftRate * dt * dt;

}



/* calculate single satellite pvt and then save in SAT_MIDRES */
int Sat_PVT(const int prn, const GNSS_Sys sys, const GPS_TIME& t, GNSS_EPHREC_DATA& eph, SAT_MIDRES& Mid)
{
	// Set some Constants
	double mu=0.0;			// earth's gravitational constant
	double omega_e=0.0;	    // value of earth's rotation rate ( AKA OMEGA_E^Dot )
	
	// set ellipsoid para
	if (sys == GPS)
	{
		mu = WGS84.GM_Earth;
		omega_e = WGS84.Omega;
	}
	else if(sys == BDS)
	{
		mu = CGCS2000.GM_Earth;
		omega_e = CGCS2000.Omega;
	}


	// Use ephemeris to calculate satellite possition and velocity

	double n0 = sqrt(mu) / pow(eph.SqrtA, 3);	// mean motion
	double n = n0 + eph.DeltaN;					// Corrected mean motion
												
	double tk = GPST_DIFF(t,eph.TOE);			// Time from ephemeris refernce epoch
	double M = eph.M0 + n * tk;					// Mean anomaly

	double Ek = 0;								// Eccentric anomaly
	double E0 = M;								// Iteration initial value

	// Solve eccentric anomaly by iteration
	while (abs(Ek - E0) > 1e-15)					
	{							
		double temp = Ek;		
		Ek = M + eph.e * sin(E0);
		E0 = temp;				
	}							

	// Get True Anomaly
	double cosv = (cos(Ek) - eph.e) / (1 - eph.e * cos(Ek));					  // cosV
	double sinv = sqrt(1.0 - pow(eph.e, 2)) * sin(Ek) / (1.0 - eph.e * cos(Ek));  // sinV
	double Vk = atan2(sinv, cosv);												  // True Anomaly
																				  
	double u_0 = eph.omega + Vk;												  // Argument of Latitule ( AKA PHI_k )
	double r_0 = eph.SqrtA * eph.SqrtA * (1.0 - eph.e * cos(Ek));				  // Radius(UnCorrected)
																				  
	// Second Harmonic Pertubations												  
	double delta_u = eph.Cuc * cos(2 * u_0) + eph.Cus * sin(2 * u_0);			  // Argument of Latitude Correction
	double delta_r = eph.Crc * cos(2 * u_0) + eph.Crs * sin(2 * u_0);			  // Radius Correction
	double delta_i = eph.Cic * cos(2 * u_0) + eph.Cis * sin(2 * u_0);			  // Inclination Correction
																				  
	// Corrections																  
	double uk = u_0 + delta_u;													  // Corrected Argument of Latitude
	double rk = r_0 + delta_r;													  // Corrected  Radius
	double ik = eph.i0 + eph.iDot * tk + delta_i;								  // Correct Inclination
																			
	// Position in orbital plane
	double xk = rk * cos(uk);
	double yk = rk * sin(uk);

	

	double OMEGAk = 0;							// Corrected longitude of ascending node
	double sinO = 0, cosO = 0;					// Auxiliary for sin(OMEGAk) cos(OMEGAk)
	double Xgk = 0, Ygk = 0, Zgk = 0;			// Intertial Coordinates for GEO
	double X = 0, Y = 0, Z = 0;					// Earth-fixed Coordinates
	double sin_ik = sin(ik);					// Auxiliary for sin(ik)
	double cos_ik = cos(ik);					// Auxiliary for cos(ik)

	double COS_5 = cos(5.0 * PI / 180.0);		// COS(5 degrees)
	double SIN_5 = sin(5.0 * PI / 180.0);		// SIN(5 degrees)

	
	double sino = sin(omega_e * tk);			// Auxiliary for sin(omega_e * tk)
	double coso = cos(omega_e * tk);			// Auxiliary for cos(omega_e * tk)

	/* -------------Mind the difference of BDS GEO satellate!----------- */
	if (sys == BDS && eph.i0<30.0*PI/180.0) 
	{
		OMEGAk = eph.OMEGA0 + eph.OMEGADot * tk - omega_e * eph.TOE.SecOfWeek;
		sinO = sin(OMEGAk);
		cosO = cos(OMEGAk);
		// Intertial Coordinates for GEO
		Xgk = xk * cosO - yk * cos_ik * sinO;
		Ygk = xk * sinO + yk * cos_ik * cosO;
		Zgk = yk * sin_ik;

	
		

		// Earth-Fix Corrdinate for GEO in CGCS2000
		X =  Xgk * coso  + Ygk * sino * COS_5 - Zgk * sino * SIN_5;
		Y = -Xgk * sino  + Ygk * coso * COS_5 - Zgk * coso * SIN_5;
		Z = Ygk * SIN_5 + Zgk * COS_5;
	}
	else 
	{
		// Corrected longitude of ascending node
		OMEGAk = eph.OMEGA0 + (eph.OMEGADot - omega_e) * tk - omega_e * eph.TOE.SecOfWeek;
		sinO = sin(OMEGAk);
		cosO = cos(OMEGAk);

		// Earth-fixed Coordinates by Rotaion
		X = xk * cosO - yk * cos(ik) * sinO;
		Y = xk * sinO + yk * cos(ik) * cosO;
		Z = yk * sin_ik;
	}


	// Use Eccentric anomaly to Calculate Rlativistic correction term (sec)
	double F = -2.0 * sqrt(mu) / (CLIGHT * CLIGHT);
	double delta_tr = F * eph.e * eph.SqrtA * sin(Ek);
	
	// Calculate delta_ts Again
	double delta_ts = eph.ClkBias + eph.ClkDrift * tk + eph.ClkDriftRate * (tk * tk) + delta_tr;

	// Calculate Deviraitve
	double Ek_dot = n / (1.0 - eph.e * cos(Ek));															// Eccentric Anomaly Rate
	double Vk_dot = Ek_dot * sqrt(1.0 - pow(eph.e, 2)) / (1.0 - eph.e * cos(Ek));							// True Anomaly Rate
	double ik_dot = eph.iDot + 2.0 * Vk_dot * (eph.Cis * cos(2.0 * u_0) - eph.Cic * sin(2.0 * u_0));		// Corrected Inclination Rate
	double uk_dot = Vk_dot + 2.0 * Vk_dot * (eph.Cus * cos(2.0 * u_0) - eph.Cuc * sin(2.0 * u_0));				// Corrected Argument of Latitude
	double rk_dot = eph.SqrtA * eph.SqrtA * eph.e * sin(Ek) * Ek_dot + 2.0 * (eph.Crs * cos(2.0 * u_0) - eph.Crc * sin(2.0 * u_0)) * Vk_dot; 	// Corrected Radius Rate 
	
	// Earth-fixed Velocity
	double X_dot = 0, Y_dot = 0, Z_dot = 0;


	/* -------------Mind the difference of BDS GEO satellate!----------- */
	if (sys == BDS && eph.i0 < 30.0 * PI / 180.0)
	{
		double OMEGAk_dot = eph.OMEGADot;

		// Calculate In-plane Velocity ----- Inertial Coordinate
		double xk_dot = rk_dot * cos(uk) - rk * uk_dot * sin(uk);
		double yk_dot = rk_dot * sin(uk) + rk * uk_dot * cos(uk);


		// Calculate Inertial Velocity (m/s)
		double Xgk_dot = -xk * OMEGAk_dot * sinO + xk_dot * cosO - yk_dot * sinO * cos_ik
							- yk * (OMEGAk_dot * cosO * cos_ik - ik_dot * sinO * sin_ik);
		double Ygk_dot = xk * OMEGAk_dot * cosO + xk_dot * sinO + yk_dot * cosO * cos_ik
							- yk * (OMEGAk_dot * sinO * cos_ik + ik_dot * cosO * sin_ik);
		double Zgk_dot = yk_dot * sin_ik + yk * ik_dot * cos_ik;

		
		// Earth-Fix Velocity for GEO in CGCS2000
		X_dot = Xgk_dot * coso + Ygk_dot * sino * COS_5 - Zgk_dot * sino * SIN_5;
		Y_dot = -Xgk_dot * sino +  Ygk_dot * coso * COS_5 - Zgk_dot * coso * SIN_5;
		Z_dot = Ygk_dot * SIN_5 + Zgk_dot * COS_5;

		X_dot += -omega_e * sino * Xgk + omega_e * coso * COS_5 * Ygk - omega_e * coso * SIN_5 * Zgk;
		Y_dot += -omega_e * coso * Xgk - omega_e * sino * COS_5 * Ygk + omega_e * sino * SIN_5 * Zgk;
	}
	else
	{
		// Longitude of Ascending Node Rate
		double OMEGAk_dot = eph.OMEGADot - omega_e;

		// Calculate In-plane Velocity
		double xk_dot = rk_dot * cos(uk) - rk * uk_dot * sin(uk);
		double yk_dot = rk_dot * sin(uk) + rk * uk_dot * cos(uk);

		// Calculate Earth-fixed Velocity (m/s)
		X_dot = -xk * OMEGAk_dot * sinO + xk_dot * cosO - yk_dot * sinO * cos_ik
			- yk * (OMEGAk_dot * cosO * cos_ik - ik_dot * sinO * sin_ik);
		Y_dot = xk * OMEGAk_dot * cosO + xk_dot * sinO + yk_dot * cosO * cos_ik
			- yk * (OMEGAk_dot * sinO * cos_ik + ik_dot * cosO * sin_ik);
		Z_dot = yk_dot * sin_ik + yk * ik_dot * cos_ik;
	}



	// Use Ek_dot to Calculate delta_ts_dot
	double delta_tr_dot = F * eph.e * eph.SqrtA * cos(Ek) * Ek_dot;
	double delta_ts_dot = eph.ClkDrift + 2 * eph.ClkDriftRate * tk + delta_tr_dot;


	// Put the Result Into SAT_MIDRES
	Mid.SatPos.x = X;
	Mid.SatPos.y = Y;
	Mid.SatPos.z = Z;

	Mid.SatVel.x = X_dot;
	Mid.SatVel.y = Y_dot;
	Mid.SatVel.z = Z_dot;

	Mid.SatClkOft = delta_ts;
	Mid.SatClkSft = delta_ts_dot;

	Mid.Tgd1 = eph.TGD1;
	Mid.Tgd2 = eph.TGD2;

	return 1;
}


/* saastamoinen model -----------------------------------------------------------
* compute tropospheric delay by standard atmosphere and saastamoinen model
* args   :
*          BLH    pos       I   receiver position {lat,lon,h} (rad,m)
*          double az el     I   azimuth/elevation angle {az,el} (rad)
*          double humi      I   relative humidity
* return : tropospheric delay (m)
*-----------------------------------------------------------------------------*/
double Saastamoinen(BLH_Coord pos, const double Az,const double El,double humi)
{
	const double temp0 = 15.0; /* temparature at sea level */
	double hgt, pres, temp, e, z, trph, trpw;


	// Check if is out of the Modle Range
	if (pos.H < -1000.0 || 1E4 < pos.H || El <= 0) return 0.0;

	/* standard atmosphere */
	hgt = pos.H < 0.0 ? 0.0 : pos.H;

	// MY ADDITION
	if (fabs(humi) < 1e-5)
		humi = 0.5 * exp(-6.396E-4 * hgt);

	pres = 1013.25 * pow(1.0 - 2.2557E-5 * hgt, 5.2568);
	temp = temp0 - 6.5E-3 * hgt + 273.16;
	e = 6.108 * humi * exp((17.15 * temp - 4684.0) / (temp - 38.45));

	/* saastamoninen model */
	z = PI / 2.0 - El;
	trph = 0.0022768 * pres / (1.0 - 0.00266 * cos(2.0 * pos.B) - 0.00028 * hgt / 1E3) / cos(z);
	trpw = 0.002277 * (1255.0 / temp + 0.05) * e / cos(z);
	return trph + trpw;
}


/* Hopfield model -----------------------------------------------------------
* compute tropospheric delay by standard atmosphere and Hopfield model
* args   :
*          double H         I   receiver Hight (m)
*          double az el     I   azimuth/elevation angle {az,el} (rad)
* return : tropospheric delay (m)
*-----------------------------------------------------------------------------*/
double Hopfield(const double H, const double El)
{
	// Check if is out of the Modle Range
	if (H < -1000.0 || 1E4 < H || El <= 0) return 0.0;

	double RH = 0.5 * exp(-6.396E-4 * H);
	double p = 1013.25 * pow(1.0 - 2.2557E-5 * H, 5.2568);
	double T0 = 15.0 + 273.16;
	double T = T0 - 6.5E-3 * H;

	double e = RH * exp(-37.2465 + 0.213166 * T - 0.00256908 * T * T);
	double h_w = 11000.0;
	double h_d = 40136.0 + 148.72 * (T0 - 273.16);

	double K_w = 155.2E-7 * 4810.0 * e * (h_w - H) / (T * T);
	double K_d = 155.2E-7 * p * (h_d - H) / T;

	double El_deg = El * 180.0 / PI;
	double d_trop = K_d / sin(sqrt(El_deg * El_deg + 6.25) * PI / 180.0) + K_w / sin(sqrt(El_deg * El_deg + 2.25) * PI / 180.0);

	return d_trop;
}


/* Earth Rotation Correction */
void Rotation_Correction(GNSS_Sys sys, const XYZ_Coord& X_rev, SAT_MIDRES& Mid)
{
	double omega_e = 0;
	if (sys == GPS) omega_e = WGS84.Omega;
	if (sys == BDS)	omega_e = CGCS2000.Omega;

	double rou = Calc_XYZ_dist(X_rev, Mid.SatPos);
	double alpha = omega_e * rou / CLIGHT;

	double sinA = sin(alpha);
	double cosA = cos(alpha);

	// mind the change of median value
	double x  = Mid.SatPos.x;
	double y  = Mid.SatPos.y;
	double vx = Mid.SatVel.x;
	double vy = Mid.SatVel.y;

	Mid.SatPos.x = cosA  * x + sinA * y;
	Mid.SatPos.y = -sinA * x + cosA * y;

	Mid.SatVel.x =  cosA * vx + sinA * vy;
	Mid.SatVel.y = -sinA * vx + cosA * vy;
}


/* Validate Observation Data and Compute Combine Obs */
void Detect_Outlier(EPOCH_OBS& obs)
{
	// Prepare a new combine obs array
	COMB_OBS new_combine[MAX_CHANNEL_NUM];
	double lambda1 = 0.0, lambda2 = 0.0, freq1 = 0.0, freq2 = 0.0;

	for (int i = 0; i < obs.SatNum; i++)
	{
		/* -------------------check obs data is valid--------------------- */
		if (fabs(obs.SatObs[i].p[0]) <= 1E-9 ||
			fabs(obs.SatObs[i].p[1]) <= 1E-9 ||
			fabs(obs.SatObs[i].l[0]) <= 1E-9 ||
			fabs(obs.SatObs[i].l[1]) <= 1E-9)
		{
			// add necceray info 
			new_combine[i].Prn = obs.SatObs[i].Prn;
			new_combine[i].Sys = obs.SatObs[i].System;
			new_combine[i].valid_epo_num = 0;
			obs.SatObs[i].Valid = false;
			continue;
		}

		/* ------------calculate epoch combine observation------------ */
		// get frequence and wave-length
		if (obs.SatObs[i].System == GPS)
		{
			freq1 = FREQ_GPS_L1;
			freq2 = FREQ_GPS_L2;
			lambda1 = CLIGHT / FREQ_GPS_L1;
			lambda2 = CLIGHT / FREQ_GPS_L2;
		}
		else if (obs.SatObs[i].System == BDS)
		{
			freq1 = FREQ_BDS_B1;
			freq2 = FREQ_BDS_B3;
			lambda1 = CLIGHT / FREQ_BDS_B1;
			lambda2 = CLIGHT / FREQ_BDS_B3;
		}
		
		// MW and GF in this Epoch
		double L1 = lambda1 * obs.SatObs[i].l[0];   // m
		double L2 = lambda2 * obs.SatObs[i].l[1];	// m
		double GF = L1 - L2;						// m

		double MW = (freq1 * L1 - freq2 * L2) / (freq1 - freq2) - (freq1 * obs.SatObs[i].p[0] + freq2 * obs.SatObs[i].p[1]) / (freq1 + freq2);

		// Extract MW and GF in Last Epoch
		double GF_0, MW_bar;
		long k_0 = 0;
		bool flag = false;

		for (int j = 0; j < MAX_CHANNEL_NUM; j++)		
		{
			if (obs.ComObs[j].Prn != obs.SatObs[i].Prn || obs.ComObs[j].Sys != obs.SatObs[i].System)
			{
				continue;
			}
			else /* find the same sat */
			{
				flag = true;
				k_0 = obs.ComObs[j].valid_epo_num;
				GF_0 = obs.ComObs[j].GF;
				MW_bar = obs.ComObs[j].MW;
				break;
			}
		}

		// add info into new_combine
		new_combine[i].Prn = obs.SatObs[i].Prn;
		new_combine[i].Sys = obs.SatObs[i].System;
		new_combine[i].LockTime_Last[0] = obs.SatObs[i].LockTime[0];
		new_combine[i].LockTime_Last[1] = obs.SatObs[i].LockTime[1];

		// if is new valid epoch
		if (flag == false || k_0==0)
		{
			obs.SatObs[i].Valid = true;
			new_combine[i].GF = GF;
			new_combine[i].MW = MW;
			new_combine[i].PIF = (freq1 * freq1 * obs.SatObs[i].p[0] - freq2 * freq2 * obs.SatObs[i].p[1]) / (freq1 * freq1 - freq2 * freq2);
			new_combine[i].valid_epo_num = 1;

		} // check MW and GF difference with last epoch
		else if(fabs(GF - GF_0) > 0.05 || fabs(MW - MW_bar) > 3)
		{
			obs.SatObs[i].Valid = false;
			new_combine[i].GF = GF;
			new_combine[i].MW = MW;
			new_combine[i].valid_epo_num = 0;
		}
		else
		{
			obs.SatObs[i].Valid = true;
			MW_bar = (k_0 * MW_bar + MW) / (k_0 + 1);
			new_combine[i].GF = GF;
			new_combine[i].MW = MW_bar;
			new_combine[i].PIF = (freq1 * freq1 * obs.SatObs[i].p[0] - freq2 * freq2 * obs.SatObs[i].p[1]) / (freq1 * freq1 - freq2 * freq2);
			new_combine[i].valid_epo_num = k_0 + 1;
		}

	}

	//// Can use memcpy
	//memcpy(obs.SatPVT, new_combine,sizeof(new_combine));

	// Add new_combine epoch obs data
	for (int i = 0; i < MAX_CHANNEL_NUM; i++)
	{
		if (i < obs.SatNum)
		{
			obs.ComObs[i] = new_combine[i];
		}
		else
		{
			obs.ComObs[i] = COMB_OBS();
		}
		
	}
}


/* Compute Statellite PVT at Transmission Time */
void Sat_PVT_At_Trans(EPOCH_OBS& obs, GNSS_EPHREC& gps_eph, GNSS_EPHREC& bds_eph)
{
	// receiver clock face time
	GPS_TIME t0 = obs.Time;
	GNSS_EPHREC_DATA eph;

	// for-loop over every obervered satellite
	for (int i = 0; i < obs.SatNum; i++)
	{
		// exclude unvalid obs sat
		if (obs.SatObs[i].Valid == false)
		{
			obs.SatPVT[i].Valid = false;
			continue;
		}

		// Turn BDS_OBS GPS_Time into BDS_Time
		GPS_TIME tr = t0;
		if (obs.SatObs[i].System == BDS)
		{
			tr.Week = t0.Week - GPS_BDS_WEEK_DIFF;
			tr.SecOfWeek = t0.SecOfWeek - 14;
		}

		// get corresponding epherise ---> eph
		if (get_eph_data(obs.SatObs[i].Prn, obs.SatObs[i].System, tr, gps_eph, bds_eph, obs.SatPVT[i], eph) != 1)
		{
			obs.SatPVT[i].Valid = false;
			continue;
		}



		// calculate transmission time
		GPS_TIME trans_uncor = GPST_SUB(tr, obs.SatObs[i].p[0] / CLIGHT);
		GPS_TIME trans;
		// clock bias correction
		Calc_ClkOft(trans_uncor, eph, obs.SatPVT[i]);

		// iterate to get more presice transmisson time
		for (int j = 0; j < 2; j++)
		{
			trans = GPST_SUB(trans_uncor, obs.SatPVT[i].SatClkOft);
			Calc_ClkOft(trans, eph, obs.SatPVT[i]);
		}

		// compute sattlelite PVT at trans time
		Sat_PVT(obs.SatObs[i].Prn, obs.SatObs[i].System, trans, eph, obs.SatPVT[i]);
	}

}




// Called when decode an observation message
bool SPP(EPOCH_OBS& obs, GNSS_EPHREC& gps_eph, GNSS_EPHREC& bds_eph,XYZ_Coord& pos)
{
	// Initialize Parameters
	Eigen::MatrixXd X(5, 1);
	X << pos.x, pos.y, pos.z, 0.0, 0.0;

	// validate the obs data
	Detect_Outlier(obs);

	int iter_count = 1;
	int max_iter_count = 10;

	while (1)
	{
		// Matrix Initial
		Eigen::MatrixXd B = Eigen::MatrixXd::Zero(obs.SatNum, 5);
		Eigen::MatrixXd P = Eigen::MatrixXd::Zero(obs.SatNum, obs.SatNum);
		Eigen::MatrixXd L = Eigen::MatrixXd::Zero(obs.SatNum, 1);

		// Remember Valid Sat Index
		int valid_index[MAX_CHANNEL_NUM];
		int neccessary_num = 5;
		memset(valid_index, 0, sizeof(valid_index));

		// compute sattlelite PVT at trans time
		Sat_PVT_At_Trans(obs, gps_eph, bds_eph);

		int valid_obs = 0;
		int GPS_obs = 0;
		int BDS_obs = 0;


		// Construct Equation
		for (int i = 0; i < obs.SatNum; i++)
		{
			// Validation of OBS and Sat_Pos data
			if (obs.SatObs[i].Valid == false || obs.SatPVT[i].Valid == false)
			{
				continue;
			}

			// Earth Rotation Correction
			Rotation_Correction(obs.SatObs[i].System, pos, obs.SatPVT[i]);


			// Calculate Az and El
			obs.SatPVT[i].TropCorr = 0;
			BLH_Coord pos_blh;

			
			if (obs.SatObs[i].System == GPS)
			{
				Calc_SatElAz(pos, obs.SatPVT[i].SatPos, WGS84, obs.SatPVT[i].Elevation, obs.SatPVT[i].Azimuth);
				XYZ2BLH(pos, pos_blh, WGS84);
			}
			else
			{
				Calc_SatElAz(pos, obs.SatPVT[i].SatPos, CGCS2000, obs.SatPVT[i].Elevation, obs.SatPVT[i].Azimuth);
				XYZ2BLH(pos, pos_blh, CGCS2000);
			}


			// Elevation Angle Mask
			if (iter_count > 1 && obs.SatPVT[i].Elevation <= Elev_Mask)
			{
				obs.SatPVT[i].Valid = false;
				continue;
			}


			if (iter_count > 1)
			{
				// Troposphere Correction
				obs.SatPVT[i].TropCorr = Hopfield(pos_blh.H, obs.SatPVT[i].Elevation);
			}

			
			// Fill Matrixs
			XYZ_Coord diff = XYZ_DIFF(pos, obs.SatPVT[i].SatPos);
			XYZ_Coord DC = XYZ_NORMALIZE(diff);
			double rou = sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
			B(valid_obs, 0) = DC.x;
			B(valid_obs, 1) = DC.y;
			B(valid_obs, 2) = DC.z;


			if (obs.SatObs[i].System == GPS)
			{
				B(valid_obs, 3) = 1;

				// Test for Elevation-Base-Obs Weighting
				//obs.SatPVT[i].Weight = 25.0/(4.0 * 4.0 + 3.0 * 3.0 / (sin(obs.SatPVT[i].Elevation) * sin(obs.SatPVT[i].Elevation)));
				//obs.SatPVT[i].Weight = 1.0 / (cos(obs.SatPVT[i].Elevation) * cos(obs.SatPVT[i].Elevation));
				//obs.SatPVT[i].Weight = sin(obs.SatPVT[i].Elevation)*sin(obs.SatPVT[i].Elevation);
				


				P(valid_obs, valid_obs) = obs.SatPVT[i].Weight;
				L(valid_obs, 0) = obs.ComObs[i].PIF - (rou + obs.SatPVT[i].TropCorr - CLIGHT * obs.SatPVT[i].SatClkOft + X(3, 0));
				GPS_obs++;
			}
			else
			{
				double TGD = (FREQ_BDS_B1 * FREQ_BDS_B1) / (FREQ_BDS_B1 * FREQ_BDS_B1 - FREQ_BDS_B3 * FREQ_BDS_B3) * obs.SatPVT[i].Tgd1;  // s
				
				B(valid_obs, 4) = 1;
				
				L(valid_obs, 0) = obs.ComObs[i].PIF - (rou + obs.SatPVT[i].TropCorr - CLIGHT * obs.SatPVT[i].SatClkOft + X(4, 0) + CLIGHT * TGD);
				P(valid_obs, valid_obs) = obs.SatPVT[i].Weight;
				BDS_obs++;
			}
			
			valid_index[valid_obs] = i;
			valid_obs++;
		}


		// Matrix Reconsctruction
		if (GPS_obs == 0)
		{
			Eigen::MatrixXd B_Temp(obs.SatNum, 4);
			Eigen::MatrixXd L_Temp(valid_obs, 1);

			B_Temp << B.leftCols(3), B.rightCols(1);
			L_Temp << L.topRows(valid_obs);

			B = B_Temp.topRows(valid_obs);
			L = L_Temp;

			neccessary_num = 4;
		}
		else if (BDS_obs == 0)
		{
			Eigen::MatrixXd B_Temp(obs.SatNum, 4);
			Eigen::MatrixXd L_Temp(valid_obs, 1);

			B_Temp = B.block(0, 0, valid_obs, 4);
			L_Temp << L.topRows(valid_obs);

			B = B_Temp;
			L = L_Temp;

			neccessary_num = 4;
		}
		else
		{
			Eigen::MatrixXd B_Temp(valid_obs, 5);
			Eigen::MatrixXd L_Temp(valid_obs, 1);

			B_Temp = B.block(0, 0, valid_obs, 5);
			L_Temp << L.topRows(valid_obs);

			B = B_Temp;
			L = L_Temp;
		}

		Eigen::MatrixXd P_Temp(valid_obs, valid_obs);
		P_Temp = P.block(0, 0, valid_obs, valid_obs);
		P = P_Temp;


		// Check if Observation is enough to Compute Unkonws
		if (B.rows() < B.cols())
		{
			return 0;
		}


		// LS Positioning
		Eigen::MatrixXd x;
		Eigen::MatrixXd v;

		Eigen::MatrixXd Qxx = (B.transpose() * P * B).inverse();
		x = Qxx * B.transpose() * P * L;
		v = B * x - L;
		

		if (GPS_obs == 0)
		{
			X(0, 0) += x(0, 0);
			X(1, 0) += x(1, 0);
			X(2, 0) += x(2, 0);
			X(4, 0) += x(3, 0);
		}
		else if (BDS_obs == 0)
		{
			X(0, 0) += x(0, 0);
			X(1, 0) += x(1, 0);
			X(2, 0) += x(2, 0);
			X(3, 0) += x(3, 0);
		}
		else
		{
			X += x;
		}


		
		
		// Positioning Result Output
		pos.x = X(0, 0);
		pos.y = X(1, 0);
		pos.z = X(2, 0);

		// Check LS is Converged
		if ((x.norm()) < 0.01 || iter_count == max_iter_count)
		{
			// Error Dectection
			Eigen::MatrixXd Qvv = (P.inverse() - B * (B.transpose() * P * B).inverse() * B.transpose());

			int Biggest_Outlier = -1;
			double Biggest_Omega = 0;
			for (int i = 0; i < valid_obs; i++)
			{
				int sat_num = valid_index[i];
				double omega_i = fabs(v(i, 0) / sqrt(Qvv(i, i)));

				if (omega_i > 3 && omega_i > Biggest_Omega)
				{
					Biggest_Outlier = sat_num;
					Biggest_Omega = omega_i;
				}
			}

			if (Biggest_Outlier != -1)
			{
				obs.SatPVT[Biggest_Outlier].Valid = false;
				// std::cout << obs.SatObs[Biggest_Outlier].System << "  " << obs.SatObs[Biggest_Outlier].Prn << "  Outlier!!" << std::endl;

				// If Unconverged
				if (iter_count == max_iter_count)
				{
					max_iter_count += 5;
				}

				iter_count++;

				continue;
			}

			

			// Positioning Result Output
			obs.my_result.time = obs.Time;
			obs.my_result.track_satnum = obs.SatNum;

			obs.my_result.Pos.x = X(0, 0);
			obs.my_result.Pos.y = X(1, 0);
			obs.my_result.Pos.z = X(2, 0);


			if (GPS_obs > 0)
				obs.my_result.RcvClkOft[0] = X(3, 0);
			else if (BDS_obs > 0)
				obs.my_result.RcvClkOft[0] = X(4, 0);
			else
				obs.my_result.RcvClkOft[0] = 0.0;
			
			obs.my_result.used_satnum[0] = GPS_obs;
			obs.my_result.used_satnum[1] = BDS_obs;

			
			obs.my_result.IsSuccess = true;
			obs.my_result.PDOP = sqrt(Qxx(0, 0) + Qxx(1, 1) + Qxx(2, 2));
			obs.my_result.Sigma_Pos = sqrt(((v.transpose() * P * v) / (valid_obs - neccessary_num))(0, 0));

			break;
		}

		iter_count++;

	}
	return 1;

}


// Called After SPP Solution
bool SPV(EPOCH_OBS& obs, XYZ_Coord& pos)
{
	// Matrix Initialization
	Eigen::MatrixXd B = Eigen::MatrixXd::Zero(obs.SatNum, 4);
	Eigen::MatrixXd P = Eigen::MatrixXd::Zero(obs.SatNum, obs.SatNum);
	Eigen::MatrixXd L = Eigen::MatrixXd::Zero(obs.SatNum, 1);


	// Numbers of Valid-obs (for Matrix Trimming)
	int valid_obs = 0;


	for (int i = 0; i < obs.SatNum; i++)
	{
		// Validation of OBS and Sat_Pos data
		if (obs.SatObs[i].Valid == false || obs.SatPVT[i].Valid == false)
		{
			continue;
		}

		// Fill Matrixs
		XYZ_Coord diff = XYZ_DIFF(pos, obs.SatPVT[i].SatPos);
		XYZ_Coord DC = XYZ_NORMALIZE(diff);
		double rou = sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);

		B(valid_obs, 0) = DC.x;
		B(valid_obs, 1) = DC.y;
		B(valid_obs, 2) = DC.z;
		B(valid_obs, 3) = 1.0;


		double d = 0;
		if (obs.SatObs[i].System == GPS)
		{
			d = -obs.SatObs[i].d[0] * CLIGHT / FREQ_GPS_L1;
		}
		else
		{
			d = -obs.SatObs[i].d[0] * CLIGHT / FREQ_BDS_B1;
		}

		L(valid_obs, 0) = d - ((obs.SatPVT[i].SatPos.x - pos.x) * obs.SatPVT[i].SatVel.x + (obs.SatPVT[i].SatPos.y - pos.y) * obs.SatPVT[i].SatVel.y + (obs.SatPVT[i].SatPos.z - pos.z) * obs.SatPVT[i].SatVel.z) / rou + CLIGHT * obs.SatPVT[i].SatClkSft;

		P(valid_obs, valid_obs) = 1.0;

	
		valid_obs++;
	}
	
	// Trim Matrix 
	Eigen::MatrixXd B_Temp(valid_obs, 4);
	Eigen::MatrixXd L_Temp(valid_obs, 1);
	Eigen::MatrixXd P_Temp(valid_obs, valid_obs);

	B_Temp = B.block(0, 0, valid_obs, 4);
	L_Temp << L.topRows(valid_obs);
	P_Temp = P.block(0, 0, valid_obs, valid_obs);

	B = B_Temp;
	L = L_Temp;
	P = P_Temp;

	
	// Check if Observation is enough to Compute Unkonws
	if (B.rows() < B.cols())
	{
		return 0;
	}


	// LS Positioning
	Eigen::MatrixXd X;
	Eigen::MatrixXd v;

	Eigen::MatrixXd Qxx = (B.transpose() * P * B).inverse();
	X = Qxx * B.transpose() * L;
	v = B * X - L;


	// Precision Validation
	obs.my_result.Sigma_Vel = sqrt(((v.transpose() * P * v) / (valid_obs - 4))(0, 0));

	// Velocity OutPut
	obs.my_result.Vel.x = X(0, 0);
	obs.my_result.Vel.y = X(1, 0);
	obs.my_result.Vel.z = X(2, 0);


	return 1;
}