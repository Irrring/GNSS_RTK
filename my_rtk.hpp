#include<map>
#include <stdexcept>
#include<iostream>
#include <winsock2.h>
#include <cstdio>

#include "Eigen/core"
#include "Eigen/dense"

using namespace std;

/*
*	CONSTANTS and STRUCTURES
*/

#ifndef RTK_Structures
#define RTK_Structures



/* --------------------Constants------------------- */
const double PI = 3.141592653589793;
const double CLIGHT = 299792458.0;        /* speed of light (m/s) */
const size_t MAXRAWLEN = 16384;

const double MJD_GPS_EPOCH = 44244.0;    // GPST Start Time in MJD
const double MJD_BDS_EPOCH = 53736.0;    // BDST Start Time in MJD
const double SECONDS_IN_WEEK = 604800.0; // How Many Seconds in a week
const int    LEAP_SECOND = 37;           // Leap second todate

const double GPS_EPH_MAX = 7200.0;
const double BDS_EPH_MAX = 3600.0;


// GNSS System Annotation
const enum GNSS_Sys { UNKS = 0, GPS, BDS, GLONASS, GALILEO, QZSS }; 
const enum CODE_TYPE 
{
    NONE = 0, L1C, L1P, L1W, L1Y, L1M, L1N, L1S, L1L, L1E, L1A, L1B, L1X, L1Z,
    L2C, L2D, L2S, L2L, L2X, L2P, L2W, L2Y, L2M, L2N, L5I, L5Q, L5X, L7I, L7Q, L7X, L6A, L6B,
    L6C, L6X, L6Z, L6S, L6L, L8I, L8Q, L8X, L2I, L2Q, L6I, L6Q, L3I, L3Q, L3X, L1I, L1Q, L5A,
    L5B, L5C, L9A, L9B, L9C, L9X, L1D, L5D, L5P, L5Z, L6E, L7D, L7P, L7Z, L8D, L8P, L4A, L4B, L4X
};


const int MAX_CHANNEL_NUM = 36;  // Only Consider GPS and BDS 


const double FREQ_GPS_L1 = 1575.42  * 1e6;  // frequency of GPS L1
const double FREQ_GPS_L2 = 1227.60  * 1e6;  // frequency of GPS L2
const double FREQ_BDS_B1 = 1561.098 * 1e6;  // frequency of GPS B1
const double FREQ_BDS_B3 = 1268.52  * 1e6;  // frequency of GPS B3

const int GPS_BDS_WEEK_DIFF = 1356;

// Correctrion Model
const enum Trop_Model { NO = 0, hopfield, saastamoinen };



/* -------------------CONFIGERATION-----------------*/
const short FREQ_NUM = 2;
const size_t MSG_BUFF_SIZE = 32768;

const double delta_pseudo = 0.3;   // Measurement Noise of Pseudorange
const double delta_phase = 0.01;   // Measurement Noise of Carrier Phase


/* -------------------Ellipsoid Definition--------------- */
struct Ellipsoid
{
    double R;           /* Radius Earth [m] */
    double F;           /* Flattening of Earth */
    double Omega;       /* [rad/s], the earth rotation rate */
    double GM_Earth;    /* [m^3/s^2]; */
    double e_2;         /* Eccentricity ratio square */

    Ellipsoid(double r,double f_inv,double omega,double GM)
    {
        R = r;
        F = 1.0 / f_inv;
        Omega = omega;
        GM_Earth = GM;

        double b = R - R * F;
        e_2 = (R * R - b * b) / (R * R);
    }
};
const Ellipsoid WGS84(6378137.0, 298.257223563, 7.2921151467e-5, 398600.5e+9);
const Ellipsoid CGCS2000(6378137.0, 298.257222101, 7.2921150e-5, 398600.4418e+9);


/* -------------------Time System------------------*/
struct COMMON_TIME
{
	short		   Year;
	unsigned short Month;
	unsigned short Day;
	unsigned short Hour;
	unsigned short Minute;
	double		   Second;

    COMMON_TIME()
	{
		Year = 0;
		Month = 0;
		Day = 0;
		Hour = 0;
		Minute = 0;
		Second = 0.0;
	}
};

struct GPS_TIME              
{
    unsigned short Week;
    double         SecOfWeek;

    GPS_TIME()
    {
        Week = 0;
        SecOfWeek = 0.0;
    }

    GPS_TIME(unsigned short week, double sof):Week(week),SecOfWeek(sof)
    {
    }
};

struct MJD_TIME             
{
    int Days;
    double FracDay;

    MJD_TIME()
    {
        Days = 0;
        FracDay = 0.0;
    }
};

struct BDS_TIME
{
    unsigned short Week;
    double         SecOfWeek;

    BDS_TIME()
    {
        Week = 0;
        SecOfWeek = 0.0;
    }

    BDS_TIME(unsigned short week, double sof) :Week(week), SecOfWeek(sof)
    {
    }
};


/* Methods of Time System Convertion */
void MJD2Common(const MJD_TIME& MJD, COMMON_TIME& CT);
void MJD2GPST(const MJD_TIME& MJD, GPS_TIME& GPST);
void MJD2BDST(const MJD_TIME& MJD, BDS_TIME& BDST);

void Common2MJD(const COMMON_TIME& CT, MJD_TIME& MJD);
void GPST2MJD(const GPS_TIME& GPST, MJD_TIME& MJD);
void BDST2MJD(const BDS_TIME& BDST, MJD_TIME& MJD);

void Common2GPST(const COMMON_TIME& CT, GPS_TIME& GPST);
void Common2BDST(const COMMON_TIME& CT, BDS_TIME& BDST);

void GPST2Common(const GPS_TIME& GPST, COMMON_TIME& CT);
void BDST2Common(const BDS_TIME& BDST, COMMON_TIME& CT);

double GPST_DIFF(const GPS_TIME& t1, const GPS_TIME& t2);
GPS_TIME& GPST_SUB(const GPS_TIME& t, const double t2);


/* -------------------Coordinate System------------------ */
struct XYZ_Coord /* GeoFrame Frame in XYZ(m) */
{
    double x;
    double y;
    double z;

    XYZ_Coord()
    {
        x = y = z = 0;
    }

    XYZ_Coord(double x0,double y0,double z0):x(x0),y(y0),z(z0){}
};

struct BLH_Coord /* GeoFrame in Latitude/Longitude/Heigh (degree) */
{
    double B;
    double L;
    double H;

    BLH_Coord()
    {
        B = L = H = 0;
    }

    BLH_Coord(double b, double l, double h) :B(b), L(l), H(h) {}
};

struct NEU_Coord /* LocalFrame in North/East/Up (m) */
{
    double N;
    double E;
    double U;
};


/* Methods of Coordinate System Convertion */
void XYZ2BLH(const XYZ_Coord& xzy, BLH_Coord& blh, Ellipsoid ellip);
void BLH2XYZ(const BLH_Coord& blh, XYZ_Coord& xyz, Ellipsoid ellip);

XYZ_Coord XYZ_DIFF(const XYZ_Coord& x1, const XYZ_Coord& x2);
XYZ_Coord XYZ_NORMALIZE(const XYZ_Coord& x);


/* Calculate Distance/Error in ENU */
void Calc_dEnu(const XYZ_Coord& X0, const XYZ_Coord& Xr, NEU_Coord& dNeu, Ellipsoid ellip);
/* Calculate Elevating and Azimuth Angle Between Reciever and Satellite */
void Calc_SatElAz(const XYZ_Coord& Xr, const XYZ_Coord& Xs, Ellipsoid ellip, double& Elev, double& Azim);
/* Calculate Distance in XYZ */
double Calc_XYZ_dist(const XYZ_Coord& X0, const XYZ_Coord& Xr);


/* ----------------Data Prossess System----------------*/

/* EPHREC for Single Satellite */
struct GNSS_EPHREC_DATA
{
    unsigned short PRN;          // Pseudorandom Noise Number
    GNSS_Sys Sys;                // GNSS System
    GPS_TIME TOC;                // Time of Clock (TOC)
    GPS_TIME TOE;                // Time of Ephemeris (TOE)
    short  SVHealth;             // Satellite Vehicle Health Status (0:OK 1:NOT)
    double ClkBias;              // Clock Bias
    double ClkDrift;             // Clock Drift
    double ClkDriftRate;         // Clock Drift Rate
    double IODE;                 // Issue of Data Ephemeris
    double IODC;                 // Issue of Data Clock
    double TGD1, TGD2;           // Time Group Delay
    double SqrtA;                // Square Root of Semi-Major Axis
    double e;                    // Eccentricity
    double M0;                   // Mean Anomaly at TOC
    double OMEGA0;                // Longitude of Ascending Node
    double i0;                   // Inclination Angle at TOC
    double omega;                // Argument of Perigee
    double OMEGADot;             // Rate of Right Ascension
    double iDot;                 // Rate of Inclination
    double DeltaN;               // Mean Motion difference 
    double Crs, Cuc, Cus, Cic, Cis, Crc; // Harmonic coefficients
    double SVAccuracy;              // Satellite Vehicle Accuracy ---> URA value(m2)

    GNSS_EPHREC_DATA() {
        PRN = SVHealth = 0;
        Sys = UNKS;
        ClkBias = ClkDrift = ClkDriftRate = IODE = IODC = TGD1 = TGD2 = 0.0;
        SqrtA = e = M0 = OMEGA0 = i0 = omega = OMEGADot = iDot = DeltaN = 0.0;
        Crs = Cuc = Cus = Cic = Cis = Crc = SVAccuracy = 0.0;
    }
};

typedef std::map<unsigned short, GNSS_EPHREC_DATA> GNSS_EPHREC;


/* Oberservation/RANGE for Single Statellite */
struct SAT_OBS
{
    short         Prn;
    GNSS_Sys      System;
    double        p[FREQ_NUM];                // m
    double        l[FREQ_NUM];                // cycles
    double        d[FREQ_NUM];                // Hz
    double        cn0[FREQ_NUM];              // dB-Hz
    double        LockTime[FREQ_NUM];         // s
    unsigned char half_Added[FREQ_NUM];       // parity knowed and half cycle added
    bool          no_cycle_slip[FREQ_NUM];    // no cycle-slip in single freq
    bool          Valid;                      // no (dual-fre) cycle-slip

    SAT_OBS()
    {
        Prn = 0;
        System = UNKS;
        Valid = true;

        for (int i = 0; i < FREQ_NUM; i++)
        {
            p[i] = l[i] = d[i] = cn0[i] = LockTime[i]  = half_Added[i] = 0;
            no_cycle_slip[i] = true;
        }
    }
};


/* Middiate result for each satellete */
struct SAT_MIDRES
{
    XYZ_Coord SatPos, SatVel;
    double SatClkOft, SatClkSft;
    double Elevation, Azimuth;
    double TropCorr;
    double Tgd1, Tgd2;
    double Weight;
    bool   Valid;             //false --> 没有星历或星历过期  true --> 计算成功

    SAT_MIDRES()
    {
        Azimuth = 0.0;
        Elevation = PI / 2.0;
        SatClkOft = SatClkSft = 0.0;
        Tgd1 = Tgd2 = TropCorr = 0.0;
        Weight = 1;
        Valid = true;
    }
};


/* Combinations of Observations */
struct COMB_OBS
{
    short    Prn;
    GNSS_Sys Sys;
    double   MW, GF, PIF;
    double   LockTime_Last[FREQ_NUM];    // s - For Cycle-Slip Dectection 
    long     valid_epo_num;

    COMB_OBS()
    {
        Prn = 0;
        Sys = UNKS;
        MW = GF = PIF = 0.0;
        valid_epo_num = 0;

        for (int i = 0; i < FREQ_NUM; i++)
        {
            LockTime_Last[i] = 0.0;
        }
    }
};


/* Possitioning and Velociting Result */
struct POS_RES
{
    GPS_TIME time;
    XYZ_Coord Pos;
    XYZ_Coord Vel;

    double RcvClkOft[2];    // 0-->GPS  1-->BDS
    int  used_satnum[2];    // 0-->GPS  1-->BDS
	

    double PDOP, Sigma_Pos, Sigma_Vel;

    int track_satnum;       // For 2 System
    bool   IsSuccess;       // SPP Result is Success or Not
    

    POS_RES()
    {
        for(int i=0;i<3;i++) 
        {
            Pos.x = Pos.y = Pos.z = Vel.x = Vel.y = Vel.z = 0.0;
            Vel.x = Vel.y = Vel.z = 0.0;
        }

		for (int i = 0; i < 2; i++)
		{
			RcvClkOft[i] = 0.0;
			used_satnum[i] = 0;
		}

        PDOP = Sigma_Pos = Sigma_Vel = 0;
        track_satnum = 0;
    }

};




/* All Observation in an Ephoch --> Time Seperated  */
struct EPOCH_OBS 
{
    GPS_TIME   Time;                            // 接收时刻的接收机钟面时，为 GPS Time
    short      SatNum;                          // 观测到的卫星数
    SAT_OBS    SatObs[MAX_CHANNEL_NUM];         // 每一颗卫星的观测值等
    SAT_MIDRES SatPVT[MAX_CHANNEL_NUM];         // 卫星位置速度等计算结果，数组索引与SatObs相同
    COMB_OBS   ComObs[MAX_CHANNEL_NUM];         // 当前历元的组合观测值，  数组索引与SatObs相同
    POS_RES    rcv_result;                      // 保存基站或NovAtel接收机定位结果
    POS_RES    my_result;

    EPOCH_OBS()
    {
        SatNum = 0;
    }

    void reset()
    {
        SatNum = 0;

        Time = GPS_TIME();
        my_result = POS_RES();
        
        for (int i = 0; i < MAX_CHANNEL_NUM; ++i)
        {
            SatObs[i] = SAT_OBS();       // 使用 SAT_OBS 的默认构造函数重置
            SatPVT[i] = SAT_MIDRES();    // 使用 SAT_MIDRES 的默认构造函数重置
            // ComObs[i] = COMB_OBS();   // 使用 COMB_OBS 的默认构造函数重置 这个不能reset，要保留
        }
    }
};

struct OEM7_MSG
{
    unsigned char message[MAXRAWLEN];  // message content in bytes
    int msg_len;                       // exactlly is Header + Data length -- for buffer-wise decode
    int buff_len;                      // bytes added in message buffer ----- for byte-wise decode

    OEM7_MSG()
    {
        memset(message, 0, sizeof(message));
        msg_len = 0;
        buff_len = 0;
    }

};



/* --------------- Structure For RTK ---------------- */

/* Inter-station Single-Difference Observation for a Single Satellite  */
struct SD_SAT_OBS
{
    short     Prn;
    GNSS_Sys  System;
    short     Valid_status;         // 0: Have Cycle Slip   1: Have parity or No dual-freq Obs  2: Have fine dual-freq Obs 
    double    dP[2], dL[2];         // m   cycles
    short     Bas_idx, Rov_idx;     // Coresponding Index of Raw Obs in Base and Rover Station 

    SD_SAT_OBS()
    {
        Prn = Bas_idx = Rov_idx = 0;
        System = UNKS;
        dP[0] = dL[0] = dP[1] = dL[1] = 0.0;
        Valid_status = 0;
    }
};


/* Inter-station Single-Difference Observation of All Satellite in an Epoch  */
struct SD_EPOCH_OBS
{
    GPS_TIME     Time;
    short        SatNum;
    SD_SAT_OBS   SdSatObs[MAX_CHANNEL_NUM];
    COMB_OBS     SdCombObs[MAX_CHANNEL_NUM];

    SD_EPOCH_OBS()
    {
        SatNum = 0;
    }

    void reset()
    {
        SatNum = 0;
        Time = GPS_TIME();
        
        for (int i = 0; i < MAX_CHANNEL_NUM; i++)
        {
            SdSatObs[i] = SD_SAT_OBS();
        }
    }
};



/*  Double-Difference Epoch Observation in an Epoch --> Inter-station and Inter-satellite  */
struct DD_OBS
{
    XYZ_Coord   dPos;                               // 基线向量
    int         ref_prn[2], ref_idx[2];             // 参考星卫星号与存储位置，0 -> GPS | 1 -> BDS
    int         Sats, DDSatNum[2];                  // 待估的双差模糊度数量，  0 -> GPS | 1 -> BDS
    double      FixedAmb[MAX_CHANNEL_NUM * 4];      // 包括双频最优解[0,AmbNum]和次优解[AmbNum,2*AmbNum]
    double      ResAmb[2], Ratio;                   // LAMBDA浮点解中的模糊度残差
    float       FixRMS[2];                          // 固定解定位中rms误差
    bool        bFixed;                             // true为固定，false为未固定


    DD_OBS()
    {
        int i;
        for (i = 0; i < 2; i++) {
            DDSatNum[i] = 0;    // 各卫星系统的双差数量
            ref_idx[i] = ref_prn[i] = -1;
        }

        Sats = 0;              // 双差卫星总数
        ResAmb[0] = ResAmb[1] = FixRMS[0] = FixRMS[1] = Ratio = 0.0;
        bFixed = false;
        for (i = 0; i < MAX_CHANNEL_NUM * 2; i++)
        {
            FixedAmb[2 * i + 0] = FixedAmb[2 * i + 1] = 0.0;
        }
    }

    void reset()
    {
        int i;
        for (i = 0; i < 2; i++) {
            DDSatNum[i] = 0;    // 各卫星系统的双差数量
            ref_idx[i] = ref_prn[i] = -1;
        }

        dPos = XYZ_Coord();
        Sats = 0;              // 双差卫星总数
        ResAmb[0] = ResAmb[1] = FixRMS[0] = FixRMS[1] = Ratio = 0.0;
        bFixed = false;
        for (i = 0; i < MAX_CHANNEL_NUM * 2; i++)
        {
            FixedAmb[2 * i + 0] = FixedAmb[2 * i + 1] = 0.0;
        }
    }
};




/*  All the Data Require in RTK Positioning  */
struct RTK_RAW {
    OEM7_MSG     Message;
    EPOCH_OBS    BasEpk;
    EPOCH_OBS    RovEpk;
    SD_EPOCH_OBS SdObs;
    DD_OBS       DDObs;
    GNSS_EPHREC  GpsEph, BdsEph;
};


/*  Ambiguity Information in EKF  */
struct Amb_Info
{
	double value[2];   //  The Value of Ambiguity ---> Double Frequency
	int    index;      //  The Index of Ambiguity  -1 --> float  else --> fixed
	bool   isUsed;     //  The Ambiguity is used or not in the new epoch

	Amb_Info()
	{
        isUsed = false;
		value[0] = value[1] = 0.0;
		index = -1;    //  Count the Satellite Sit But Not Ambiguity(have to *2)
	}
};


// Map to Index the ambiguity information in EKF
typedef std::map<int, Amb_Info> Amb_Map;


/*  RTK Positioning in EKF */
struct RTK_EKF
{
	GPS_TIME Time;
    XYZ_Coord Pos;      // Status--Position
	XYZ_Coord Vel;      // Status--Velocity

    Amb_Map GPS_Amb;    // key   --> PRN
	Amb_Map BDS_Amb;    // value --> Ambiguity Information(value[2],index)

    DD_OBS Last_DDObs;

	Eigen::MatrixXd P;  // Covariance Matrix of Status -- meter

	int  state;         // 1 --> Static   0 --> Kinematic
	bool isInit;        // EKF is Initialized or Not


	RTK_EKF()
	{
		isInit = false;
        state = 0;
		Time = GPS_TIME();
		Pos = XYZ_Coord();
		Vel = XYZ_Coord();
        P = Eigen::MatrixXd::Zero(3 + 2 * MAX_CHANNEL_NUM, 3 + 2 * MAX_CHANNEL_NUM);
	}

};



/* --------------- Date Sourece Handler -----------------*/

// 输入类型枚举（核心设计）
enum class InputType {
    NONE,
    FILE_INPUT,
    SOCKET_INPUT
};

class InputHandle {
public:
    // 成员变量
    InputType type_ = InputType::NONE;
    union {
        FILE* fp_;
        SOCKET sock_;
    };

    size_t valid_size = 0;
    size_t buff_len = 0;
    unsigned char msg_buff[MSG_BUFF_SIZE];
    unsigned char* buff_ptr = msg_buff;

    // 构造函数
    InputHandle(const char* filepath);
    InputHandle(const char IP[], const unsigned short Port);

    // 析构函数
    ~InputHandle();

    // 禁用拷贝
    InputHandle(const InputHandle&) = delete;
    InputHandle& operator=(const InputHandle&) = delete;

    // 读取接口
    int read();

private:
    // 资源关闭统一方法
    void close_resource();

    bool OpenSocket(SOCKET& sock, const char* IP, unsigned short port);
    void CloseSocket(SOCKET sock);
    
};



/* --------------- Configurations -----------------*/
struct config_ 
{
    /* ------ About Data Source and Save --------- */

	short IsFileData;        // 0: Socket  1: File
	int BasPort, RovPort;    // Base and Rover Port

	string BasNetIP, RovNetIP;   // IP of Base and Rover Station
	string Base_File, RovFile;   // File Path of Base and Rover Data Source

	string LogFile, OutputFile;  // File Path of Output and Log Files

    string Visual_IP;
    int Visual_Port;
    bool Enable_Visual;    // If Open RTKPLOT Realtime Visual


	/* ------ About RTK Process Config --------- */
    short RoverMode;       // 0: Kinematic  1: Static
    short RTKProcMode;     // 0: Least Square  1: EKF
	short Trop_Model;      // 0: No Correction  1: Hopfield  2: Saastamoinen
	short Ban_GEO;         // 0: No Ban GEO  1: Ban GEO

	double Ratio_Thres;       
    double Elevation_Mask;
};






/* Methods of Data Decode */
int decode_gps_ephemb(OEM7_MSG& msg, GNSS_EPHREC& eph);
int decode_bds_ephemeriseb(OEM7_MSG& msg, GNSS_EPHREC& eph);
int decode_rangeb(OEM7_MSG& msg, EPOCH_OBS& obs);
int decode_rangecmpb(OEM7_MSG& msg, EPOCH_OBS& obs);
int decode_bestpos(OEM7_MSG& msg, EPOCH_OBS& obs);

int decode_NovOem7_Message(OEM7_MSG& msg, EPOCH_OBS& obs, GNSS_EPHREC& gps_eph, GNSS_EPHREC& bds_eph);
int decode_NovOem7_Byte(OEM7_MSG& msg, uint8_t data, EPOCH_OBS& obs, GNSS_EPHREC& gps_eph, GNSS_EPHREC& bds_eph);
int decode_NovOem7_Buff(unsigned char*& buff, size_t size,size_t& Len, OEM7_MSG& msg, EPOCH_OBS& obs, GNSS_EPHREC& gps_eph, GNSS_EPHREC& bds_eph);



/* ----------------SPP Algorithm ----------------*/

int  get_eph_data(const int prn, const GNSS_Sys sys, const GPS_TIME& t, const GNSS_EPHREC& gps_eph, const GNSS_EPHREC& bds_eph, SAT_MIDRES& Mid, GNSS_EPHREC_DATA& eph);
void Calc_ClkOft(const GPS_TIME& t, GNSS_EPHREC_DATA& eph, SAT_MIDRES& Mid);

void Detect_Outlier(EPOCH_OBS& obs);

void   Rotation_Correction(GNSS_Sys sys, const XYZ_Coord& X_rev, SAT_MIDRES& Mid);
double Hopfield(const double H, const double El);
double Saastamoinen(BLH_Coord pos, const double Az, const double El, double humi = 0.0);



int Sat_PVT(const int prn, const GNSS_Sys sys, const GPS_TIME& t, GNSS_EPHREC_DATA& eph, SAT_MIDRES& Mid, short Ban_GEO);


bool SPP(EPOCH_OBS& obs, GNSS_EPHREC& gps_eph, GNSS_EPHREC& bds_eph, XYZ_Coord& pos, config_ config);
bool SPV(EPOCH_OBS& obs, XYZ_Coord& pos);




/* --------------------Lambda Algorithm Function Signatures------------------- */
/* LD factorization (Q=L'*diag(D)*L) */
int LD(int n, const Eigen::MatrixXd& Q, Eigen::MatrixXd& L, Eigen::VectorXd& D);
/* integer gauss transformation */
void gauss(int n, Eigen::MatrixXd& L, Eigen::MatrixXd& Z, int i, int j);
/* permutations */
void perm(int n, Eigen::MatrixXd& L, Eigen::VectorXd& D, int j, double del, Eigen::MatrixXd& Z);
/* lambda reduction (z=Z'*a, Qz=Z'*Q*Z=L'*diag(D)*L) */
void reduction(int n, Eigen::MatrixXd& L, Eigen::VectorXd& D, Eigen::MatrixXd& Z);
/* modified lambda (mlambda) search */
int search(int n, int m, const Eigen::MatrixXd& L, const Eigen::VectorXd& D, const Eigen::VectorXd& zs, Eigen::MatrixXd& zn, Eigen::VectorXd& s);
/* lambda/mlambda integer least-square estimation */
int lambda(int n, int m, const Eigen::VectorXd& a, const Eigen::MatrixXd& Q, Eigen::MatrixXd& F, Eigen::VectorXd& s);
/* --------------------End of Lambda Algorithm Function Signatures------------- */



// 读取配置文件
bool LoadConfig(const string& filePath, config_& config);

/* --------------------RTK Algorithm Function Signatures------------------- */
// 获取同步观测数据
int GetSynObs(InputHandle& rover_src, InputHandle& base_src, RTK_RAW& raw);

// 检查原始观测数据是否有周跳
void Check_Raw_Obs(EPOCH_OBS& base_obs, EPOCH_OBS& rover_obs);

// 构造单差观测值
void Construct_SD_Obs(const EPOCH_OBS& base_obs, const EPOCH_OBS& rover_obs, SD_EPOCH_OBS& SD_obs);

// 验证单差观测值并计算组合值
void Check_SD_Obs(SD_EPOCH_OBS& sd_obs);

// 选择双差观测的参考卫星
bool Seletct_Ref_Sat(const EPOCH_OBS& base_obs, const EPOCH_OBS& rover_obs, SD_EPOCH_OBS& SD_obs, DD_OBS& DD_obs);

// 初始化扩展卡尔曼滤波器的状态向量和协方差矩阵
void RTK_EKF_INIT(RTK_RAW& raw, RTK_EKF& kf);

// 检测参考卫星是否发生变化
void check_RefChange(RTK_RAW& raw, RTK_EKF& kf, int* d_ref);

// 处理EKF中的模糊度状态
void ProcessAmbiguityState(RTK_RAW& Raw, RTK_EKF& EKF, int* d_ref,
    Eigen::MatrixXd& X, Eigen::MatrixXd& Phi_Amb,
    Eigen::MatrixXd& Q);

// 执行扩展卡尔曼滤波器的时间更新步骤
int TimeEstimate(RTK_RAW& Raw, RTK_EKF& EKF,
    Eigen::MatrixXd& X_k_k_1, Eigen::MatrixXd& P_k_k_1);

// 执行扩展卡尔曼滤波器的测量更新步骤
int MeasurementUpdate(RTK_RAW& Raw, const Eigen::MatrixXd& X_k_k_1,
    const Eigen::MatrixXd& P_k_k_1, Eigen::MatrixXd& X_k_k, Eigen::MatrixXd& P_k_k);

// 清理不再使用的卫星模糊度
void RemoveUnusedAmbiguities(Amb_Map& amb_map);

// 更新EKF中的模糊度信息
void UpdateAmbiguityInfo(const RTK_RAW& Raw, RTK_EKF& EKF, const Eigen::MatrixXd& X_k_k);


// 计算RTK浮点解，使用最小二乘法估计位置和浮点模糊度
bool RTK_Float(RTK_RAW& Raw, Eigen::VectorXd& FloatAmb, Eigen::MatrixXd& Qxx);

// 计算RTK固定解，使用LAMBDA方法将浮点模糊度固定为整数
int RTK_FIX(RTK_RAW& Raw, Eigen::VectorXd& FloatAmb, Eigen::MatrixXd& Qxx, double ratio_thres);

// 将RTK定位结果转换为标准格式的输出字符串
string Form_Pos_String(RTK_RAW& raw, int type);

// 实现基于最小二乘的实时差分定位处理
int RTK_LS(config_& config);

// 实现基于卡尔曼滤波的实时差分定位处理
int RTK_KF(config_& config);




#endif // !RTK_Structures





