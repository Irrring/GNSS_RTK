#include"my_rtk.hpp"
#include<iostream>

// using namespace std;


/* Constant */
const double MAXVAL = 8388608.0;
const unsigned long CRC32_POLYNOMIAL = 0xEDB88320L;

/* Sync header */
const unsigned char OEM4SYNC1 = 0xaa;   /* oem7 message start sync code 1 */
const unsigned char OEM4SYNC2 = 0x44;   /* oem7 message start sync code 2 */
const unsigned char OEM4SYNC3 = 0x12;   /* oem7 message start sync code 3 */
const unsigned char OEM4HLEN = 28;      /* oem7 message header length (bytes) */

/* Message ID */
const int ID_RANGECMP     = 140;         /* oem7 range compressed */
const int ID_RANGE        = 43;          /* oem7 range measurement */
const int ID_RAWEPHEM     = 41;          /* oem7 raw ephemeris */
const int ID_GPSEPHEM     = 7;           /* oem7 gps ephemeris */
const int ID_BDSEPHEMERIS = 1696;        /* oem7 gps ephemeris */
const int ID_PSRPOS       = 47;          /* position computed by the receiver */
const int ID_BESTPOS      = 42;          /* position computed by the receiver */

/* Get fields (little-endian) */
#define U1(p)   (*((uint8_t *)(p)))
#define I1(p)   (*((int8_t  *)(p)))
static uint16_t U2(uint8_t* p) { uint16_t u; memcpy(&u, p, 2); return u; }
static uint32_t U4(uint8_t* p) { uint32_t u; memcpy(&u, p, 4); return u; }
static int32_t  I4(uint8_t* p) { int32_t  i; memcpy(&i, p, 4); return i; }
static float    R4(uint8_t* p) { float    r; memcpy(&r, p, 4); return r; }
static double   R8(uint8_t* p) { double   r; memcpy(&r, p, 8); return r; }

/* extend sign ---------------------------------------------------------------*/
static int32_t exsign(uint32_t v, int bits)
{
    return (int32_t)(v & (1 << (bits - 1)) ? v | (~0u << bits) : v);
}


/* signal 5type to obs code ---------------------------------------------------*/
static CODE_TYPE sig2code(GNSS_Sys& sys, int sigtype)
{
    CODE_TYPE code = NONE;

    if (sys == GPS) {
        switch (sigtype) {
        case  0: return code=L1C; /* L1C/A */
        case  5: return code=L2P; /* L2P    (OEM7) */
        case  9: return code=L2W; /* L2P(Y),semi-codeless */
        case 14: return code=L5Q; /* L5Q    (OEM6) */
        case 16: return code=L1L; /* L1C(P) (OEM7) */
        case 17: return code=L2S; /* L2C(M) (OEM7) */
        }
    }
    else if (sys == BDS) {
        switch (sigtype) {
        case  0: return code=L2I; /* B1I with D1 (OEM6) */
        case  1: return code=L7I; /* B2I with D1 (OEM6) */
        case  2: return code=L6I; /* B3I with D1 (OEM7) */
        case  4: return code=L2I; /* B1I with D2 (OEM6) */
        case  5: return code=L7I; /* B2I with D2 (OEM6) */
        case  6: return code=L6I; /* B3I with D2 (OEM7) */
        case  7: return code=L1P; /* B1C(P) (OEM7) */
        case  9: return code=L5P; /* B2a(P) (OEM7) */
        case 11: return code=L7D; /* B2b(I) (OEM7,F/W 7.08) */
        }
    }
   
    return code;
}


/* obs code to frequency index ---------------------------------------------------*/
static int code2freq_idx(GNSS_Sys& sys, CODE_TYPE& code)
{
    if (sys == GPS)
    {
        if (code == L1C)
        {
            return 0;
        }
        else if (code == L2W)
        {
            return 1;
        }
        else
        {
            // std::cerr << "Not Supported Signal Type" << std::endl;
            return -1;
        }
       
    }
    else if(sys==BDS)
    {
        if (code == L2I)
        {
            return 0;
        }
        else if (code == L6I)
        {
            return 1;
        }
        else
        {
            // std::cerr << "Not Supported Signal Type" << std::endl;
            return -1;
        }
    }
}

/* Maybe get back into hpp */
static double freq_idx2freq(GNSS_Sys& sys, int idx)
{
    if (sys == GPS)
    {
        switch (idx)
        {
        case 0:return 1575.42 * 1e6; // L1
        case 1:return 1227.60 * 1e6; // L2
        case 2:return 1176.45 * 1e6;
        default: return -1;
        }
    }
    else if (sys == BDS)
    {
        switch (idx)
        {
        case 0:return 1561.098 * 1e6; // B1I
        case 1:return 1268.52 * 1e6;  // B3I
        default: return -1;
        }
    }
}



/* --------------------------------------------------------------------------
Calculate a CRC value to be used by CRC calculation functions.
-------------------------------------------------------------------------- */
unsigned long CRC32Value(int i) {

	int j;
	unsigned long ulCRC;
	ulCRC = i;

	for (j = 8; j > 0; j--) {

		if (ulCRC & 1)
			ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
		else
			ulCRC >>= 1;
	}
	return ulCRC;
}

/* --------------------------------------------------------------------------
Calculates the CRC-32 of a block of data all at once

ulCount - Number of bytes in the data block
ucBuffer - Data block
-------------------------------------------------------------------------- */
unsigned long CalculateBlockCRC32(unsigned long ulCount, unsigned char* ucBuffer) {

    unsigned long ulTemp1;
    unsigned long ulTemp2;
    unsigned long ulCRC = 0;

    while (ulCount-- != 0) {
        ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
        ulTemp2 = CRC32Value(((int)ulCRC ^ *ucBuffer++) & 0xFF);
        ulCRC = ulTemp1 ^ ulTemp2;
    }
    return(ulCRC);
}


/* Get Sync Code of AA4412 */
static int sync_oem4(uint8_t* buff, uint8_t data)
{
    buff[0] = buff[1]; buff[1] = buff[2]; buff[2] = data;
    return buff[0] == OEM4SYNC1 && buff[1] == OEM4SYNC2 && buff[2] == OEM4SYNC3;
}


/* Decode Channel tracking status word */
static int decode_track_stat(uint32_t stat, GNSS_Sys& sys, int* track,
    int* plock, int* clock, int* parity, int* halfc)
{
    CODE_TYPE code;

    int satsys, sigtype;
    int freq_idx;

    // Bit Wise Decode
    code = NONE;
    *track = stat & 0x1F; // -------> ********43210 & 000000011111 ===== 43210 ------> Tracking state
    *plock = (stat >> 10) & 1; // -------> *********0 & 000000000001 ===== 0 ------> Get the 10 bit
    *parity = (stat >> 11) & 1;
    *clock = (stat >> 12) & 1;
    satsys = (stat >> 16) & 7; // 7 ---> 0000001111
    *halfc = (stat >> 28) & 1;
    sigtype = (stat >> 21) & 0x1F;

    // Get GNSS_System
    switch (satsys) 
    {
    case 0: sys = GPS; break;
    //case 1: *sys = SYS_GLO; break;
    //case 2: *sys = SYS_SBS; break;
    //case 3: *sys = SYS_GAL; break;
    case 4: sys = BDS; break;
    //case 5: *sys = SYS_QZS; break; 
    //case 6: *sys = SYS_IRN; break;
    default:
        // std::cerr << "System is NOT supported except for GPS/BDS right now" << std::endl;
        return -1;
    }

    // Get Code Type
    code = sig2code(sys, sigtype);

    if (!code)
    {
        // std::cerr << "Unknown Code Type" << std::endl;
        return -1;
    }

    // Get Freq of Code
    freq_idx = code2freq_idx(sys, code);
    
    return freq_idx;
}


int decode_rangecmpb(OEM7_MSG& msg, EPOCH_OBS& obs)
{
    double psr, adr, adr_rolls, lockt, dop, cn0, freq, glo_bias = 0.0;
    int i, prn, idx, track, plock, clock, parity, halfc;

    GNSS_Sys sys = UNKS;
    CODE_TYPE code = NONE;

    // ret set memory of obs
    obs.reset();

    // Get Observation time (Week and Tow) from Header
    int week = U2(msg.message + 14);
    double tow = U4(msg.message + 16) * 0.001;

    // Pointer of the Current Bytes in Message->Data
    uint8_t* p = msg.message + OEM4HLEN;


    // Number of satellite observations with information to follow
    int nobs = U4(p); 

    // Check rangecmpb data length error
    if (msg.msg_len < OEM4HLEN + 4 + nobs * 24)
    {
        // std::cerr << "oem4 rangecmpb length error: len" << msg.msg_len <<"nobs = " << nobs << std::endl;
        return -1;
    }

    // Decode all Stattle Obs Data in a for-loop
    for (i = 0, p += 4; i < nobs; i++, p += 24)
    {
        // Channel tracking status word
        if ((idx = decode_track_stat(U4(p), sys, &track, &plock, &clock, &parity, &halfc)) < 0) 
        {
            continue;  // check is support freq 
        }

        // Get prn dop psr
        prn = U1(p + 17);
        dop = exsign(U4(p + 4) & 0xFFFFFFF, 28) / 256.0;
        psr = (U4(p + 7) >> 4) / 128.0 + U1(p + 11) * 2097152.0;

        // Get freq
        freq = freq_idx2freq(sys, idx);

        if (freq != 0.0)
        {
            adr = I4(p + 12) / 256.0;
            adr_rolls = (psr * freq / CLIGHT + adr) / MAXVAL;
            adr = -adr + MAXVAL * floor(adr_rolls + (adr_rolls <= 0 ? -0.5 : 0.5));
        }
        else {
            adr = 1e-9;
        }

        // lock time
        lockt = (U4(p + 18) & 0x1FFFFF) / 32.0;

        // Get C_N0
        cn0 = ((U2(p + 20) & 0x3FF) >> 5) + 20.0;

        if (!clock) psr = 0.0;               /* code unlock */
        if (!plock) adr = dop = 0.0;         /* phase unlock */


        // Input Data to Obs Struct

        bool flag = false; // check Prn and Sys already added

        for (int j = 0; j < obs.SatNum; j++) /* for-loop in added sat */
        {
            if (obs.SatObs[j].Prn != prn || obs.SatObs[j].System != sys)
            {
                continue;  // check is support freq 
            }
            else /* find the same sat */
            {
                flag = true;
                
                obs.SatObs[j].cn0[idx] = cn0;
                obs.SatObs[j].p[idx] = psr;
                obs.SatObs[j].l[idx] = adr;
                obs.SatObs[j].d[idx] = dop;
                obs.SatObs[j].LockTime[idx] = lockt;

                break;
            }
        }

        if (!flag)
        {
            obs.SatObs[obs.SatNum].Prn = prn;
            obs.SatObs[obs.SatNum].System = sys;
            obs.SatObs[obs.SatNum].cn0[idx] = cn0;
            obs.SatObs[obs.SatNum].p[idx] = psr;
            obs.SatObs[obs.SatNum].l[idx] = adr;
            obs.SatObs[obs.SatNum].d[idx] = dop;
            obs.SatObs[obs.SatNum].LockTime[idx] = lockt;

            obs.SatNum++;
        }
    }

    // Input Observation Time
    obs.Time.Week = week;
    obs.Time.SecOfWeek = tow;

    return 1;
}

int decode_rangeb(OEM7_MSG& msg, EPOCH_OBS& obs)
{
    // Declarations
    double psr, adr, lockt, dop, cn0;
    int i, prn, idx, track, plock, clock, parity, halfc;

    GNSS_Sys sys = UNKS;
    CODE_TYPE code = NONE;

    // Reset memory of obs
    obs.reset();

    // Get Observation time (Week and Tow) from Header
    int week = U2(msg.message + 14);
    double tow = U4(msg.message + 16) * 0.001;

    // Pointer of the Current Bytes in Message->Data
    uint8_t* p = msg.message + OEM4HLEN;

    // Number of satellite observations with information to follow
    int nobs = U4(p);

    // Check rangecmpb data length error
    if (msg.msg_len < OEM4HLEN + 4 + nobs * 24)
    {
        // std::cerr << "oem4 rangecmpb length error: len" << msg.msg_len << "nobs = " << nobs << std::endl;
        return -1;
    }

    // Decode all Stattle Obs Data in a for-loop
    for (i = 0, p += 4; i < nobs; i++, p += 44)
    {
        // Channel tracking status word
        if ((idx = decode_track_stat(U4(p+40), sys, &track, &plock, &clock,&parity, &halfc)) < 0) 
        {
            continue; // check is support freq 
        }

        // Get Basic Data
        prn   = U2(p);
        psr   = R8(p + 4 );
        adr   = -1.0 * R8(p + 16);
        dop   = R4(p + 28);
        cn0   = R4(p + 32);
        lockt = R4(p + 36);

        if (!clock) psr = 0.0;              /* code  unlock */
        if (!plock) adr = dop = 0.0;        /* phase unlock */

        // ---------- Put Data into Obs Struct --------------

        bool flag = false; // check Prn and Sys already added

        for (int j = 0; j < obs.SatNum; j++) /* for-loop in added sat */
        {
            if (obs.SatObs[j].Prn != prn || obs.SatObs[j].System != sys)
            {
                continue;
            }
            else /* find the same sat */
            {
                flag = true;

                obs.SatObs[j].cn0[idx] = cn0;
                obs.SatObs[j].p[idx] = psr;
                obs.SatObs[j].l[idx] = adr;
                obs.SatObs[j].d[idx] = dop;
                obs.SatObs[j].LockTime[idx] = lockt;
                obs.SatObs[j].half_Added[idx] = parity;

                break;
            }
        }

        if (!flag)
        {
            obs.SatObs[obs.SatNum].Prn = prn;
            obs.SatObs[obs.SatNum].System = sys;
            obs.SatObs[obs.SatNum].cn0[idx] = cn0;
            obs.SatObs[obs.SatNum].p[idx] = psr;
            obs.SatObs[obs.SatNum].l[idx] = adr;
            obs.SatObs[obs.SatNum].d[idx] = dop;
            obs.SatObs[obs.SatNum].LockTime[idx] = lockt;
            obs.SatObs[obs.SatNum].half_Added[idx] = parity;

            obs.SatNum++;
        }
    }

    // Input Observation Time
    obs.Time.Week = week;
    obs.Time.SecOfWeek = tow;
    return 1;
}

int decode_gps_ephemb(OEM7_MSG& msg, GNSS_EPHREC& eph)
{
    double tow, N;
    unsigned short health, IODE1, IODE2, week, z_week;
    bool AS;


    GNSS_EPHREC_DATA data;

    uint8_t* p = msg.message + OEM4HLEN;

    data.Sys    = GPS;
    data.PRN    = U4(p);       p += 4;
    tow         = R8(p);       p += 8;
    data.SVHealth = (U4(p)>>6) &1 ;  p += 4;
    IODE1       = U4(p); p += 4;
    IODE2       = U4(p); p += 4;
    week        = U4(p); p += 4;
    z_week      = U4(p); p += 4;
    data.TOE.SecOfWeek = R8(p); p += 8;
    data.SqrtA  = sqrt(R8(p)); p += 8;
    data.DeltaN = R8(p);   p += 8;
    data.M0     = R8(p);   p += 8;
    data.e      = R8(p);   p += 8;
    data.omega  = R8(p);   p += 8;
    data.Cuc    = R8(p);   p += 8;
    data.Cus    = R8(p);   p += 8;
    data.Crc    = R8(p);   p += 8;
    data.Crs    = R8(p);   p += 8;
    data.Cic    = R8(p);   p += 8;
    data.Cis    = R8(p);   p += 8;
    data.i0     = R8(p);   p += 8;
    data.iDot   = R8(p);   p += 8;
    data.OMEGA0 = R8(p);   p += 8;
    data.OMEGADot = R8(p);   p += 8;
    data.IODC     = U4(p);   p += 4; /* AODC */
    data.TOC.SecOfWeek = R8(p); p += 8;
    data.TGD1     = R8(p);   p += 8;
    data.ClkBias  = R8(p);   p += 8;
    data.ClkDrift = R8(p);   p += 8;
    data.ClkDriftRate = R8(p);   p += 8;
    AS = U4(p);              p += 4;
    N = R8(p);               p += 8;
    data.SVAccuracy = R8(p);

    data.TOC.Week = data.TOE.Week = week;
    data.IODE = IODE1;

    eph[data.PRN] = data;

    return 0;
}

int decode_bds_ephemeriseb(OEM7_MSG& msg, GNSS_EPHREC& eph)
{
    GNSS_EPHREC_DATA data;
    unsigned short week;

    uint8_t* p = msg.message + OEM4HLEN;

    data.Sys = BDS;
    data.PRN = U4(p);              p += 4;
  
    week                = U4(p);   p += 4; /* Satellite Vehicle Accuracy--->URA value(m) */
    data.SVAccuracy     = R8(p);   p += 8; 
    data.SVHealth       = U4(p)&1; p += 4;
    data.TGD1           = R8(p);   p += 8; /* TGD1 for B1 (s) */
    data.TGD2           = R8(p);   p += 8; /* TGD2 for B2 (s) */
    data.IODC           = U4(p);   p += 4; /* AODC */
    data.TOC.SecOfWeek  = U4(p);   p += 4;
    data.ClkBias        = R8(p);   p += 8;
    data.ClkDrift       = R8(p);   p += 8;
    data.ClkDriftRate   = R8(p);   p += 8;
    data.IODE           = U4(p);   p += 4; /* AODE */
    data.TOE.SecOfWeek  = U4(p);   p += 4;
    data.SqrtA          = R8(p);   p += 8;
    data.e              = R8(p);   p += 8;
    data.omega          = R8(p);   p += 8;
    data.DeltaN         = R8(p);   p += 8;
    data.M0             = R8(p);   p += 8;
    data.OMEGA0         = R8(p);   p += 8;
    data.OMEGADot       = R8(p);   p += 8;
    data.i0             = R8(p);   p += 8;
    data.iDot           = R8(p);   p += 8;
    data.Cuc            = R8(p);   p += 8;
    data.Cus            = R8(p);   p += 8;
    data.Crc            = R8(p);   p += 8;
    data.Crs            = R8(p);   p += 8;
    data.Cic            = R8(p);   p += 8;
    data.Cis            = R8(p);

    data.TOC.Week = data.TOE.Week = week;

    eph[data.PRN] = data;

    return 0;
}


int decode_bestpos(OEM7_MSG& msg, EPOCH_OBS& obs)
{
    XYZ_Coord xyz;
    BLH_Coord blh;

    obs.rcv_result.time.Week = U2(msg.message + 14);
    obs.rcv_result.time.SecOfWeek = U4(msg.message + 16) * 0.001;

    uint8_t* p = msg.message + OEM4HLEN;      

    blh.B =  R8(p + 8 );  // degree
    blh.L =  R8(p + 16);  // degree
    blh.H =  R8(p + 24);  // m ----> Height above mean sea level (metres)
    blh.H += R4(p + 32);  // m ----> Undulation - the relationship between thend  geoid athe ellipsoid (m) of the chosen datum

    obs.rcv_result.track_satnum = U1(p + 64);
    obs.rcv_result.used_satnum[0] = U1(p + 65);

    BLH2XYZ(blh, xyz, WGS84);

    obs.rcv_result.Pos = xyz;


    return 0;
}



/* Shift Message Buffer Data -----------------------------------------------------------
* Call when buff data is almost used and shift data from the end of buffer to the front
* args   :
*          unsigned char*& buff  I   buffer pointer's reference
*          size_t& len           I   bytes undecoded remain in buffer
*          size_t& size          I   buffer vaild data size (include decoded datas)
* return : 0                     O   if success
*-----------------------------------------------------------------------------*/
//int shift_msg_buff(unsigned char*& buff, size_t& len, size_t& size)
//{
//    size_t rest = size - len;
//    unsigned char* head = buff - rest;
//
//    for (size_t i = 0; i < len; i++)
//    {
//        head[i] = buff[i];
//    }
//
//    buff = head;
//
//    return 0;
//}

int shift_msg_buff(unsigned char*& buff, size_t& len, size_t& size)
{
    // 获取缓冲区的初始指针
    unsigned char* buffer_start = buff - (size - len);

    // 使用memmove安全地移动数据（处理重叠情况）
    if (buff != buffer_start && len > 0) {
        memmove(buffer_start, buff, len);
    }
    //else
    //{
    //    cout<<"heyheyhey"<<endl;
    //}

    // 更新缓冲区指针
    buff = buffer_start;

    return 0;
}



/* Decode NovAtel OEM7 Message */
/* Return
*  1 ----> Success and Read Obs data
*  0 ----> Success and Read Nav data
*  -1 ---> Some Error (e.g. CRC)
*/
int decode_NovOem7_Message(OEM7_MSG& msg, EPOCH_OBS& obs, GNSS_EPHREC& gps_eph, GNSS_EPHREC& bds_eph)
{
    
    /* check crc32 */
	if (CalculateBlockCRC32(msg.msg_len, msg.message) != U4(msg.message + msg.msg_len))
	{
		// std::cerr<<"Message Fails in CRC32 Check";
		return -1;
	}

    /* message type: 0=binary,1=ascii */
    int msg_type = (U1(msg.message + 6) >> 4) & 0x3;
    if (msg_type != 0)
    {
        // std::cerr << "Message is ASCII Type" << std::endl;
        return -1;
    }

     
    /* Indicates the quality of the GPS reference time */
    int stat = U1(msg.message + 13);
    if (stat == 20) 
    {
        double tow = U4(msg.message + 16) * 0.001;
        // std::cerr << "Message quality of the GPS reference time too bad" << "Message Time: " << tow << std::endl;
        return -1;
    }

    /* Message ID */
    int type = U2(msg.message + 4);

    /* Decode function based on Message ID */
	switch (type)
	{
	//    case ID_RANGECMP:       return decode_rangecmpb(msg, obs);
	    case ID_RANGE:          return decode_rangeb(msg, obs);
	    case ID_GPSEPHEM:       return decode_gps_ephemb(msg, gps_eph);
        case ID_BDSEPHEMERIS:   return decode_bds_ephemeriseb(msg, bds_eph);
        case ID_BESTPOS:        return decode_bestpos(msg, obs);
	}

    // If read obs data
    if (type == ID_RANGE ) return 1;

    // If read nav data
    return 0;
}

/* input NovAtel OEM7 raw data from stream -------------------------------
*  fetch next NovAtel OEM7 raw data and input a mesasge from stream
*---------------------------------------------------------------------------*/
int decode_NovOem7_Byte(OEM7_MSG& msg, uint8_t data, EPOCH_OBS& obs, GNSS_EPHREC& gps_eph, GNSS_EPHREC& bds_eph)
{
    // Find Syn Header
    if (msg.buff_len == 0) {
        if (sync_oem4(msg.message, data))
            msg.buff_len = 3;
        return 0;
    }

    // Add a byte data if in a message 
    msg.message[msg.buff_len] = data;
    msg.buff_len++;

    // Check message length
    if (msg.buff_len == 10 && (msg.msg_len = U2(msg.message + 8) + OEM4HLEN) > MAXRAWLEN - 4)
    {
        // std::cerr << "Message Length Out of Range" << std::endl;
        msg.buff_len = 0;
        return -1;
    }

    // Trim a message
    if (msg.buff_len < 10 || msg.buff_len < msg.msg_len + 4) return 0;
    msg.buff_len = 0;

    /* decode oem7/6/4 message */
    return decode_NovOem7_Message(msg, obs, gps_eph, bds_eph);
}



/* Decode OEM7 from a Buffer -----------------------------------------------------------
* Called after message buffer filled
* args   :
*          unsigned char*& buff  I   reference of msg buffer pointer 
*          size_t& len           I   bytes undecoded remain in buffer
*          size_t& size          I   buffer vaild data size (include decoded datas) ---> Incase Buffer is not full 
*          OEM7_MSG& msg         IO  Extrected single message for decode
*          EPOCH_OBS obs         IO  Obeservation data for an epoch
*          GNSS_EPHREC& gps_eph  IO  GPS statellite ephemeries
*          GNSS_EPHREC& bds_eph  IO  BDS statellite ephemeries
* return : 
*          0                     O   Buffer almost decoded   ---> Get out of the function and add data into buffer again
*          1                     O   Get An observation data ---> Get out to have a spp spv solution
* -----------------------------------------------------------------------------*/
int decode_NovOem7_Buff(unsigned char*& buff, size_t size,size_t &Len, OEM7_MSG& msg, EPOCH_OBS& obs, GNSS_EPHREC& gps_eph, GNSS_EPHREC& bds_eph)
{
    // Loop to Extract Messages
    while (true)
    {
        // Find Syn Header
        while (true)
        {
            if (Len <= 3)
            {
                shift_msg_buff(buff, Len, size);
                return 0;  // -------> Get out of the function and add data into buffer again
            }
                

            if (!buff)
            {
                cout << "heyheyhey" << endl;
            }

            if (buff[0] == OEM4SYNC1 && buff[1] == OEM4SYNC2 && buff[2] == OEM4SYNC3)
            {
                break;
            } 
            else
            {
                buff++;
                Len--;
            } 
        }
        
        // reset message
        memset(&msg, 0, sizeof(OEM7_MSG));

        // Get Message Length
        msg.msg_len = U2(buff + 8) + OEM4HLEN;

        // Check Buffer Length
        if ((OEM4HLEN + msg.msg_len + 4) > Len)
        {
            shift_msg_buff(buff, Len,size);
            return 0;    // -------> Get out of the function and add data into buffer again
        }

        // Extract a message
        memcpy(msg.message, buff, (static_cast<size_t>(msg.msg_len) + 4));

        // decode oem7 message
        int result = decode_NovOem7_Message(msg, obs, gps_eph, bds_eph);

        if (result == -1) // Something Error in the message e.g. CRC / Message quality
        {
            buff += 3;
            Len -= 3;
            continue;   // -------> next loop to find another message
        }

        // Subtract Len and Shift buff*
        Len -= (msg.msg_len + 4);
        buff += (msg.msg_len + 4);

        if (result == 1) // Get An observation data ------> get out to have a solution
        {
            return 1;
        }  
    }

}

