#include"my_rtk.hpp"
#include<cmath>

/* Methods of Time System Convertion */
void MJD2Common(const MJD_TIME& MJD, COMMON_TIME& CT)
{
    // ֱ�Ӵ� MJD ��ȡ�������ֺ�С������
    int A = MJD.Days + 2400001;  // JD = MJD + 2400000.5, ��ֱ�Ӽ� 0.5 ������������ + 1
    double F = MJD.FracDay;      // ֱ��ʹ�� MJD С������

    int B = A + 1537;
    int C = static_cast<int>((B - 122.1) / 365.25);
    int D = static_cast<int>(365.25 * C);
    int E = static_cast<int>((B - D) / 30.6001);
    int day = static_cast<int>(B - D - static_cast<int>(30.6001 * E) + F);
    int month = E - 1 - 12 * static_cast<int>(E / 14);
    int year = C - 4715 - static_cast<int>((7 + month) / 10);

    // ����С�����ֵ�Сʱ�����Ӻ���
    double dayFrac = F * 24.0;
    int hour = static_cast<int>(dayFrac);
    double minFrac = (dayFrac - hour) * 60.0;
    int minute = static_cast<int>(minFrac);
    double second = (minFrac - minute) * 60.0;

    // ��� COMMON_TIME �ṹ��
    CT.Year = static_cast<short>(year);
    CT.Month = static_cast<unsigned short>(month);
    CT.Day = static_cast<unsigned short>(day);
    CT.Hour = static_cast<unsigned short>(hour);
    CT.Minute = static_cast<unsigned short>(minute);
    CT.Second = second;
}



void MJD2GPST(const MJD_TIME& MJD, GPS_TIME& GPST)
{
    double totalDays = MJD.Days + MJD.FracDay;                                // MJD��������
    double deltaDays = totalDays - MJD_GPS_EPOCH;                             // ���� GPS �ο�ʱ�������
    GPST.Week = static_cast<unsigned short>(deltaDays / 7.0);                 // ��������
    GPST.SecOfWeek = (MJD.Days - MJD_GPS_EPOCH  - GPST.Week * 7+ MJD.FracDay) * 86400.0;    // ������������(�������پ�����ʧ��
}


void MJD2BDST(const MJD_TIME& MJD, BDS_TIME& BDST)
{
    double totalDays = MJD.Days + MJD.FracDay;                                              // MJD��������
    double deltaDays = totalDays - MJD_BDS_EPOCH;                                           // ���� BDS �ο�ʱ�������
    BDST.Week = static_cast<unsigned short>(deltaDays / 7.0);                               // ��������
    BDST.SecOfWeek = (MJD.Days - MJD_BDS_EPOCH  - BDST.Week * 7 + MJD.FracDay) * 86400.0;    // ������������(�������پ�����ʧ��
}

void Common2MJD(const COMMON_TIME& CT, MJD_TIME& MJD)
{
    int y, m;

    if (CT.Month <= 2)
    {
        y = CT.Year - 1;
        m = CT.Month + 12;
    }
    else
    {
        y = CT.Year;
        m = CT.Month;
    }

    MJD.FracDay = (CT.Hour + (CT.Minute + CT.Second / 60.0) / 60.0) / 24.0;
    double Mjd = static_cast<int>(365.25 * y) + static_cast<int>(30.6001 * (m + 1)) + CT.Day + MJD.FracDay + 1720981.5 - 2400000.5;
    MJD.Days = static_cast<int>(Mjd);
}



void GPST2MJD(const GPS_TIME& GPST, MJD_TIME& MJD)
{
    double totalDays = GPST.Week * 7.0 + GPST.SecOfWeek / 86400.0;                         // GPS ʱ���ʾ��������
    MJD.Days = static_cast<int>(MJD_GPS_EPOCH + totalDays);                                // ת��Ϊ MJD ����������
    MJD.FracDay = GPST.SecOfWeek / 86400.0 - static_cast<int>(GPST.SecOfWeek / 86400.0);   // ����С������
}

void BDST2MJD(const BDS_TIME& BDST, MJD_TIME& MJD)
{
    double totalDays = BDST.Week * 7.0 + BDST.SecOfWeek / 86400.0;                          // BDS ʱ���ʾ��������
    MJD.Days = static_cast<int>(MJD_BDS_EPOCH + totalDays);                                 // ת��Ϊ MJD ����������
    MJD.FracDay = BDST.SecOfWeek / 86400.0 - static_cast<int>(BDST.SecOfWeek / 86400.0);    // ����С������
}

void Common2GPST(const COMMON_TIME& CT, GPS_TIME& GPST)
{
    MJD_TIME MJD;
    Common2MJD(CT, MJD);  // ���Ƚ�����ת��Ϊ MJD
    MJD2GPST(MJD, GPST);  // �ٴ� MJD ת��Ϊ GPS
}

void Common2BDST(const COMMON_TIME& CT, BDS_TIME& BDST)
{
    MJD_TIME MJD;
    Common2MJD(CT, MJD);  // ���Ƚ�����ת��Ϊ MJD
    MJD2BDST(MJD, BDST);  // �ٴ� MJD ת��Ϊ BDS
}

void BDST2Common(const BDS_TIME& BDST, COMMON_TIME& CT)
{
    MJD_TIME MJD;
    BDST2MJD(BDST, MJD);   // ���Ƚ� BDS ת��Ϊ MJD
    MJD2Common(MJD, CT);  // �ٴ�  MJD ת��Ϊ����
}

void GPST2Common(const GPS_TIME& GPST, COMMON_TIME& CT)
{
    MJD_TIME MJD;
    GPST2MJD(GPST, MJD);  // ���Ƚ� GPS ת��Ϊ MJD
    MJD2Common(MJD, CT);  // �ٴ� MJD ת��Ϊ����
}


double GPST_DIFF(const GPS_TIME& t1,const GPS_TIME& t2)
{
    return 86400.0 * (t1.Week - t2.Week) + t1.SecOfWeek - t2.SecOfWeek;
}

GPS_TIME& GPST_SUB(const GPS_TIME& t, const double t2)
{
    GPS_TIME t1 = t;

    t1.SecOfWeek -= t2;
    if (t1.SecOfWeek < 0)
    {
        t1.SecOfWeek += SECONDS_IN_WEEK;
        t1.Week--;
    }

    if (t1.SecOfWeek > SECONDS_IN_WEEK)
    {
        t1.SecOfWeek -= SECONDS_IN_WEEK;
        t1.Week++;
    }

    return t1;
}




/* Methods of Coordinate System Convertion */
void XYZ2BLH(const XYZ_Coord& xyz, BLH_Coord& blh, Ellipsoid ellip)
{
    double x = xyz.x;
    double y = xyz.y;
    double z = xyz.z;

    double a = ellip.R;
    double e = sqrt(ellip.e_2);

    double l = atan2(y,x) * 180.0 / PI;

    double r = sqrt(x * x + y * y);
    double t0 = z / r;
    double k = a * e * e / r;
    double p = 1.0 - e * e;

    double t1 = t0;
    double temp = t0;

    do{

        temp = t1;
        t1 = t0 + k * temp / sqrt(1 + p * temp * temp);

    }while (abs(t1 - temp) > 1e-10);

    double b_rad = atan(t1);
    double b = b_rad * 180.0 / PI;

    double N_i = a / sqrt(1 - e * e * sin(b_rad) * sin(b_rad));
    double h = r / cos(b_rad) - N_i;

    blh.B = b;
    blh.L = l;
    blh.H = h;
}


void BLH2XYZ(const BLH_Coord& blh, XYZ_Coord& xyz, Ellipsoid ellip)
{
    double b = blh.B * PI / 180.0;
    double l = blh.L * PI / 180.0;
    double h = blh.H;

    double a = ellip.R;    
    double e_2 = ellip.e_2;

    double W = sqrt(1 - e_2 * sin(b) * sin(b));
    double N_i = a / W;

    xyz.x = (N_i + h) * cos(b) * cos(l);
    xyz.y = (N_i + h) * cos(b) * sin(l);
    xyz.z = (N_i * (1 - e_2) + h) * sin(b);
}


void Calc_dEnu(const XYZ_Coord& X0, const XYZ_Coord& Xr, NEU_Coord& dNeu, Ellipsoid ellip)
{
    BLH_Coord blh;
    XYZ2BLH(X0, blh, ellip);

    // Degree2Rad
    double B = blh.B * PI / 180.0;
    double L = blh.L * PI / 180.0;

    double dx = Xr.x - X0.x;
    double dy = Xr.y - X0.y;
    double dz = Xr.z - X0.z;

    // Return dNeu
    dNeu.N = -sin(B) * cos(L) * dx - sin(B) * sin(L) * dy + cos(B) * dz;
    dNeu.E = -sin(L) * dx + cos(L) * dy;
    dNeu.U = cos(B) * cos(L) * dx + cos(B) * sin(L) * dy + sin(B) * dz;

}

void Calc_SatElAz(const XYZ_Coord& Xr, const XYZ_Coord& Xs, Ellipsoid ellip, double& Elev, double& Azim)
{
    // Calculate dNeu First
    NEU_Coord d_neu;
    Calc_dEnu(Xr, Xs, d_neu, ellip);

    // Calculate Elev and Azim
    Elev = atan(d_neu.U / sqrt(d_neu.N * d_neu.N + d_neu.E * d_neu.E));
    Azim = atan2(d_neu.E, d_neu.N);
}

double Calc_XYZ_dist(const XYZ_Coord& X0, const XYZ_Coord& Xr)
{
    double dx = X0.x - Xr.x;
    double dy = X0.y - Xr.y;
    double dz = X0.z - Xr.z;

    return sqrt(dx * dx + dy * dy + dz * dz);
}


XYZ_Coord XYZ_DIFF(const XYZ_Coord& x1, const XYZ_Coord& x2)
{
    XYZ_Coord x;
    x.x = x1.x - x2.x;
    x.y = x1.y - x2.y;
    x.z = x1.z - x2.z;
    return x;
}



XYZ_Coord XYZ_NORMALIZE(const XYZ_Coord& x)
{
    XYZ_Coord x_norm;

    double norm = sqrt((x.x * x.x + x.y * x.y + x.z * x.z));

    x_norm.x = x.x / norm;
    x_norm.y = x.y / norm;
    x_norm.z = x.z / norm;

    return x_norm;
}