#pragma once
#include<iostream>
#include<fstream>
#include<algorithm>
#include<cstdio>
#include<cmath>
#include<time.h>
#include<string>
#include<cstring>
#include<vector>
#include<map>
#include<set>
using namespace std;

/* global variables */
extern int rec_num;                     // receiver number

/* constants -----------------------------------------------------------------*/
#define SQR(x)      ((x)*(x))

#define PI          3.1415926535897932  // pi
#define D2R         (PI/180.0)          // deg to rad
#define R2D         (180.0/PI)          // rad to deg
#define AU          149597870691.0      // 1 AU (m)
#define AS2R        (D2R/3600.0)        // arc sec to radian
#define CLIGHT      299792458.0         // speed of light (m/s)

/* earth parameters -----------------------------------------------------------*/
#define OMGE        7.2921151467E-5     // earth angular velocity (IS-GPS) (rad/s)
#define RE_WGS84    6378137.0           // earth semimajor axis (WGS84) (m)
#define FE_WGS84    (1.0/298.257223563) // earth flattening (WGS84)

/* max value ------------------------------------------------------------------*/
#define MAXPOSHEAD  1024                // max head line position
#define MAXANT      64                  // max length of station name/antenna type
#define MAXRCV      64                  // max receiver number (1 to MAXRCV)
#define MAXOBS      64                  // max number of obs in an epoch
#define MAXLEAPS    64                  // max number of leap seconds table
#define MAXOBSTYPE  64					// max obs type
#define MAXRNXLEN   (16*MAXOBSTYPE+4)   // max rinex record length
#define MAXINFILE   128	                // max input files
#define MAXOUTFILE  50	                // max output files
#define MAXSTRPATH  1024                // max length of stream path

/* time system ----------------------------------------------------------------*/
#define DTTOL       0.005               // tolerance of time difference (s)

#define TIMES_GPST  0                   // time formart: gps */
#define TIMES_UTC   1                   // time formart: utc */
#define TIMES_JST   2                   // time formart: jst */

#define MAXDTOE_GPS 7200.0              // max time difference to GPS Toe (s)
#define MAXDTOE_GAL 10800.0             // max time difference to Galileo Toe (s)
#define MAXDTOE_CMP 21600.0             // max time difference to BeiDou Toe (s)
#define MAXDTOE_GLO 1800.0              // max time difference to GLONASS Toe (s)
#define MAXDTOE_S   86400.0             // max time difference to ephem toe (s) for other

#define TSYS_GPS    0                   // time system: GPS time
#define TSYS_UTC    1                   // time system: UTC
#define TSYS_GLO    2                   // time system: GLONASS time
#define TSYS_GAL    3                   // time system: Galileo time
#define TSYS_BDS    4                   // time system: BeiDou time

/* satellite system (GPS,GLO,GAL,BDS) -----------------------------------------*/
#define MINPRNGPS   1                       // min satellite PRN number of GPS
#define MAXPRNGPS   32                      // max satellite PRN number of GPS
#define NSATGPS     (MAXPRNGPS-MINPRNGPS+1) // number of GPS satellites
#define NSYSGPS     1

#define MINPRNGLO   1                       // min satellite slot number of GLONASS
#define MAXPRNGLO   27                      // max satellite slot number of GLONASS
#define NSATGLO     (MAXPRNGLO-MINPRNGLO+1) // number of GLONASS satellites
#define NSYSGLO     1

#define MINPRNGAL   1                       // min satellite PRN number of Galileo
#define MAXPRNGAL   30                      // max satellite PRN number of Galileo
#define NSATGAL    (MAXPRNGAL-MINPRNGAL+1)  // number of Galileo satellites
#define NSYSGAL     1

#define MINPRNBDS   1                       // min satellite sat number of BeiDou
#define MAXPRNBDS   35                      // max satellite sat number of BeiDou
#define NSATBDS     (MAXPRNBDS-MINPRNBDS+1) // number of BeiDou satellites
#define NSYSBDS     1

#define MAXSAT      (NSATGPS+NSATGLO+NSATGAL+NSATBDS)

#define NSYS_USED   5                   // number of used satellite systems, GPS/GLO/BDS/GAL/QZS
#define NUMSYS      4                   // number of GNSS system

/* freq system ------------------------------------------------------------*/
#define FREQ1       1.57542E9           // L1/E1  frequency (Hz) 
#define FREQ2       1.22760E9           // L2     frequency (Hz) 
#define FREQ5       1.17645E9           // L5/E5a frequency (Hz) 
#define FREQ6       1.27875E9           // E6/LEX frequency (Hz) 
#define FREQ7       1.20714E9           // E5b    frequency (Hz) 
#define FREQ8       1.191795E9          // E5a+b  frequency (Hz) 
#define FREQ9       2.492028E9          // S      frequency (Hz) 

#define FREQ1_GLO   1.60200E9           // GLONASS G1 base frequency (Hz) 
#define DFRQ1_GLO   0.56250E6           // GLONASS G1 bias frequency (Hz/n) 
#define FREQ2_GLO   1.24600E9           // GLONASS G2 base frequency (Hz) 
#define DFRQ2_GLO   0.43750E6           // GLONASS G2 bias frequency (Hz/n) 
#define FREQ3_GLO   1.202025E9          // GLONASS G3 frequency (Hz) 

#define FREQ1_BDS   1.561098E9          // BeiDou B1 frequency (Hz)
#define FREQ2_BDS   1.20714E9           // BeiDou B2 frequency (Hz)
#define FREQ3_BDS   1.26852E9           // BeiDou B3 frequency (Hz)

#define NFREQ       3                   // number of carrier frequencies
#define MAXFREQ     7                   // max NFREQ

/* navigation system -----------------------------------------------------*/
#define SYS_NONE    0x00                // navigation system: none
#define SYS_GPS     0x01                // navigation system: GPS
#define SYS_GLO     0x02                // navigation system: GLONASS
#define SYS_GAL     0x04                // navigation system: Galileo
#define SYS_BDS     0x08                // navigation system: BeiDou
#define SYS_ALL     0xFF                // navigation system: all

/* obs code --------------------------------------------------------------*/
#define CODE_NONE   0                   // obs code: none or unknown 
#define CODE_L1C    1                   // obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) 
#define CODE_L1P    2                   // obs code: L1P,G1P    (GPS,GLO) 
#define CODE_L1W    3                   // obs code: L1 Z-track (GPS) 
#define CODE_L1Y    4                   // obs code: L1Y        (GPS) 
#define CODE_L1M    5                   // obs code: L1M        (GPS) 
#define CODE_L1N    6                   // obs code: L1codeless (GPS) 
#define CODE_L1S    7                   // obs code: L1C(D)     (GPS,QZS) 
#define CODE_L1L    8                   // obs code: L1C(P)     (GPS,QZS) 
#define CODE_L1E    9                   // (not used) 
#define CODE_L1A    10                  // obs code: E1A        (GAL) 
#define CODE_L1B    11                  // obs code: E1B        (GAL) 
#define CODE_L1X    12                  // obs code: E1B+C,L1C(D+P) (GAL,QZS) 
#define CODE_L1Z    13                  // obs code: E1A+B+C,L1SAIF (GAL,QZS) 
#define CODE_L2C    14                  // obs code: L2C/A,G1C/A (GPS,GLO) 
#define CODE_L2D    15                  // obs code: L2 L1C/A-(P2-P1) (GPS) 
#define CODE_L2S    16                  // obs code: L2C(M)     (GPS,QZS) 
#define CODE_L2L    17                  // obs code: L2C(L)     (GPS,QZS) 
#define CODE_L2X    18                  // obs code: L2C(M+L),B1I+Q (GPS,QZS,CMP) 
#define CODE_L2P    19                  // obs code: L2P,G2P    (GPS,GLO) 
#define CODE_L2W    20                  // obs code: L2 Z-track (GPS) 
#define CODE_L2Y    21                  // obs code: L2Y        (GPS) 
#define CODE_L2M    22                  // obs code: L2M        (GPS) 
#define CODE_L2N    23                  // obs code: L2codeless (GPS) 
#define CODE_L5I    24                  // obs code: L5/E5aI    (GPS,GAL,QZS,SBS) 
#define CODE_L5Q    25                  // obs code: L5/E5aQ    (GPS,GAL,QZS,SBS) 
#define CODE_L5X    26                  // obs code: L5/E5aI+Q/L5B+C (GPS,GAL,QZS,IRN,SBS) 
#define CODE_L7I    27                  // obs code: E5bI,B2I   (GAL,CMP) 
#define CODE_L7Q    28                  // obs code: E5bQ,B2Q   (GAL,CMP) 
#define CODE_L7X    29                  // obs code: E5bI+Q,B2I+Q (GAL,CMP) 
#define CODE_L6A    30                  // obs code: E6A        (GAL) 
#define CODE_L6B    31                  // obs code: E6B        (GAL) 
#define CODE_L6C    32                  // obs code: E6C        (GAL) 
#define CODE_L6X    33                  // obs code: E6B+C,LEXS+L,B3I+Q (GAL,QZS,CMP) 
#define CODE_L6Z    34                  // obs code: E6A+B+C    (GAL) 
#define CODE_L6S    35                  // obs code: LEXS       (QZS) 
#define CODE_L6L    36                  // obs code: LEXL       (QZS) 
#define CODE_L8I    37                  // obs code: E5(a+b)I   (GAL) 
#define CODE_L8Q    38                  // obs code: E5(a+b)Q   (GAL) 
#define CODE_L8X    39                  // obs code: E5(a+b)I+Q (GAL) 
#define CODE_L2I    40                  // obs code: B1I        (BDS) 
#define CODE_L2Q    41                  // obs code: B1Q        (BDS) 
#define CODE_L6I    42                  // obs code: B3I        (BDS) 
#define CODE_L6Q    43                  // obs code: B3Q        (BDS) 
#define CODE_L3I    44                  // obs code: G3I        (GLO) 
#define CODE_L3Q    45                  // obs code: G3Q        (GLO) 
#define CODE_L3X    46                  // obs code: G3I+Q      (GLO) 
#define CODE_L1I    47                  // obs code: B1I        (BDS) 
#define CODE_L1Q    48                  // obs code: B1Q        (BDS) 
#define CODE_L5A    49                  // obs code: L5A SPS    (IRN) 
#define CODE_L5B    50                  // obs code: L5B RS(D)  (IRN) 
#define CODE_L5C    51                  // obs code: L5C RS(P)  (IRN) 
#define CODE_L9A    52                  // obs code: SA SPS     (IRN) 
#define CODE_L9B    53                  // obs code: SB RS(D)   (IRN) 
#define CODE_L9C    54                  // obs code: SC RS(P)   (IRN) 
#define CODE_L9X    55                  // obs code: SB+C       (IRN) 
#define MAXCODE     55                  // max number of obs code 

/* positioning mode -----------------------------------------------------*/
#define PMODE_SINGLE           0        // positioning mode: single 
#define PMODE_DGPS             1        // positioning mode: DGPS/DGNSS 
#define PMODE_KINEMA           2        // positioning mode: kinematic 
#define PMODE_STATIC           3        // positioning mode: static 
#define PMODE_MOVEB            4        // positioning mode: moving-base 
#define PMODE_FIXED            5        // positioning mode: fixed 
#define PMODE_PPP_KINEMA       6        // positioning mode: PPP-kinemaric 
#define PMODE_PPP_STATIC       7        // positioning mode: PPP-static 
#define PMODE_PPP_FIXED        8        // positioning mode: PPP-fixed 

/* ephemeris option -----------------------------------------------------*/
#define EPHOPT_BRDC            0        // ephemeris option: broadcast ephemeris 
#define EPHOPT_PREC            1        // ephemeris option: precise ephemeris 
#define EPHOPT_SBAS            2        // ephemeris option: broadcast + SBAS 
#define EPHOPT_SSRAPC          3        // ephemeris option: broadcast + SSR_APC 
#define EPHOPT_SSRCOM          4        // ephemeris option: broadcast + SSR_COM 
#define EPHOPT_LEX             5        // ephemeris option: QZSS LEX ephemeris 

/* solution status -------------------------------------------------------*/
#define SOLQ_NONE              0        // solution status: no solution
#define SOLQ_FIX               1        // solution status: fix 
#define SOLQ_FLOAT             2        // solution status: float 
#define SOLQ_SBAS              3        // solution status: SBAS 
#define SOLQ_DGPS              4        // solution status: DGPS/DGNSS 
#define SOLQ_SINGLE            5        // solution status: single 
#define SOLQ_PPP               6        // solution status: PPP 
#define SOLQ_DR                7        // solution status: dead reconing 

#define MAXSOLQ                7        // max number of solution status 

/* ionosphere option -----------------------------------------------------*/
#define IONOOPT_OFF            0        // ionosphere option: correction off 
#define IONOOPT_BRDC           1        // ionosphere option: broadcast model 
#define IONOOPT_IF12           2        // ionosphere option: L1/L2 or L1/L5 iono-free LC 
#define IONOOPT_UC1            3        // ionosphere option: estimation 
#define IONOOPT_UC12           4        // ionosphere option: estimation 
#define IONOOPT_TEC            5        // ionosphere option: IONEX TEC model 
#define IONOOPT_QZS            6        // ionosphere option: QZSS broadcast model 
#define IONOOPT_LEX            7        // ionosphere option: QZSS LEX ionospehre 
#define IONOOPT_STEC           8        // ionosphere option: SLANT TEC model 

/* troposphere option ----------------------------------------------------*/
#define TROPOPT_OFF            0        // troposphere option: correction off 
#define TROPOPT_SAAS           1        // troposphere option: Saastamoinen model 
#define TROPOPT_SBAS           2        // troposphere option: SBAS model 
#define TROPOPT_EST            3        // troposphere option: ZTD estimation 
#define TROPOPT_ESTG           4        // troposphere option: ZTD+grad estimation 
#define TROPOPT_ZTD            5        // troposphere option: ZTD correction 

/* error factor ----------------------------------------------------------*/
#define EFACT_GPS              1.0      // error factor: GPS 
#define EFACT_GLO              1.5      // error factor: GLONASS
#define EFACT_GAL              1.0      // error factor: Galileo 
#define EFACT_BDS              1.0      // error factor: BeiDou 

/* weighting option ------------------------------------------------------*/
#define WEIGHTOPT_ELEVATION    0        // weighting option: elevation 
#define WEIGHTOPT_SNR          1        // weighting option: snr 

const double lam_carr[MAXFREQ] = {      // carrier wave length (m) list
    CLIGHT / FREQ1, CLIGHT / FREQ2, CLIGHT / FREQ5,
    CLIGHT / FREQ6, CLIGHT / FREQ7, CLIGHT / FREQ8,
    CLIGHT / FREQ9
};

/* class & struct -----------------------------------------------------------*/

/* GPS time struct ----------------------------------------------------------*/
struct GpsTime_t
{
	time_t time;		// time (s) expressed by standard time_t
	double sec;		    // fraction of second under 1 s

    bool operator < (const GpsTime_t &t) const{
        double t1 = (double)this->time + this->sec;
        double t2 = (double)t.time + t.sec;
        return (t1 < t2);
    }
};

/* sattion information -----------------------------------------------------*/
struct Station_t
{
    string name2;
    char name[MAXANT];                  // marker name 
    char marker[MAXANT];                // marker number 
    char antdes[MAXANT];                // antenna descriptor 
    char antsno[MAXANT];                // antenna serial number 
    char rectype[MAXANT];               // receiver type descriptor 
    char recver[MAXANT];                // receiver firmware version 
    char recsno[MAXANT];                // receiver serial number 
    int antsetup;                       // antenna setup id 
    int itrf;                           // ITRF realization year 
    int deltype;                        // antenna delta type (0:enu,1:xyz) 
    double pos[3];                      // station position (ecef) (m) 
    double del[3];                      // antenna position delta (e/n/u or x/y/z) (m) 
    double hgt;                         // antenna height (m) 
};

/* Ephemeris data struct ------------------------------------------------------------*/
/* GPS/GAL/BDS nav ephemeris at an epoch --------------------------------------------*/
struct NavData_t 
{
    /* time parameters */
    GpsTime_t toe, toc, ttr;            // time of ephemeris\clock\transmit
    double f0, f1, f2;                  // SV clock parameters (af0,af1,af2)
    int week;                           // GPS/QZS: gps week, GAL: galileo week
    double toes;                        // Toe (s) in week
    /* SV orbit parameters */
    double A;                           // long semi-axis (m)
    double e;                           // eccentricity
    double i0;                          // orbit inclination (rad)
    double OMG0;                        // ascending node(rad)
    double omg;                         // perigee angular distance (rad)
    double M0;                          // mean anomaly (rad)
    double deln, OMGd, idot;            // modification parameters for M0\OMG0\i0
    double crc, crs, cuc, cus, cic, cis;// cos/sin modification parameters for A\OMG0\i0
    
    int iode, iodc;                     // index of data epoch/clock
    int sva;                            // SV accuracy (URA index)
    int svh;                            // SV health (0:ok)
    int code;                           // GPS/QZS: code on L2, GAL/BDS: data sources
    int flag;                           // GPS/QZS: L2 P data flag, BDS: nav type
    double fit;                         // fit interval (h)
    double tgd[4];                      // group delay parameters
                                        // GPS/QZS:tgd[0]=TGD
                                        // GAL    :tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1
                                        // BDS    :tgd[0]=BGD1,tgd[1]=BGD2
    //double Adot, ndot;                  // Adot,ndot for CNAV
};
/* GLO nav ephemeris at an epoch ----------------------------------------------------*/
struct NavDataGlo_t 
{
    GpsTime_t toe;                      // epoch of epherides (gpst)
    GpsTime_t tof;                      // message frame time (gpst)
    int iode;                           // IODE (0-6 bit of tb field)
    int frq;                            // satellite frequency number
    int svh, sva, age;                  // satellite health, accuracy, age of operation
    
    double pos[3];                      // satellite position (ecef) (m)
    double vel[3];                      // satellite velocity (ecef) (m/s)
    double acc[3];                      // satellite acceleration (ecef) (m/s^2)
    double taun, gamn;                  // SV clock bias (s)/relative freq bias
    double dtaun;                       // delay between L1 and L2 (s)
};
/* GPS/GAL/BDS nav ephemeris of a satellite -----------------------------------------*/
struct NavSatData_t 
{
    int sat;
    map<GpsTime_t, NavData_t> naveph;
};
/* GLO nav ephemeris of a satellite -------------------------------------------------*/
struct NavSatDataGlo_t 
{
    int sat;
    map<GpsTime_t, NavDataGlo_t> naveph;
};
/* precise ephemeris of an epoch ----------------------------------------------------*/
struct PrecNav_t
{
    int index;          // ephemeris index for multiple files
    double pos[4];      // satellite position/clock (ecef) (m|s)
    float  std[4];      // satellite position/clock std (m|s)
    double vel[4];      // satellite velocity/clk-rate (m/s|s/s)
    float  vst[4];      // satellite velocity/clk-rate std (m/s|s/s)
    //float  cov[3];      // satellite position covariance (m^2)
    //float  vco[3];      // satellite velocity covariance (m^2)
};
/* precise clock of an epoch --------------------------------------------------------*/
struct PrecClk_t
{
    int index;          // clock index for multiple files
    double clk;         // satellite clock (s)
    float  std;         // satellite clock std (s)
};

/* earth rotation parameters --------------------------------------------------------*/
struct Erp_t
{
    double mjd;         // mjd (days)
    double xp, yp;      // pole offset (rad)
    double xpr, ypr;    // pole offset rate (rad/day)
    double ut1_utc;     // ut1-utc (s)
    double lod;         // length of day (s/day)
};

/* antenna parameter type ------------------------------------------------------------*/
struct PCV_t
{
    int sat;                                // satellite number (0:receiver)
    GpsTime_t ts, te;                       // valid time start and end
    char type[MAXANT];                      // antenna type
    char code[MAXANT];                      // serial number or satellite code
    double off[NSYS_USED * NFREQ][3];       // phase center offset e/n/u or x/y/z (m)
    double var[NSYS_USED * NFREQ][80 * 50]; // phase center variation (m)
                                            // el=90,85,...,0 or nadir=0,1,2,3,... (deg)
    double dazi;                            // Increment of the azimuth：0 to 360 with increment 'DAZI'(in degrees).
    double zen1, zen2, dzen;                // Receiver  antenna: Definition of the grid in zenith angle.
                                            // Satellite antenna: Definition of the grid in nadir angle.
};

/* nav product data package type -----------------------------------------------------*/
struct NavPack_t 
{
    int n = 0;          // number of broadcast ephemeris
    int ng;             // number of glonass ephemeris
    int ne;             // number of precise ephemeris
    int nc;             // number of precise clock
    int leaps;          // leap seconds (s)

    map<int, map<GpsTime_t, NavData_t>>     eph;    // GPS/GAL/BDS ephemeris
    map<int, map<GpsTime_t, NavDataGlo_t>> geph;    // GLO         ephemeris
    map<int, map<GpsTime_t, PrecNav_t>>    peph;    // precise ephemeris
    map<int, map<GpsTime_t, PrecClk_t>>    pclk;    // precise clock

    map<int, map<string, double>>           dcb;    // DCB (0:p1-p2,1:p1-c1,2:p2-c2,3:p1-p3,4:p2-p3) (m)
    vector<Erp_t>                           erp;    // earth rotation parameters
    vector<vector<double>>                  otl;    // ocean tide parameters
    map<int, PCV_t>                     sat_pcv;    // satellite PCO/PCV 

    map<int, vector<double>>                lam;    // wave-length of sat
    
    double utc_gps[4];      // GPS delta-UTC parameters {A0,A1,T,W}
    double utc_glo[4];      // GLONASS UTC GPS time parameters
    double utc_gal[4];      // Galileo UTC GPS time parameters
    double utc_bds[4];      // BeiDou  UTC parameters

    double ion_gps[8];      // GPS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}
    double ion_gal[4];      // Galileo iono model parameters {ai0,ai1,ai2,0}
    double ion_bds[8];      // BeiDou iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}

    double glo_cpbias[4];              // glonass code-phase bias {1C,1P,2C,2P} (m)
    char   glo_fcn[MAXPRNGLO + 1];     // glonass frequency channel number + 8

    //int nt;             // number of tec grid data
    //tec_t* tec;         /* tec grid data */
    //double wlbias[MAXSAT];             // wide-lane bias (cycle)
};

/* obs data struct --------------------------------------------------------------------*/
/* obs data of a sat at an epoch ------------------------------------------------------*/
struct ObsData_t 
{
    unsigned char LLI[NFREQ];			// loss of lock indicator
    unsigned char code[NFREQ];			// code indicator   (CODE_???)
    char* type[NFREQ];					// type of obs data (CLC,C1P...)
    double P[NFREQ];					// observation data pseudorange   (m)
    double L[NFREQ];					// observation data carrier-phase (cycle)
};
/* obs data of an epoch --------------------------------------------------------------*/
struct ObsEphData_t 
{
    GpsTime_t eph;                      // gps time of current epoch
    int nsat;                           // number of satellite at current epoch
    map<int, ObsData_t> obssat;         // key: sat  value: ObsData_t
};
/* obs data of a receiver ------------------------------------------------------------*/
struct ObsRecData_t 
{
    int rec;                            // receiver #
    int neph;                           // number of epoches
    vector<ObsEphData_t> obseph;        // ObsEphData_t vector
};

/* signal index type -----------------------------------------------------------------*/
struct Sigind_t 
{
    int n;                              // number of index
    int frq[MAXOBSTYPE];                // signal frequency (1:L1,2:L2,...)
    int pos[MAXOBSTYPE];                // signal index in ObsData_t (-1:no)
    unsigned char pri[MAXOBSTYPE];		// signal priority (15-0)
    unsigned char type[MAXOBSTYPE];     // type (0:C,1:L,2:D,3:S)
    unsigned char code[MAXOBSTYPE];     // obs code (CODE_L??)
    double shift[MAXOBSTYPE];           // phase shift (cycle)
};

/* file options type -----------------------------------------------------------------*/
struct FileOpt_t
{
    vector<const char*> infile;     // RINEX file
    vector<const char*> outfile;    // output files
    vector<const char*> atxfile;    // ATX file
    vector<const char*> dcbfile;    // DCB files
    vector<const char*> mdcbfile;   // MGEX DCB file
    vector<const char*> precfile;   // SP3 and CLK files
    vector<const char*> erpfile;    // ERP file
    vector<const char*> blqfile;    // BLQ file

    vector<const char*> dbgfile;    // Debug file

    /* 未用到 */
    char stapos[MAXSTRPATH]; /* station positions file */
    char geoid[MAXSTRPATH]; /* external geoid data file */
    char ionf[MAXSTRPATH]; /* ionosphere data file */
    char snxf[MAXSTRPATH]; /* sinex file */
    //char tempdir[MAXSTRPATH]; /* ftp/http temporaly directory */
    //char geexe[MAXSTRPATH]; /* google earth exec file */
    //char solstat[MAXSTRPATH]; /* solution statistics file */
    //char trace[MAXSTRPATH]; /* debug trace file */
};

/* processing options type -----------------------------------------------------------*/
struct ProcOpt_t
{
    GpsTime_t ts;           // start time
    GpsTime_t te;           // end time
    double ti = 30.0;       // time interval
    int ieph;               // index of epoch

    int pos_mode;           // mode: positioning mode (PMODE_???)
    int sol_mode;           // mode: solution mode (0:forward,1:backward,2:combined)
    
    int eph_opt;            // option: ephemeris option (EPHOPT_???)
    int iono_opt;           // option: ionosphere option (IONOOPT_???)
    int trop_opt;           // option: troposphere option (TROPOPT_???)
    int tide_opt;           // option: correct earth tide option (0:off 1:solid 2:solid+otl+pole)
    int wgt_opt = 0;        // option: weighting option (0:elevation 1:SNR)

    int nf;                 // number of frequencies (1:L1,2:L1+L2,3:L1+L2+L5) */
    int nav_sys;            // navigation system 
    
    unsigned char exsats[MAXSAT]; /* excluded satellites (1:excluded,2:included) */

    double thres_GDOP;      // reject threshold of GDOP
    double thres_PDOP;      // reject threshold of PDOP
    
    double thresGF;         // slip threshold of geometry-free phase (m)
    double thresMW;         // slip threshold of MW phase (m)
    int     flagGF;         // 使用GF标记符
    int     flagMW;         // 使用MW标记符

    double err_ratio[NFREQ];   // code/phase error ratio
    double err[5];             // measurement error factor
                               // [0]:reserved 
                               // [1-3]:error factor a/b/c of phase (m) 
                               // [4]:doppler frequency (hz) 

    int rovpos;         /* rover position for fixed mode */
    int refpos;         /* base position for relative mode (0:pos in prcopt 1:average of single pos 2:read from file 3:rinex header, 4:rtcm pos */
    double ru[3];       /* rover position for fixed mode {x,y,z} (ecef) (m) */
    double rb[3];       /* base position for relative mode {x,y,z} (ecef) (m) */
    
    double elmin;       /* elevation mask angle (rad) */

    //int gnsisb;         /* stochastic modeling for ISBs in multi-GNSS processing */
    //int gloicb;         /* considering GLONASS code inter-channel biases */
    //snrmask_t snrmask;  /* SNR mask */
    //int sateph;         /* satellite ephemeris/clock (EPHOPT_???) */
    //int modear;         /* AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold,4:ppp-ar) */
    //int glomodear;      /* GLONASS AR mode (0:off,1:on,2:auto cal,3:ext cal) */
    //int bdsmodear;      /* BeiDou AR mode (0:off,1:on) */
    //int maxout;         /* obs outage count to reset bias */
    //int minlock;        /* min lock count to fix ambiguity */
    //int minfix;         /* min fix count to hold ambiguity */
    //int armaxiter;      /* max iteration to resolve ambiguity */
    //int dynamics;       /* dynamics model (0:none,1:velociy,2:accel) */
    //int niter;          /* number of filter iteration */
    //int codesmooth;     /* code smoothing window size (0:none) */
    //int intpref;        /* interpolate reference obs (for post mission) */
    //double std[3];      /* initial-state std [0]bias,[1]iono [2]trop */
    //double prn[6];      /* process-noise std [0]bias,[1]iono [2]trop [3]acch [4]accv [5] pos */
    //double sclkstab;    /* satellite clock stability (sec/sec) */
    //double thresar[8];  /* AR validation threshold */
    //double elmaskar;    /* elevation mask of AR for rising satellite (deg) */
    //double elmaskhold;  /* elevation mask to hold ambiguity (deg) */
    //double thresslip;   /* slip threshold of geometry-free phase (m) */
    //double maxtdiff;    /* max difference of time (sec) */
    //double maxinno;     /* reject threshold of innovation (m) */
    //double baseline[2]; /* baseline length constraint {const,sigma} (m) */
    //char anttype[MAXANT]; /* antenna types {rover,base} */
    //double antdel[3];   /* antenna delta {{rov_e,rov_n,rov_u},{ref_e,ref_n,ref_u}} */
    //PCV_t pcvr;         /* receiver antenna parameters {rov,base} */
    //int  maxaveep;      /* max averaging epoches */
    //int  initrst;       /* initialize by restart */
    //int  outsingle;     /* output single by dgps/float/fix/ppp outage */
    //char rnxopt[256];   /* rinex options {rover,base} */
    //int  posopt[6];     /* positioning options */
    //int  syncsol;       /* solution sync mode (0:off,1:on) */
    //exterr_t exterr;    /* extended receiver error model */
    //int freqopt;        /* disable L2-AR */
    //char pppopt[256];   /* ppp option */
};

/* solution options type ----------------------------------------------------------*/
struct Solopt_t
{
    int pos_fmt;          // solution format (SOLF_???) 
    int time_sys;         // time system (TIMES_???) 
    int time_fmt;         // time format (0:sssss.s,1:yyyy/mm/dd hh:mm:ss.s) 
    int time_dig;         // time digits under decimal point */

    //int deg_fmt;          // latitude/longitude format (0:ddd.ddd,1:ddd mm ss) 
    //int outhead;          // output header (0:no,1:yes) 
    //int out_opt;          /* output processing options (0:no,1:yes) */
    //int datum;            /* datum (0:WGS84,1:Tokyo) */
    //int height;           /* height (0:ellipsoidal,1:geodetic) */
    //int geoid;            /* geoid model (0:EGM96,1:JGD2000) */
    //int solstatic;        /* solution of static mode (0:all,1:single) */
    //int sstat;            /* solution statistics level (0:off,1:states,2:residuals) */
    //int trace;            /* debug trace level (0:off,1-5:debug) */
    //double nmeaintv[2];   /* nmea output interval (s) (<0:no,0:all) */
    //                      /* nmeaintv[0]:gprmc,gpgga,nmeaintv[1]:gpgsv */
    //char sep[64];         /* field separator */
    //char prog[64];        /* program name */
    //double maxsolstd;     /* max std-dev for solution output (m) (0:all) */

    //int fpout[MAXOUTFILE];  //for solution files output
};

/* solution type ------------------------------------------------------------------*/
struct Sol_t
{       
    GpsTime_t time;             // time (GPST)
    double tt;                  // time difference
    double rr_snx[3];           // position from SNX file
    double rr[6];               // position/velocity (m|m/s) {x,y,z,vx,vy,vz} or {e,n,u,ve,vn,vu}
    double dtr[6];              // receiver clock bias to time systems (s)
    float  qr[6];               // position variance/covariance (m^2)
                                // {c_xx,c_yy,c_zz,c_xy,c_yz,c_zx}
                                // {c_ee,c_nn,c_uu,c_en,c_nu,c_ue}
    float  qv[6];               // velocity variance/covariance (m^2)
                                // {c_xx,c_yy,c_zz,c_xy,c_yz,c_zx}
                                // {c_ee,c_nn,c_uu,c_en,c_nu,c_ue}
    double dop[4];              // DOPs {GDOP,PDOP,HDOP,VDOP}
    unsigned char type;         // type (0:xyz-ecef,1:enu-baseline)
    unsigned char stat;         // solution status (SOLQ_???)
    unsigned char ns;           // number of valid satellites
    
    float age;                  // age of differential (s)
    float ratio;                // AR ratio factor for valiation
    float thres;                // AR ratio threshold for valiation
    double rms;
    double dcb_rcv;             // receiver DCB

    /* QHY add debug */
    int n_ite = 0;              // bad epoch: epoch not iterated
    int n_les = 0;              // bad epoch: epoch satellite number not enough
    FILE* fp_sat;               // Debug file pointer: satellite select position/clock file
    FILE* fp_itr;               // Debug file pointer: resduial value file
};

/* satellite status type ---------------------------------------------------------*/
struct Sat_t
{
    unsigned char sys;          // navigation system 
    unsigned char vs;           // valid satellite flag single 
    double azel[2];             // azimuth/elevation angles {az,el} (rad)
    double resp[NFREQ];         // residuals of pseudorange (m) 
    double resc[NFREQ];         // residuals of carrier-phase (m) 
    double resp_pri[NFREQ];     // residuals of pseudorange (m) 
    double resc_pri[NFREQ];     // residuals of carrier-phase (m) 
    double resp_pos[NFREQ];     // residuals of pseudorange (m) */
    double resc_pos[NFREQ];     // residuals of carrier-phase (m) 
    double PC, LC;              // ionosphere-free pseudorange and carrier phase observations 
    unsigned char vsat[NFREQ];  // valid satellite flag 

    double pcv[3];              // sat pcv (m)
    double phw;                 // phase windup (cycle) 

    //unsigned char snr[NFREQ];   // signal strength (0.25 dBHz) 
    //unsigned char fix[NFREQ];   // ambiguity fix flag (1:fix,2:float,3:hold) 
    //unsigned char slip[NFREQ];  // cycle-slip flag 
    //unsigned char half[NFREQ];  // half-cycle valid flag 
    //int lock[NFREQ];            // lock counter of phase 
    //unsigned int outc[NFREQ];   // obs outage counter of phase 
    //unsigned int slipc[NFREQ];  // cycle-slip counter 
    //unsigned int rejc[NFREQ];   // reject counter 
    //double  gf;                 // geometry-free phase L1-L2 (m) 
    //double  gf2;                // geometry-free phase L1-L5 (m) 
    //double  mw;                 // MW-LC (m) 
    //GpsTime_t pt[2][NFREQ];     // previous carrier-phase time 
    //double  ph[2][NFREQ];       // previous carrier-phase observable (cycle) 
};

/* PPP global status ------------------------------------------------------------*/
struct PPP_Glob_t
{
    double ep[6] = { 0 };
    double clk_jump = 0.0;
    map<int, vector<double>> obs_past;

    map<int, double> ecli_f;
};
static PPP_Glob_t pppglob;

/* functions --------------------------------------------------------------------*/
/* comm_fun.cpp -----------------------------------------------------------------*/

/* sting functions -----------------------------------------*/

/**
* @brief set string without tail space
*
* @param <char> [*dst] output string
* @param <char> [*src] input string
* @param <int> [n] string width
*
* @return void
*/
extern void setstr(char* dst, const char* src, int n);
/**
* @brief convert substring in string to number
*
* @param <char> [*s] string ("... nnn.nnn ...")
* @param <int> [i,n] substring position and width
*
* @return <double> [value] converted number
*/
extern double str2num(const char* s, int i, int n);

/* time functions ------------------------------------------*/

/**
* @brief convert substring in string to gtime_t struct
*
* @param <char> [*s] string ("... yyyy mm dd hh mm ss ...")
* @param <int> [i,n] substring position and width
* @param <GpsTime_t> [*t] gps time
*
* @return status (0:ok,-1:error)
*/
extern int str2time(const char* s, int i, int n, GpsTime_t* t);
/**
* @brief convert gtime_t struct to string
*
* @param <GpsTime_t> [t] gps time
* @param <char> [*s] string ("yyyy/mm/dd hh:mm:ss.ssss")
* @param <int> [n] number of decimals
*
* @return void
*/
extern void time2str(GpsTime_t t, char* s, int n);
/**
* @brief get time string
*
* @param <GpsTime_t> [t] gps time
* @param <int> [n] number of decimals
*
* @return <char> [*buff] string ("yyyy/mm/dd hh:mm:ss.ssss")
*/
extern char* time_str(GpsTime_t t, int n);
/**
* @brief convert calendar day/time to time
*
* @param <double> [*ep] day/time {year,month,day,hour,min,sec}
*
* @return <GpsTime_t> [time] converted gps time struct
*/
extern GpsTime_t epoch2time(const double* ep);
/**
* @brief convert gps time to calendar day/time
*
* @param <GpsTime_t> [t] gps time struct
* @param <double> [*ep] day/time {year,month,day,hour,min,sec}
*
* @return void
*/
extern void time2epoch(const GpsTime_t t,double* ep);
/**
* @brief convert utc time to gps time
*
* @param <GpsTime_t> [t] utc time
*
* @return <GpsTime_t> [t] time expressed in gps
*/
extern GpsTime_t utc2gpst(GpsTime_t t);
/*utc to gmst---------------------------------------------------------------- -
*convert utc to gmst(Greenwich mean sidereal time)
* args   : gtime_t t        I   time expressed in utc
* double ut1_utc   I   UT1 - UTC(s)
* return : gmst(rad)
* ---------------------------------------------------------------------------- - */
extern double utc2gmst(GpsTime_t t, double ut1_utc);
/**
* @brief convert gps time to utc time
*
* @param <GpsTime_t> [t] gps time
*
* @return <GpsTime_t> [t] time expressed in utc
*/
extern GpsTime_t gpst2utc(GpsTime_t t);
/**
* @brief convert gtime_t struct to week and tow in gps time
*
* @param <GpsTime_t> [t] gps time
* @param <int> [*week] week number in gps time (NULL: no output)
*
* @return <double> time of week in gps time (s)
*/
extern double time2gpst(GpsTime_t t, int* week);
/**
* @brief convert week and tow to gps time struct
*
* @param <int> [week] week number in gps time
* @param <double> [sec] time of week in gps time (s)
*
* @return <GpsTime_t> time in gtime_t struct
*/
extern GpsTime_t gpst2time(const int week, double sec);
/**
* @brief convert beidou time (beidou navigation satellite system time) to gps time
*
* @param <GpsTime_t> [t] time expressed in bdt
*
* @return <GpsTime_t> [t] time expressed in gpstime
*/
extern GpsTime_t bdt2gpst(GpsTime_t t);
/**
* @brief convert beidou week and tow to gps time struct
*
* @param <int> [week] week in beidou time
* @param <double> [sec] time of week in beidou time(s)
*
* @return <GpsTime_t> time expressed in gpstime
*/
extern GpsTime_t bdt2time(const int week, double sec);
/**
* @brief convert year, day of year to gps time
*
* @param <int> [yyyy] year
* @param <int> [doy] day of year 
* 
* @return <GpsTime_t> gps time of this doy
*/
extern GpsTime_t yrdoy2time(const int yyyy, const int doy);
/**
* @brief add time to gtime_t struct
*
* @param <GpsTime_t> [t] gps time
* @param <double> [sec] time to add (s)
*
* @return <GpsTime_t> [t] gps time (t+sec)
*/
extern GpsTime_t timeadd(GpsTime_t t, double sec);
/**
* @brief difference between GpsTime_t structs
*
* @param <GpsTime_t> [t1,t2] gps time
*
* @return <double> time difference (t1-t2) (s)
*/
extern double timediff(GpsTime_t t1, GpsTime_t t2);
/**
* @brief screening by time start, time end, and time interval
*
* @param <GpsTime_t> [time] present epoch
* @param <GpsTime_t> [ts] time start (ts.time==0:no screening by ts)
* @param <GpsTime_t> [te] time end   (te.time==0:no screening by te)
* @param <double> [tint] time interval (s) (0.0:no screen by tint)
*
* @return <int> 1:on condition, 0:not on condition
*/
extern int screent(GpsTime_t time, GpsTime_t ts, GpsTime_t te, double tint);

/* sat functions ---------------------------------------------*/

/**
* @brief satellite id to satellite number
*
* @param <char> [*id] satellite id (nn,Gnn,Rnn,Enn,Jnn,Cnn,Inn or Snn)
*
* @return <int> satellite number (0: error)
*/
extern int satid2no(const char* id);
/**
* @brief satellite number to satellite id
*
* @param <int> [sat] satellite number
*
* @return <char> [*id] satellite id (Gnn,Rnn,Enn,Jnn,Cnn,Inn or nnn)
*/
extern void satno2id(int sat, char* id);
/**
* @brief obs type string to obs code
*
* @param <char> [*obs] obs code string ("1C","1P","1Y",...)
* @param <int> [*freq] frequency (1:L1,2:L2,3:L5,4:L6,5:L7,6:L8,0:err,NULL:no output)
*
* @return <usigned char> [i] obs code (CODE_???)
*/
extern unsigned char obs2code(const char* obs, int* freq);
/**
* @brief convert obs code to obs code string
*
* @param <unsigned char> [code] obs code (CODE_???)
* @param <int> [*freq] frequency (1:L1/E1, 2:L2/B1, 3:L5/E5a/L3, 4:L6/LEX/B3,
                                  5:E5b/B2, 6:E5(a+b), 7:S NULL: no output)
*
* @return obs code string ("1C","1P","1P",...)
*/
extern const char* code2obs(unsigned char code, int* freq);
/**
* @brief satellite code to satellite system
*
* @param <char> [code] satellite system code
*
* @return <int> [sys] satellite system (SYS_GPS..)
*/
extern int code2sys(char code);
/**
* @brief satellite system+prn/slot number to satellite number
*
* @param <int> [sys] satellite system (SYS_GPS,SYS_GLO,...)
* @param <int> [prn] satellite prn/slot number
*
* @return <int> satellite number (0:error)
*/
extern int satno(int sys, int prn);
/**
* @brief convert satellite number to satellite system
*
* @param <int> [sat] satellite number
* @param <int> [*prn] satellite prn/slot number (NULL: no output)
*
* @return <int> [sys] satellite system (SYS_GPS,SYS_GLO,...)
*/
extern int satsys(int sat, int* prn);
/**
* @brief test excluded satellite
*
* @param <int> [sat] satellite number
* @param <map> [*var_sat] variance of ephemeris (m^2)
* @param <map> [*svh] sv health flag
* @param <ProcOpt_t> [*popt] process option
*
* @return 0:not excluded 1:excluded
*/
extern int sat_exclude(int sat, map<int, vector<double>>* var_sat, map<int, vector<int>>* svh, const ProcOpt_t* popt);
/**
* @brief get wave length of satellite in frequence #
*
* @param <int> [sat] satellite number
* @param <int> [frq] frequency number
* @param <NavPack_t> [*navall] nav data pack
*
* @return <double> wave length (m)
*/
extern double sat_wavelen(int sat, int frq, NavPack_t* navall);

/* ephemeris functions ---------------------------------------*/

/**
* @brief broadcast ephemeris to satellite clock bias
*
* @param <GpsTime_t> [time] time by satellite clock
* @param <NavData_t> [*eph] broadcast ephemeris
*
* @return <double> satellite clock bias (s) without relativeity correction
*/
extern double eph2clk(GpsTime_t time, const NavData_t* eph);
/**
* @brief glonass broadcast ephemeris to satellite clock bias
*
* @param <GpsTime_t> [time] time by satellite clock
* @param <NavDataGlo_t> [*geph] glonass broadcast ephemeris
*
* @return <double> satellite clock bias (s) without relativeity correction
*/
extern double geph2clk(GpsTime_t time, const NavDataGlo_t* geph);

/* coordinate functions ---------------------------------------*/

/**
* @brief transform ecef to geodetic postion
*
* @param <vector> [r] ecef position {x,y,z} (m)
*
* @return <vector> [LLH] geodetic position {lat,lon,h} (rad,m)
*/
extern vector<double> ecef2pos(const vector<double> r);
/**
* @brief compute ecef to local coordinate transfromation matrix
*
* @param <vector> [pos] geodetic position {lat,lon} (rad)
*
* @return <vector> [R] Rotation matirix (3*3)
*/
extern vector<vector<double>> rot_matrix(const vector<double> pos);
/**
* @brief transform ecef vector to local tangental coordinate
*
* @param <vector> [pos] geodetic position {lat,lon} (rad)
* @param <vector> [r] vector in ecef coordinate {x,y,z}
*
* @return <vector> [e]  vector in local tangental coordinate {e,n,u}
*/
extern vector<double> ecef2enu(const vector<double> pos, const vector<double> r);

/**
* @brief compute geometric distance and receiver-to-satellite unit vector
*
* @param <int> [sat] satellilte number
* @param <map> [*rs] satellilte position (ecef at transmission) (m)
* @param <vector> [rr] receiver position (ecef at transmission) (m)
* @param <vector> [*e] line-of-sight vector (ecef)
*
* @return <double> [e]  geometric distance (m) (0>:error/no satellite position)
*/
extern double geodist(int sat, map<int, vector<double>>* rs, const vector<double> rr, vector<double>* e);
/**
* @brief compute satellite azimuth/elevation angle
*
* @param <int> [sat] satellilte number
* @param <vector> [pos] geodetic position {lat,lon,h} (rad,m)
* @param <vector> [e] receiver-to-satellilte unit vevtor (ecef)
* @param <map> [*azel] satellite azimuth/elevation angle
*
* @return <double> [el] satellite elevation angle
*/
extern double satazel(const int sat, const vector<double> pos, const vector<double> e, map<int, vector<double>>* azel);
/* sun and moon position -------------------------------------------------------
* get sun and moon position in ecef
* args   : gtime_t tut      I   time in ut1
*          double *erpv     I   erp value {xp,yp,ut1_utc,lod} (rad,rad,s,s/d)
*          double *rsun     IO  sun position in ecef  (m) (NULL: not output)
*          double *rmoon    IO  moon position in ecef (m) (NULL: not output)
*          double *gmst     O   gmst (rad)
* return : none
*-----------------------------------------------------------------------------*/
extern void sun_moon_pos(GpsTime_t tutc, const double* erpv, double* rsun, double* rmoon, double* gmst);


/* code functions ---------------------------------------------*/

/**
* @brief convert obs code to obs type
*
* @param <double> [ver] rinex version
* @param <int> [sys] satellite system
* @param <char> [*str] obs code ('P1','P2'...)
* @param <char> [*type] obs type ('P1C','P2P'...)
*
* @return void
*/
extern void convcode(double ver, int sys, const char* str, char* type);
/**
* @brief get code priority for multiple codes in a frequency
*
* @param <int> [sys] satellite system
* @param <unsigned char> [code] obs code (CODE_???)
* @param <char> [*opt] code options (NULL:no option)
*
* @return <int> priority (15:highest - 1:lowest 0:error)
*/
extern int getcodepri(const int sys, unsigned char code, const char* opt);
/**
* @brief set code priority for multiple codes in a frequency
*
* @param <int> [sys] satellite system
* @param <int> [freq] satellite frequency
* @param <char> [*pri] code priority (15 - 1)
*
* @return void
*/
extern void setcodepri(int sys, int freq, const char* pri);

/* math functions ---------------------------------------------*/

/**
* @brief upper/downer round of number
*
* @param <double> [dNum] float number
*
* @return rounfed number
*/
extern int myRound(const double dNum);
/**
* @brief inner product of vectors (double[] version)
*
* @param <double> [*a,*b] vector a,b (n*1)
* @param <int> [n] size of vector a,b
*
* @return <double> [c] a'*b
*/
extern double dot(const double* a, const double* b, int n);
/**
* @brief inner product of vectors (vector version)
*
* @param <vector> [a,b] vector a,b (n*1)
* @param <int> [n] size of vector a,b
*
* @return <double> [c] a'*b
*/
extern double dot(const vector<double> a, const vector<double> b, int n);
/* ??? */
extern double dot(const vector<double> a, const double* b, int n);
/**
* @brief euclid norm of vector (double[] version)
*
* @param <double> [*a] vector a (n*1)
* @param <int> [n] size of vector a
*
* @return <double> || a ||
*/
extern double norm(const double* a, int n);
/**
* @brief euclid norm of vector (vector version)
*
* @param <vector> [a] vector a (n*1)
* @param <int> [n] size of vector a
*
* @return <double> || a ||
*/
extern double norm(const vector<double> a, int n);
/* normalize 3d vector ---------------------------------------------------------
* normalize 3d vector
* args   : double *a        I   vector a (3 x 1)
*          double *b        O   normlized vector (3 x 1) || b || = 1
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int normv3(const double* a, double* b);
extern int normv3(vector<double> a, double* b);

/* multiply matrix */

/* copy matrix -----------------------------------------------------------------
* copy matrix
* args   : double *A        O   destination matrix A (n x m)
*          double *B        I   source matrix B (n x m)
*          int    n,m       I   number of rows and columns of matrix
* return : none
*-----------------------------------------------------------------------------*/
extern void matcpy(double* A, const double* B, int n, int m);

extern void matmul(const char* tr, int n, int k, int m, double alpha, 
    const double* A, const double* B, double beta, double* C);
/**
* @brief matrix multiplication
*
* @param <char> [*tr] operation Indicator ('NN','NT'...)
* @param <int> [pA] dimension of matrix A
* @param <int> [pAB] commom dimension of matrix A and matrix B
* @param <int> [pB] dimension of matrix B
* @param <double> [alpha,beta] multiply factor
* @param <vector> [A,B] matrix A and matrix B
* @param <vector> [*C] putput matrix C
*
* @return void
*/
extern void matmul(const char* tr, int pA, int pAB, int pB, double alpha,
    const vector<vector<double>>* A, const  vector<vector<double>>* B, double beta, vector<vector<double>>* C);
/**
* @brief LU decomposition
*
* @param <vector> [*A] origen matrix
* @param <int> [n] dimedion of matrix
* @param <vector> [*L,*U] lower triangle matrix/upper triangle matrix
*
* @return <int> -1:error 0:ok
*/
extern int LU_dcmp(vector<vector<double>> A, int n, vector<vector<double>>* L, vector<vector<double>>* U);
/**
* @brief invert L,U matrix to calculate inversion matrix
*
* @param <vector> [*L,*U] lower triangle matrix/upper triangle matrix
* @param <int> [n] dimedion of matrix
*
* @return <vector> [*R] inversion matrix
*/
extern void LU_back(vector<vector<double>>* L, vector<vector<double>>* U, int n, vector<vector<double>>* R);
/**
* @brief calculate inversion matrix
*
* @param <vector> [*A] origen matrix
* @param <int> [n] dimedion of matrix
*
* @return <vector> [*A] A^-1
*/
extern int mat_invert(vector<vector<double>>* A, int n);
/**
* @brief calculate least square
*
* @param <vector> [*A] coefficent matrix (ny*nx)
* @param <vector> [*y] residual vector (ny*1)
* @param <int> [nx,ny] dimedion of matrix
* @param <vector> [*Q] covariance matrix (nx*nx)
*
* @return <int> 0:error 1:ok
*/
extern int lsq(vector<vector<double>>* A, vector<double>* y, int nx, int ny, vector<double>* x, vector<vector<double>>* Q);

/* delay functions ---------------------------------------------*/

/**
* @brief compute ionospheric delay by Kolabhor model
*
* @param <GpsTime_t> [time] epoch time
* @param <int> [sat] satellite number
* @param <double> [*ion] ionosphere broadcast parameters
* @param <vector> [pos] geodetic position {lat,lon,h} (rad,m)
* @param <map> [*azel] satellite azimuth/elevation angle (rad)
*
* @return <double> ionospheric delay (m)
*/
extern double iono_model(GpsTime_t time, int sat, const double* ion, vector<double> pos, map<int, vector<double>>* azel);
/**
* @brief compute tropospheric delay by standard atmosphere and Saastamoinen model
*
* @param <GpsTime_t> [time] epoch time
* @param <int> [sat] satellite number
* @param <vector> [pos] geodetic position {lat,lon,h} (rad,m)
* @param <map> [*azel] satellite azimuth/elevation angle (rad)
* @param <double> [humi] elative humidity
* @param <double> [*zwd] zenith wet delay
*
* @return <double> zenith dry delay (m)
*/
extern double trop_model(GpsTime_t time, int sat, vector<double> pos, map<int, vector<double>>* azel, double humi, double* zwd);

/* other functions ---------------------------------------------*/

/**
* @brief initialize station info struct
*
* @param <Station_t> [*sta] station info
*
* @return void
*/
extern void init_sta(Station_t* sta);
/**
* @brief sort obs epoch data by time
*
* @param <ObsEphData_t> [eph1,eph2] obs epoch data
*
* @return 0:< 1:>=
*/
extern int compare_eph(ObsEphData_t eph1, ObsEphData_t eph2);
/**
* @brief sort earth rotation data by time
*
* @param <Erp_t> [erp1,erp2] earth rotation data
*
* @return 0:< 1:>=
*/
extern int compare_erp(Erp_t erp1, Erp_t erp2);
/**
* @brief compuate cycle silp by process option and obs sample rate
*
* @param <ProcOpt_t> [*popt] process option
* @param <double> [sample] obs sample rate (s)
*
* @return <int> [stat] 0:error 1:ok
*/
extern int cal_CS_thres(ProcOpt_t* popt, const double sample);
/**
* @brief compute DOP (dilution of precision)
*
* @param <int> [ns] number of satellites
* @param <map> [*azel] satellite azimuth/elevation angle (rad)
* @param <double> [elmin] elevation cutoff angle (rad)
*
* @return <vector> [dops] DOPs {GDOP,PDOP,HDOP,VDOP}
*/
extern vector<double> cal_dops(int ns, map<int, vector<double>>* azel, double elmin);
/**
* @brief open output file
*
* @param <char> [*outfile] output file path
*
* @return <FILE*> file pointer
*/
extern FILE* openfile(const char* outfile);


/* outer product of 3d vectors -------------------------------------------------
* outer product of 3d vectors
* args   : double *a,*b     I   vector a,b (3 x 1)
*          double *c        O   outer product (a x b) (3 x 1)
* return : none
*-----------------------------------------------------------------------------*/
extern void cross3(const double* a, const double* b, double* c);
/* ??? */
extern int cal_Eclips(int prn, map<int, vector<double>>* rs, double* sunp, double TTAG,
    double SANTXYZ[3], const NavPack_t* navall);

/* readfile.cpp ----------------------------------------------------------------*/

extern int read_obsnav(GpsTime_t ts, GpsTime_t te, double ti, vector<const char*> infile, vector<ObsRecData_t>* obsall, NavPack_t* navall, Station_t* sta);
/**
* @brief read antenna parameters (ATX)
*
* @param <vector<const char*>> [atxfile] ATX file path
* @param <vector<PCV_t>> [*pcvs] pcv vector
*
* @return 0: error 1: ok
*/
extern int read_pcv(vector<const char*> atxfile, vector<PCV_t>* pcvs);
/**
* @brief set PCO/PCV into nav packet by time and sat#
*
* @param <GpsTime_t> [time] epoch time
* @param <ProcOpt_t> [*popt] process option
* @param <NavPack_t> [*navall] nav packet
* @param <Station_t> [*sta] station information
*
* @return void
*/
extern void set_pcv(GpsTime_t time, ProcOpt_t* popt, NavPack_t* navall, vector<PCV_t>* pcvs, const Station_t* sta);
/**
* @brief read dcb parameters
*
* @param <vector<const char*>> [dcbfile] dcb file path
* @param <NavPack_t> [*navall] nav data packet
*
* @return 0: error 1: ok
*/
extern int read_dcb(vector<const char*> dcbfile, NavPack_t* navall);
/**
* @brief read MGEX dcb parameters
*
* @param <vector<const char*>> [mdcbfile] MGEX dcb file path
* @param <NavPack_t> [*navall] nav data packet
* @param <GpsTime_t> [time] reference time
*
* @return 0: error 1: ok
*/
extern int read_dcb_mgex(vector<const char*> mdcbfile, NavPack_t* navall, const GpsTime_t time);
/**
* @brief read precise ephemeris/clock parameters
*
* @param <vector<const char*>> [precfile] precise ephemeris/clock file path
* @param <NavPack_t> [*navall] nav data packet
*
* @return 0: error 1: ok
*/
extern int read_prec_eph(vector<const char*> precfile, NavPack_t* navall);
/**
* @brief read earth rotation parameters
*
* @param <vector<const char*>> [erpfile] ERP file path
* @param <NavPack_t> [*navall] nav data packet
*
* @return 0: error 1: ok
*/
extern int read_erp(vector<const char*> erpfile, NavPack_t* navall);
/**
* @brief read BLQ ocean tide loading parameters
*
* @param <vector<const char*>> [erpfile] BLQ file path
* @param <char> [*sta] station name
* @param <vector<vector<double>>> [*otl] ocean tide parameters
*
* @return 0: error 1: ok
*/
extern int read_blq(vector<const char*> blqfile, const char* sta, vector<vector<double>>* otl);

/* gnsspos.cpp ----------------------------------------------------------------*/

/**
* @brief API of position process
*
* @param <GpsTime_t> [ts,te] strat/end time
* @param <double>    [ti] time unit
* @param <ProcOpt_t> [*popt] process option
* @param <FileOpt_t> [*fopt] file option
* @param <SolOpt_t>  [*sopt] solution option
*
* @return 0: error 1: ok
*/
extern int posFDU(GpsTime_t ts, GpsTime_t te, double ti, ProcOpt_t* popt, FileOpt_t* fopt, Solopt_t* sopt);

/* spp.cpp --------------------------------------------------------------------*/

/**
* @brief single point position by L1 pseudorange
*
* @param <ObsEphData_t> [*obs] obs epoch data
* @param <int> [n] number of obs in current epoch
* @param <NavPack_t> [*navall] nav package
* @param <ProcOpt_t> [*popt] process option
* @param <Sol_t> [*sol] positioning solution
* @param <map> [*sat_stat] sat status
*
* @return 0: error 1: ok
*/
extern int spp(ObsEphData_t* obs, int n, NavPack_t* navall, const ProcOpt_t* popt, Sol_t* sol, map<int, Sat_t>* sat_stat);

/* spp.cpp --------------------------------------------------------------------*/

extern int ppp(ObsEphData_t* obs, int n, NavPack_t* navall, const ProcOpt_t* popt, Sol_t* sol, map<int, Sat_t>* sat_stat);

/* broad_eph.cpp --------------------------------------------------------------*/

extern void cal_satpos(GpsTime_t teph, ObsEphData_t* obs, int n, NavPack_t* navall, int ephopt,
    map<int, vector<double>>* rs, map<int, vector<double>>* dts, map<int, vector<double>>* var, map<int, vector<int>>* svh);

/* results.cpp ----------------------------------------------------------------*/

extern void out_sol(FILE* fp, const Sol_t* sol, const ProcOpt_t* popt, const Solopt_t* sopt);