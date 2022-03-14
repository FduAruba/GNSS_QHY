#pragma once
#include"gnss.h"
using namespace std;

#define MINFREQ_GLO -7                  /* min frequency number glonass */
#define MAXFREQ_GLO 13                  /* max frequency number glonass */

/* global variables ------------------------------------------------*/
static const char syscodes[] = "GREC";	  // satellite system codes
static const char obscodes[] = "CLDS";    // obs type codes
static const char frqcodes[] = "125678";  // frequency codes
static const double ura_eph[] = {         // ura values (ref [3] 20.3.3.3.1.1)
	2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
	3072.0,6144.0,0.0
};

/* static functions ------------------------------------------------*/

/**
* @brief adjust time considering week handover
*
* @param <GpsTime_t> [t,t0] gpstime
*
* @return <GpsTime_t> [t] gpstime adjusted by week
*/
static GpsTime_t adjweek(GpsTime_t t, GpsTime_t t0);
/**
* @brief adjust time considering day handover
*
* @param <GpsTime_t> [t,t0] gpstime
*
* @return <GpsTime_t> [t] gpstime adjusted by day
*/
static GpsTime_t adjday(GpsTime_t t, GpsTime_t t0);
/**
* @brief ura value (m) to ura index
*
* @param <double> [value] ura value
*
* @return <int> [i] position in ura index
*/
static int ura_index(double value);
/**
* @brief save cycle slips
*
* @param <char> [slips] cycle slip array
* @param <map<int, ObsData_t>::iterator> [it] map iterator
*
* @return void
*/
static void saveslips(unsigned char slips[][NFREQ], map<int, ObsData_t>::iterator it);
/**
* @brief restore cycle slips
*
* @param <char> [slips] cycle slip array
* @param <map<int, ObsData_t>::iterator> [it] map iterator
*
* @return void
*/
static void restslips(unsigned char slips[][NFREQ], map<int, ObsData_t>::iterator it);

/* read RINEX file ----------------------------------------------------------*/

/**
* @brief save obs type (.v2)
*
* @param <FILE> [*fp] file pointer
* @param <double> [ver] rinex version
* @param <char> [*buff] buffer
* @param <char> [tobs] obs type array
*
* @return void
*/
static void obsTypeV2(FILE* fp, double ver, char* buff, char tobs[][MAXOBSTYPE][4]);
/**
* @brief save obs type (.v3)
*
* @param <FILE> [*fp] file pointer
* @param <char> [*buff] buffer
* @param <char> [tobs] obs type array
*
* @return void
*/
static void obsTypeV3(FILE* fp, char* buff, char tobs[][MAXOBSTYPE][4]);
/**
* @brief set obs type index
*
* @param <double> [ver] rinex version
* @param <int> [sys] satellite system
* @param <char> [tobs] obs type array
* @param <Sigind_t> [ind] index struct
* @param <char> [opt] system option
*
* @return void
*/
static void set_index(double ver, int sys, char tobs[MAXOBSTYPE][4], Sigind_t* ind, const char* opt);
/**
* @brief add single ephemeris to NavSatData_t data(GPS/GAL/BDS)
*
* @param <NavSatData_t> [*sateph] sat ephemeris
* @param <int> [sat] sat #
* @param <NavData_t> [*eph] ephemeris data
*
* @return <int> 0: error 1: right
*/
static int add_eph(NavSatData_t* sateph, const int sat, const NavData_t* eph);
/**
* @brief add single ephemeris to NavSatData_t data(GLO)
*
* @param <NavSatDataGlo_t> [*sateph] sat ephemeris
* @param <int> [sat] sat #
* @param <NavData_t> [*geph] ephemeris data
*
* @return <int> 0: error 1: right
*/
static int add_geph(NavSatDataGlo_t* sateph, const int sat, const NavDataGlo_t* geph);
/**
* @brief add sat ephemeris to NavPack_t data(GPS/GLO/GAL/BDS)
*
* @param <NavSatData_t> [*sateph] sat ephemeris (GPS/GAL/BDS)
* @param <NavSatDataGlo_t> [*gsateph] sat ephemeris (GLO)
* @param <NavPack_t> [*navall] ephemeris set
*
* @return void
*/
static void add_sat_eph(NavSatData_t* sateph, NavSatDataGlo_t* gsateph, NavPack_t* navall);
/**
* @brief add obs epoch data to ObsRecData_t
*
* @param <ObsRecData_t> [*recdata] receiver obs data struct
* @param <ObsEphData_t> [*data] epoch obs data
*
* @return <int> 0: error 1: right
*/
static int add_obs_data(ObsRecData_t* recdata, const ObsEphData_t* data);
/**
* @brief sort obs data and remove duplicated data
*
* @param <ObsRecData_t> [*recdata] receiver obs data struct
*
* @return void
*/
static void sort_obs(ObsRecData_t* recdata);
/**
* @brief get station information from rnx head
*
* @param <Station_t> [*sta] station information struct
* @param <char> [*buff] buffer
* @param <char> [*label] label in buffer
*
* @return void
*/
static void sta_info(Station_t* sta, char* buff, char* label);
/**
* @brief decode GPS\GAL\BDS ephemeris data
*
* @param <double> [ver] rinex verison
* @param <int> [sat] sat#
* @param <double> [*data] nav data array
* @param <NavData_t> [*eph] GPS/GAL/BDS ephemeris data struct
* 
* @return <int> 0: error 1: ok
*/
static int decode_eph(double ver, int sat, GpsTime_t toc, const double* data, NavData_t* eph);
/**
* @brief decode GLO ephemeris data
*
* @param <double> [ver] rinex verison
* @param <int> [sat] sat#
* @param <GpsTime_t> [toc] time of clock
* @param <double> [*data] nav data array
* @param <NavData_t> [*eph] GLO ephemeris data struct
*
* @return <int> 0: error 1: ok
*/
static int decode_geph(double ver, int sat, GpsTime_t toc, double* data, NavDataGlo_t* geph);
/**
* @brief decode nav file head
*
* @param <char> [*buff] buffer pointer
* @param <NavPack_t> [*nav] nav data packet
*
* @return void
*/
static void decode_navh(char* buff, NavPack_t* nav);
/**
* @brief decode obs file head
*
* @param <FILE> [*fp] file pointer
* @param <char> [*buff] buffer
* @param <double> [*ver] rinex version
* @param <int> [*tsys] time system
* @param <char> [tobs] type of obs array
* @param <NavPack_t> [*navall] nav data packet
* @param <Station_t> [*sta] station information struct
*
* @return void
*/
static void decode_obsh(FILE* fp, char* buff, double ver, int* tsys, char tobs[][MAXOBSTYPE][4], NavPack_t* navall, Station_t* sta);
/**
* @brief decode obs epoch
*
* @param <FILE> [*fp] file pointer
* @param <char> [*buff] buffer
* @param <double> [*ver] rinex version
* @param <GpsTime_t> [*time] gps time
* @param <int> [*flag] event flag
* @param <int> [*sats] sat # array
*
* @return <int> [n] number of sats
*/
static int decode_obsepoch(FILE* fp, char* buff, double ver, GpsTime_t* time, int* flag, int* sats);
/**
* @brief decode obs data st an epoch
*
* @param <FILE> [*fp] file pointer
* @param <char> [*buff] buffer
* @param <double> [ver] rinex version
* @param <Sigind_t> [*index] singal index
* @param <ObsEphData_t> [*obs] epoch obs data struct
* @param <int> [*sats] sat # array
*
* @return <int> 0: error 1: ok
*/
static int decode_obsdata(FILE* fp, char* buff, double ver, int mask, Sigind_t* index, ObsEphData_t* obs, int* sats);
/**
* @brief read obs body
*
* @param <FILE> [*fp] file pointer
* @param <double> [ver] rinex version
* @param <char> [tobs] obs type array
* @param <ObsEphData_t> [*data] obs epoch data struct
*
* @return <int> [n] obs data number -1: error
*/
static int read_rnx_obsb(FILE* fp, double ver, char tobs[][MAXOBSTYPE][4], ObsEphData_t* data);
/**
* @brief read obs data
*
* @param <FILE> [*fp] file pointer
* @param <double> [ver] rinex version
* @param <int> [tsys] time system
* @param <char> [tobs] obs type array
* @param <vector<ObsRecData_t>> [*obsall] obs data vector
*
* @return <int> [stat] 0: error 1: ok
*/
static int read_rnx_obs(FILE* fp, double ver, int tsys, char tobs[][MAXOBSTYPE][4], vector<ObsRecData_t>* obsall);
/**
* @brief read nav data body
*
* @param <FILE> [*fp] file pointer
* @param <double> [ver] rinex version
* @param <int> [sys] rinex system
* @param <int> [*sat] sat #
* @param <int> [*type] gps/gal/bds nav: 0 glo: 1
* @param <NavData_t> [*eph] gps/gal/bds nav data struct
* @param <NavDataGlo_t> [*geph] glo nav data struct
*
* @return <int> 0: error 1: ok -1: read file fail
*/
static int read_rnx_navb(FILE* fp, double ver, int sys, int* sat, int* type, NavData_t* eph, NavDataGlo_t* geph);
/**
* @brief read nav data
*
* @param <FILE> [*fp] file pointer
* @param <double> [ver] rinex version
* @param <int> [sys] rinex system
* @param <NavPack_t> [*navall] nav data packet
*
* @return <int> 0: error 1: ok
*/
static int read_rnx_nav(FILE* fp, double ver, int sys, NavPack_t* navall);
/**
* @brief read rinex data head
*
* @param <FILE> [*fp] file pointer
* @param <double> [*ver] rinex version
* @param <char> [*type] rinex data type
* @param <int> [*sys] rinex system
* @param <int> [*tsys] time system
* @param <char> [tobs] obs type array
* @param <NavPack_t> [*navall] nav data packet
* @param <Station_t> [*sta] station information
*
* @return <int> 0: error 1: ok
*/
static int read_rnxh(FILE* fp, double* ver, char* type, int* sys, int* tsys, char tobs[][MAXOBSTYPE][4], NavPack_t* navall, Station_t* sta);
/**
* @brief read rinex data file point
*
* @param <FILE> [*fp] file pointer
* @param <char> [*type] rinex data type
* @param <vector<ObsRecData_t>> [*obsall] obs receiver data vector
* @param <NavPack_t> [*navall] nav data packet
* @param <Station_t> [*sta] station information
*
* @return <int> 0: error 1: ok
*/
static int read_rnxfp(FILE* fp, char* type, vector<ObsRecData_t>* obsall, NavPack_t* navall, Station_t* sta);

/* read ATX file ----------------------------------------------------------*/

/**
* @brief decode atx data
*
* @param <char> [*p] buffer pointer
* @param <int> [n] number of pcv value
* @param <double> [*v] pcv value array
*
* @return <int> [i] number of pcv value
*/
static int decode_atx(char* p, int n, double* v);
/**
* @brief earch antenna parameter (PCV/PCO)
*
* @param <int> [sat] sat#
* @param <char> [*type] antenna type
* @param <GpsTime_t> [time] time to search parameters
* @param <vector> [*pcvs] all PCV parameters read from file
*
* @return <PCV_t> [*pcv] PCV of sat# fit time
*/
static int search_pcv(int sat, const char* type, GpsTime_t time, vector<PCV_t>* pcvs, PCV_t* pcv);

/* read SP3/CLK file -----------------------------------------------------*/

/**
* @brief read SP3 file head
*
* @param <FILE> [*fp] SP3 file pointer
* @param <GpsTime_t> [*time] ephemeris time
* @param <char> [*type] ephemeris type ('P','V'...)
* @param <double> [*bfact] bias factors b0,b1
* @param <char> [*tsys] time system (TSYS_GPS,TSYS_UTC...)
*
* @return <int> [k] sat numbers in SP3 file
*/
static int read_sp3h(FILE* fp, GpsTime_t* time, char* type, double* bfact, char* tsys);
/**
* @brief read SP3 file body
*
* @param <FILE> [*fp] SP3 file pointer
* @param <char> [*type] ephemeris type ('P','V'...)
* @param <int> [ns] satellite numbers
* @param <double> [*bfact] bias factors b0,b1
* @param <char> [*tsys] time system (TSYS_GPS,TSYS_UTC...)
* @param <int> [idx] file index (unless now)
* @param <NavPack_t> [*navall] nav packet
*
* @return void
*/
static void read_sp3b(FILE* fp, char type, int ns, double* bfact, char* tsys, int index, NavPack_t* navall);
/**
* @brief read SP3 file
*
* @param <char> [*file] SP3 file path
* @param <NavPack_t> [*navall] nav packet
*
* @return void
*/
static void read_sp3(const char* file, NavPack_t* navall);
/**
* @brief read CLK file head
*
* @param <FILE> [*fp] CLK file pointer
* @param <char> [*type] clock type
*
* @return <int> 0: error 1: ok
*/
static int read_clkh(FILE* fp, char* type);
/**
* @brief read CLK file body
*
* @param <FILE> [*fp] CLK file pointer
* @param <char> [type] clock type
* @param <int> [index] file index (unless now)
* @param <NavPack_t> [*navall] nav packet
*
* @return <int> 0: error 1: ok
*/
static int read_clkb(FILE* fp, char type, int index, NavPack_t* navall);
/**
* @brief read rinex clock files
*
* @param <char> [*file] CLK file path
* @param <NavPack_t> [*navall] nav packet
*
* @return void
*/
static void read_clk(const char* file, NavPack_t* navall);