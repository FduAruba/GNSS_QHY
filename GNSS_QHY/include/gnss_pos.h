#pragma once
#include "gnss.h"

/* static variables ------------------------------------------------*/

static vector<ObsRecData_t> obsall;		  // obs data set
static NavPack_t navall = { 0 };		  // nav packet
static Station_t sta;					  // station infomation
static vector<PCV_t> pcvs;				  // pcv data
static map<int, Sat_t> sat_stat;		  // sat status
static vector<FILE*> fps = vector<FILE*>(50, NULL);	// file pointers
//static PPP_Glob_t pppglob;

/* functions -------------------------------------------------------*/

static int clk_Repair(ObsEphData_t* obs, NavPack_t* navall);
/**
* @brief API of specific position processes (SPP, PPP...)
*
* @param <ObsEphData_t> [*obs]  obs data in i-th epoch
* @param <int>          [n]     number of obs in i-th epoch
* @param <ProcOpt_t>    [*popt] process option
* @param <Sol_t>        [*sol]  solution
*
* @return <int> -1:error 1:ok
*/
static int position(ObsEphData_t* obs, int n, NavPack_t* navall, ProcOpt_t* popt, Sol_t* sol);
/**
* @brief scan obs data and exclude satellite by system
*
* @param <ProcOpt_t>    [*popt] process option
* @param <ObsEphData_t> [*obs]  obs data in i-th epoch
*
* @return <int> number of obs after sort (0:error)
*/
static int obsScan_SPP(const ProcOpt_t* popt, ObsEphData_t* obs);
/**
* @brief scan obs data and exclude satellite by double frequency carruier phace and code coarse difference
*
* @param <ProcOpt_t>    [*popt] process option
* @param <ObsEphData_t> [*obs]  obs data in i-th epoch
*
* @return <int> number of obs after sort (0:error)
*/
static int obsScan_PPP(const  ProcOpt_t* popt, ObsEphData_t* obs);
/**
* @brief input obs data from obs set
*
* @param <vector>       [*obsall] obs data set
* @param <int>          [revs]    0:forward 1:backward
* @param <int>          [rec]     receiber#
* @param <int>          [*ieph]   i-th epoch
* @param <ObsEphData_t> [*obs]    obs data in i-th epoch
*
* @return <int> number of obs in current epoch (-1:error)
*/
static int inputobs(vector<ObsRecData_t>* obsall, int revs, int rcv, int* ieph, ObsEphData_t* obs);
/**
* @brief input obs data at this epoch, process position and output solution
*
* @param <ProcOpt_t> [*popt] process option
* @param <SolOpt_t>  [*sopt] solution option
* @param <int>       [mode]  process mode (forward/backward)
* @param <Sol_t>     [*sol]  solution
*
* @return void
*/
static void procpos(ProcOpt_t* popt, Solopt_t* sopt, int mode, Sol_t* sol);
/**
* @brief execute process (forward/backward)
*
* @param <ProcOpt_t> [*popt] process option
* @param <FileOpt_t> [*fopt] file option
* @param <SolOpt_t>  [*sopt] solution option
*
* @return 0: error 1: ok
*/
static int execses(ProcOpt_t* popt, FileOpt_t* fopt, Solopt_t* sopt);
/**
* @brief read obs data, ephemeris data and opher external products form files
*
* @param <GpsTime_t> [ts,te] start time/end time
* @param <double>    [ti]    time unit
* @param <ProcOpt_t> [ts,te] strat/end time
* @param <ProcOpt_t> [*popt] process option
* @param <FileOpt_t> [*fopt] file option
*
* @return 0: error 1: ok
*/
static int read_products(GpsTime_t ts, GpsTime_t te, double ti, ProcOpt_t* popt, FileOpt_t* fopt);