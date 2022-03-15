#include "gnss_pos.h"

/* post process functions ------------------------------------------*/

static int position(ObsEphData_t* obs, int n, NavPack_t* navall, ProcOpt_t* popt, Sol_t* sol)
{
	/* 局部变量定义 ========================================================= */
	GpsTime_t time;						// GPS时间变量
	int nu;								// 有效obs数
	char msg[128] = "";					// 错误信息字符串
	int stat;
	/* ====================================================================== */

	sol->stat = SOLQ_NONE;
	time = sol->time;	/* previous epoch */

	/* 1.rover position by single point positioning */
	if ((stat = spp(obs, n, navall, popt, sol, &sat_stat)) == 0) {
		//printf("*** ERROR: spp error\n");
		sta.nbad++;
		return -1;
	}

	//if (time.time != 0) rtk->tt = timediff(rtk->sol.time, time);

	/* 2.通过筛选，更新有效卫星个数nu */
	//nu = n;
	//obsScan_PPP(opt, obs, n, &nu);
	//if (nu <= 4) {
	//	sprintf(PPP_Glo.chMsg, "*** WARNING: There are only %d satellites observed, skip PPP!\n", nu);
	//	outDebug(OUTWIN, OUTFIL, 0);
	//	return 0;
	//}

	///* 3.clock jump repair 钟跳修复 */
	//clkRepair(obs, nu);

	///* 4.precise point positioning 执行PPP定位 */
	//if (opt->mode >= PMODE_PPP_KINEMA) {
	//	pppos(rtk, obs, nu, nav);
	//}
	//else {
	//	return 1;
	//}

	////calculate DOPs
	//calDop(rtk, obs, nu);

	////save the information for current epoch
	//keepEpInfo(rtk, obs, nu, nav);
	return 1;
}

static int obsScan_SPP(const ProcOpt_t* popt, ObsEphData_t* obs, const int nobs)
{
	/* 局部变量定义 ========================================================= */
	double dt;					// 判断符(是否为0)
	int i, j;					// 循环遍历变量
	int n, sat, sys;			// 有效obs个数/卫星号/系统号
	map<int, ObsData_t>::iterator it;
	/* ====================================================================== */

	for (it = obs->obssat.begin(), n = 0; it != obs->obssat.end();) {
		sat = it->first;
		sys = satsys(sat, NULL);

		if (!(sys & popt->nav_sys) || popt->exsats[sat - 1] || norm(it->second.P, 3) <= 0.0) {
			it = obs->obssat.erase(it);
			continue;
		}

		it++;
	}

	if (obs->obssat.size() > 0) {
		obs->nsat = obs->obssat.size();
		return obs->nsat;
	}

	return 0;
}

static int inputobs(vector<ObsRecData_t>* obsall, int revs, int rcv, int* ieph, ObsEphData_t* obs)
{
	double ep[6];

	if (!revs) {    /* input forward data */
		if (rcv <= (*obsall).size() && *ieph < (*obsall)[rcv].neph && *ieph >= 0) {
			*obs = (*obsall)[rcv].obseph[*ieph];
			(*ieph)++;
			if ((*obs).nsat < 4) {
				time2epoch((*obs).eph, ep);
				printf("%04d-%02d-%02d %02d:%02d:%05.2f number of sat = %2d<4\n",
					(int)ep[0], (int)ep[1], (int)ep[2], (int)ep[3], (int)ep[4], ep[5], (*obs).nsat);
				return -1;
			}
		}
		else { return -1; }
	}
	else {
		if (rcv <= (*obsall).size() && *ieph < (*obsall)[rcv].neph && *ieph >= 0) {
			*obs = (*obsall)[rcv].obseph[*ieph];
			(*ieph)--;
			if ((*obs).nsat < 4) {
				time2epoch((*obs).eph, ep);
				printf("%04d-%02d-%02d %02d:%02d:%05.2f number of sat = %2d<4\n",
					(int)ep[0], (int)ep[1], (int)ep[2], (int)ep[3], (int)ep[4], ep[5], (*obs).nsat);
				return -1;
			}
		}
		else { return -1; }
	}

	return (*obs).nsat;
}

static void procpos(ProcOpt_t* popt, Solopt_t* sopt, int mode, Sol_t* sol)
{
	/* 局部变量定义 ========================================================= */
	Sol_t sol_tmp = { {0} };				// 解算结果结构体(全0)
	GpsTime_t time_tmp = { 0 };				// gts时间变量
	ObsEphData_t obs;					// obs结构体变量
	int i, j, k = 0;					// 循环遍历变量
	int nep = 0, nobs, nfit, solstat;	// 历元个数/obs个数/有效obs个数/解算状态标记
	int pri[] = { 0,1,2,3,4,5,1,6 };	// 优先级数组
	/* ====================================================================== */

	//solstatic = sopt->solstatic && popt->mode == PMODE_PPP_STATIC;
	solstat = 0;

	/* processing epoch-wise */
	/* 1.obs获取一个历元内所有的obs数据，并返回obs个数nobs */
	while ((nobs = inputobs(&obsall, popt->sol_mode, 0, &(popt->ieph), &obs)) >= 0) {

		if (k == 0) { popt->ts = obs.eph; }
		k++;
		// 计算30min包含多少个历元
		nep = (int)(30 * (60 / popt->ti));

		/* 2.pseudorange observation checking 通过筛选，更新有效卫星个数n */
		nfit = obsScan_SPP(popt, &obs, nobs);

		if (nfit < 4) { printf("sat number %d<4, can't precess SPP\n", nfit); continue; }

		/* 3.执行当前历元定位 */
		i = position(&obs, nfit, &navall, popt, sol);

		if		(i == -1) { (*sol).stat = SOLQ_NONE; }
		else if (i == 0) { continue; }

		if (mode == 0) {  /* forward/backward */
			//outResult(rtk, sopt, &navs);

			if (!solstat && fps[0]) {
				out_sol(fps[0], sol, popt, sopt);
			}
			else if (time_tmp.time == 0 || pri[(*sol).stat] <= pri[sol_tmp.stat]) {
				sol_tmp = (*sol);
				if (time_tmp.time == 0 || timediff((*sol).time, time_tmp) < 0.0) {
					time_tmp = (*sol).time;
				}
			}
		}
	}
	return;
}

static int execses(ProcOpt_t* popt, FileOpt_t* fopt, Solopt_t* sopt)
{
	Sol_t sol = { {0} };

	/* 1.设置周跳检测阈值 */
	if (fabs(popt->thresGF) < 0.01 || 
		fabs(popt->thresMW) < 0.01) {
		cal_CS_thres(popt, popt->ti);
	}

	/* 2.设置前向/后向处理 */
	if (popt->sol_mode == 0) {
		popt->ieph = 0;
		popt->ts = obsall[0].obseph[0].eph;
		popt->te = obsall[0].obseph[obsall[0].neph - 1].eph;

		for (auto it = fopt->dbgfile.begin(); it != fopt->dbgfile.end(); it++) {
			if (strstr(*it, "sele")) {
				sol.fp_sat = fopen(*it, "w");
			}
			else if (strstr(*it, "iter")) {
				sol.fp_itr = fopen(*it, "w");
			}
		}
		
		procpos(popt, sopt, 0, &sol);

		if (sol.fp_sat) { fclose(sol.fp_sat); sol.fp_sat = NULL; }
		if (sol.fp_itr) { fclose(sol.fp_itr); sol.fp_sat = NULL; }
	}
	else if (popt->sol_mode == 1) { 
		popt->ieph = obsall[0].neph - 1;
		popt->ts = obsall[0].obseph[0].eph;
		popt->te = obsall[0].obseph[obsall[0].neph - 1].eph;
		procpos(popt, sopt, 0, &sol);
	}

	printf("bad epoch not iterated=%d\n", sol.n_ite);
	printf("bad epoch ns not enough=%d\n", sol.n_les);
	return 0;
}

static int read_products(GpsTime_t ts, GpsTime_t te, double ti, ProcOpt_t* popt, FileOpt_t* fopt)
{
	int stat;

	/* read RINEX file ---------------------------------*/
	if (fopt->infile.size()) {
		if (!(stat = read_obsnav(ts, te, 30.0, fopt->infile, &obsall, &navall, &sta))) {
			return 0;
		}
	}
	/* read ATX file -----------------------------------*/
	if (fopt->atxfile.size()) {
		if (!(stat = read_pcv(fopt->atxfile, &pcvs))) {
			printf("*** PCV/PCO not correct\n");
			//return -1;
		}
		else {
			set_pcv(obsall[0].obseph[0].eph, popt, &navall, &pcvs, &sta);
			vector<PCV_t> tmp; pcvs.swap(tmp);	// 释放内存
		}
	}
	/* read DCB file -----------------------------------*/
	if (fopt->dcbfile.size()) {
		if (!(stat = read_dcb(fopt->dcbfile, &navall))) {
			printf("*** DCB not correct\n");
			//return -1;
		}
	}
	if (fopt->mdcbfile.size()) {
		if (!(stat = read_dcb_mgex(fopt->mdcbfile, &navall, obsall[0].obseph[0].eph))) {
			printf("*** MDCB not correct\n");
			//return -1;
		}
	}
	/* read ERP file -----------------------------------*/
	if (fopt->erpfile.size()) {
		if (!(stat = read_erp(fopt->erpfile, &navall))) {
			printf("*** Earth rotation parameters not correct\n");
			//return -1;
		}
	}
	/* read SP3 & CLK file -----------------------------*/
	if (fopt->precfile.size() && popt->eph_opt == EPHOPT_PREC) {
		if (!(stat = read_prec_eph(fopt->precfile, &navall))) {
			printf("*** Precise ephemeris & clock not correct\n");
			return 0;
		}
	}
	/* read BLQ file -----------------------------------*/
	if (fopt->blqfile.size()) {
		if (!(stat = read_blq(fopt->blqfile, sta.name, &navall.otl))) {
			printf("*** Ocean tide load not correct\n");
			//return -1;
		}
	}

	return 1;
}

extern int posFDU(GpsTime_t ts, GpsTime_t te, double ti, ProcOpt_t* popt, FileOpt_t* fopt, Solopt_t* sopt)
{
	int stat;

	/* preparation work =========================================================*/
	init_sta(&sta);
	/* set outfile pointers ----------------------------*/
	for (int i = 0; i < fopt->outfile.size(); i++) {
		fps[i] = openfile(fopt->outfile[i]);
	}
	/* set rinex code priority for precise clock -------*/
	if (PMODE_PPP_KINEMA <= popt->pos_mode) {
		setcodepri(SYS_GPS, 1, popt->eph_opt == EPHOPT_PREC ? "PYWC" : "CPYW");
	}
	/* ==========================================================================*/

	/* read files ===============================================================*/
	if (!(stat = read_products(ts, te, ti, popt, fopt))) {
		printf("*** Read products error, program exit\n");
		return 0;
	}

	execses(popt, fopt, sopt);

	for (int i = 0; i < fps.size(); i++) {
		if (fps[i]) {
			fclose(fps[i]); fps[i] = NULL;
		}
	}

	return 1;
}