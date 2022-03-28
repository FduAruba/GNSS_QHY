#include "gnss_pos.h"

/* post process functions ------------------------------------------*/

static int clk_Repair(ObsEphData_t* obs, NavPack_t* navall)
{
	int sat, n_vaild = 0, n_cj = 0;
	double delta0 = 0.0, delta1 = 0.0, d1, d2, d3, d4, ddd1 = 0.0, ddd2 = 0.0;
	double CJ_F1, CJ_F2;
	vector<double> lam;
	map<int, ObsData_t>::iterator it;
	long t1, t2;

	t1 = clock();

	// 1.遍历obs，计算当前历元和上个历元的双频伪距，相位偏差
	for (it = obs->obssat.begin(); it != obs->obssat.end(); it++) {
		sat = it->first;
		lam = navall->lam[sat];

		if (obs->obssat[sat].P[0] * obs->obssat[sat].P[1] *
			obs->obssat[sat].L[0] * obs->obssat[sat].L[1] == 0.0) {
			continue;
		}

		if (pppglob.obs_past.find(sat) == pppglob.obs_past.end()) {
			pppglob.obs_past.insert(pair<int, vector<double>>(sat, vector<double>(4, 0.0)));
			continue;
		}

		if (pppglob.obs_past[sat][0] * pppglob.obs_past[sat][1] *
			pppglob.obs_past[sat][2] * pppglob.obs_past[sat][3] == 0.0) {
			continue;;
		}

		n_vaild++;

		d1 = obs->obssat[sat].P[0] - pppglob.obs_past[sat][0];
		d2 = obs->obssat[sat].P[1] - pppglob.obs_past[sat][1];
		d3 = (obs->obssat[sat].L[0] - pppglob.obs_past[sat][2]) * lam[0];
		d4 = (obs->obssat[sat].L[1] - pppglob.obs_past[sat][3]) * lam[1];

		if (fabs(d1 - d3) > 290000) {   //ms clock jump
			delta0 += d1 - d3;
			delta1 += d2 - d4;
			n_cj++;
		}
	}

	// 2.判断当前历元是否发生钟跳，并更新钟跳值 (仅当前全部obs都发生钟跳才修复)
	if (n_cj != 0 && n_cj == n_vaild)
	{
		d1 = delta0 / n_cj;
		d2 = delta1 / n_cj;

		CJ_F1 = 0.0;   //flag for clock jump
		CJ_F2 = 0.0;
		CJ_F1 = d1 / CLIGHT * 1000.0;
		CJ_F2 = myRound(CJ_F1);

		if (fabs(CJ_F1 - CJ_F2) < 2.5E-2) {
			pppglob.clk_jump += static_cast<int>(CJ_F2);
			printf("*** WARNING: clock jump=%d(ms)\n", (int)pppglob.clk_jump);
		}
		else { ; }
	}

	// 3.更新pppglob的obs状态，并对双频相位进行钟跳修复
	for (auto it = pppglob.obs_past.begin(); it != pppglob.obs_past.end(); it++) {
		sat = it->first;
		lam = navall->lam[sat];

		if (obs->obssat.find(sat) == obs->obssat.end()) {
			pppglob.obs_past[sat][0] = 0.0;
			pppglob.obs_past[sat][1] = 0.0;
			pppglob.obs_past[sat][2] = 0.0;
			pppglob.obs_past[sat][3] = 0.0;
			continue;
		}
		else {
			pppglob.obs_past[sat][0] = obs->obssat[sat].P[0];
			pppglob.obs_past[sat][1] = obs->obssat[sat].P[1];
			pppglob.obs_past[sat][2] = obs->obssat[sat].L[0];
			pppglob.obs_past[sat][3] = obs->obssat[sat].L[1];
		}
		
		ddd1 = ddd2 = pppglob.clk_jump * CLIGHT / 1000.0;

		//repair for phase observations
		if (obs->obssat[sat].L[0] != 0.0) { obs->obssat[sat].L[0] += ddd1 / lam[0]; }
		if (obs->obssat[sat].L[1] != 0.0) { obs->obssat[sat].L[1] += ddd2 / lam[1]; }
	}
	
	t2 = clock();

	// printf("* The total time for running the program: %6.3f seconds\n%c", (double)(t2 - t1) / CLOCKS_PER_SEC, '\0');
	return 1;
}

static int position(ObsEphData_t* obs, int n, NavPack_t* navall, ProcOpt_t* popt, Sol_t* sol)
{
	/* 局部变量定义 ========================================================= */
	GpsTime_t time;						// GPS时间变量
	int n_ppp;							// PPP算法有效obs数
	char msg[128] = "";					// 错误信息字符串
	int stat;
	/* ====================================================================== */

	sol->stat = SOLQ_NONE;
	time = sol->time;	/* previous epoch */

	/* 1.rover position by single point positioning */
	if ((stat = spp(obs, n, navall, popt, sol, &sat_stat)) == 0) {
		//printf("*** ERROR: spp error\n");
		return -1;
	}

	if (time.time != 0) { sol->tt = timediff(sol->time, time); }

	/* 2.PPP obs 数据筛选 */
	n_ppp = obsScan_PPP(popt, obs);
	if (n_ppp < 5) {
		printf("*** WARNING: There are only %d satellites observed, skip PPP!\n", n_ppp);
		return 0;
	}

	/* 3.clock jump repair */
	clk_Repair(obs, navall);

	/* 4.precise point positioning 执行PPP定位 */
	if (popt->pos_mode >= PMODE_PPP_KINEMA) {
		ppp(obs, n_ppp, navall, popt, sol, &sat_stat);
	}
	else { return 1; }

	////calculate DOPs
	//calDop(rtk, obs, nu);

	////save the information for current epoch
	//keepEpInfo(rtk, obs, nu, nav);
	return 1;
}

static int obsScan_SPP(const ProcOpt_t* popt, ObsEphData_t* obs)
{
	/* 局部变量定义 ========================================= */
	int sat, sys;						// 卫星号/系统号
	map<int, ObsData_t>::iterator it;   // obs迭代指针
	/* ====================================================== */

	for (it = obs->obssat.begin(); it != obs->obssat.end();) {
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

static int obsScan_PPP(const  ProcOpt_t* popt, ObsEphData_t* obs)
{
	/* 局部变量定义 ========================================= */
	int sat, sys;						// 卫星号/系统号
	map<int, ObsData_t>::iterator it;   // obs迭代指针
	/* ====================================================== */

	for (it = obs->obssat.begin(); it != obs->obssat.end();) {
		sat = it->first;
		sys = satsys(sat, NULL);

		if (sat = 14 && it->second.P[0] == 0.0) {
			int tt = 1;
		}

		if (popt->pos_mode >= PMODE_PPP_KINEMA) {
			// 判断obs L1,L2频率上是否相位均存在，且双频伪距差值较小 (粗差检测)
			if ((it->second.L[0] * it->second.L[1] == 0.0) || (fabs(it->second.P[0] - it->second.P[1]) >= 200.0)) {
				it = obs->obssat.erase(it);
				continue;
			}
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
	int nep = 0, nobs, n_spp, solstat;	// 历元个数/obs个数/有效obs个数/解算状态标记
	int pri[] = { 0,1,2,3,4,5,1,6 };	// 优先级数组
	/* ====================================================================== */

	//solstatic = sopt->solstatic && popt->mode == PMODE_PPP_STATIC;
	solstat = 0;

	/* processing epoch-wise */
	/* 1.obs获取一个历元内所有的obs数据，并返回obs个数nobs */
	while ((nobs = inputobs(&obsall, popt->sol_mode, 0, &(popt->ieph), &obs)) >= 0) {

		if (k == 0) { popt->ts = obs.eph; }
		k++;
		time2epoch(obs.eph, pppglob.ep);
		// 计算30min包含多少个历元
		nep = (int)(30 * (60 / popt->ti));

		/* 2.pseudorange observation checking 通过筛选，更新有效卫星个数n */
		n_spp = obsScan_SPP(popt, &obs);

		if (n_spp < 4) { printf("sat number %d<4, can't precess SPP\n", n_spp); continue; }

		/* 3.执行当前历元定位 */
		i = position(&obs, n_spp, &navall, popt, sol);

		if		(i == -1) { (*sol).stat = SOLQ_NONE; }
		else if (i == 0)  { continue; }

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
			else if (strstr(*it, "ppp")) {
				sol.fp_sat_ppp = fopen(*it, "w");
			}
		}
		
		procpos(popt, sopt, 0, &sol);

		if (sol.fp_sat) { fclose(sol.fp_sat); sol.fp_sat = NULL; }
		if (sol.fp_itr) { fclose(sol.fp_itr); sol.fp_sat = NULL; }
		if (sol.fp_sat_ppp) { fclose(sol.fp_sat_ppp); sol.fp_sat_ppp = NULL; }
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