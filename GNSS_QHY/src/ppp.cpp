#include"gnss.h"

//calculate satellite elevation
extern void calElev(Sol_t* sol, const ObsEphData_t* obs, int n, map<int, vector<double>>* rs, map<int, Sat_t>* sat_stat)
{
	int i, sat;
	double r;
	vector<double> rr = vector<double>(3, 0.0);
	vector<double> pos = vector<double>(3, 0.0);
	vector<double> e = vector<double>(3, 0.0);
	pair<int, vector<double>> azel;
	map<int, vector<double>> azel_tmp;

	// 1.卫星方位角/仰角清零
	for (auto it = (*sat_stat).begin(); it != (*sat_stat).end(); it++) {
		it->second.azel[0] = it->second.azel[1] = 0.0;
	}
	// 2.获取接收机位置
	if (sol->rr_snx[0] != 0.0) {
		for (int i = 0; i < 3; i++) {
			rr[i] = sol->rr_snx[i];
		}
	}
	else{
		for (int i = 0; i < 3; i++) {
			rr[i] = sol->rr[i];
		}
	}
	if (norm(rr, 3) <= 100.0) { return; }

	pos = ecef2pos(rr);
	// 3.重新计算当前历元卫星仰角
	for (auto it = obs->obssat.begin(); it != obs->obssat.end(); it++) {
		sat = it->first;

		if (satsys(sat, NULL) == SYS_NONE)          { continue; }
		if ((r = geodist(sat, rs, rr, &e)) < 0)     { continue; }
		if (sat_stat->find(sat) == sat_stat->end()) { continue; }

		azel.first = sat;
		for (int i = 0; i < 2; i++) {
			azel.second.push_back((*sat_stat)[sat].azel[i]);
		}
		
		azel_tmp.insert(azel);
		satazel(sat, pos, e, &azel_tmp);
	}
	// 4.将方位角/仰角赋值回sat_stat
	for (auto it = azel_tmp.begin(); it != azel_tmp.end(); it++) {
		sat = it->first;

		for (int i = 0; i < 2; i++) { 
			(*sat_stat)[sat].azel[i] = azel_tmp[sat][i]; 
		}
	}
}

/* exclude meas of eclipsing satellite (block IIA) ---------------------------*/
static void test_eclipse(const ObsEphData_t* obs, int n, const NavPack_t* navall, map<int, vector<double>>* rs)
{
	double rsun[3], esun[3], r, ang, erpv[5] = { 0 }, cosa;
	int i, j, sat;
	const char* type;
	double sec, ex[3] = { 0.0,0.0,0.0 };
	int week;
	/* unit vector of sun direction (ecef) */
	sun_moon_pos(gpst2utc(obs->eph), erpv, rsun, NULL, NULL);
	normv3(rsun, esun);

	for (auto it = (*rs).begin(); it != (*rs).end(); it++) {
		sec = time2gpst(obs->eph, &week);
		sat = it->first;
		week = cal_Eclips(sat, rs, rsun, sec, ex, navall);

		if (week && 1) {
			char id[10] = { '\0' };
			satno2id(sat, id);
			printf("*** WARNING: %s ecliType=%d\n", id, week);
			pppglob.ecli_f[sat] = 4.0;
		}
		else{ 
			pppglob.ecli_f[sat] = 1.0;
		}
			
		type = (*navall).sat_pcv.at(sat).type;

		if ((r = norm((*rs)[sat], 3)) <= 0.0) { continue; }

		/* only block IIA */
		if (*type && !strstr(type, "BLOCK IIA")) { continue; }

		/* sun-earth-satellite angle */
		cosa = dot((*rs)[sat], esun, 3) / r;
		cosa = cosa < -1.0 ? -1.0 : (cosa > 1.0 ? 1.0 : cosa);
		ang = acos(cosa);

		/* test eclipse */
		if (ang<PI / 2.0 || r * sin(ang)>RE_WGS84) { continue; }

		//sprintf(PPP_Glo.chMsg, "*** WARNING: eclipsing sat excluded sat=%s\n", PPP_Glo.sFlag[obs[i].sat - 1].id);
		//outDebug(OUTWIN, OUTFIL, OUTTIM);

		for (j = 0; j < 3; j++) { (*rs)[sat][j] = 0.0; }
	}
}

/* precise point positioning -------------------------------------------------*/
extern int ppp(ObsEphData_t* obs, int n, NavPack_t* navall, const ProcOpt_t* popt, Sol_t* sol, map<int, Sat_t>* sat_stat)
{
	/* 局部变量定义 ========================================================= */
	//const prcopt_t* opt = &rtk->opt;
	//double* rs, * dts, * var, * v, * H, * R, * azel, * xp, * Pp;
	double rr[3], pos[3], dr[3] = { 0 }, enu[3] = { 0 }, dtdx[3] = { 0 };
	double dtrp = 0.0, shd = 0.0, vart = 0.0;
	double dantr[NFREQ] = { 0 }, dants[NFREQ] = { 0 };
	double cosaz, sinaz, cosel, sinel;
	char str[32];
	int i, j, nv, sat, info, exc[MAXOBS] = { 0 }, stat = SOLQ_SINGLE;

	map<int, vector<int>> vsat;			// 卫星状态标记(1:ok  0:error)
	map<int, vector<int>> svh;			// 卫星健康标识符(0:ok -1:error)
	map<int, vector<double>> rs;		// 卫星坐标&速度(Px,Py,Pz,Vx,Vy,Vz)
	map<int, vector<double>> dts;		// 卫星钟差&钟漂(dts,ddts)
	map<int, vector<double>> var;		// 卫星定位方差
	map<int, vector<double>> azel;		// 卫星仰角&方位角(az,el)
	map<int, vector<double>> resp;		// 定位残差
	/* ====================================================================== */

	time2str(obs->eph, str, 2);

	for (auto it = obs->obssat.begin(); it != obs->obssat.end(); it++) {
		rs.insert(pair<int, vector<double>>(it->first, vector<double>(6, 0.0)));
		dts.insert(pair<int, vector<double>>(it->first, vector<double>(2, 0.0)));
		var.insert(pair<int, vector<double>>(it->first, vector<double>(1, 0.0)));
		azel.insert(pair<int, vector<double>>(it->first, vector<double>(2, 0.0)));
		resp.insert(pair<int, vector<double>>(it->first, vector<double>(1, 0.0)));
		svh.insert(pair<int, vector<int>>(it->first, vector<int>(1, -1)));
		vsat.insert(pair<int, vector<int>>(it->first, vector<int>(1, 0)));
	}
	
	//rs = mat(6, n); dts = mat(2, n); var = mat(1, n); azel = zeros(2, n);

	//for (i = 0; i < MAXSAT; i++) for (j = 0; j < opt->nf; j++) rtk->ssat[i].fix[j] = 0;

	/* 1.satellite positions and clocks */
	cal_satpos(obs->eph, obs, n, navall, popt->eph_opt, &rs, &dts, &var, &svh);

	/* 2.exclude measurements of eclipsing satellite (block IIA) 没搞懂 */
	test_eclipse(obs, n, navall, &rs);

	// 3.calculate satellite elevation
	calElev(sol, obs, n, &rs, sat_stat);

	for (i = 0; i < n; i++) {
		sat = obs[i].sat;
		for (j = 0; j < 3; j++) PPP_Glo.ssat_Ex[sat - 1].rs[j] = rs[j + i * 6];

		/* satellite and receiver antenna model */
		satantpcv(sat, rs + i * 6, PPP_Glo.rr, nav->pcvs + sat - 1, PPP_Glo.ssat_Ex[sat - 1].dants);
		antmodel(sat, &opt->pcvr, opt->antdel, rtk->ssat[sat - 1].azel, 1, PPP_Glo.ssat_Ex[sat - 1].dantr);

		/* phase windup model */
		if (!model_phw(rtk->sol.time, sat, nav->pcvs[sat - 1].type, 2, rs + i * 6, PPP_Glo.rr, &PPP_Glo.ssat_Ex[sat - 1].phw)) {
			continue;
		}
	}

	////to compute the elements of initialized PPP files
	//if (PPP_Glo.outFp[OFILE_IPPP]) {
	//	//epoch time
	//	PPP_Info.t = obs[0].time;

	//	for (i = 0; i < 3; i++) rr[i] = PPP_Glo.crdTrue[i];
	//	if (norm(rr, 3) == 0.0) rr[i] = rtk->sol.rr[i];
	//	ecef2pos(rr, pos);

	//	//the displacements by earth tides
	//	tidedisp(gpst2utc(obs[0].time), rr, 7, &nav->erp, opt->odisp[0], dr);

	//	for (i = 0; i < MAXSAT; i++) PPP_Info.ssat[i].vs = 0;
	//	for (i = 0; i < n && i < MAXOBS; i++) {
	//		sat = obs[i].sat;
	//		PPP_Info.ssat[sat - 1].vs = 1;

	//		//satellite position and clock offsets
	//		matcpy(PPP_Info.ssat[sat - 1].satpos, rs + i * 6, 6, 1);
	//		PPP_Info.ssat[sat - 1].satclk = dts[i * 2] * CLIGHT;

	//		//the eclipse satellites
	//		PPP_Info.ssat[sat - 1].flag = 0;
	//		if (norm(PPP_Info.ssat[sat - 1].satpos, 3) == 0.0) PPP_Info.ssat[sat - 1].flag = 1;

	//		//sagnac effect
	//		PPP_Info.ssat[sat - 1].dsag = sagnac(rs + i * 6, rr);

	//		//satellite azimuth and elevation
	//		PPP_Info.ssat[sat - 1].azel[0] = rtk->ssat[sat - 1].azel[0];
	//		PPP_Info.ssat[sat - 1].azel[1] = rtk->ssat[sat - 1].azel[1];

	//		//line-of-sight (LOS) unit vector
	//		cosel = cos(PPP_Info.ssat[sat - 1].azel[1]); sinel = sin(PPP_Info.ssat[sat - 1].azel[1]);
	//		cosaz = cos(PPP_Info.ssat[sat - 1].azel[0]); sinaz = sin(PPP_Info.ssat[sat - 1].azel[0]);

	//		//convert 3D tidal displacements to LOS range
	//		ecef2enu(pos, dr, enu);
	//		PPP_Info.ssat[sat - 1].dtid = enu[1] * cosel * cosaz + enu[0] * cosel * sinaz + enu[2] * sinel;

	//		//tropospheric zenith total delays (ZTDs)
	//		if (!model_trop(obs[i].time, pos, PPP_Info.ssat[sat - 1].azel, opt,
	//			rtk->x, dtdx, nav, &dtrp, &shd, &vart)) continue;
	//		PPP_Info.ssat[sat - 1].dtrp = dtrp;
	//		PPP_Info.ssat[sat - 1].shd = shd;
	//		PPP_Info.ssat[sat - 1].wmap = dtdx[0];

	//		//satellite and receiver antenna model
	//		satantpcv(sat, rs + i * 6, rr, nav->pcvs + sat - 1, dants);
	//		antmodel(sat, &opt->pcvr, opt->antdel, PPP_Info.ssat[sat - 1].azel, 1, dantr);
	//		for (j = 0; j < NFREQ; j++) PPP_Info.ssat[sat - 1].dant[j] = dants[j] + dantr[j];

	//		/* phase windup model */
	//		if (!model_phw(rtk->sol.time, sat, nav->pcvs[sat - 1].type, 2, rs + i * 6, rr,
	//			&PPP_Info.ssat[sat - 1].phw)) continue;

	//		for (j = 0; j < NFREQ; j++) {
	//			PPP_Info.ssat[sat - 1].L[j] = PPP_Info.ssat[sat - 1].P[j] = 0.0;
	//			PPP_Info.ssat[sat - 1].L[j] = obs[i].L[j];
	//			PPP_Info.ssat[sat - 1].P[j] = obs[i].P[j];
	//		}
	//	}
	//}

	//detecs(rtk, obs, n, nav);

	///* temporal update of ekf states */
	//udstate_ppp(rtk, obs, n, nav);

	//nv = n * rtk->opt.nf * 2 + MAXSAT + 3;
	//xp = mat(rtk->nx, 1); Pp = zeros(rtk->nx, rtk->nx);
	//v = mat(nv, 1); H = mat(rtk->nx, nv); R = mat(nv, nv);
	//for (i = 0; i < MAX_ITER; i++) {
	//	matcpy(xp, rtk->x, rtk->nx, 1);
	//	matcpy(Pp, rtk->P, rtk->nx, rtk->nx);

	//	/* prefit residuals */
	//	if (!(nv = ppp_res(0, obs, n, rs, dts, var, svh, exc, nav, xp, rtk, v, H, R, azel))) {
	//		sprintf(PPP_Glo.chMsg, "*** WARNING: %s ppp (%d) no valid obs data\n", str, i + 1);
	//		outDebug(OUTWIN, OUTFIL, OUTTIM);
	//		break;
	//	}
	//	/* measurement update of ekf states */
	//	if ((info = filter(xp, Pp, H, v, R, rtk->nx, nv))) {
	//		sprintf(PPP_Glo.chMsg, "*** ERROR: %s ppp (%d) filter error info=%d\n", str, i + 1, info);
	//		outDebug(OUTWIN, OUTFIL, OUTTIM);
	//		break;
	//	}
	//	/* postfit residuals */
	//	if (ppp_res(i + 1, obs, n, rs, dts, var, svh, exc, nav, xp, rtk, v, H, R, azel)) {
	//		matcpy(rtk->x, xp, rtk->nx, 1);
	//		matcpy(rtk->P, Pp, rtk->nx, rtk->nx);
	//		stat = SOLQ_PPP;
	//		break;
	//	}
	//}
	//if (i >= MAX_ITER) {
	//	sprintf(PPP_Glo.chMsg, "*** WARNING: %s ppp (%d) iteration overflows\n", str, i);
	//	outDebug(OUTWIN, OUTFIL, OUTTIM);
	//}
	//if (stat == SOLQ_PPP) {
	//	/* ambiguity resolution in ppp */
	//	/*if (ppp_ar(rtk,obs,n,exc,nav,azel,xp,Pp)&&
	//	ppp_res(9,obs,n,rs,dts,var,svh,dr,exc,nav,xp,rtk,v,H,R,azel)) {

	//	matcpy(rtk->xa,xp,rtk->nx,1);
	//	matcpy(rtk->Pa,Pp,rtk->nx,rtk->nx);

	//	for (i=0;i<3;i++) std[i]=sqrt(Pp[i+i*rtk->nx]);
	//	if (norm(std,3)<MAX_STD_FIX) stat=SOLQ_FIX;
	//	}
	//	else {*/
	//	rtk->nfix = 0;
	//	//}
	//	/* update solution status */
	//	update_stat(rtk, obs, n, stat);
	//}
	//free(rs); free(dts); free(var); free(azel);
	//free(xp); free(Pp); free(v); free(H); free(R);
return 1;
}