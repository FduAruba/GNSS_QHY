#include"gnss.h"

/* nominal yaw-angle ---------------------------------------------------------*/
static double yaw_nominal(double beta, double mu)
{
	if (fabs(beta) < 1E-12 && fabs(mu) < 1E-12) { return PI; }
	return atan2(-tan(beta), sin(mu)) + PI;
}

/* yaw-angle of satellite ----------------------------------------------------*/
extern int yaw_angle(int sat, const char* type, int opt, double beta, double mu, double* yaw)
{
	*yaw = yaw_nominal(beta, mu);
	return 1;
}

/* satellite attitude model --------------------------------------------------*/
static int sat_yaw(GpsTime_t time, int sat, const char* type, int opt, map<int, vector<double>>* rs, double* exs, double* eys)
{
	double rsun[3], ri[6], es[3], esun[3], n[3], p[3], en[3], ep[3], ex[3], E, beta, mu;
	double yaw, cosy, siny, erpv[5] = { 0 };
	int i;
	double rs_tmp[3];

	for (i = 0; i < 3; i++) { rs_tmp[i] = (*rs)[sat][i]; }

	sun_moon_pos(gpst2utc(time), erpv, rsun, NULL, NULL);

	/* beta and orbit angle */
	matcpy(ri, rs_tmp, 6, 1);
	ri[3] -= OMGE * ri[1];
	ri[4] += OMGE * ri[0];
	cross3(ri, ri + 3, n);
	cross3(rsun, n, p);
	if (!normv3(rs_tmp, es) || !normv3(rsun, esun) ||
		!normv3(n, en) || !normv3(p, ep)) {
		return 0;
	}
	beta = PI / 2.0 - acos(dot(esun, en, 3));
	E = acos(dot(es, ep, 3));
	mu = PI / 2.0 + (dot(es, esun, 3) <= 0 ? -E : E);
	if (mu < -PI / 2.0) mu += 2.0 * PI;
	else if (mu >= PI / 2.0) mu -= 2.0 * PI;

	/* yaw-angle of satellite */
	if (!yaw_angle(sat, type, opt, beta, mu, &yaw)) { return 0; }

	/* satellite fixed x,y-vector */
	cross3(en, es, ex);
	cosy = cos(yaw);
	siny = sin(yaw);
	for (i = 0; i < 3; i++) {
		exs[i] = -siny * en[i] + cosy * ex[i];
		eys[i] = -cosy * en[i] - siny * ex[i];
	}

	return 1;
}

/* phase windup model --------------------------------------------------------*/
static int model_phw(GpsTime_t time, int sat, map<int, PCV_t>* pcv_s, int opt, 
	map<int, vector<double>>* rs, const double* rr, map<int, Sat_t>* sat_stat)
{
	double exs[3], eys[3], ek[3], exr[3], eyr[3], eks[3], ekr[3];
	double dr[3], ds[3], drs[3], r[3], cosp, ph;
	int i;

	vector<double>       pos = vector<double>(3, 0.0);
	vector<double>    rr_tmp = vector<double>(3, 0.0);
	vector<vector<double>> E = vector<vector<double>>(3, vector<double>(3, 0.0));

	if (opt <= 0) { return 1; }	/* no phase windup */

	if (norm(rr, 3) <= 0.0) { return 0; }

	for (i = 0; i < 3; i++) { rr_tmp[i] = rr[i]; }

	/* satellite yaw attitude model */
	if (!sat_yaw(time, sat, (*pcv_s)[sat].type, opt, rs, exs, eys)) { return 0; }

	/* unit vector satellite to receiver */
	for (i = 0; i < 3; i++) { r[i] = rr_tmp[i] - (*rs)[sat][i]; }
	if (!normv3(r, ek)) { return 0; }

	/* unit vectors of receiver antenna */
	pos = ecef2pos(rr_tmp);
	//ecef2pos(rr, pos);
	E = rot_matrix(rr_tmp);
	//xyz2enu(pos, E);
	exr[0] = E[1][0];  exr[1] =  E[1][1]; exr[2] =  E[1][2]; /* x = north */
	eyr[0] = -E[0][0]; eyr[1] = -E[0][1]; eyr[2] = -E[0][2]; /* y = west  */

	/* phase windup effect */
	cross3(ek, eys, eks);
	cross3(ek, eyr, ekr);
	for (i = 0; i < 3; i++) {
		ds[i] = exs[i] - ek[i] * dot(ek, exs, 3) - eks[i];
		dr[i] = exr[i] - ek[i] * dot(ek, exr, 3) + ekr[i];
	}
	cosp = dot(ds, dr, 3) / norm(ds, 3) / norm(dr, 3);
	if (cosp < -1.0) cosp = -1.0;
	else if (cosp > 1.0) cosp = 1.0;
	//acos（-1） invalid
	if (fabs(fabs(cosp) - 1.0) < 1.0e-10) { return 0; }

	ph = acos(cosp) / 2.0 / PI;
	cross3(ds, dr, drs);
	if (dot(ek, drs, 3) < 0.0) ph = -ph;

	(*sat_stat)[sat].phw = ph + floor((*sat_stat)[sat].phw - ph + 0.5); /* in cycle */

	return 1;
}
/* interpolate antenna phase center variation --------------------------------*/
static double interpvar0(int sat, double ang, const double* var, int bsat)
{
	int i, sys, limit = 18;
	char sid[30];
	double a;

	if (bsat) {	// 计算卫星pcv
		ang = ang / 5.0;
		//sys = satsys(sat, NULL);
		//if (sys==SYS_GPS) limit=14;
		//else if (sys==SYS_GLO) limit=15;

		if (ang >= limit) {
			if (ang > limit + 0.25) {
				satno2id(sat, sid);
				printf("*** %s (nadir=%f) >= %2d°\n", sid, ang, limit);
			}
			return var[limit];
		}
		if (ang < 0) {
			printf("*** ERROR: compute satellite antenna offset: nadir < 0\n");
			return var[0];
		}

		i = (int)ang;

		return var[i] * (1.0 + i - ang) + var[i + 1] * (ang - i);
	}
	else {
		a = ang / 5.0; /* ang=0-90 */
		i = (int)a;
		if (i < 0) {
			printf("*** ERROR: compute receiver antenna offset: i<0\n");
			return var[0];
		}
		else if (i > 18) {
			printf("*** ERROR: compute receiver antenna offset: i>18\n");
			return var[18];
		}
		return var[i] * (1.0 - a + i) + var[i + 1] * (a - i);
	}
}
/* satellite antenna model ------------------------------------------------------
* compute satellite antenna phase center parameters
* args   : pcv_t *pcv       I   antenna phase center parameters
*          double nadir     I   nadir angle for satellite (rad)
*          double *dant     O   range offsets for each frequency (m)
* return : none
*-----------------------------------------------------------------------------*/
extern void antmodel_s(int sat, map<int, PCV_t>* pcvs, double nadir, double* pcv)
{
	int i, sys;

	sys = satsys(sat, NULL);
	for (i = 0; i < NFREQ; i++) {
		//在interpvar函数里对nadir也除以了5.0，这是正确的吗？
		//输出nadir*R2D的值，都在14°以内；
		//分别使用两种方案计算alrt站2010年的数据，发现除以5.0的结果高程方向偏差系统为正；
		//不除以5.0的结果高程方向无系统偏差
		//dant[i]=interpvar(sat, nadir*R2D*5.0, pcv->var[i], true);
		if (sys == SYS_GPS) {
			pcv[i] = interpvar0(sat, nadir * R2D * 5.0, (*pcvs)[sat].var[i], 1);
			if (i == 2) {
				pcv[i] = interpvar0(sat, nadir * R2D * 5.0, (*pcvs)[sat].var[1], 1);
			}
		}
		else if (sys == SYS_GLO) {
			pcv[i] = interpvar0(sat, nadir * R2D * 5.0, (*pcvs)[sat].var[i + NFREQ], 1);
			if (i == 2) {
				pcv[i] = interpvar0(sat, nadir * R2D * 5.0, (*pcvs)[sat].var[1 + NFREQ], 1);
			}
		}
		else if (sys == SYS_BDS) {
			pcv[i] = interpvar0(sat, nadir * R2D * 5.0, (*pcvs)[sat].var[i + 2 * NFREQ], 1);
			if (i == 2) {
				pcv[i] = interpvar0(sat, nadir * R2D * 5.0, (*pcvs)[sat].var[1 + 2 * NFREQ], 1);
			}
		}
		else if (sys == SYS_GAL) {
			pcv[i] = interpvar0(sat, nadir * R2D * 5.0, (*pcvs)[sat].var[i + 3 * NFREQ], 1);
			if (i == 2) {
				pcv[i] = interpvar0(sat, nadir * R2D * 5.0, (*pcvs)[sat].var[1 + 3 * NFREQ], 1);
			}
		}
		/*else if (sys == SYS_QZS) {
			pcv[i] = interpvar0(sat, nadir * R2D * 5.0, pcv->var[i + 4 * NFREQ], 1);
			if (i == 2) {
				pcv[i] = interpvar0(sat, nadir * R2D * 5.0, pcv->var[1 + 4 * NFREQ], 1);
			}
		}*/
	}
}

/* satellite antenna phase center variation ----------------------------------*/
static void cal_sat_PCV(int sat, map<int, vector<double>>* rs, const double* rr, map<int, PCV_t>* pcvs, double* pcv)
{
	double ru[3], rz[3], eu[3], ez[3], nadir, cosa;
	int i;

	for (i = 0; i < 3; i++) {
		ru[i] = rr[i] - (*rs)[sat][i];
		rz[i] = -(*rs)[sat][i];
	}
	if (!normv3(ru, eu) || !normv3(rz, ez)) { return; }

	cosa = dot(eu, ez, 3);
	cosa = cosa < -1.0 ? -1.0 : (cosa > 1.0 ? 1.0 : cosa);
	nadir = acos(cosa);

	antmodel_s(sat, pcvs, nadir, pcv);
}

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

	// 1.satellite positions and clocks
	cal_satpos(obs->eph, obs, n, navall, popt->eph_opt, &rs, &dts, &var, &svh);

	// 2.exclude measurements of eclipsing satellite (block IIA) 没搞懂
	test_eclipse(obs, n, navall, &rs);

	// 3.calculate satellite elevation
	calElev(sol, obs, n, &rs, sat_stat);

	for (auto it = obs->obssat.begin(); it != obs->obssat.end(); it++) {
		sat = it->first;

		cal_sat_PCV(sat, &rs, sol->rr, &(navall->sat_pcv), (*sat_stat)[sat].pcv);

		/* 【未完成】接收机PCV计算，如果不是公开站，则不计算 */
		//antmodel(sat, &opt->pcvr, opt->antdel, rtk->ssat[sat - 1].azel, 1, PPP_Glo.ssat_Ex[sat - 1].dantr);

		if (!model_phw(sol->time, sat, &(navall->sat_pcv), 2, &rs, sol->rr, sat_stat)) {
			continue;
		}

	}

	//for (i = 0; i < n; i++) {
	//	sat = obs[i].sat;
	//	for (j = 0; j < 3; j++) PPP_Glo.ssat_Ex[sat - 1].rs[j] = rs[j + i * 6];

	//	/* satellite and receiver antenna model */
	//	satantpcv(sat, rs + i * 6, PPP_Glo.rr, nav->pcvs + sat - 1, PPP_Glo.ssat_Ex[sat - 1].dants);
	//	antmodel(sat, &opt->pcvr, opt->antdel, rtk->ssat[sat - 1].azel, 1, PPP_Glo.ssat_Ex[sat - 1].dantr);

	//	/* phase windup model */
	//	if (!model_phw(rtk->sol.time, sat, nav->pcvs[sat - 1].type, 2, rs + i * 6, PPP_Glo.rr, &PPP_Glo.ssat_Ex[sat - 1].phw)) {
	//		continue;
	//	}
	//}

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