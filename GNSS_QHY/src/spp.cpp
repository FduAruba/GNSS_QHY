#include"gnss.h"

#define MAX(x,y)    ((x)>=(y)?(x):(y))

#define NX          (4+3)       /* # of estimated parameters */

#define MAXITR      30          /* max number of iteration for point pos */
#define ERR_ION     5.0         /* ionospheric delay std (m) */
#define ERR_TROP    3.0         /* tropspheric delay std (m) */
#define ERR_SAAS    0.3         /* saastamoinen model error std (m) */
#define ERR_BRDCI   0.5         /* broadcast iono model error factor */
#define ERR_CBIAS   0.3         /* code bias error std (m) */
#define REL_HUMI    0.7         /* relative humidity for saastamoinen model */

const double chisqr[100] = {    /* chi-sqr(n) (alpha=0.001) */
	10.8,13.8,16.3,18.5,20.5,22.5,24.3,26.1,27.9,29.6,
	31.3,32.9,34.5,36.1,37.7,39.3,40.8,42.3,43.8,45.3,
	46.8,48.3,49.7,51.2,52.6,54.1,55.5,56.9,58.3,59.7,
	61.1,62.5,63.9,65.2,66.6,68.0,69.3,70.7,72.1,73.4,
	74.7,76.0,77.3,78.6,80.0,81.3,82.6,84.0,85.4,86.7,
	88.0,89.3,90.6,91.9,93.3,94.7,96.0,97.4,98.7,100 ,
	101 ,102 ,103 ,104 ,105 ,107 ,108 ,109 ,110 ,112 ,
	113 ,114 ,115 ,116 ,118 ,119 ,120 ,122 ,123 ,125 ,
	126 ,127 ,128 ,129 ,131 ,132 ,133 ,134 ,135 ,137 ,
	138 ,139 ,140 ,142 ,143 ,144 ,145 ,147 ,148 ,149
};

/* validate solution ---------------------------------------------------------*/
static int valsol(map<int, vector<double>>* azel, vector<int>* idx, int n, const ProcOpt_t* popt, 
	vector<double>* v, int nv, int nx, Sol_t* sol)
{
	double azels[MAXOBS * 2], vv;
	int i, ns;
	int sat;
	map<int, vector<double>> azel_tmp;
	vector<double> dops = vector<double>(4, 0.0);
	
	/* 1.chi-square validation of residuals */
	vv = dot((*v), (*v), nv);
	//if (nv > nx && vv > chisqr[nv - nx - 1]) {
	//	printf("Warning: large chi-square error nv=%d vv=%.1f cs=%.1f\n", nv, vv, chisqr[nv - nx - 1]);
	//	/* return 0; */ /* threshold too strict for all use cases, report error but continue on */
	//}
	/* 2.large gdop check */
	for (i = ns = 0; i < (*idx).size(); i++) {
		if ((sat = (*idx)[i]) <= 0) { continue; }
		azel_tmp[sat].push_back((*azel)[sat][0]);
		azel_tmp[sat].push_back((*azel)[sat][1]);

		ns++;
	}

	dops = cal_dops(ns, &azel_tmp, popt->elmin);

	if (dops[0] <= 0.0 || dops[0] > popt->thres_GDOP) {
		printf("GDOP error nv=%d gdop=%.1f", nv, dops[0]);
		return 0;
	}
	if (dops[1] <= 0.0 || dops[1] > popt->thres_PDOP) {
		printf("PDOP error nv=%d pdop=%.1f", nv, dops[1]);
		return 0;
	}
	/* 3.save dops */
	for (int i = 0; i < 4; i++) { sol->dop[i] = dops[i]; }

	return 1;
}

/* pseudorange measurement error variance ------------------------------------*/
static double varerr(const ProcOpt_t* popt, double el, int sys, double snr_rover)
{
	double a, b, snr_max;
	double fact = popt->err[0];
	double sinel = sin(el);

	switch (sys)
	{
	case SYS_GLO: {fact *= EFACT_GLO; break; }
	case SYS_GAL: {fact *= EFACT_GAL; break; }
	case SYS_BDS: {fact *= EFACT_BDS; break; }
	default: {fact *= EFACT_GPS; break; }
	}

	a = fact * popt->err[1];
	b = fact * popt->err[2];
	snr_max = popt->err[4];

	/* note: SQR(3.0) is approximated scale factor for error variance
	   in the case of iono-free combination */
	fact = (popt->iono_opt == IONOOPT_IF12) ? SQR(3.0) : 1.0;

	switch (popt->wgt_opt)
	{
	case WEIGHTOPT_ELEVATION: {
		return fact * (SQR(a) + SQR(b / sinel));
	}
	case WEIGHTOPT_SNR: {
		return fact * SQR(a) * pow(10, 0.1 * MAX(snr_max - snr_rover, 0));
	}
	default: {
		return 0.0;
	}
	}
}

/* trospheric delay (m) ------------------------------------------------------*/
static int trop_corr(GpsTime_t time, const int sat, NavPack_t* navall, vector<double> pos,
	map<int, vector<double>>* azel, const ProcOpt_t* popt, double* dtrp, double* vtrp)
{
	/* 局部变量定义 ========================================================= */
	double trpw = 0.0;				// 电离层湿延迟(m)
	/* 局部变量定义 ========================================================= */
	*dtrp = 0.0;
	//pos[0] = 31.301470533; pos[1] = 121.500711419; pos[2] = 12.9655;

	/* saastamoinen model */
	if (popt->trop_opt == TROPOPT_SAAS || popt->trop_opt == TROPOPT_EST) {
		*dtrp = trop_model(time, sat, pos, azel, REL_HUMI, &trpw);
		*dtrp += trpw;
		*vtrp = SQR(ERR_SAAS / (sin((*azel)[sat][1]) + 0.1));

		if (*dtrp > 100.0) { *dtrp = 100.0; }
		else if (*dtrp < 0.05) { *dtrp = 0.05; }

		return 1;
	}

	/* no correction */
	*dtrp = 0.0;
	*vtrp = popt->trop_opt == TROPOPT_OFF ? 0.0 : SQR(ERR_TROP);

	return 1;
}

/* ionospheric delay (m) -----------------------------------------------------*/
static int iono_corr(GpsTime_t time, const int sat, NavPack_t* navall, vector<double> pos,
	map<int, vector<double>>* azel, const ProcOpt_t* popt, double* dion, double* vion)
{
	//pos[0] = 31.301470533; pos[1] = 121.500711419; pos[2] = 12.9655;

	/* 1.broadcast model 广播模型 */
	if (popt->iono_opt == IONOOPT_BRDC) {
		*dion = iono_model(time, sat, navall->ion_gps, pos, azel);
		*vion = SQR((*dion) * ERR_BRDCI);
		return 1;
	}

	*dion = 0.0;
	*vion = popt->iono_opt == IONOOPT_OFF ? SQR(ERR_ION) : 0.0;
	return 1;
}

/* get tgd parameter (m) -----------------------------------------------------*/
static double gettgd(int sat, NavPack_t* navall, double* tgd1, double* tgd2)
{
	int sys;
	map<int, map<GpsTime_t, NavData_t>>::iterator it;

	if ((it = navall->eph.find(sat)) == navall->eph.end()) { return 0.0; }
	if ((sys = satsys(sat, NULL)) == SYS_NONE) { return 0.0; }

	if (sys == SYS_BDS) {
		if (tgd1) { *tgd1 = CLIGHT * it->second.begin()->second.tgd[0]; }
		if (tgd2) { *tgd2 = CLIGHT * it->second.begin()->second.tgd[2]; }

		return CLIGHT * it->second.begin()->second.tgd[0];
	}

	return CLIGHT * it->second.begin()->second.tgd[0];
}

/* geometric distances (m) ---------------------------------------------------*/
static double prange(int sat, ObsEphData_t* obs, NavPack_t* navall, const ProcOpt_t* popt, double* var)
{
	double PC;
	double P1, P2;
	double P1_P2 = 0.0, P1_C1 = 0.0, P2_C2 = 0.0;
	double gamma;
	double tgd1, tgd2;
	int i = 0, j = 1, sys;
	vector<double> lam;
	map<string, double>::iterator it;

	if ((sys = satsys(sat, NULL)) == SYS_NONE) { return 0.0; }

	if ((navall->geph.find(sat) == navall->geph.end() && navall->eph.find(sat) == navall->eph.end()) ||
		obs->obssat.find(sat) == obs->obssat.end()) {
		return 0.0;
	}

	if (obs->obssat[sat].P[0] == 0.0) {
		//printf("sat=%d\n", sat);
		return 0.0;
	}

	*var = 0.0;
	lam = navall->lam[sat];

	gamma = SQR(lam[j]) / SQR(lam[i]);		/* f1^2/f2^2 */
	P1 = obs->obssat[sat].P[i];
	P2 = obs->obssat[sat].P[j];
	if (navall->dcb.find(sat) != navall->dcb.end()) {
		for (it = navall->dcb[sat].begin(); it != navall->dcb[sat].end(); it++) {
			if (it->first == "P1-P2") { P1_P2 = it->second; }
			else if (it->first == "P1-C1") { P1_C1 = it->second; }
			else if (it->first == "P2-C2") { P2_C2 = it->second; }
		}
	}

	if (P1 <= 100000) { return 0.0; }

	//P1_P2 = 0.0; P1_C1 = 0.0; P2_C2 = 0.0;
	/* 1.apply P1-C1,P2-C2 code corrections from DCB file */
	if (popt->iono_opt == IONOOPT_IF12 && P2 != 0.0) {
		if (P1 == 0.0 || P2 == 0.0) { return 0.0; }

		if (obs->obssat[sat].code[i] == CODE_L1C) { P1 += P1_C1; } /* C1->P1 */
		if (obs->obssat[sat].code[j] == CODE_L2C) { P2 += P2_C2; } /* C2->P2 */

		/* iono-free combination */
		PC = (gamma * P1 - P2) / (gamma - 1.0);  /* k1P1 - k2P2 */

		if (sys == SYS_BDS) {
			P1_P2 = gettgd(sat, navall, &tgd1, &tgd2);
			PC = PC + (tgd2 - gamma * tgd1) / (gamma - 1.0);
		}
	}
	/* 2.generate pseudo iono-free LC from L1 and P1-P2 code correction */
	else {
		/* if no P1-P2 DCB, use TGD from nav file instead */
		if (P1_P2 == 0.0 && (sys & (SYS_GPS | SYS_GAL | SYS_BDS))) {
			P1_P2 = (1.0 - gamma) * gettgd(sat, navall, &tgd1, &tgd2);
		}
		if (obs->obssat[sat].code[i] == CODE_L1C) { P1 += P1_C1; } /* C1->P1 */

		/* iono-free combination or P1 peeudorange with tgd corrected */
		PC = P1 - P1_P2 / (1.0 - gamma);
	}

	*var = SQR(ERR_CBIAS);

	return PC;
}

/* pseudorange residuals -----------------------------------------------------*/
static int rescode(int iter, ObsEphData_t* obs, int n, NavPack_t* navall, const ProcOpt_t* popt,
	map<int, vector<double>>* rs, map<int, vector<double>>* dts, map<int, vector<double>>* var_sat, map<int, vector<int>>* svh,
	vector<double>* x, vector<double>* v, vector<vector<double>>* H, vector<double>* var, map<int, vector<double>>* azel,
	map<int, vector<int>>* vsat, map<int, vector<double>>* resp, int* ns, vector<int>* idx, Sol_t* sol)
{
	double r, el, dion=0.0, dtrp=0.0, vmeas, vion=0.0, vtrp=0.0, dtr, P = 0.0, Pres = 0.0 ,lam_L1;
	double snr_rover = popt->err[4];
	int i, j, nv = 0, sys, mask[4] = { 0 }, sat;
	int stat;

	map<int, ObsData_t>::iterator it;
	vector<double>  rr = vector<double>(3, 0.0);
	vector<double> pos = vector<double>(3, 0.0);
	vector<double>   e = vector<double>(3, 0.0);

	(*v)   = vector<double>(n + NUMSYS, 0.0);
	(*var) = vector<double>(n + NUMSYS, 0.0);
	(*H)   = vector<vector<double>>(n + NUMSYS, vector<double>(NX, 0.0));
	(*idx) = vector<int>(n + NUMSYS, 0);

	/* 数据准备阶段，获取接收机上一历元的位置和钟差，将位置XYZ转为LLH */
	for (i = 0; i < 3; i++) { rr[i] = (*x)[i]; }
	pos = ecef2pos(rr);
	dtr = (*x)[3];

	if (popt->ieph == 302) { 
		int tt = 1; }
	/* 遍历当前历元obs，计算残差 */
	for (it = obs->obssat.begin(), *ns = 0; it != obs->obssat.end(); it++) {
		sat = it->first;
		(*vsat)[sat][0] = 0;
		(*azel)[sat][0] = (*azel)[sat][1] = 0.0;

		if ((sys = satsys(sat, NULL)) == SYS_NONE) { continue; }

		/* 1.geometric distance/azimuth/elevation angle */
		if ((r = geodist(sat, rs, rr, &e)) <= 0.0 ||
			(el = satazel(sat, pos, e, azel)) < popt->elmin) {
			continue;
		}

		if (sat == 25) {
			int bk = 1l;
		}
		/* 2.psudorange with code bias correction */
		if ((P = prange(sat, obs, navall, popt, &vmeas)) == 0.0) { 
			continue; 
		}

		/* 3.excluded satellite */
		if ((stat = sat_exclude(sat, var_sat, svh, popt)) == 1) { 
			continue; 
		}

		/* 4.ionospheric corrections */
		if ((stat = iono_corr(obs->eph, sat, navall, pos, azel, popt, &dion, &vion)) == 0) {
			continue;
		}
		/* GPS-L1 -> L1/B1 将GPS-L1电离层延迟转换成其他系统L1延迟 */
		if ((lam_L1 = navall->lam[sat][0]) > 0.0) {
			dion *= SQR(lam_L1 / lam_carr[0]);
		}

		//if (vion > 10) { dion = 0.0; vion = 0.0; }
		/* 5.tropospheric corrections */
		if ((stat = trop_corr(obs->eph, sat, navall, pos, azel, popt, &dtrp, &vtrp)) == 0) {
			continue;
		}

		/* 6.pseudorange residual */
		Pres = P - (r + dtr - CLIGHT * (*dts)[sat][0] + dion + dtrp);
		//if (popt->ieph > 1 && iter > 3 && fabs(Pres) >= 20.0) { continue; }
		(*v)[nv] = Pres;
		(*idx)[nv] = sat;

		/* 7.design matrix */
		for (j = 0; j < NX; j++) { (*H)[nv][j] = j < 3 ? -e[j] : (j == 3 ? 1.0 : 0.0); }

		/* 8.time system and receiver bias offset correction */
		if		(sys == SYS_GLO) { (*v)[nv] -= (*x)[4]; (*H)[nv][4] = 1.0; mask[1] = 1; }
		else if (sys == SYS_GAL) { (*v)[nv] -= (*x)[5]; (*H)[nv][5] = 1.0; mask[2] = 1; }
		else if (sys == SYS_BDS) { (*v)[nv] -= (*x)[6]; (*H)[nv][6] = 1.0; mask[3] = 1; }
		else { mask[0] = 1; }

		(*vsat)[sat][0] = 1; (*resp)[sat][0] = (*v)[nv]; (*ns)++;

		/* 9.error variance */
		(*var)[nv] = varerr(popt, (*azel)[sat][1], sys, snr_rover) + (*var_sat)[sat][0] + vmeas + vion + vtrp;
		
		if ((*azel)[sat][1] < popt->elmin) { (*var)[nv] *= 100.0; }

		/* 调试代码：输出每颗卫星每次迭代的各项误差 */
		if (sol->fp_itr) {
			fprintf(sol->fp_itr, "%03d az=%7.2f el=%7.2f r=%16.4f P=%16.4f dtr=%16.4f dts=%16.4f dion=%7.4f dtrp=%7.4f v=%16.4f var=%8.4f\n",
				sat, (*azel)[sat][0] * R2D, (*azel)[sat][1] * R2D, r, P, dtr, CLIGHT * (*dts)[sat][0], dion, dtrp, (*v)[nv], (*var)[nv]);
		}
		
		nv++;
	}

	for (i = 0; i < 4; i++) {
		if (mask[i]) {
			continue;
		}
		else {
			(*v)[nv] = 0.0;
			(*H)[nv][i + 3] = 1.0;
			(*var)[nv] = 0.01;
			(*idx)[nv] = (-1 - i);
			nv++;
		}
	}

	return nv;
}

/* estimate position ---------------------------------------------------------*/
static int estpos(ObsEphData_t* obs, int n, NavPack_t* navall, const ProcOpt_t* popt,
	map<int, vector<double>>* rs, map<int, vector<double>>* dts, map<int, vector<double>>* var_sat, map<int, vector<int>>* svh,
	Sol_t* sol, map<int, vector<double>>* azel, map<int, vector<int>>* vsat, map<int, vector<double>>* resp)
{
	double sig;
	int i, j, k, info, stat = 0, nv, ns;
	vector<double>         x = vector<double>(NX, 0.0);
	vector<double>        dx = vector<double>(NX, 0.0);
	vector<double>         v = vector<double>(n + NUMSYS, 0.0);
	vector<double>       var = vector<double>(n + NUMSYS, 0.0);
	vector<vector<double>> Q = vector<vector<double>>(NX, vector<double>(NX, 0.0));
	vector<vector<double>> H = vector<vector<double>>(n + NUMSYS, vector<double>(NX, 0.0));
	vector<int>          idx = vector<int>(n + NUMSYS, 0);

	vector<vector<double>>dx_t(NX, vector<double>(1, 0.0));
	vector<vector<double>>v_t(n + NUMSYS, vector<double>(1, 0.0));

	for (i = 0; i < 3; i++) { x[i] = sol->rr[i]; }
	x[3] = 0.0;

	if (sol->fp_itr) {
		fprintf(sol->fp_itr, "%s\n", time_str(obs->eph, 0));	// 调试代码：输出时间
	}
	
	for (i = 0; i < MAXITR; i++) {
		if (sol->fp_itr) {
			fprintf(sol->fp_itr, "iter=%d\n", i + 1);		   // 调试代码：输出迭代次数
		}
		/* 1.pseudorange residuals */
		nv = rescode(i, obs, n, navall, popt, rs, dts, var_sat, svh, &x, &v, &H, &var, azel, vsat, resp, &ns, &idx, sol);
		if (nv < NX) {
			printf("%s iepoch=%04d ***lack of valid sats ns=%d\n", time_str(obs->eph, 0), popt->ieph, ns);
			sol->n_les++;
			break;
		}
		
		//if (nv < n + 4) {	// 剔除多余行
		//	v.erase(v.begin() + nv, v.end());
		//	H.erase(H.begin() + nv, H.end());
		//	var.erase(var.begin() + nv, var.end());
		//	idx.erase(idx.begin() + nv, idx.end());
		//}

		/* 调试代码 */
		/*printf("sat=\n");
		for (int j = 0; j < nv; j++) { printf("%16d", idx[j]); } printf("\n");
		printf("v=\n");
		for (int j = 0; j < nv; j++) { printf("%16.4f", v[j]); } printf("\n");*/
		/*printf("var=\n");
		for (int j = 0; j < nv; j++) { printf("%16.4f", var[j]); } printf("\n");*/

		/* 2.weight by variance */
		for (j = 0; j < nv; j++) {
			sig = sqrt(var[j]);
			v[j] /= sig;
			for (k = 0; k < NX; k++) { H[j][k] /= sig; }
		}

		/* 3.least square estimation */
		if ((info = lsq(&H, &v, NX, nv, &dx, &Q)) == 0) {
			printf("***ERROR:lsq error info=%d\n", info);
			break;
		}

		/* 调试代码：计算后验残差并输出 */
		/*for (int ii = 0; ii < NX; ii++) { dx_t[ii][0] = dx[ii]; }
		for (int ii = 0; ii < nv; ii++) { v_t[ii][0] = v[ii]; }
		matmul("NN", nv, NX, 1, -1.0, &H, &dx_t, 1.0, &v_t);*/

		for (j = 0; j < NX; j++) { x[j] += dx[j]; }

		/* 4.residuals converged and save result to sol */
		if (norm(dx, NX) < 1E-4) {
			sol->type = 0;
			sol->time = timeadd(obs->eph, -x[3] / CLIGHT);
			sol->dtr[0] = x[3] / CLIGHT;	/* receiver clock bias (s) */
			sol->dtr[1] = x[4] / CLIGHT;	/* glo-gps time offset (s) */
			sol->dtr[2] = x[5] / CLIGHT;	/* gal-gps time offset (s) */
			sol->dtr[3] = x[6] / CLIGHT;	/* bds-gps time offset (s) */
			for (j = 0; j < 6; j++) { sol->rr[j] = j < 3 ? x[j] : 0.0; }
			for (j = 0; j < 3; j++) { sol->qr[j] = (float)Q[j][j]; }
			sol->qr[3] = (float)Q[0][1];    /* cov xy */
			sol->qr[4] = (float)Q[1][2];	/* cov yz */
			sol->qr[5] = (float)Q[0][2];    /* cov zx */
			sol->ns = (unsigned char)ns;
			sol->age = sol->ratio = 0.0;

			/* 5.validate solution */
			if ((stat = valsol(azel, &idx, n, popt, &v, nv, NX, sol)) == 1) {
				sol->stat = popt->eph_opt == EPHOPT_SBAS ? SOLQ_SBAS : SOLQ_SINGLE;
			}
			break;
		}
	}

	if (sol->fp_itr) {
		fprintf(sol->fp_itr, "\n");	// 调试代码
	}

	if (i >= MAXITR) { 
		printf("%s iepoch=%04d ***iteration divergent i=%d\n", time_str(obs->eph, 0), popt->ieph, i);
		sol->n_ite++;
	}

	x.clear(); dx.clear(); v.clear(); var.clear(); Q.clear(); idx.clear();
	
	return stat;
}

/* single point position -----------------------------------------------------*/
extern int spp(ObsEphData_t* obs, int n, NavPack_t* navall, const ProcOpt_t* popt, Sol_t* sol, map<int, Sat_t>* sat_stat)
{
	/* 局部变量定义 ========================================================= */
	int i;								// 循环遍历变量
	int sat, stat = 1; 					// 卫星号/状态标识符
	map<int, vector<int>> vsat;			// 卫星状态标记(1:ok  0:error)
	map<int, vector<int>> svh;			// 卫星健康标识符(0:ok -1:error)
	map<int, vector<double>> rs;		// 卫星坐标&速度(Px,Py,Pz,Vx,Vy,Vz)
	map<int, vector<double>> dts;		// 卫星钟差&钟漂(dts,ddts)
	map<int, vector<double>> var;		// 卫星定位方差
	map<int, vector<double>> azel;		// 卫星仰角&方位角(az,el)
	map<int, vector<double>> resp;		// 定位残差
	ProcOpt_t popt_spp = *popt;
	Sat_t sat_tmp = { 0 };
	unsigned char buff[4096] = { '\0' };
	unsigned char* p = buff;
	/* ====================================================================== */

	sol->stat = SOLQ_NONE;
	if (n < 4) { printf("sat number = %d<4\n", n); return 0; }
	else	   { sol->time = obs->eph; }
	
	// 初始化矩阵
	for (auto it = obs->obssat.begin(); it != obs->obssat.end(); it++) {
		rs  .insert(pair<int, vector<double>>(it->first, vector<double>(6, 0.0)));
		dts .insert(pair<int, vector<double>>(it->first, vector<double>(2, 0.0)));
		var .insert(pair<int, vector<double>>(it->first, vector<double>(1, 0.0)));
		azel.insert(pair<int, vector<double>>(it->first, vector<double>(2, 0.0)));
		resp.insert(pair<int, vector<double>>(it->first, vector<double>(1, 0.0)));
		svh. insert(pair<int, vector<int>>(it->first, vector<int>(1, -1)));
		vsat.insert(pair<int, vector<int>>(it->first, vector<int>(1, 0)));
	}

	popt_spp.eph_opt  = EPHOPT_BRDC;
	popt_spp.iono_opt = IONOOPT_BRDC;
	popt_spp.trop_opt = TROPOPT_SAAS;

	/* 1.satellite positons, velocities and clocks 计算卫星位置、速度、钟差 */
	cal_satpos(obs->eph, obs, n, navall, popt_spp.eph_opt, &rs, &dts, &var, &svh);

	/* 调试代码：输出卫星位置 */
	if (sol->fp_sat) {
		fprintf(sol->fp_sat, "%s ns=%02d\n", time_str(obs->eph, 0), rs.size());
		for (auto it = rs.begin(); it != rs.end(); it++) {
			int sat = it->first;
			fprintf(sol->fp_sat, "%03d %16.4f %16.4f %16.4f %16.4f\n",
				it->first, it->second[0], it->second[1], it->second[2], CLIGHT * dts[sat][0]);
		}
		fprintf(sol->fp_sat, "\n");
	}
	
	/* 2.estimate receiver position with pseudorange 通过伪距估计接收机位置 */
	stat = estpos(obs, n, navall, &popt_spp, &rs, &dts, &var, &svh, sol, &azel, &vsat, &resp);

	if (sat_stat) {
		for (auto it = vsat.begin(); it != vsat.end(); it++) {
			sat_tmp = { 0 };
;			sat = it->first;
			if ((it->second)[0] == 0) { continue; }
			
			sat_tmp.vs = 1;
			sat_tmp.azel[0] = azel[sat][0];
			sat_tmp.azel[1] = azel[sat][1];
			sat_tmp.resp_pos[0] = resp[sat][0];

			(*sat_stat)[sat] = sat_tmp;
		}
	}

	return stat;
}