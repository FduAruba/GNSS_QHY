#include"gnss.h"

#define RE_GLO   6378136.0        /* radius of earth (m)            ref [2] */
#define MU_GPS   3.9860050E14     /* gravitational constant         ref [1] */
#define MU_GLO   3.9860044E14     /* gravitational constant         ref [2] */
#define MU_GAL   3.986004418E14   /* earth gravitational constant   ref [7] */
#define MU_CMP   3.986004418E14   /* earth gravitational constant   ref [9] */
#define J2_GLO   1.0826257E-3     /* 2nd zonal harmonic of geopot   ref [2] */

#define OMGE_GLO 7.292115E-5      /* earth angular velocity (rad/s) ref [2] */
#define OMGE_GAL 7.2921151467E-5  /* earth angular velocity (rad/s) ref [7] */
#define OMGE_CMP 7.292115E-5      /* earth angular velocity (rad/s) ref [9] */

#define SIN_5 -0.0871557427476582 /* sin(-5.0 deg) */
#define COS_5  0.9961946980917456 /* cos(-5.0 deg) */

#define ERREPH_GLO 5.0            /* error of glonass ephemeris (m) */
#define TSTEP    60.0             /* integration step glonass ephemeris (s) */
#define RTOL_KEPLER 1E-13         /* relative tolerance for Kepler equation */

#define DEFURASSR 0.15            /* default accurary of ssr corr (m) */
#define MAXECORSSR 10.0           /* max orbit correction of ssr (m) */
#define MAXCCORSSR (1E-6*CLIGHT)  /* max clock correction of ssr (m) */
#define MAXAGESSR 90.0            /* max age of ssr orbit and clock (s) */
#define MAXAGESSR_HRCLK 10.0      /* max age of ssr high-rate clock (s) */
#define STD_BRDCCLK 30.0          /* error of broadcast clock (m) */

#define MAX_ITER_KEPLER 30        /* max number of iteration of Kelpler */

/* variance by ura ephemeris (ref [1] 20.3.3.3.1.1) --------------------------*/
static double var_uraeph(int ura)
{
	const double ura_value[] = {
		2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
		3072.0,6144.0
	};
	return ura < 0 || 15 < ura ? SQR(6144.0) : SQR(ura_value[ura]);
}

/* glonass orbit differential equations --------------------------------------*/
static void deq(const double* x, double* xdot, const double* acc)
{
	double a, b, c, r2 = dot(x, x, 3), r3 = r2 * sqrt(r2), omg2 = SQR(OMGE_GLO);

	if (r2 <= 0.0) {
		xdot[0] = xdot[1] = xdot[2] = xdot[3] = xdot[4] = xdot[5] = 0.0;
		return;
	}
	/* ref [2] A.3.1.2 with bug fix for xdot[4],xdot[5] */
	a = 1.5 * J2_GLO * MU_GLO * SQR(RE_GLO) / r2 / r3; /* 3/2*J2*mu*Ae^2/r^5 */
	b = 5.0 * x[2] * x[2] / r2;                    /* 5*z^2/r^2 */
	c = -MU_GLO / r3 - a * (1.0 - b);                /* -mu/r^3-a(1-b) */
	xdot[0] = x[3]; xdot[1] = x[4]; xdot[2] = x[5];
	xdot[3] = (c + omg2) * x[0] + 2.0 * OMGE_GLO * x[4] + acc[0];
	xdot[4] = (c + omg2) * x[1] - 2.0 * OMGE_GLO * x[3] + acc[1];
	xdot[5] = (c - 2.0 * a) * x[2] + acc[2];
}

/* glonass position and velocity by numerical integration --------------------*/
static void glorbit(double t, double* x, const double* acc)
{
	double k1[6], k2[6], k3[6], k4[6], w[6];
	int i;

	deq(x, k1, acc); for (i = 0; i < 6; i++) { w[i] = x[i] + k1[i] * t / 2.0; }
	deq(w, k2, acc); for (i = 0; i < 6; i++) { w[i] = x[i] + k2[i] * t / 2.0; }
	deq(w, k3, acc); for (i = 0; i < 6; i++) { w[i] = x[i] + k3[i] * t; }
	deq(w, k4, acc);
	for (i = 0; i < 6; i++) { 
		x[i] += (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) * t / 6.0; 
	}
}

/* select ephememeris --------------------------------------------------------*/
static NavData_t* sel_eph(GpsTime_t time, int sat, int iode, NavPack_t* navall)
{
	/* 局部变量定义 ========================================================= */
	double t, tmax, tmin;			// 时间变量/最大时间/最小时间
	map<GpsTime_t, NavData_t>::iterator ite;
	/* ====================================================================== */

	switch (satsys(sat, NULL)) {
	case SYS_GAL: {tmax = MAXDTOE_GAL + 1.0; break; }
	case SYS_BDS: {tmax = MAXDTOE_CMP + 1.0; break; }
	default:	  {tmax = MAXDTOE_GPS + 1.0; break; }
	}
	tmin = tmax + 1.0;

	/* 1.检查当前星历是否有sat对应的星历数据 */
	if (navall->eph.find(sat) == navall->eph.end()) { return NULL; }
	ite = navall->eph[sat].end();

	for (auto it = navall->eph[sat].begin(); it != navall->eph[sat].end(); it++) {
		if (iode >= 0 && it->second.iode != iode) { continue; }
		if ((t = fabs(timediff(it->first, time))) > tmax) { continue; }
		if (iode > 0) { return &(it->second); }		// 如果输入了iode，则根据其返回星历
		if (t <= tmin) { ite = it; tmin = t; }		// 否则遍历查找时间最接近星历
	}
	if (iode >= 0 || ite == navall->eph[sat].end()) {
		printf("*** ERROR: no broadcast ephemeris: %s sat=%2d iode=%3d\n", time_str(time, 0), sat, iode);
		return NULL;
	}

	return &(ite->second);
}

/* select glonass ephememeris ------------------------------------------------*/
static NavDataGlo_t* sel_geph(GpsTime_t time, int sat, int iode, NavPack_t* navall)
{
	double t, tmax = MAXDTOE_GLO, tmin = tmax + 1.0;
	map<GpsTime_t, NavDataGlo_t>::iterator ite;

	/* 1.检查当前星历是否有sat对应的星历数据 */
	if (navall->geph.find(sat) == navall->geph.end()) { return NULL; }
	ite = navall->geph[sat].end();

	for (auto it = navall->geph[sat].begin(); it != navall->geph[sat].end(); it++) {
		if (iode >= 0 && it->second.iode != iode) { continue; }
		if ((t = fabs(timediff(it->first, time))) > tmax) { continue; }
		if (iode > 0) { return &(it->second); }		// 如果输入了iode，则根据其返回星历
		if (t <= tmin) { ite = it; tmin = t; }		// 否则遍历查找时间最接近星历
	}
	if (iode >= 0 || ite == navall->geph[sat].end()) {
		printf("*** ERROR: no glonass ephemeris  : %s sat=%2d iode=%2d\n", time_str(time, 0), sat, iode);
		return NULL;
	}

	return &(ite->second);
}

/* satellite clock with broadcast ephemeris ----------------------------------*/
static int ephclk(GpsTime_t time, GpsTime_t teph, int sat, NavPack_t* navall, double* dts)
{
	/* 局部变量定义 ========================================================= */
	NavData_t* eph;					// 星历变量
	NavDataGlo_t* geph;				// GLONASS星历变量
	int sys;						// 卫星系统
	/* ====================================================================== */

	sys = satsys(sat, NULL);

	if (sys == SYS_GPS || sys == SYS_GAL || sys == SYS_BDS) {
		if (!(eph = sel_eph(teph, sat, -1, navall))) { return 0; }
		*dts = eph2clk(time, eph);
	}
	else if (sys == SYS_GLO) {
		if (!(geph = sel_geph(teph, sat, -1, navall))) { return 0; }
		*dts = geph2clk(time, geph);
	}
	else { return 0; }

	return 1;
}

extern void eph2pos(GpsTime_t time, const NavData_t* eph, int sat,
	map<int, vector<double>>* rs, map<int, vector<double>>* dts, map<int, vector<double>>* var)
{
	double tk, M, E, Ek, sinE, cosE, u, r, i, O, sin2u, cos2u, x, y, sinO, cosO, cosi, mu, omge;
	double xg, yg, zg, sino, coso;
	int n, sys, prn;

	double tmp;

	if (eph->A <= 0.0) {
		(*rs)[sat][0] = (*rs)[sat][1] = (*rs)[sat][2] = 0.0;
		(*dts)[sat][0] = (*var)[sat][0] = 0.0;
		//rs[0] = rs[1] = rs[2] = *dts = *var = 0.0;
		return;
	}
	tk = timediff(time, eph->toe);

	switch ((sys = satsys(sat, &prn)))
	{
	case SYS_GAL: {mu = MU_GAL; omge = OMGE_GAL; break; }
	case SYS_BDS: {mu = MU_CMP; omge = OMGE_CMP; break; }
	default:	  {mu = MU_GPS; omge = OMGE;     break; }
	}
	M = eph->M0 + (sqrt(mu / (eph->A * eph->A * eph->A)) + eph->deln) * tk;

	for (n = 0, E = M, Ek = 0.0; fabs(E - Ek) > RTOL_KEPLER && n < MAX_ITER_KEPLER; n++) {
		Ek = E; 
		E -= (E - eph->e * sin(E) - M) / (1.0 - eph->e * cos(E));
	}
	if (n >= MAX_ITER_KEPLER) {
		printf(" ***ERROR: eph2pos: kepler iteration overflow sat = % 2d\n", sat);
		return;
	}
	sinE = sin(E); 
	cosE = cos(E);

	u = atan2(sqrt(1.0 - eph->e * eph->e) * sinE, cosE - eph->e) + eph->omg;
	r = eph->A * (1.0 - eph->e * cosE);
	i = eph->i0 + eph->idot * tk;
	sin2u = sin(2.0 * u); cos2u = cos(2.0 * u);
	u += eph->cus * sin2u + eph->cuc * cos2u;
	r += eph->crs * sin2u + eph->crc * cos2u;
	i += eph->cis * sin2u + eph->cic * cos2u;
	x = r * cos(u); y = r * sin(u); cosi = cos(i);

	/* beidou GEO satellite (ref [9]) */
	if (sys == SYS_BDS && prn <= 5) {
		O = eph->OMG0 + eph->OMGd * tk - omge * eph->toes;
		sinO = sin(O); cosO = cos(O);
		xg = x * cosO - y * cosi * sinO;
		yg = x * sinO + y * cosi * cosO;
		zg = y * sin(i);
		sino = sin(omge * tk); coso = cos(omge * tk);

		(*rs)[sat][0]= xg * coso + yg * sino * COS_5 + zg * sino * SIN_5;
		(*rs)[sat][1]= -xg * sino + yg * coso * COS_5 + zg * coso * SIN_5;
		(*rs)[sat][2]= -yg * SIN_5 + zg * COS_5;
	}
	else {
		O = eph->OMG0 + (eph->OMGd - omge) * tk - omge * eph->toes;
		sinO = sin(O); cosO = cos(O);
		(*rs)[sat][0] = x * cosO - y * cosi * sinO;
		(*rs)[sat][1] = x * sinO + y * cosi * cosO;
		(*rs)[sat][2] = y * sin(i);
	}

	tk = timediff(time, eph->toc);
	(*dts)[sat][0] = eph->f0 + eph->f1 * tk + eph->f2 * tk * tk;

	/* relativity correction */
	(*dts)[sat][0] -= 2.0 * sqrt(mu * eph->A) * eph->e * sinE / SQR(CLIGHT);

	/* position and clock error variance */
	(*var)[sat][0] = var_uraeph(eph->sva);
}

extern void geph2pos(GpsTime_t time, const NavDataGlo_t* geph, int sat,
	map<int, vector<double>>* rs, map<int, vector<double>>* dts, map<int, vector<double>>* var)
{
	double t, tt, x[6];
	int i;

	t = timediff(time, geph->toe);

	(*dts)[sat][0] = -geph->taun + geph->gamn * t;

	for (i = 0; i < 3; i++) {
		x[i]     = geph->pos[i];
		x[i + 3] = geph->vel[i];
	}
	for (tt = t < 0.0 ? -TSTEP : TSTEP; fabs(t) > 1E-9; t -= tt) {
		if (fabs(t) < TSTEP) { tt = t; }
		glorbit(tt, x, geph->acc);
	}

	for (i = 0; i < 3; i++) { (*rs)[sat][i] = x[i]; }

	(*var)[sat][0] = SQR(ERREPH_GLO);
}

/* satellite position and clock by broadcast ephemeris -----------------------*/
static int ephpos(GpsTime_t time, GpsTime_t teph, int sat, NavPack_t* nav, int iode,
	map<int, vector<double>>* rs, map<int, vector<double>>* dts, map<int, vector<double>>* var, map<int, vector<int>>* svh)
{
	/* 局部变量定义 ========================================================= */
	NavData_t* eph;							// 星历变量
	NavDataGlo_t* geph;						// GLONASS星历变量
	double tt = 1E-3;
	int i, sys;							// 循环遍历遍历/卫星系统
	map<int, vector<double>> rs_tmp = *(rs);
	map<int, vector<double>> dts_tmp = *(dts);
	/* ===================================================================== */

	sys = satsys(sat, NULL);

	(*svh)[sat][0] = -1;

	if (sys == SYS_GPS || sys == SYS_GAL || sys == SYS_BDS) {
		if (!(eph = sel_eph(teph, sat, iode, nav))) { return 0; }
		eph2pos(time, eph, sat, rs, dts, var);
		time = timeadd(time, tt);
		eph2pos(time, eph, sat, &rs_tmp, &dts_tmp, var);
		(*svh)[sat][0] = eph->svh;
	}
	else if (sys == SYS_GLO) {
		if (!(geph = sel_geph(teph, sat, iode, nav))) { return 0; }
		geph2pos(time, geph, sat, rs, dts, var);
		time = timeadd(time, tt);
		geph2pos(time, geph, sat, &rs_tmp, &dts_tmp, var);
		(*svh)[sat][0] = geph->svh;
	}
	else { return 0; }

	/* satellite velocity and clock drift by differential approx */
	for (i = 0; i < 3; i++) { 
		(*rs)[sat][i + 3] = (rs_tmp[sat][i] - (*rs)[sat][i]) / tt;
	}
	(*dts)[sat][1] = (dts_tmp[sat][0] - (*dts)[sat][0]) / tt;

	return 1;
}

extern int satpos(GpsTime_t time, GpsTime_t teph, int sat, int eph_opt, NavPack_t* nav,
	map<int, vector<double>>* rs, map<int, vector<double>>* dts, map<int, vector<double>>* var, map<int, vector<int>>* svh)
{
	(*svh)[sat][0] = 0;

	switch (eph_opt)
	{
	case EPHOPT_BRDC: { return ephpos(time, teph, sat, nav, -1, rs, dts, var, svh); }
	/*case EPHOPT_PREC: {
		if (!peph2pos(time, sat, nav, 1, rs, dts, var)) {
			break;
		}
		else { return 1; }
	}*/
	}
	(*svh)[sat][0] = -1;
	return 0;
}

extern void cal_satpos(GpsTime_t teph, ObsEphData_t* obs, int n, NavPack_t* navall, int eph_opt, 
	map<int,vector<double>>* rs, map<int, vector<double>>* dts, map<int, vector<double>>* var, map<int, vector<int>>* svh)
{
	/* 局部变量定义 ========================================================= */
	//GpsTime_t time[MAXOBS] = { {0} };		// GPS时间变量
	double dt, pr;							// 卫星钟差变量/伪距变量
	int i, j;								// 循环遍历变量
	map<int, ObsData_t>::iterator it;
	map<int, GpsTime_t> time;
	int sat;
	/* ====================================================================== */

	for (it = obs->obssat.begin(), pr = 0.0; it != obs->obssat.end(); it++) {
		/* search any psuedorange 查找3频内是至少有一个伪距非0 */
		for (j = 0, pr = 0.0; j < NFREQ; j++) {
			if ((pr = it->second.P[j]) != 0.0) { break; }
		}
		if (j >= NFREQ) {
			printf("*** WARNING: no pseudorange %s sat=%2d\n", time_str(obs->eph, 3), it->first);
			continue;
		}

		/* transmission time by satellite clock 计算卫星发送信号时间(接收时间time-伪距/光速) */
		sat = it->first;
		time[sat] = timeadd(obs->eph, -pr / CLIGHT);

		/* satellite clock bias by broadcast ephemeris 从星历获取卫星钟差 */
		if (!ephclk(time[sat], teph, sat, navall, &dt)) { continue; }
		time[sat] = timeadd(time[sat], -dt);

		/* satellite position and clock at transmission time 计算卫星位置/钟差 */
		if (!satpos(time[sat], teph, sat, eph_opt, navall, rs, dts, var, svh)) {
			printf("*** WARNING: no ephemeris %s sat=%2d\n", time_str(time[sat], 3), sat);
			continue;
		}
		/* if no precise clock available, use broadcast clock instead */
		if ((*dts)[sat][0] == 0.0) {
			if (!ephclk(time[sat], teph, sat, navall, &dt)) { continue; }
			(*dts)[sat][0] = dt;
			(*dts)[sat][1] = 0.0;
			(*var)[sat][0] = SQR(STD_BRDCCLK);
		}
	}
}