#include"gnss.h"

#define NMAX        10              /* order of polynomial interpolation */
#define MAXDTE      900.0           /* max time difference to ephem time (s) */
#define EXTERR_CLK  1E-3            /* extrapolation error for clock (m/s) */
#define EXTERR_EPH  5E-7            /* extrapolation error for ephem (m/s^2) */

/* polynomial interpolation by Neville's algorithm ---------------------------*/
static double interppol(const double* x, double* y, int n)
{
	int i, j;

	for (j = 1; j < n; j++) {
		for (i = 0; i < n - j; i++) {
			y[i] = (x[i + j] * y[i] - x[i] * y[i + 1]) / (x[i + j] - x[i]);
}
	}
	return y[0];
}

static void posWithEarhRotation(const int k, double pos[3], double p[3][NMAX + 1], double dt)
{
	double sinl, cosl;
#if 0
	p[0][k] = pos[0];
	p[1][k] = pos[1];
#else
	/* correciton for earh rotation ver.2.4.0 */
	sinl = sin(OMGE * dt);
	cosl = cos(OMGE * dt);
	p[0][k] = cosl * pos[0] - sinl * pos[1];
	p[1][k] = sinl * pos[0] + cosl * pos[1];
#endif
	p[2][k] = pos[2];
}

/* satellite position by precise ephemeris -----------------------------------*/
static int peph2pos(GpsTime_t time, int sat, NavPack_t* navall,
	map<int, vector<double>>* rs, map<int, vector<double>>* dts, double* vare, double* varc)
{
	double t[NMAX + 1], p[3][NMAX + 1], c[2], * pos, std = 0.0, s[3], sinl;
	int i, j, k, index, sys;
	int id[NMAX + 1], kInd, bBadClk;

	//GpsTime_t tk;
	map<GpsTime_t, PrecNav_t>::iterator it, ib, ik, ikp;
	//map<GpsTime_t, PrecNav_t>::reverse_iterator ir;
	double tt, t_tmp = 1000.0;
	
	//rs[0] = rs[1] = rs[2] = dts[0] = 0.0;

	if (navall->peph.find(sat) == navall->peph.end()) { return 0; }

	if (navall->peph[sat].size() < NMAX + 1 ||
		timediff(time, navall->peph[sat].begin()->first) < -MAXDTE ||
		timediff(time, navall->peph[sat].rbegin()->first) > MAXDTE) {
		return 0;
	}

	for (it = navall->peph[sat].begin(), ik = navall->peph[sat].end(); it != navall->peph[sat].end(); it++) {
		tt = timediff(it->first, time);
		if (fabs(tt) < t_tmp) {
			t_tmp = fabs(tt);
			ik = it;
		}
	}
	if (ik == navall->peph[sat].end()) { return 0; }

	for (k = 0, it = ib = ik, ib--; it != navall->peph[sat].end(); it++, ib--) {
		// 前向未到开始位置
		if (ib != navall->peph[sat].begin() && k <= NMAX) {
			t[k] = timediff(ib->first, time);
			pos = ib->second.pos;
			if (norm(pos, 3) > 0.0) {
				posWithEarhRotation(k, pos, p, t[k]);
				k++;
			}
		}
		// 前向已到开始位置
		else if (ib == navall->peph[sat].begin() && k <= NMAX) {
			t[k] = timediff(ib->first, time);
			pos = ib->second.pos;
			if (norm(pos, 3) > 0.0) {
				posWithEarhRotation(k, pos, p, t[k]);
				k++;
			}
			break;
		}
		if (k == NMAX + 1) { break; }

		// 后向未到结束位置
		if (it != navall->peph[sat].end() && k <= NMAX) {
			t[k] = timediff(it->first, time);
			pos = it->second.pos;
			if (norm(pos, 3) > 0.0) {
				posWithEarhRotation(k, pos, p, t[k]);
				k++;
			}
		}
		if (k == NMAX + 1) { break; }
	}
	if (k <= NMAX) { return 0; }

	/* 根据时间差t由小到大排序，采用容器交换的方法 */
	for (i = 0; i <= NMAX; i++) {
		for (j = i + 1; j <= NMAX; j++) {
			if (t[i] <= t[j]) { continue; }

			sinl = t[j];  t[j] = t[i];   t[i] = sinl;
			k   = id[j]; id[j] = id[i]; id[i] = k;
			for (k = 0; k < 3; k++) {
				sinl    = p[k][j];
				p[k][j] = p[k][i];
				p[k][i] = sinl;
			}
		}
	}
	if (t[0] > 900.0 || t[NMAX] < -900.0) { return 0; }

	/* Neville插值法，计算10阶多项式插值 */
	for (i = 0; i < 3; i++) { (*rs)[sat][i] = interppol(t, p[i], NMAX + 1); }

	/* 计算位置插值误差 */
	if (vare) {
		for (i = 0; i < 3; i++) { s[i] = ik->second.std[i]; }
		std = norm(s, 3);

		/* extrapolation error for orbit */
		if		(t[0] > 0.0)    { std += EXTERR_EPH * SQR(t[0]) / 2.0; }
		else if (t[NMAX] < 0.0) { std += EXTERR_EPH * SQR(t[NMAX]) / 2.0; }
		*vare = SQR(std);
	}
	/* linear interpolation for clock */
	if (varc) {
		ikp = ik; ikp++;

		t[0] = timediff(time, ik->first);
		t[1] = timediff(time, ikp->first);
		c[0] = ik->second.pos[3];
		c[1] = ikp->second.pos[3];

		bBadClk = 0;
		if (t[0] <= 0.0) {
			if (((*dts)[sat][0] = c[0]) == 0.0) { bBadClk = 1; }
			std = ik->second.std[3] * CLIGHT - EXTERR_CLK * t[0];
		}
		else if (t[1] >= 0.0) {
			if (((*dts)[sat][0] = c[1]) == 0.0) { bBadClk = 1; }
			std = ikp->second.std[3] * CLIGHT * CLIGHT + EXTERR_CLK * t[1];
		}
		else if (c[0] != 0.0 && c[1] != 0.0) {
			(*dts)[sat][0] = (c[1] * t[0] - c[0] * t[1]) / (t[0] - t[1]);
			if (t[0] < -t[1]) {
				std = ik->second.std[3] + EXTERR_CLK * fabs(t[i]);
			}
			else {
				std = ikp->second.std[3] + EXTERR_CLK * fabs(t[i]);
			}
		}
		else { bBadClk = 1; }

		*varc = SQR(std);
	}
	
	return 1;

	/*if (nav->ne < NMAX + 1 || timediff(time, nav->peph[0].time) < -MAXDTE ||
		timediff(time, nav->peph[nav->ne - 1].time) > MAXDTE) {
		return 0;
	}*/

	///* binary search 二分法搜索时间最接近的精密星历 */
	//for (i = 0, j = nav->ne - 1; i < j;) {
	//	k = (i + j) / 2;
	//	if (timediff(nav->peph[k].time, time) < 0.0) { i = k + 1; }
	//	else { j = k; }
	//}
	//index = i <= 0 ? 0 : i - 1;	// 历元索引

	///* polynomial interpolation for orbit */
	//i = index - (NMAX + 1) / 2;
	//if (i < 0) { i = 0; }
	//else if (i + NMAX >= nav->ne) { i = nav->ne - NMAX - 1; }

	/* Neville插值法，搜索在index前后10个点，并记录其在peph中的位置到id[]中 */
	//for (j = k = 0; j < NMAX * 50; j++) {
	//	// index 时间后的点
	//	if (index + j >= 0 && index + j < nav->ne && k <= NMAX) {
	//		id[k] = index + j;
	//		t[k] = timediff(nav->peph[id[k]].time, time);
	//		pos = nav->peph[id[k]].pos[sat - 1];

	//		if (norm(pos, 3) > 0.0) {
	//			posWithEarhRotation(k, pos, p, t[k]);
	//			k++;
	//		}
	//	}
	//	if (k == NMAX + 1) { break; }
	//	// index 时间前的点
	//	if (index - j >= 0 && index - j < nav->ne && k <= NMAX && j != 0) {
	//		id[k] = index - j;
	//		t[k] = timediff(nav->peph[id[k]].time, time);
	//		pos = nav->peph[id[k]].pos[sat - 1];

	//		if (norm(pos, 3) > 0.0) {
	//			posWithEarhRotation(k, pos, p, t[k]);
	//			k++;
	//		}
	//	}
	//	if (k == NMAX + 1) { break; }
	//}

	//if (k <= NMAX) { return 0; }
	///* 根据时间差t由小到大排序，采用容器交换的方法 */
	//for (i = 0; i <= NMAX; i++) {
	//	for (j = i + 1; j <= NMAX; j++) {
	//		if (t[i] <= t[j]) { continue; }

	//		sinl = t[j];  t[j] = t[i];   t[i] = sinl;
	//		k = id[j]; id[j] = id[i]; id[i] = k;
	//		for (k = 0; k < 3; k++) {
	//			sinl = p[k][j];
	//			p[k][j] = p[k][i];
	//			p[k][i] = sinl;
	//		}
	//	}
	//}
	///* 查找时间差最小的插值点 */
	//kInd = 0;
	//for (i = 0; i <= NMAX; i++) {
	//	if (fabs(t[kInd]) <= fabs(t[i])) kInd = i;
	//}
	//index = id[kInd];

	/*if (t[0] > 900.0 || t[NMAX] < -900.0) {
		sprintf(PPP_Glo.chMsg, "%s t[0]=%-5.1f t[%d]=%-5.1f\n", PPP_Glo.sFlag[sat - 1].id, t[0], NMAX, t[NMAX]);
		outDebug(0, 0, 0);
		return 0;
	}*/
	///* Neville插值法，计算10阶多项式插值 */
	//for (i = 0; i < 3; i++) {
	//	rs[i] = interppol(t, p[i], NMAX + 1);
	//}
	/* 计算位置插值误差 */
	//if (vare) {
	//	for (i = 0; i < 3; i++) { s[i] = nav->peph[index].std[sat - 1][i]; }
	//	std = norm(s, 3);

	//	/* extrapolation error for orbit */
	//	if (t[0] > 0.0) { std += EXTERR_EPH * SQR(t[0]) / 2.0; }
	//	else if (t[NMAX] < 0.0) { std += EXTERR_EPH * SQR(t[NMAX]) / 2.0; }
	//	*vare = SQR(std);
	//}
	///* linear interpolation for clock */
	//t[0] = timediff(time, nav->peph[index].time);
	//t[1] = timediff(time, nav->peph[index + 1].time);
	//c[0] = nav->peph[index].pos[sat - 1][3];
	//c[1] = nav->peph[index + 1].pos[sat - 1][3];

	//bBadClk = 0;
	//if (t[0] <= 0.0) {
	//	if ((dts[0] = c[0]) == 0.0) { bBadClk = 1; }
	//	std = nav->peph[index].std[sat - 1][3] * CLIGHT - EXTERR_CLK * t[0];
	//}
	//else if (t[1] >= 0.0) {
	//	if ((dts[0] = c[1]) == 0.0) bBadClk = 1;
	//	std = nav->peph[index + 1].std[sat - 1][3] * CLIGHT + EXTERR_CLK * t[1];
	//}
	//else if (c[0] != 0.0 && c[1] != 0.0) {
	//	dts[0] = (c[1] * t[0] - c[0] * t[1]) / (t[0] - t[1]);
	//	i = t[0] < -t[1] ? 0 : 1;
	//	std = nav->peph[index + i].std[sat - 1][3] + EXTERR_CLK * fabs(t[i]);
	//}
	//else {
	//	bBadClk = 1;
	//}
	//if (varc) { *varc = SQR(std); }

	//sys = PPP_Glo.sFlag[sat - 1].sys;
	//if (sys == SYS_CMP && bBadClk) { return 1; }

	//if (bBadClk) {
	//	//return 0;
	//}
	//return 1;
}
/* satellite position/clock by precise ephemeris/clock -------------------------
* compute satellite position/clock with precise ephemeris/clock
* args   : gtime_t time       I   time (gpst)
*          int    sat         I   satellite number
*          nav_t  *nav        I   navigation data
*          int    opt         I   sat postion option
*                                 (0: center of mass, 1: antenna phase center)
*          double *rs         O   sat position and velocity (ecef)
*                                 {x,y,z,vx,vy,vz} (m|m/s)
*          double *dts        O   sat clock {bias,drift} (s|s/s)
*          double *var        IO  sat position and clock error variance (m)
*                                 (NULL: no output)
* return : status (1:ok,0:error or data outage)
* notes  : clock includes relativistic correction but does not contain code bias
*          before calling the function, nav->peph, nav->ne, nav->pclk and
*          nav->nc must be set by calling readsp3(), readrnx() or readrnxt()
*          if precise clocks are not set, clocks in sp3 are used instead
*-----------------------------------------------------------------------------*/
extern int peph_pos(GpsTime_t time, int sat, NavPack_t* navall, int opt,
	map<int, vector<double>>* rs, map<int, vector<double>>* dts, map<int, vector<double>>* var)
{
	double rss[3], rst[3], dtss[1], dtst[1], dant[3] = { 0 }, vare = 0.0, varc = 0.0, tt = 1E-3;
	int i;

	if (sat <= 0 || MAXSAT < sat) { return 0; }

	if (!peph2pos(time, sat, navall, rs, dts, &vare, &varc)) { return 0; }



	///* satellite position and clock bias */
	//if (!peph2pos(time, sat, navall, rs, dts, &vare, &varc) || !pephclk(time, sat, nav, dtss, &varc)) { return 0; }

	//time = timeadd(time, tt);
	//if (!pephpos(time, sat, nav, rst, dtst, NULL, NULL) || !pephclk(time, sat, nav, dtst, NULL)) { return 0; }

	///* satellite antenna offset correction */
	//if (opt) {
	//	satantoff(time, rss, sat, nav, dant);
	//}

	//for (i = 0; i < 3; i++) {
	//	rs[i] = rss[i] + dant[i];
	//	rs[i + 3] = (rst[i] - rss[i]) / tt;
	//}
	///* relativistic effect correction */
	//if (dtss[0] != 0.0) {
	//	dts[0] = dtss[0] - 2.0 * dot(rs, rs + 3, 3) / CLIGHT / CLIGHT;
	//	dts[1] = (dtst[0] - dtss[0]) / tt;
	//}
	//else    /* no precise clock */
	//	dts[0] = dts[1] = 0.0;

	//*var = vare + varc;

	return 1;
}