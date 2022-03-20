#include"gnss.h"

#define NMAX        10              /* order of polynomial interpolation */
#define MAXDTE      900.0           /* max time difference to ephem time (s) */
#define MAXDTEC     30.0            /* max time difference to precise clock time (s) */
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

static void posWithEarthRotation(const int k, double pos[3], double p[3][NMAX + 1], double dt)
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

static int pephclk(GpsTime_t time, int sat, NavPack_t* navall, map<int, vector<double>>* dts, double* varc) 
{
	double t[2], c[2], std;
	int i, j, k, index, ic;
	double tt, t_tmp = 1000.0, n = 0.0;
	bool search_f = true, search_b = true;
	double t1, t2;
	GpsTime_t ta;
	map<GpsTime_t, PrecClk_t>::iterator it, ip, iq, ik, ikp;
	vector<map<GpsTime_t, PrecClk_t>::iterator> its;

	if (navall->pclk.find(sat) == navall->pclk.end()) { 
		return 0; 
	}

	// 判断当前精密时钟是能进行插值
	if (navall->pclk[sat].size() < 2 ||
		timediff(time, navall->pclk[sat].begin()->first) < -MAXDTEC ||
		timediff(time, navall->pclk[sat].rbegin()->first) > MAXDTEC) {
		return 1;
	}

	// 1.查找距离当前历元最近的时钟，用ik索引
	for (it = navall->pclk[sat].begin(), ik = iq = navall->pclk[sat].end(), iq--; n <= navall->pclk[sat].size() / 2;)
	{
		t1 = timediff(it->first, time); t2 = timediff(iq->first, time);

		if (t1 < 0.0 && search_f == true) {
			ta = timeadd(it->first, -(round(t1 / MAXDTEC) + n) * MAXDTEC);
			if (navall->pclk[sat].find(ta) != navall->pclk[sat].end()) {
				it = navall->pclk[sat].find(ta);
				search_f = false;
				if (fabs(timediff(it->first, time)) < t_tmp) {
					t_tmp = fabs(timediff(it->first, time));
					ik = it;
				}
			}
		}
		else { ik = iq = it; continue; }

		if (t2 > 0.0 && search_b == true) {
			ta = timeadd(iq->first, -(round(t2 / MAXDTEC) - n) * MAXDTEC);
			if (navall->pclk[sat].find(ta) != navall->pclk[sat].end()) {
				iq = navall->pclk[sat].find(ta);
				search_b = false;
				if (fabs(timediff(iq->first, time)) < t_tmp) {
					t_tmp = fabs(timediff(it->first, time));
					ik = iq;
				}
			}
		}
		else { ik = it = iq; continue; }

		if (search_f == true && search_b == true) { n++; }
		else { break; }
	}
	if (ik == navall->pclk[sat].end()) { return 0; }
	else if (ik == navall->pclk[sat].begin()) { ikp = ik; ikp++; }
	else { ikp = ik; ik--; }
	/*for (ic = 0, ip = it = navall->pclk[sat].begin(), ik = iq = navall->pclk[sat].end(), iq--;
		(it->first < iq->first) && ip != navall->pclk[sat].end(); ip++)
	{
		if (++ic >= 15) { break; }

		tt = timediff(ip->first, time);

		if (tt <= 0.0) {
			ta = timeadd(ip->first, -round(tt / MAXDTEC) * MAXDTEC);
			if (navall->pclk[sat].find(ta) != navall->pclk[sat].end()) {
				it = navall->pclk[sat].find(ta);
				if (fabs(timediff(it->first, time)) < t_tmp) {
					t_tmp = fabs(timediff(it->first, time));
					ik = ip = it;
				}
			}
		}
		else if (tt > 0.0) {
			ta = timeadd(ip->first, -round(tt / MAXDTEC) * MAXDTEC);
			if (navall->pclk[sat].find(ta) != navall->pclk[sat].end()) {
				iq = navall->pclk[sat].find(ta);
				if (fabs(timediff(iq->first, time)) < t_tmp) {
					t_tmp = fabs(timediff(it->first, time));
					ik = ip = iq;
				}
			}
		}
	}
	if (ik == navall->pclk[sat].end()) { return 0; }*/

	t[0] = timediff(time, ik->first);
	t[1] = timediff(time, ikp->first);
	c[0] = ik->second.clk;
	c[1] = ikp->second.clk;

	
	if (t[0] <= 0.0) {
		if (((*dts)[sat][0] = c[0]) == 0.0) { return 0; }
		std = ik->second.std * CLIGHT - EXTERR_CLK * t[0];
	}
	else if (t[1] >= 0.0) {
		if (((*dts)[sat][0] = c[1]) == 0.0) { return 0; }
		std = ikp->second.std * CLIGHT + EXTERR_CLK * t[1];
	}
	else if (c[0] != 0.0 && c[1] != 0.0) {
		(*dts)[sat][0] = (c[1] * t[0] - c[0] * t[1]) / (t[0] - t[1]);

		if (t[0] < -t[1]) {
			std = ik->second.std; i = 0;
		}
		else {
			std = ikp->second.std; i = 1;
		}

		if (std * CLIGHT > 0.05) {
			std = std + EXTERR_CLK * fabs(t[i]);
		}
		else {
			std = std * CLIGHT + EXTERR_CLK * fabs(t[i]);
		}
	}
	else { return 0; }

	if (varc) { *varc = SQR(std); }
	
	return 1;
}

/* satellite position by precise ephemeris -----------------------------------*/
static int peph2pos(GpsTime_t time, int sat, NavPack_t* navall,
	map<int, vector<double>>* rs, map<int, vector<double>>* dts, double* vare, double* varc)
{
	double t[NMAX + 1], p[3][NMAX + 1], c[2], * pos, std = 0.0, s[3], sinl;
	int i, j, k, index, sys, it_f = 1, ib_f = 1;
	int id[NMAX + 1], bBadClk, n;
	double tt, t_tmp = 1000.0;

	map<GpsTime_t, PrecNav_t>::iterator it, ib, ik, ikp;
	vector<map<GpsTime_t, PrecNav_t>::iterator> its;
	
	// 判断当前星历是否包含卫星
	if (navall->peph.find(sat) == navall->peph.end()) {	
		return 0;
	}
	// 判断当前星历是能进行插值
	if (navall->peph[sat].size() < NMAX + 1 ||
		timediff(time, navall->peph[sat].begin()->first) < -MAXDTE ||
		timediff(time, navall->peph[sat].rbegin()->first) > MAXDTE) {
		return 0;
	}
	// 1.查找距离当前历元最近的星历，用ik索引
	for (it = navall->peph[sat].begin(), ik = navall->peph[sat].end(); it != navall->peph[sat].end(); it++) {
		tt = timediff(it->first, time);
		if (fabs(tt) < t_tmp) {
			t_tmp = fabs(tt);
			ik = it;
		}
	}
	if (ik == navall->peph[sat].end()) {
		return 0;
	}
	// 2.查找ik附近10个历元的星历，并记录时间差和卫星位置
	for (it = ib = ik, --ib, n = 0;; ++it, --ib) {
		// 前向查找
		if (it_f) {
			if (it != navall->peph[sat].end()) {
				t[n] = timediff(it->first, time);
				pos = it->second.pos;
				if (norm(pos, 3) > 0.0) {
					posWithEarthRotation(n, pos, p, t[n]);
					its.push_back(it); 
					n++;
				}
				if (n == NMAX + 1) { break; }
			}
			else if (it == navall->peph[sat].end()) {
				it_f = 0;
			}
		}
		// 后向查找
		if (ib_f) {
			if (ib != navall->peph[sat].begin()) {
				t[n] = timediff(ib->first, time);
				pos = ib->second.pos;
				if (norm(pos, 3) > 0.0) {
					posWithEarthRotation(n, pos, p, t[n]);
					its.push_back(ib);
					n++;
				}
				if (n == NMAX + 1) { break; }
			}
			else if (ib == navall->peph[sat].begin()) {
				ib_f = 0;
				t[n] = timediff(ib->first, time);
				pos = ib->second.pos;
				if (norm(pos, 3) > 0.0) {
					posWithEarthRotation(n, pos, p, t[n]);
					its.push_back(ib);
					n++;
				}
				if (n == NMAX + 1) { break; }
			}
		}
		// 如果前/后向均超过索引范围，跳出循环
		if (it_f == 0 && ib_f == 0) { break; }
	}
	if (n <= NMAX) { return 0; }

	//for (k = 0, it = ib = ik, ib--; it != navall->peph[sat].end(); it++, ib--) {
	//	// 前向未到开始位置
	//	if (ib != navall->peph[sat].begin() && k <= NMAX) {
	//		t[k] = timediff(ib->first, time);
	//		pos = ib->second.pos;
	//		if (norm(pos, 3) > 0.0) {
	//			posWithEarthRotation(k, pos, p, t[k]);
	//			its.push_back(ib);
	//			k++;
	//		}
	//	}
	//	// 前向已到开始位置
	//	else if (ib == navall->peph[sat].begin() && k <= NMAX) {
	//		t[k] = timediff(ib->first, time);
	//		pos = ib->second.pos;
	//		if (norm(pos, 3) > 0.0) {
	//			posWithEarthRotation(k, pos, p, t[k]);
	//			its.push_back(ib);
	//			k++;
	//		}
	//		break;
	//	}
	//	if (k == NMAX + 1) { break; }
	//	// 后向未到结束位置
	//	if (it != navall->peph[sat].end() && k <= NMAX) {
	//		t[k] = timediff(it->first, time);
	//		pos = it->second.pos;
	//		if (norm(pos, 3) > 0.0) {
	//			posWithEarthRotation(k, pos, p, t[k]);
	//			its.push_back(it);
	//			k++;
	//		}
	//	}
	//	if (k == NMAX + 1) { break; }
	//}
	//if (k <= NMAX) { return 0; }

	// 3.根据时间差t由小到大排序，并采用容器交换的方法拓展至位置pos和指针its
	for (i = 0; i <= NMAX; i++) {
		for (j = i + 1; j <= NMAX; j++) {
			if (t[i] <= t[j]) { continue; }

			sinl = t[j]; t[j] = t[i]; t[i] = sinl;
			k = id[j]; id[j] = id[i]; id[i] = k;
			it = its[j]; its[j] = its[i]; its[i] = it;
			for (k = 0; k < 3; k++) {
				sinl = p[k][j];
				p[k][j] = p[k][i];
				p[k][i] = sinl;
			}
		}
	}
	// 获取时间差最大的星历指针
	ik = fabs(t[0]) >= fabs(t[NMAX]) ? its[0] : its[NMAX];
	
	if (t[0] > 900.0 || t[NMAX] < -900.0) { return 0; }

	// 4.Neville插值法，计算10阶多项式插值
	for (i = 0; i < 3; i++) { (*rs)[sat][i] = interppol(t, p[i], NMAX + 1); }

	// 5.计算位置插值误差
	if (vare) {
		for (i = 0; i < 3; i++) { s[i] = ik->second.std[i]; }
		std = norm(s, 3);

		// 轨道外插，需要增加额外误差
		if		(t[0] > 0.0)    { std += EXTERR_EPH * SQR(t[0]) / 2.0; }
		else if (t[NMAX] < 0.0) { std += EXTERR_EPH * SQR(t[NMAX]) / 2.0; }

		*vare = SQR(std);
	}
	/* 6.计算卫星钟差插值误差 */
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

	if (varc) { *varc = SQR(std); }
	
	return 1;
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
	double rss[3], dtst[1], dant[3] = { 0 }, vare = 0.0, varc = 0.0, tt = 1E-3;
	int i;

	map<int, vector<double>> rs_tmp = (*rs);
	map<int, vector<double>> dts_tmp = (*dts);
	vector<double> rs_p, rs_v;

	if (sat <= 0 || MAXSAT < sat) { return 0; }

	/* satellite position and clock bias */
	if (!peph2pos(time, sat, navall, rs, dts, &vare, &varc) || 
		!pephclk(time, sat, navall, dts, &varc)) {
		return 0; 
	}

	time = timeadd(time, tt);
	if (!peph2pos(time, sat, navall, &rs_tmp, &dts_tmp, NULL, NULL) ||
		!pephclk(time, sat, navall, &dts_tmp, NULL)) {
		return 0;
	}

	/* satellite antenna offset correction */
	if (opt) { sat_pco(time, sat, rs, navall, dant); }

	for (i = 0; i < 3; i++) {
		(*rs)[sat][i] += dant[i];
		(*rs)[sat][i + 3] = (rs_tmp[sat][i] - (*rs)[sat][i]) / tt;
	}
	/* relativistic effect correction */
	if ((*dts)[sat][0] != 0.0) {
		for (int i = 0; i < 3; i++) { 
			rs_p.push_back((*rs)[sat][i]);
			rs_v.push_back((*rs)[sat][i+3]);
		}
		(*dts)[sat][1] = (dts_tmp[sat][0] - (*dts)[sat][0]) / tt;
		(*dts)[sat][0] -= 2.0 * dot(rs_p, rs_v, 3) / CLIGHT / CLIGHT;
	}
	else {   /* no precise clock */
		(*dts)[sat][0] = (*dts)[sat][1] = 0.0;
	}

	(*var)[sat][0] = vare + varc;

	return 1;
}