#include"gnss.h"
#include<exception>

#define SQRT(x)     ((x)<0.0||(x)!=(x)?0.0:sqrt(x))
#define MAX_VAR_EPH SQR(300.0)  /* max variance eph to reject satellite (m^2) */

static const double gpst0[] = { 1980,1, 6,0,0,0 }; /* gps time reference */
static const double bdt0[]  = { 2006,1, 1,0,0,0 }; /* beidou time reference */


static const char* obscodes[] = {       /* observation code strings */

	""  ,"1C","1P","1W","1Y", "1M","1N","1S","1L","1E", /*  0- 9 */
	"1A","1B","1X","1Z","2C", "2D","2S","2L","2X","2P", /* 10-19 */
	"2W","2Y","2M","2N","5I", "5Q","5X","7I","7Q","7X", /* 20-29 */
	"6A","6B","6C","6X","6Z", "6S","6L","8L","8Q","8X", /* 30-39 */
	"2I","2Q","6I","6Q","3I", "3Q","3X","1I","1Q","5A"  /* 40-49 */
	"5B","5C","9A","9B","9C", "9X",""  ,""  ,""  ,""    /* 50-59 */
};

static unsigned char obsfreqs[] = {
	/* 1:L1/E1, 2:L2/B1, 3:L5/E5a/L3, 4:L6/LEX/B3, 5:E5b/B2, 6:E5(a+b), 7:S */
	0, 1, 1, 1, 1,  1, 1, 1, 1, 1, /*  0- 9 */
	1, 1, 1, 1, 2,  2, 2, 2, 2, 2, /* 10-19 */
	2, 2, 2, 2, 3,  3, 3, 5, 5, 5, /* 20-29 */
	4, 4, 4, 4, 4,  4, 4, 6, 6, 6, /* 30-39 */
	2, 2, 4, 4, 3,  3, 3, 1, 1, 3, /* 40-49 */
	3, 3, 7, 7, 7,  7, 0, 0, 0, 0  /* 50-59 */
};

static char codepris[4][MAXFREQ][16] = {  /* code priority table */

   /* L1/E1      L2/B1        L5/E5a/L3 L6/LEX/B3 E5b/B2    E5(a+b)  S */
	{"CPYWMNSL","PYWCMNDSLX","IQX"     ,""       ,""       ,""      ,""    }, /* GPS */
	{"PC"      ,"PC"        ,"IQX"     ,""       ,""       ,""      ,""    }, /* GLO */
	{"CABXZ"   ,""          ,"IQX"     ,"ABCXZ"  ,"IQX"    ,"IQX"   ,""    }, /* GAL */
	{"IQX"     ,"IQX"       ,"IQX"     ,"IQX"    ,"IQX"    ,""      ,""    }, /* BDS */
};

static double leaps[MAXLEAPS + 1][7] = { /* leap seconds (y,m,d,h,m,s,utc-gpst) */
	{2017,1,1,0,0,0,-18},
	{2015,7,1,0,0,0,-17},
	{2012,7,1,0,0,0,-16},
	{2009,1,1,0,0,0,-15},
	{2006,1,1,0,0,0,-14},
	{1999,1,1,0,0,0,-13},
	{1997,7,1,0,0,0,-12},
	{1996,1,1,0,0,0,-11},
	{1994,7,1,0,0,0,-10},
	{1993,7,1,0,0,0, -9},
	{1992,7,1,0,0,0, -8},
	{1991,1,1,0,0,0, -7},
	{1990,1,1,0,0,0, -6},
	{1988,1,1,0,0,0, -5},
	{1985,7,1,0,0,0, -4},
	{1983,7,1,0,0,0, -3},
	{1982,7,1,0,0,0, -2},
	{1981,7,1,0,0,0, -1},
	{0}
};

/* functions ---------------------------------------------------------------------*/

extern void setstr(char* dst, const char* src, int n) 
{
	char*       p = dst;
	const char* q = src;

	while (*q && q < src + n) {
		*p++ = *q++;
	}
	*p-- = '\0';
	while (p >= dst && *p == ' ') {
		*p-- = '\0';
	}
}

extern double str2num(const char* s, int i, int n) 
{
	double value;
	char str[256];
	char* p = str;

	if ((i < 0) || ((int)strlen(s) < i) || ((int)sizeof(str) - 1 < n)) {
		return 0.0;
	}
	for (s += i; *s && --n >= 0; s++) { 
		*p++ = (*s == 'd' || *s == 'D') ? 'E' : *s;
	}
	*p = '\0';
	if (sscanf_s(str, "%lf", &value) == 1) { return value; }
	else								   { return 0.0; }
}

extern int str2time(const char* s, int i, int n, GpsTime_t* t) 
{
	/* 局部变量定义 ======================================= */
	double ep[6];					// 历元时刻
	char str[256], * p = str;		// 字符串变量/字符串指针
	/* ==================================================== */

	// 如果字符串异常，返回-1
	if (i < 0 || (int)strlen(s) < i || (int)sizeof(str) - 1 < i) { return -1; }
	// 字符串复制
	for (s += i; *s && --n >= 0;) { *p++ = *s++; }
	*p = '\0';
	// 将str中的字符转存至ep中，若出错，返回-1
	if (sscanf(str, "%lf %lf %lf %lf %lf %lf", ep, ep + 1, ep + 2, ep + 3, ep + 4, ep + 5) < 6) {
		return -1;
	}
	// 将ep中的数字转成gpst格式
	if (ep[0] < 100.0) { ep[0] += ep[0] < 80.0 ? 2000.0 : 1900.0; };
	*t = epoch2time(ep);
	return 0;
}

extern void time2str(GpsTime_t t, char* s, int n)
{
	double ep[6];

	if (n < 0) { n = 0; }
	else if (n > 12) { n = 12; }
	if (1.0 - t.sec < 0.5 / pow(10.0, n)) { t.time++; t.sec = 0.0; };
	time2epoch(t, ep);
	sprintf(s, "%04.0f/%02.0f/%02.0f %02.0f:%02.0f:%0*.*f\0",
		ep[0], ep[1], ep[2], ep[3], ep[4], n <= 0 ? 2 : n + 3, n <= 0 ? 0 : n, ep[5]);
}

extern char* time_str(GpsTime_t t, int n)
{
	static char buff[64];
	time2str(t, buff, n);
	return buff;
}

extern GpsTime_t epoch2time(const double* ep) 
{
	const int doy[] = { 1,32,60,91,121,152,182,213,244,274,305,335 };
	GpsTime_t time = { 0 };
	int days, sec, year = (int)ep[0], mon = (int)ep[1], day = (int)ep[2];

	if (year < 1970 || 2099 < year || mon < 1 || 12 < mon) { return time; }

	/* leap year if year%4==0 in 1901-2099 */
	days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] + day - 2 + (year % 4 == 0 && mon >= 3 ? 1 : 0);
	sec  = (int)floor(ep[5]);
	time.time = (time_t)days * 86400 + static_cast<time_t>(ep[3]) * 3600 + static_cast<time_t>(ep[4]) * 60 + sec;
	time.sec = ep[5] - sec;

	return time;
}

extern void time2epoch(const GpsTime_t t, double* ep)
{
	const int mday[] = { /* # of days in a month */
		31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
		31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
	};
	int days, sec, mon, day;

	/* leap year if year%4==0 in 1901-2099 */
	days = (int)(t.time / 86400);
	sec = (int)(t.time - (time_t)days * 86400);
	for (day = days % 1461, mon = 0; mon < 48; mon++) {
		if (day >= mday[mon]) day -= mday[mon]; 
		else { break; }
	}

	ep[0] = 1970 + days / 1461 * 4 + mon / 12; 
	ep[1] = mon % 12 + 1; 
	ep[2] = day + 1;
	ep[3] = sec / 3600; 
	ep[4] = sec % 3600 / 60; 
	ep[5] = sec % 60 + t.sec;
}

extern GpsTime_t utc2gpst(GpsTime_t t) 
{
	int i;

	for (i = 0; leaps[i][0] > 0; i++) {
		if (timediff(t, epoch2time(leaps[i])) >= 0.0) { 
			return timeadd(t, -leaps[i][6]); 
		}
	}
	return t;
}

extern GpsTime_t gpst2utc(GpsTime_t t)
{
	GpsTime_t tu;
	int i;

	for (i = 0; leaps[i][0] > 0; i++) {
		tu = timeadd(t, leaps[i][6]);
		if (timediff(tu, epoch2time(leaps[i])) >= 0.0) { return tu; }
	}

	return t;
}

extern double time2gpst(GpsTime_t t, int* week) 
{
	GpsTime_t t0 = epoch2time(gpst0);
	time_t sec = t.time - t0.time;
	int w = (int)(sec / (86400 * 7));

	if (week) *week = w;
	return (double)(sec - (double)w * 86400 * 7) + t.sec;
}

extern GpsTime_t gpst2time(const int week, double sec) 
{
	GpsTime_t t = epoch2time(gpst0);

	if (sec < -1E9 || 1E9 < sec) { sec = 0.0; }
	t.time += (time_t)86400 * 7 * week + (int)sec;
	t.sec = sec - (int)sec;
	return t;
}

extern GpsTime_t bdt2gpst(GpsTime_t t) 
{
	return timeadd(t, 14.0);
}

extern GpsTime_t bdt2time(const int week, double sec) 
{
	GpsTime_t t = epoch2time(bdt0);

	if (sec < -1E9 || 1E9 < sec) { sec = 0.0; }
	t.time += (time_t)86400 * 7 * week + (int)sec;
	t.sec = sec - (int)sec;
	return t;
}

extern GpsTime_t yrdoy2time(const int yyyy, const int doy)
{
	int days_in_month[12] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
	int mm, dd, id;
	double ep[6] = { 0.0 };

	/* check if yyyy is leap year */
	if (yyyy % 4 == 0 && (yyyy % 100 != 0 || yyyy % 400 == 0)) { 
		days_in_month[1] = 29; 
	}

	id = doy;
	dd = 0;
	for (mm = 0; mm < 12; mm++) {
		id -= days_in_month[mm];
		if (id > 0) { continue; }
		dd = id + days_in_month[mm];
		break;
	}
	mm++;
	ep[0] = (double)yyyy; ep[1] = (double)mm; ep[2] = (double)dd;

	return epoch2time(ep);
}

extern GpsTime_t timeadd(GpsTime_t t, double sec) 
{
	double tt;

	t.sec += sec;
	tt = floor(t.sec);
	t.time += (int)tt;
	t.sec -= tt;

	return t;
}

extern double timediff(GpsTime_t t1, GpsTime_t t2)
{
	return (difftime(t1.time, t2.time) + t1.sec - t2.sec);
}

extern int screent(GpsTime_t time, GpsTime_t ts, GpsTime_t te, double tint)
{
	return (tint <= 0.0 || fmod(time2gpst(time, NULL) + DTTOL, tint) <= DTTOL * 2.0) &&
		(ts.time == 0 || timediff(time, ts) >= -DTTOL) &&
		(te.time == 0 || timediff(time, te) < DTTOL);
}

extern int satid2no(const char* id) 
{
	int sys, prn;
	char code;

	if (sscanf(id, "%d", &prn) == 1) {
		if (MINPRNGPS <= prn && prn <= MAXPRNGPS) { sys = SYS_GPS; }
		else { return 0; }
		return satno(sys, prn);
	}
	if (sscanf(id, "%c%d", &code, &prn) < 2) { return 0; }

	switch (code) {
	case 'G': {sys = SYS_GPS; prn += MINPRNGPS - 1; break; }
	case 'R': {sys = SYS_GLO; prn += MINPRNGLO - 1; break; }
	case 'E': {sys = SYS_GAL; prn += MINPRNGAL - 1; break; }
	case 'C': {sys = SYS_BDS; prn += MINPRNBDS - 1; break; }
	default: return 0;
	}
	return satno(sys, prn);
}

extern void satno2id(int sat, char* id)
{
	int prn;

	switch (satsys(sat, &prn)) {
	case SYS_GPS: { sprintf(id, "G%02d", prn - MINPRNGPS + 1); return; }
	case SYS_GLO: { sprintf(id, "R%02d", prn - MINPRNGLO + 1); return; }
	case SYS_GAL: { sprintf(id, "E%02d", prn - MINPRNGAL + 1); return; }
	case SYS_BDS: { sprintf(id, "C%02d", prn - MINPRNBDS + 1); return; }
	}
	strcpy(id, "");
}

extern int satsys(int sat, int* prn) 
{
	int sys = SYS_NONE;

	if (sat <= 0 || MAXSAT < sat) { sat = 0; }
	else if (sat <= NSATGPS) {
		sys = SYS_GPS; sat += MINPRNGPS - 1;
	}
	else if ((sat -= NSATGPS) <= NSATGLO) {
		sys = SYS_GLO; sat += MINPRNGLO - 1;
	}
	else if ((sat -= NSATGLO) <= NSATGAL) {
		sys = SYS_GAL; sat += MINPRNGAL - 1;
	}
	else if ((sat -= NSATGAL) <= NSATBDS) {
		sys = SYS_BDS; sat += MINPRNBDS - 1;
	}
	else { sat = 0; }
	if (prn) { *prn = sat; }
	return sys;
}

extern int sat_exclude(int sat, map<int, vector<double>>* var_sat, map<int, vector<int>>* svh, const ProcOpt_t* popt)
{
	int sys = satsys(sat, NULL);

	if (sys == SYS_NONE || var_sat->find(sat) == var_sat->end() || svh->find(sat) == svh->end()) { return 1; }

	if ((*svh)[sat][0] < 0) { return 1; }				/* ephemeris unavailable */

	if (popt) {
		if (popt->exsats[sat - 1] == 1) { return 1; }	/* excluded satellite */
		if (popt->exsats[sat - 1] == 2) { return 0; }	/* included satellite */
		if (!(sys & popt->nav_sys)) { return 1; }		/* unselected sat sys */
	}

	if ((*svh)[sat][0]) {
		//printf("unhealthy satellite: sat=%3d svh=%02X\n", sat, (*svh)[sat][0]);
		return 1;
	}

	if ((*var_sat)[sat][0] > MAX_VAR_EPH) {
		printf("invalid ura satellite: sat=%3d ura=%.2f\n", sat, sqrt((*var_sat)[sat][0]));
		return 1;
	}

	return 0;
}

extern double sat_wavelen(int sat, int frq, NavPack_t* navall)
{
	const double freq_glo[] = { FREQ1_GLO,FREQ2_GLO };
	const double dfrq_glo[] = { DFRQ1_GLO,DFRQ2_GLO };
	int i, sys = satsys(sat, NULL);

	if (sys == SYS_GLO) {								  /* L1/L2 */
		if (0 <= frq && frq <= 1) {
			return CLIGHT / (freq_glo[frq] + dfrq_glo[frq] * navall->geph[sat].begin()->second.frq);
		}
		else if (frq == 2) {						      /* L3 */
			return CLIGHT / FREQ3_GLO;
		}
	}
	else if (sys == SYS_BDS) {
		if (frq == 0) { return CLIGHT / FREQ1_BDS; }	  /* B1 */
		else if (frq == 1) { return CLIGHT / FREQ2_BDS; } /* B2 */
		else if (frq == 2) { return CLIGHT / FREQ3_BDS; } /* B3 */
	}
	//added by fzhou, the recognition of Galileo triple-frequency, 2016-09-15 10:15:30 @ GFZ
	else if (sys == SYS_GAL) {
		if (frq == 0) { return CLIGHT / FREQ1; }		  /* E1 */
		else if (frq == 1) { return CLIGHT / FREQ5; }	  /* E5a */
		else if (frq == 2) { return CLIGHT / FREQ7; }	  /* E5b */
	}
	else {
		if (frq == 0) { return CLIGHT / FREQ1; }		  /* L1/E1 */
		else if (frq == 1) { return CLIGHT / FREQ2; }	  /* L2 */
		else if (frq == 2) { return CLIGHT / FREQ5; }	  /* L5/E5a */
	}

	return 0.0;
}

extern unsigned char obs2code(const char* obs, int* freq)
{
	int i;
	if (freq) { *freq = 0; }
	for (i = 1; *obscodes[i]; i++) {
		if (strcmp(obscodes[i], obs)) { continue; }

		if (freq) { *freq = obsfreqs[i]; }

		return (unsigned char)i; // 返回i表示在obscode[]列表中的位置
								 // 返回值类型为uchar(8bit)目的为节省内存,uint(32bit)占内存大
	}
	return CODE_NONE;
}

extern const char* code2obs(unsigned char code, int* freq)
{
	if (freq) { *freq = 0; }
	if (code <= CODE_NONE || MAXCODE < code) { return ""; }
	if (freq) { *freq = obsfreqs[code]; }

	return obscodes[code];
}

extern int code2sys(char code)
{
	if (code == 'G' || code == ' ') { return SYS_GPS; }
	if (code == 'R')				{ return SYS_GLO; }
	if (code == 'E')				{ return SYS_GAL; }
	if (code == 'C')				{ return SYS_BDS; }
	return SYS_NONE;
}

extern int satno(int sys, int prn) 
{
	if (prn <= 0) return 0;
	switch (sys) {
	case SYS_GPS:
		if (prn < MINPRNGPS || MAXPRNGPS < prn) { return 0; }
		return prn - MINPRNGPS + 1;
	case SYS_GLO:
		if (prn < MINPRNGLO || MAXPRNGLO < prn) { return 0; }
		return NSATGPS + prn - MINPRNGLO + 1;
	case SYS_GAL:
		if (prn < MINPRNGAL || MAXPRNGAL < prn) { return 0; }
		return NSATGPS + NSATGLO + prn - MINPRNGAL + 1;
	case SYS_BDS:
		if (prn < MINPRNBDS || MAXPRNBDS < prn) { return 0; }
		return NSATGPS + NSATGLO + NSATGAL + prn - MINPRNBDS + 1;
	}
	return 0;
}

extern double eph2clk(GpsTime_t time, const NavData_t* eph)
{
	double t;
	int i;

	t = timediff(time, eph->toc);

	for (i = 0; i < 2; i++) {
		t -= eph->f0 + eph->f1 * t + eph->f2 * t * t;
	}
	return eph->f0 + eph->f1 * t + eph->f2 * t * t;
}

extern double geph2clk(GpsTime_t time, const NavDataGlo_t* geph)
{
	double t;
	int i;

	t = timediff(time, geph->toe);

	for (i = 0; i < 2; i++) {
		t -= -geph->taun + geph->gamn * t;
	}
	return -geph->taun + geph->gamn * t;
}

extern vector<double> ecef2pos(const vector<double> r)
{
	double e2 = FE_WGS84 * (2.0 - FE_WGS84);
	double r2 = dot(r, r, 2);
	double z, zk, v = RE_WGS84, sinp;
	vector<double> LLH = vector<double>(3, 0.0);

	for (z = r[2], zk = 0.0; fabs(z - zk) >= 1E-4;) {
		zk = z;
		sinp = z / sqrt(r2 + z * z);
		v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);
		z = r[2] + v * e2 * sinp;
	}

	LLH[0] = r2 > 1E-12 ? atan(z / sqrt(r2)) : (r[2] > 0.0 ? PI / 2.0 : -PI / 2.0);
	LLH[1] = r2 > 1E-12 ? atan2(r[1], r[0]) : 0.0;
	LLH[2] = sqrt(r2 + z * z) - v;

	return LLH;
}

extern vector<vector<double>> rot_matrix(const vector<double> pos)
{
	vector<vector<double>> R = vector<vector<double>>(3, vector<double>(3, 0.0));

	if (pos.size() < 2) { return R; }

	double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);

	R[0][0] = -sinl;    R[1][0] = -sinp * cosl;    R[2][0] = cosp * cosl;
	R[0][1] = cosl;    R[1][1] = -sinp * sinl;    R[2][1] = cosp * sinl;
	R[0][2] = 0.0;    R[1][2] = cosp;    R[2][2] = sinp;

	return R;
}

extern vector<double> ecef2enu(const vector<double> pos, const vector<double> r)
{
	vector<vector<double>>   R = vector<vector<double>>(3, vector<double>(3, 0.0));
	vector<vector<double>> r2d = vector<vector<double>>(1, vector<double>(3, 0.0));
	vector<vector<double>> e2d = vector<vector<double>>(3, vector<double>(1, 0.0));
	vector<double>           e = vector<double>(3, 0.0);

	for (int i = 0; i < 3; i++) { r2d[0][i] = r[i]; }

	R = rot_matrix(pos);
	matmul("NT", 3, 3, 1, 1.0, &R, &r2d, 0.0, &e2d);

	for (int j = 0; j < 3; j++) { e[j] = e2d[j][0]; }

	return e;
}

extern double geodist(int sat, map<int, vector<double>>* rs, const vector<double> rr, vector<double>* e)
{
	double r;
	int i;
	double rs_tmp[3];

	for (i = 0; i < 3; i++) { rs_tmp[i] = (*rs)[sat][i]; }

	if (norm(rs_tmp, 3) < RE_WGS84) { return -1.0; }

	// 射线单位向量
	for (i = 0; i < 3; i++) { (*e)[i] = rs_tmp[i] - rr[i]; }
	r = norm(*e, 3);
	for (i = 0; i < 3; i++) { (*e)[i] /= r; }

	// 返回校正地球自转(sagnac effect)的几何距离
	return r + OMGE * (rs_tmp[0] * rr[1] - rs_tmp[1] * rr[0]) / CLIGHT;
}

extern double satazel(const int sat, const vector<double> pos, const vector<double> e, map<int, vector<double>>* azel)
{
	double az = 0.0, el = PI / 2.0;
	vector<double>enu = vector<double>(3, 0.0);

	if (pos[2] > -RE_WGS84) {
		enu = ecef2enu(pos, e);
		az = dot(enu, enu, 2) < 1E-12 ? 0.0 : atan2(enu[0], enu[1]);
		if (az < 0.0) { az += 2 * PI; }
		el = asin(enu[2]);
	}
	if (azel) {
		(*azel)[sat][0] = az;
		(*azel)[sat][1] = el;
	}
	return el;
}

extern void convcode(double ver, int sys, const char* str, char* type)
{
	strcpy(type, "   ");

	if (!strcmp(str, "P1")) {				// ver.2.11 GPS L1PY, GLO L2P
		if		(sys == SYS_GPS) { sprintf(type, "%c1W", 'C'); }
		else if (sys == SYS_GLO) { sprintf(type, "%c1P", 'C'); }
	}
	else if (!strcmp(str, "P2")) {			// ver.2.11 GPS L2PY,GLO L2P
		if		(sys == SYS_GPS) { sprintf(type, "%c2W", 'C'); }
		else if (sys == SYS_GLO) { sprintf(type, "%c2P", 'C'); }
	}
	else if (!strcmp(str, "C1")) {			// ver.2.11 GPS L1C,GLO L1C/A
		if (ver >= 2.12);					/* reject C1 for 2.12 */
		else if (sys == SYS_GPS) { sprintf(type, "%c1C", 'C'); }
		else if (sys == SYS_GLO) { sprintf(type, "%c1C", 'C'); }
		else if (sys == SYS_GAL) { sprintf(type, "%c1X", 'C'); } /* ver.2.12 */
	}
	else if (!strcmp(str, "C2")) {
		if (sys == SYS_GPS) {
			if (ver >= 2.12)	 { sprintf(type, "%c2W", 'C'); } /* L2P(Y) */
			else				 { sprintf(type, "%c2X", 'C'); } /* L2C */
		}
		else if (sys == SYS_GLO) { sprintf(type, "%c2C", 'C'); }
		else if (sys == SYS_BDS) { sprintf(type, "%c1X", 'C'); } /* ver.2.12 B1 */
	}
	else if (ver >= 2.12 && str[1] == 'A') { /* ver.2.12 L1C/A */
		if		(sys == SYS_GPS) { sprintf(type, "%c1C", str[0]); }
		else if (sys == SYS_GLO) { sprintf(type, "%c1C", str[0]); }
	}
	else if (ver >= 2.12 && str[1] == 'B') { /* ver.2.12 GPS L1C */
		if		(sys == SYS_GPS) { sprintf(type, "%c1X", str[0]); }
	}
	else if (ver >= 2.12 && str[1] == 'C') { /* ver.2.12 GPS L2C */
		if		(sys == SYS_GPS) { sprintf(type, "%c2X", str[0]); }
	}
	else if (ver >= 2.12 && str[1] == 'D') { /* ver.2.12 GLO L2C/A */
		if		(sys == SYS_GLO) { sprintf(type, "%c2C", str[0]); }
	}
	else if (ver >= 2.12 && str[1] == '1') { /* ver.2.12 GPS L1PY,GLO L1P */
		if		(sys == SYS_GPS) { sprintf(type, "%c1W", str[0]); }
		else if (sys == SYS_GLO) { sprintf(type, "%c1P", str[0]); }
		else if (sys == SYS_GAL) { sprintf(type, "%c1X", str[0]); } /* tentative */
		else if (sys == SYS_BDS) { sprintf(type, "%c1X", str[0]); } /* extension */
	}
	else if (ver < 2.12 && str[1] == '1') {
		if		(sys == SYS_GPS) { sprintf(type, "%c1C", str[0]); }
		else if (sys == SYS_GLO) { sprintf(type, "%c1C", str[0]); }
		else if (sys == SYS_GAL) { sprintf(type, "%c1X", str[0]); } /* tentative */
	}
	else if (str[1] == '2') {
		if		(sys == SYS_GPS) { sprintf(type, "%c2W", str[0]); }
		else if (sys == SYS_GLO) { sprintf(type, "%c2P", str[0]); }
		else if (sys == SYS_BDS) { sprintf(type, "%c1X", str[0]); } /* ver.2.12 B1 */
	}
	else if (str[1] == '5') {
		if		(sys == SYS_GPS) { sprintf(type, "%c5X", str[0]); }
		else if (sys == SYS_GAL) { sprintf(type, "%c5X", str[0]); }
	}
	else if (str[1] == '6') {
		if		(sys == SYS_GAL) { sprintf(type, "%c6X", str[0]); }
		else if (sys == SYS_BDS) { sprintf(type, "%c6X", str[0]); } /* ver.2.12 B3 */
	}
	else if (str[1] == '7') {
		if		(sys == SYS_GAL) { sprintf(type, "%c7X", str[0]); }
		else if (sys == SYS_BDS) { sprintf(type, "%c7X", str[0]); } /* ver.2.12 B2 */
	}
	else if (str[1] == '8') {
		if		(sys == SYS_GAL) { sprintf(type, "%c8X", str[0]); }
	}
}

extern int getcodepri(const int sys, unsigned char code, const char* opt)
{
	/* 局部变量定义 ========================================================= */
	const char* p;					// 字符串位置指针
	const char* optstr;				// 系统选项字符串
	const char* obs, str[8] = "";	// obs类型/字符串变量
	int i, j;						// 循环遍历变量
	/* ====================================================================== */

	switch (sys) {
	case SYS_GPS: {i = 0; optstr = "-GL%2s"; break; }
	case SYS_GLO: {i = 1; optstr = "-RL%2s"; break; }
	case SYS_GAL: {i = 2; optstr = "-EL%2s"; break; }
	case SYS_BDS: {i = 3; optstr = "-CL%2s"; break; }
	default: return 0;
	}
	obs = code2obs(code, &j);

	/* parse code options [一般没用]*/
	for (p = opt; p && (p = strchr(p, '-')); p++) {
		if (sscanf(p, optstr, str) < 1 || str[0] != obs[0]) { continue; }
		return str[1] == obs[1] ? 15 : 0;
	}
	/* search code priority */
	return (p = strchr(codepris[i][j - 1], obs[1])) ? 14 - (int)(p - codepris[i][j - 1]) : 0;
}

extern void setcodepri(int sys, int freq, const char* pri)
{
	if (freq <= 0 || MAXFREQ < freq) { return; }
	if (sys & SYS_GPS) { strcpy(codepris[0][freq - 1], pri); }
	if (sys & SYS_GLO) { strcpy(codepris[1][freq - 1], pri); }
	if (sys & SYS_GAL) { strcpy(codepris[2][freq - 1], pri); }
	if (sys & SYS_BDS) { strcpy(codepris[3][freq - 1], pri); }
}

extern int myRound(const double dNum)
{
	int iNum;

	if (dNum >= 0) { iNum = (int)(dNum + 0.5); }
	else		   { iNum = (int)(dNum - 0.5); }

	return iNum;
}

extern double dot(const double* a, const double* b, int n)
{
	double c = 0.0;

	while (--n >= 0) { c += a[n] * b[n]; }

	return c;
}

extern double dot(const vector<double> a, const vector<double> b, int n)
{
	double c = 0.0;

	while (--n >= 0) { c += a[n] * b[n]; }

	return c;
}

extern double norm(const double* a, int n)
{
	return sqrt(dot(a, a, n));;
}

extern double norm(const vector<double> a, int n)
{
	if (n <= 0 || a.size() < n) { return -1.0; }

	return sqrt(dot(a, a, n));
}

extern void matmul(const char* tr, int pA, int pAB, int pB, double alpha,
	const vector<vector<double>>* A, const  vector<vector<double>>* B, double beta, vector<vector<double>>* C)
{
	double d;
	int i, j, x, f = tr[0] == 'N' ? (tr[1] == 'N' ? 1 : 2) : (tr[1] == 'N' ? 3 : 4);
	int bad = 0;

	if (pA <= 0 || pB <= 0 || pAB <= 0) {
		printf("***ERROR: pA=%3d, pB=%3d, pAB=%3d\n", pA, pB, pAB);
		return;
	}
	if (pA > ((*A).size() > (*A)[0].size() ? (*A).size() : (*A)[0].size())) {
		printf("***ERROR: pA=%3d > A's size(%d*%d)\n", pA, (int)(*A).size(), (int)(*A)[0].size());
		bad = 1;
	}
	if (pB > ((*B).size() > (*B)[0].size() ? (*B).size() : (*B)[0].size())) {
		printf("***ERROR: pB=%3d > B's size(%d*%d)\n", pB, (int)(*B).size(), (int)(*B)[0].size());
		bad = 1;
	}
	if (pA > ((*C).size() > (*C)[0].size() ? (*C).size() : (*C)[0].size()) ||
		pB > ((*C).size() > (*C)[0].size() ? (*C).size() : (*C)[0].size())) {
		printf("***ERROR: pA=%3d, pB=%3d > C's size(%d*%d)\n", pA, pB, (int)(*C).size(), (int)(*C)[0].size());
		bad = 1;
	}
	if (bad) { return; }

	for (i = 0; i < pA; i++) {
		for (j = 0; j < pB; j++) {
			d = 0.0;
			switch (f)
			{
			case 1: {
				if (pAB > (*A)[0].size() || pAB > (*B).size()) {
					printf("***ERROR:the commom row/cloumn=%d > A's column %d or B's row %d\n",
						pAB, (int)(*A)[0].size(), (int)(*B).size());
					return;
				}
				for (x = 0; x < pAB; x++) { d += (*A)[i][x] * (*B)[x][j]; }
				break;
			}
			case 2: {
				if (pAB > (*A)[0].size() || pAB > (*B)[0].size()) {
					printf("***ERROR:the commom row/cloumn=%d > A's column %d or B's column %d\n",
						pAB, (int)(*A)[0].size(), (int)(*B)[0].size());
					return;
				}
				for (x = 0; x < pAB; x++) { d += (*A)[i][x] * (*B)[j][x]; }
				break;
			}
			case 3: {
				if (pAB > (*A).size() || pAB > (*B).size()) {
					printf("***ERROR:the commom row/cloumn=%d > A's row %d or B's row %d\n",
						pAB, (int)(*A).size(), (int)(*B).size());
					return;
				}
				for (x = 0; x < pAB; x++) { d += (*A)[x][i] * (*B)[x][j]; }
				break;
			}
			case 4: {
				if (pAB > (*A).size() || pAB > (*B)[0].size()) {
					printf("***ERROR:the commom row/cloumn=%d > A's row %d or B's column %d\n",
						pAB, (int)(*A).size(), (int)(*B)[0].size());
					return;
				}
				for (x = 0; x < pAB; x++) { d += (*A)[x][i] * (*B)[j][x]; }
				break;
			}
			}
			if (&(*C)[i][j] != NULL) { (*C)[i][j] = alpha * d + beta * (*C)[i][j]; }
		}
	}
}

extern int LU_dcmp(vector<vector<double>> A, int n, vector<vector<double>>* L, vector<vector<double>>* U)
{
	double tmp = 1.0, big;

	for (int i = 0; i < n; i++) {
		big = 0.0;
		for (int j = 0; j < n; j++) {
			if ((tmp = fabs(A[i][j])) > big) {
				big = tmp;
			}
		}
		if (big <= 0.0) {
			printf("矩阵的伴随矩阵<0，逆矩阵不存在\n");
			return -1;
		}
	}
	//计算U矩阵的第一行
	for (int j = 0; j < n; j++) { A[0][j] = A[0][j]; }

	//计算L矩阵的第1列
	for (int i = 1; i < n; i++) { A[i][0] = A[i][0] / A[0][0]; }

	for (int k = 1; k < n; k++) {
		for (int j = k; j < n; j++) {
			double s = 0;
			for (int i = 0; i < k; i++) { s = s + A[k][i] * A[i][j]; }	//累加

			A[k][j] = A[k][j] - s;				//计算U矩阵的其他元素
		}
		for (int i = k + 1; i < n; i++) {
			double t = 0;
			for (int j = 0; j < k; j++) { t = t + A[i][j] * A[j][k]; }	//累加

			A[i][k] = (A[i][k] - t) / A[k][k];  //计算L矩阵的其他元素
		}
	}

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			if (i > j) {	//如果i>j，说明行大于列，计算矩阵的下三角部分，得出L的值，U的为0

				(*L)[i][j] = A[i][j];
				(*U)[i][j] = 0;
			}
			else {			//否则如果i<j，说明行小于列，计算矩阵的上三角部分，得出U的//值，L的为0
				(*U)[i][j] = A[i][j];
				if (i == j) { (*L)[i][j] = 1.0; }
				else { (*L)[i][j] = 0.0; }
			}
		}
	}
	for (int i = 0; i < n; i++) {
		if ((tmp = (*U)[i][i]) == 0.0) {
			printf("U的对角线元素为0，逆矩阵不存在\n");
			return -1;
		}
	}

	return 0;
}

extern void LU_back(vector<vector<double>>* L, vector<vector<double>>* U, int n, vector<vector<double>>* R)
{
	vector<vector<double>> L_b = vector<vector<double>>(n, vector<double>(n, 0.0));
	vector<vector<double>> U_b = vector<vector<double>>(n, vector<double>(n, 0.0));

	/*求矩阵U的逆 */
	for (int i = 0; i < n; i++) {
		U_b[i][i] = 1 / (*U)[i][i];	//对角元素的值，直接取倒数
		for (int k = i - 1; k >= 0; k--) {
			double s = 0.0;
			for (int j = k + 1; j <= i; j++) { s = s + (*U)[k][j] * U_b[j][i]; }

			U_b[k][i] = -s / (*U)[k][k];	//迭代计算，按列倒序依次得到每一个值，
		}
	}
	//求矩阵L的逆
	for (int i = 0; i < n; i++) {
		L_b[i][i] = 1;			   //对角元素的值，直接取倒数，这里为1
		for (int k = i + 1; k < n; k++) {
			for (int j = i; j <= k - 1; j++) {
				L_b[k][i] = L_b[k][i] - (*L)[k][j] * L_b[j][i];   //迭代计算，按列顺序依次得到每一个值
			}
		}
	}

	matmul("NN", n, n, n, 1.0, &U_b, &L_b, 0.0, R);
}

extern int mat_invert(vector<vector<double>>* A, int n)
{
	int stat;
	vector<vector<double>> L = vector<vector<double>>(n, vector<double>(n, 0.0));
	vector<vector<double>> U = vector<vector<double>>(n, vector<double>(n, 0.0));

	if ((stat = LU_dcmp((*A), n, &L, &U)) == -1) { return 0; }

	LU_back(&L, &U, n, A);

	return 1;
}

extern int lsq(vector<vector<double>>* A, vector<double>* y, int nx, int ny, vector<double>* x, vector<vector<double>>* Q)
{
	if (ny < nx) { return -1; }
	double* Ay;
	int stat;
	vector<vector<double>> x2D = vector<vector<double>>(nx, vector<double>(1, 0.0));
	vector<vector<double>> y2D = vector<vector<double>>(ny, vector<double>(1, 0.0));
	vector<vector<double>> ATy = vector<vector<double>>(nx, vector<double>(1, 0.0));


	for (int i = 0; i < ny; i++) { y2D[i][0] = (*y)[i]; }

	matmul("TN", nx, ny, 1, 1.0, A, &y2D, 0.0, &ATy); /* ATy=A^T*y */
	matmul("TN", nx, ny, nx, 1.0, A, A, 0.0, Q);	  /* Q=A^T*A */
	if ((stat = mat_invert(Q, nx)) == 1) {			  /* Q=(A^T*A)^-1 */
		matmul("NN", nx, nx, 1, 1.0, Q, &ATy, 0.0, &x2D);
		for (int i = 0; i < nx; i++) {
			(*x)[i] = x2D[i][0];
		}
		return 1;
	}

	return 0;
}

extern double iono_model(GpsTime_t time, int sat, const double* ion, vector<double> pos, map<int, vector<double>>* azel)
{
	/* 局部变量定义 ========================================================= */
	const double ion_default[] = { /* 2004/1/1 广播电离层默认参数(8个) */
		0.1118E-07,-0.7451E-08,-0.5961E-07, 0.1192E-06,
		0.1167E+06,-0.2294E+06,-0.1311E+06, 0.1049E+07
	};
	double tt, f, psi, phi, lam, amp, per, x;
	int week;
	double az, el;
	/* ====================================================================== */

	if (azel->find(sat) == azel->end()) { return 0.0; }

	az = (*azel)[sat][0];
	el = (*azel)[sat][1];
	if (pos[2] < -1E3 || el <= 0) { return 0.0; }
	if (norm(ion, 8) <= 0.0) { ion = ion_default; }

	/* 1.earth centered angle (semi-circle) */
	psi = 0.0137 / (el / PI + 0.11) - 0.022;

	/* 2.subionospheric latitude/longitude (semi-circle) */
	phi = pos[0] / PI + psi * cos(az);
	if (phi > 0.416) { phi = 0.416; }
	else if (phi < -0.416) { phi = -0.416; }

	/* 3.wave length */
	lam = pos[1] / PI + psi * sin(az) / cos(phi * PI);

	/* 4.geomagnetic latitude (semi-circle) */
	phi += 0.064 * cos((lam - 1.617) * PI);

	/* 5.local time (s) */
	tt = 43200.0 * lam + time2gpst(time, &week);
	tt -= floor(tt / 86400.0) * 86400.0; /* 0<=tt<86400 */

	/* 6.slant factor */
	f = 1.0 + 16.0 * pow(0.53 - el / PI, 3.0);

	/* 7.ionospheric delay */
	amp = ion[0] + phi * (ion[1] + phi * (ion[2] + phi * ion[3]));
	per = ion[4] + phi * (ion[5] + phi * (ion[6] + phi * ion[7]));
	amp = amp < 0.0 ? 0.0 : amp;
	per = per < 72000.0 ? 72000.0 : per;
	x = 2.0 * PI * (tt - 50400.0) / per;

	return CLIGHT * f * (fabs(x) < 1.57 ? 5E-9 + amp * (1.0 + x * x * (-0.5 + x * x / 24.0)) : 5E-9);
}

extern double trop_model(GpsTime_t time, int sat, vector<double> pos, map<int, vector<double>>* azel, double humi, double* zwd)
{
	const double temp0 = 15.0;					/* temparature at sea level */
	double hgt, pres, temp, e, z, trph, trpw;

	if (azel->find(sat) == azel->end()) { return 0.0; }

	if (pos[2] < -100.0 || 1E6 < pos[2] || (*azel)[sat][1] <= 0) { return 0.0; }

	if (pos[2] >= 1.0 / 2.2557E-5) { return 0.0; }

	hgt = pos[2] < 0.0 ? 0.0 : pos[2];

	/* standard atmosphere */
	pres = 1013.25 * pow(1.0 - 2.2557E-5 * hgt, 5.2568);
	temp = temp0 - 6.5E-3 * hgt + 273.16;
	e = 6.108 * humi * exp((17.15 * temp - 4684.0) / (temp - 38.45));

	/* saastamoninen model */
	z = PI / 2.0 - (*azel)[sat][1];
	trph = 0.0022768 * pres / (1.0 - 0.00266 * cos(2.0 * pos[0]) - 0.00028 * hgt / 1E3) / cos(z); // 干延迟
	trpw = 0.002277 * (1255.0 / temp + 0.05) * e / cos(z);										  // 湿延迟

	*zwd = trpw;

	return trph;	// 只返回干延迟
}

extern void init_sta(Station_t* sta)
{
	int i;
	*sta->name = '\0';
	*sta->marker = '\0';
	*sta->antdes = '\0';
	*sta->antsno = '\0';
	*sta->rectype = '\0';
	*sta->recver = '\0';
	*sta->recsno = '\0';
	sta->antsetup = sta->itrf = sta->deltype = 0;
	for (i = 0; i < 3; i++) { sta->pos[i] = 0.0; }
	for (i = 0; i < 3; i++) { sta->del[i] = 0.0; }
	sta->hgt = 0.0;
}

extern int compare_eph(ObsEphData_t eph1, ObsEphData_t eph2)
{
	double t1 = (double)eph1.eph.time + eph1.eph.sec;
	double t2 = (double)eph2.eph.time + eph2.eph.sec;
	return (t1 < t2);
}

extern int compare_erp(Erp_t erp1, Erp_t erp2)
{
	return (erp1.mjd < erp2.mjd);
}

extern int cal_CS_thres(ProcOpt_t* popt, const double sample)
{
	int stat = 0;

	// 根据采样率sample设置GF和MW的阈值
	if (sample > 0.0) {
		if (popt->flagGF == 1 && fabs(popt->thresGF) < 0.01) {
			if (sample <= 1.0) { popt->thresGF = 0.05; }
			else if (sample <= 20.0) { popt->thresGF = (0.10) / (20.0 - 0.0) * sample + 0.05; }
			else if (sample <= 60.0) { popt->thresGF = 0.15; }
			else if (sample <= 100.0) { popt->thresGF = 0.25; }
			else { popt->thresGF = 0.35; }

			stat = 1;
		}
		if (popt->flagMW == 1 && fabs(popt->thresMW) < 0.01) {
			if (sample <= 1.0) { popt->thresMW = 2.5; }
			else if (sample <= 20.0) { popt->thresMW = (2.5) / (20.0 - 0.0) * sample + 2.5; }
			else if (sample <= 60.0) { popt->thresMW = 5.0; }
			else { popt->thresMW = 7.5; }

			stat = 1;
		}

		return stat;
	}
	else {
		popt->thresGF = 0.15;
		popt->thresMW = 5.0;
		stat = 0;
	}

	return stat;
}

extern vector<double> cal_dops(int ns, map<int, vector<double>>* azel, double elmin)
{
	double cosEL, sinEL, cosAZ, sinAZ, el = 0.0, az = 0.0;
	int i, n = 0, sat, stat;
	vector<double> dops = vector<double>(4, 0.0);
	vector<vector<double>> H = vector<vector<double>>(ns, vector<double>(4, 0.0));
	vector<vector<double>> Q = vector<vector<double>>(4, vector<double>(4, 0.0));

	for (auto it = (*azel).begin(); it != (*azel).end(); it++) {
		sat = it->first;
		az = (*azel)[sat][0];
		el = (*azel)[sat][1];
		if (el < elmin || el <= 0.0) { continue; }

		cosEL = cos(el); sinEL = sin(el);
		cosAZ = cos(az); sinAZ = sin(az);
		H[n][0] = cosEL * sinAZ;
		H[n][1] = cosEL * cosAZ;
		H[n][2] = sinEL;
		H[n][3] = 1.0;

		n++;
	}

	if (ns < 4) {
		printf("In calculating DOPS, ns=%d<4\n", n);
		return dops;
	}

	matmul("TN", 4, n, 4, 1.0, &H, &H, 0.0, &Q);
	if ((stat = mat_invert(&Q, 4)) == 1) {
		dops[0] = SQRT(Q[0][0] + Q[1][1] + Q[2][2] + Q[3][3]);
		dops[1] = SQRT(Q[0][0] + Q[1][1] + Q[2][2]);
		dops[2] = SQRT(Q[0][0] + Q[1][1]);
		dops[3] = SQRT(Q[2][2]);
	}

	/*printf("Q=\n");
	for (int p = 0; p < 4; p++) {
		for (int q = 0; q < 4; q++) {
			printf("%8.2f", Q[p][q]);
		}
		printf("\n");
	}*/

	return dops;
}

extern FILE* openfile(const char* outfile)
{
	return !*outfile ? stdout : fopen(outfile, "w");
}
