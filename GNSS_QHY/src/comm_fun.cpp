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

/* coordinate rotation matrix ------------------------------------------------*/
#define Rx(t,X) do { \
    (X)[0]=1.0; (X)[1]=(X)[2]=(X)[3]=(X)[6]=0.0; \
    (X)[4]=(X)[8]=cos(t); (X)[7]=sin(t); (X)[5]=-(X)[7]; \
} while (0)

#define Ry(t,X) do { \
    (X)[4]=1.0; (X)[1]=(X)[3]=(X)[5]=(X)[7]=0.0; \
    (X)[0]=(X)[8]=cos(t); (X)[2]=sin(t); (X)[6]=-(X)[2]; \
} while (0)

#define Rz(t,X) do { \
    (X)[8]=1.0; (X)[2]=(X)[5]=(X)[6]=(X)[7]=0.0; \
    (X)[0]=(X)[4]=cos(t); (X)[3]=sin(t); (X)[1]=-(X)[3]; \
} while (0)

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

/* time to day and sec -------------------------------------------------------*/
static double time2sec(GpsTime_t time, GpsTime_t* day)
{
	double ep[6], sec;
	time2epoch(time, ep);
	sec = ep[3] * 3600.0 + ep[4] * 60.0 + ep[5];
	ep[3] = ep[4] = ep[5] = 0.0;
	*day = epoch2time(ep);
	return sec;
}

/*utc to gmst---------------------------------------------------------------- -
*convert utc to gmst(Greenwich mean sidereal time)
* args   : gtime_t t        I   time expressed in utc
* double ut1_utc   I   UT1 - UTC(s)
* return : gmst(rad)
* ---------------------------------------------------------------------------- - */
extern double utc2gmst(GpsTime_t t, double ut1_utc)
{
	const double ep2000[] = { 2000,1,1,12,0,0 };
	GpsTime_t tut, tut0;
	double ut, t1, t2, t3, gmst0, gmst;

	tut = timeadd(t, ut1_utc);
	ut = time2sec(tut, &tut0);
	t1 = timediff(tut0, epoch2time(ep2000)) / 86400.0 / 36525.0;
	t2 = t1 * t1; t3 = t2 * t1;
	gmst0 = 24110.54841 + 8640184.812866 * t1 + 0.093104 * t2 - 6.2E-6 * t3;
	gmst = gmst0 + 1.002737909350795 * ut;

	return fmod(gmst, 86400.0) * PI / 43200.0; /* 0 <= gmst <= 2*PI */
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

	R[0][0] = -sinl;  R[1][0] = -sinp * cosl;  R[2][0] = cosp * cosl;
	R[0][1] = cosl;   R[1][1] = -sinp * sinl;  R[2][1] = cosp * sinl;
	R[0][2] = 0.0;    R[1][2] = cosp;          R[2][2] = sinp;

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

/* iau 1980 nutation ---------------------------------------------------------*/
static void nut_iau1980(double t, const double* f, double* dpsi, double* deps)
{
	static const double nut[106][10] = {
		{   0,   0,   0,   0,   1, -6798.4, -171996, -174.2, 92025,   8.9},
		{   0,   0,   2,  -2,   2,   182.6,  -13187,   -1.6,  5736,  -3.1},
		{   0,   0,   2,   0,   2,    13.7,   -2274,   -0.2,   977,  -0.5},
		{   0,   0,   0,   0,   2, -3399.2,    2062,    0.2,  -895,   0.5},
		{   0,  -1,   0,   0,   0,  -365.3,   -1426,    3.4,    54,  -0.1},
		{   1,   0,   0,   0,   0,    27.6,     712,    0.1,    -7,   0.0},
		{   0,   1,   2,  -2,   2,   121.7,    -517,    1.2,   224,  -0.6},
		{   0,   0,   2,   0,   1,    13.6,    -386,   -0.4,   200,   0.0},
		{   1,   0,   2,   0,   2,     9.1,    -301,    0.0,   129,  -0.1},
		{   0,  -1,   2,  -2,   2,   365.2,     217,   -0.5,   -95,   0.3},
		{  -1,   0,   0,   2,   0,    31.8,     158,    0.0,    -1,   0.0},
		{   0,   0,   2,  -2,   1,   177.8,     129,    0.1,   -70,   0.0},
		{  -1,   0,   2,   0,   2,    27.1,     123,    0.0,   -53,   0.0},
		{   1,   0,   0,   0,   1,    27.7,      63,    0.1,   -33,   0.0},
		{   0,   0,   0,   2,   0,    14.8,      63,    0.0,    -2,   0.0},
		{  -1,   0,   2,   2,   2,     9.6,     -59,    0.0,    26,   0.0},
		{  -1,   0,   0,   0,   1,   -27.4,     -58,   -0.1,    32,   0.0},
		{   1,   0,   2,   0,   1,     9.1,     -51,    0.0,    27,   0.0},
		{  -2,   0,   0,   2,   0,  -205.9,     -48,    0.0,     1,   0.0},
		{  -2,   0,   2,   0,   1,  1305.5,      46,    0.0,   -24,   0.0},
		{   0,   0,   2,   2,   2,     7.1,     -38,    0.0,    16,   0.0},
		{   2,   0,   2,   0,   2,     6.9,     -31,    0.0,    13,   0.0},
		{   2,   0,   0,   0,   0,    13.8,      29,    0.0,    -1,   0.0},
		{   1,   0,   2,  -2,   2,    23.9,      29,    0.0,   -12,   0.0},
		{   0,   0,   2,   0,   0,    13.6,      26,    0.0,    -1,   0.0},
		{   0,   0,   2,  -2,   0,   173.3,     -22,    0.0,     0,   0.0},
		{  -1,   0,   2,   0,   1,    27.0,      21,    0.0,   -10,   0.0},
		{   0,   2,   0,   0,   0,   182.6,      17,   -0.1,     0,   0.0},
		{   0,   2,   2,  -2,   2,    91.3,     -16,    0.1,     7,   0.0},
		{  -1,   0,   0,   2,   1,    32.0,      16,    0.0,    -8,   0.0},
		{   0,   1,   0,   0,   1,   386.0,     -15,    0.0,     9,   0.0},
		{   1,   0,   0,  -2,   1,   -31.7,     -13,    0.0,     7,   0.0},
		{   0,  -1,   0,   0,   1,  -346.6,     -12,    0.0,     6,   0.0},
		{   2,   0,  -2,   0,   0, -1095.2,      11,    0.0,     0,   0.0},
		{  -1,   0,   2,   2,   1,     9.5,     -10,    0.0,     5,   0.0},
		{   1,   0,   2,   2,   2,     5.6,      -8,    0.0,     3,   0.0},
		{   0,  -1,   2,   0,   2,    14.2,      -7,    0.0,     3,   0.0},
		{   0,   0,   2,   2,   1,     7.1,      -7,    0.0,     3,   0.0},
		{   1,   1,   0,  -2,   0,   -34.8,      -7,    0.0,     0,   0.0},
		{   0,   1,   2,   0,   2,    13.2,       7,    0.0,    -3,   0.0},
		{  -2,   0,   0,   2,   1,  -199.8,      -6,    0.0,     3,   0.0},
		{   0,   0,   0,   2,   1,    14.8,      -6,    0.0,     3,   0.0},
		{   2,   0,   2,  -2,   2,    12.8,       6,    0.0,    -3,   0.0},
		{   1,   0,   0,   2,   0,     9.6,       6,    0.0,     0,   0.0},
		{   1,   0,   2,  -2,   1,    23.9,       6,    0.0,    -3,   0.0},
		{   0,   0,   0,  -2,   1,   -14.7,      -5,    0.0,     3,   0.0},
		{   0,  -1,   2,  -2,   1,   346.6,      -5,    0.0,     3,   0.0},
		{   2,   0,   2,   0,   1,     6.9,      -5,    0.0,     3,   0.0},
		{   1,  -1,   0,   0,   0,    29.8,       5,    0.0,     0,   0.0},
		{   1,   0,   0,  -1,   0,   411.8,      -4,    0.0,     0,   0.0},
		{   0,   0,   0,   1,   0,    29.5,      -4,    0.0,     0,   0.0},
		{   0,   1,   0,  -2,   0,   -15.4,      -4,    0.0,     0,   0.0},
		{   1,   0,  -2,   0,   0,   -26.9,       4,    0.0,     0,   0.0},
		{   2,   0,   0,  -2,   1,   212.3,       4,    0.0,    -2,   0.0},
		{   0,   1,   2,  -2,   1,   119.6,       4,    0.0,    -2,   0.0},
		{   1,   1,   0,   0,   0,    25.6,      -3,    0.0,     0,   0.0},
		{   1,  -1,   0,  -1,   0, -3232.9,      -3,    0.0,     0,   0.0},
		{  -1,  -1,   2,   2,   2,     9.8,      -3,    0.0,     1,   0.0},
		{   0,  -1,   2,   2,   2,     7.2,      -3,    0.0,     1,   0.0},
		{   1,  -1,   2,   0,   2,     9.4,      -3,    0.0,     1,   0.0},
		{   3,   0,   2,   0,   2,     5.5,      -3,    0.0,     1,   0.0},
		{  -2,   0,   2,   0,   2,  1615.7,      -3,    0.0,     1,   0.0},
		{   1,   0,   2,   0,   0,     9.1,       3,    0.0,     0,   0.0},
		{  -1,   0,   2,   4,   2,     5.8,      -2,    0.0,     1,   0.0},
		{   1,   0,   0,   0,   2,    27.8,      -2,    0.0,     1,   0.0},
		{  -1,   0,   2,  -2,   1,   -32.6,      -2,    0.0,     1,   0.0},
		{   0,  -2,   2,  -2,   1,  6786.3,      -2,    0.0,     1,   0.0},
		{  -2,   0,   0,   0,   1,   -13.7,      -2,    0.0,     1,   0.0},
		{   2,   0,   0,   0,   1,    13.8,       2,    0.0,    -1,   0.0},
		{   3,   0,   0,   0,   0,     9.2,       2,    0.0,     0,   0.0},
		{   1,   1,   2,   0,   2,     8.9,       2,    0.0,    -1,   0.0},
		{   0,   0,   2,   1,   2,     9.3,       2,    0.0,    -1,   0.0},
		{   1,   0,   0,   2,   1,     9.6,      -1,    0.0,     0,   0.0},
		{   1,   0,   2,   2,   1,     5.6,      -1,    0.0,     1,   0.0},
		{   1,   1,   0,  -2,   1,   -34.7,      -1,    0.0,     0,   0.0},
		{   0,   1,   0,   2,   0,    14.2,      -1,    0.0,     0,   0.0},
		{   0,   1,   2,  -2,   0,   117.5,      -1,    0.0,     0,   0.0},
		{   0,   1,  -2,   2,   0,  -329.8,      -1,    0.0,     0,   0.0},
		{   1,   0,  -2,   2,   0,    23.8,      -1,    0.0,     0,   0.0},
		{   1,   0,  -2,  -2,   0,    -9.5,      -1,    0.0,     0,   0.0},
		{   1,   0,   2,  -2,   0,    32.8,      -1,    0.0,     0,   0.0},
		{   1,   0,   0,  -4,   0,   -10.1,      -1,    0.0,     0,   0.0},
		{   2,   0,   0,  -4,   0,   -15.9,      -1,    0.0,     0,   0.0},
		{   0,   0,   2,   4,   2,     4.8,      -1,    0.0,     0,   0.0},
		{   0,   0,   2,  -1,   2,    25.4,      -1,    0.0,     0,   0.0},
		{  -2,   0,   2,   4,   2,     7.3,      -1,    0.0,     1,   0.0},
		{   2,   0,   2,   2,   2,     4.7,      -1,    0.0,     0,   0.0},
		{   0,  -1,   2,   0,   1,    14.2,      -1,    0.0,     0,   0.0},
		{   0,   0,  -2,   0,   1,   -13.6,      -1,    0.0,     0,   0.0},
		{   0,   0,   4,  -2,   2,    12.7,       1,    0.0,     0,   0.0},
		{   0,   1,   0,   0,   2,   409.2,       1,    0.0,     0,   0.0},
		{   1,   1,   2,  -2,   2,    22.5,       1,    0.0,    -1,   0.0},
		{   3,   0,   2,  -2,   2,     8.7,       1,    0.0,     0,   0.0},
		{  -2,   0,   2,   2,   2,    14.6,       1,    0.0,    -1,   0.0},
		{  -1,   0,   0,   0,   2,   -27.3,       1,    0.0,    -1,   0.0},
		{   0,   0,  -2,   2,   1,  -169.0,       1,    0.0,     0,   0.0},
		{   0,   1,   2,   0,   1,    13.1,       1,    0.0,     0,   0.0},
		{  -1,   0,   4,   0,   2,     9.1,       1,    0.0,     0,   0.0},
		{   2,   1,   0,  -2,   0,   131.7,       1,    0.0,     0,   0.0},
		{   2,   0,   0,   2,   0,     7.1,       1,    0.0,     0,   0.0},
		{   2,   0,   2,  -2,   1,    12.8,       1,    0.0,    -1,   0.0},
		{   2,   0,  -2,   0,   1,  -943.2,       1,    0.0,     0,   0.0},
		{   1,  -1,   0,  -2,   0,   -29.3,       1,    0.0,     0,   0.0},
		{  -1,   0,   0,   1,   1,  -388.3,       1,    0.0,     0,   0.0},
		{  -1,  -1,   0,   2,   1,    35.0,       1,    0.0,     0,   0.0},
		{   0,   1,   0,   1,   0,    27.3,       1,    0.0,     0,   0.0}
	};
	double ang;
	int i, j;

	*dpsi = *deps = 0.0;

	for (i = 0; i < 106; i++) {
		ang = 0.0;
		for (j = 0; j < 5; j++) { ang += nut[i][j] * f[j]; }
		*dpsi += (nut[i][6] + nut[i][7] * t) * sin(ang);
		*deps += (nut[i][8] + nut[i][9] * t) * cos(ang);
	}
	*dpsi *= 1E-4 * AS2R; /* 0.1 mas -> rad */
	*deps *= 1E-4 * AS2R;
}

/* astronomical arguments: f={l,l',F,D,OMG} (rad) ----------------------------*/
static void ast_args(double t, double* f)
{
	static const double fc[][5] = { /* coefficients for iau 1980 nutation */
		{ 134.96340251, 1717915923.2178,  31.8792,  0.051635, -0.00024470},
		{ 357.52910918,  129596581.0481,  -0.5532,  0.000136, -0.00001149},
		{  93.27209062, 1739527262.8478, -12.7512, -0.001037,  0.00000417},
		{ 297.85019547, 1602961601.2090,  -6.3706,  0.006593, -0.00003169},
		{ 125.04455501,   -6962890.2665,   7.4722,  0.007702, -0.00005939}
	};
	double tt[4];
	int i, j;

	for (tt[0] = t, i = 1; i < 4; i++) { tt[i] = tt[i - 1] * t; }
	for (i = 0; i < 5; i++) {
		f[i] = fc[i][0] * 3600.0;
		for (j = 0; j < 4; j++) {
			f[i] += fc[i][j + 1] * tt[j];
		}
		f[i] = fmod(f[i] * AS2R, 2.0 * PI);
	}
}

/* eci to ecef transformation matrix -------------------------------------------
* compute eci to ecef transformation matrix
* args   : gtime_t tutc     I   time in utc
*          double *erpv     I   erp values {xp,yp,ut1_utc,lod} (rad,rad,s,s/d)
*          double *U        O   eci to ecef transformation matrix (3 x 3)
*          double *gmst     IO  greenwich mean sidereal time (rad)
*                               (NULL: no output)
* return : none
* note   : see ref [3] chap 5
*          not thread-safe
*-----------------------------------------------------------------------------*/
extern void eci2ecef(GpsTime_t tutc, const double* erpv, double* U, double* gmst)
{
	const double ep2000[] = { 2000,1,1,12,0,0 };
	static GpsTime_t tutc_;
	static double U_[9], gmst_;
	GpsTime_t tgps;
	double eps, ze, th, z, t, t2, t3, dpsi, deps, gast, f[5];
	double R1[9], R2[9], R3[9], R[9], W[9], N[9], P[9], NP[9];
	int i;

	if (fabs(timediff(tutc, tutc_)) < 0.01) { /* read cache */
		for (i = 0; i < 9; i++) U[i] = U_[i];
		if (gmst) *gmst = gmst_;
		return;
	}
	tutc_ = tutc;

	/* terrestrial time */
	tgps = utc2gpst(tutc_);
	t = (timediff(tgps, epoch2time(ep2000)) + 19.0 + 32.184) / 86400.0 / 36525.0;
	t2 = t * t; t3 = t2 * t;

	/* astronomical arguments */
	ast_args(t, f);

	/* iau 1976 precession */
	ze = (2306.2181 * t + 0.30188 * t2 + 0.017998 * t3) * AS2R;
	th = (2004.3109 * t - 0.42665 * t2 - 0.041833 * t3) * AS2R;
	z = (2306.2181 * t + 1.09468 * t2 + 0.018203 * t3) * AS2R;
	eps = (84381.448 - 46.8150 * t - 0.00059 * t2 + 0.001813 * t3) * AS2R;
	Rz(-z, R1); Ry(th, R2); Rz(-ze, R3);
	matmul("NN", 3, 3, 3, 1.0, R1, R2, 0.0, R);
	matmul("NN", 3, 3, 3, 1.0, R, R3, 0.0, P); /* P=Rz(-z)*Ry(th)*Rz(-ze) */

	/* iau 1980 nutation */
	nut_iau1980(t, f, &dpsi, &deps);
	Rx(-eps - deps, R1); Rz(-dpsi, R2); Rx(eps, R3);
	matmul("NN", 3, 3, 3, 1.0, R1, R2, 0.0, R);
	matmul("NN", 3, 3, 3, 1.0, R, R3, 0.0, N); /* N=Rx(-eps)*Rz(-dspi)*Rx(eps) */

	/* greenwich aparent sidereal time (rad) */
	gmst_ = utc2gmst(tutc_, erpv[2]);
	gast = gmst_ + dpsi * cos(eps);
	gast += (0.00264 * sin(f[4]) + 0.000063 * sin(2.0 * f[4])) * AS2R;

	/* eci to ecef transformation matrix */
	Ry(-erpv[0], R1); Rx(-erpv[1], R2); Rz(gast, R3);
	matmul("NN", 3, 3, 3, 1.0, R1, R2, 0.0, W);
	matmul("NN", 3, 3, 3, 1.0, W, R3, 0.0, R); /* W=Ry(-xp)*Rx(-yp) */
	matmul("NN", 3, 3, 3, 1.0, N, P, 0.0, NP);
	matmul("NN", 3, 3, 3, 1.0, R, NP, 0.0, U_); /* U=W*Rz(gast)*N*P */

	for (i = 0; i < 9; i++) U[i] = U_[i];
	if (gmst) *gmst = gmst_;
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

/* sun and moon position in eci (ref [4] 5.1.1, 5.2.1) -----------------------*/
static void sun_moon_pos_ECI(GpsTime_t tut, double* rsun, double* rmoon)
{
	const double ep2000[] = { 2000,1,1,12,0,0 };
	double t, f[5], eps, Ms, ls, rs, lm, pm, rm, sine, cose, sinp, cosp, sinl, cosl;

	t = timediff(tut, epoch2time(ep2000)) / 86400.0 / 36525.0;

	/* astronomical arguments */
	ast_args(t, f);

	/* obliquity of the ecliptic */
	eps = 23.439291 - 0.0130042 * t;
	sine = sin(eps * D2R); cose = cos(eps * D2R);

	/* sun position in eci */
	if (rsun) {
		Ms = 357.5277233 + 35999.05034 * t;
		ls = 280.460 + 36000.770 * t + 1.914666471 * sin(Ms * D2R) + 0.019994643 * sin(2.0 * Ms * D2R);
		rs = AU * (1.000140612 - 0.016708617 * cos(Ms * D2R) - 0.000139589 * cos(2.0 * Ms * D2R));
		sinl = sin(ls * D2R); cosl = cos(ls * D2R);
		rsun[0] = rs * cosl;
		rsun[1] = rs * cose * sinl;
		rsun[2] = rs * sine * sinl;
	}
	/* moon position in eci */
	if (rmoon) {
		lm = 218.32 + 481267.883 * t + 6.29 * sin(f[0]) - 1.27 * sin(f[0] - 2.0 * f[3]) +
			0.66 * sin(2.0 * f[3]) + 0.21 * sin(2.0 * f[0]) - 0.19 * sin(f[1]) - 0.11 * sin(2.0 * f[2]);
		pm = 5.13 * sin(f[2]) + 0.28 * sin(f[0] + f[2]) - 0.28 * sin(f[2] - f[0]) -
			0.17 * sin(f[2] - 2.0 * f[3]);
		rm = RE_WGS84 / sin((0.9508 + 0.0518 * cos(f[0]) + 0.0095 * cos(f[0] - 2.0 * f[3]) +
			0.0078 * cos(2.0 * f[3]) + 0.0028 * cos(2.0 * f[0])) * D2R);
		sinl = sin(lm * D2R); cosl = cos(lm * D2R);
		sinp = sin(pm * D2R); cosp = cos(pm * D2R);
		rmoon[0] = rm * cosp * cosl;
		rmoon[1] = rm * (cose * cosp * sinl - sine * sinp);
		rmoon[2] = rm * (sine * cosp * sinl + cose * sinp);
	}
}

extern void sun_moon_pos(GpsTime_t tutc, const double* erpv, double* rsun, double* rmoon, double* gmst)
{
	GpsTime_t tut;
	double rs[3], rm[3], U[9], gmst_;

	tut = timeadd(tutc, erpv[2]); /* utc -> ut1 */

	/* sun and moon position in eci */
	sun_moon_pos_ECI(tut, rsun ? rs : NULL, rmoon ? rm : NULL);

	/* eci to ecef transformation matrix */
	eci2ecef(tutc, erpv, U, &gmst_);

	/* sun and moon postion in ecef */
	if (rsun)  { matmul("NN", 3, 1, 3, 1.0, U, rs, 0.0, rsun); }
	if (rmoon) { matmul("NN", 3, 1, 3, 1.0, U, rm, 0.0, rmoon); }
	if (gmst)  { *gmst = gmst_; }
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

extern double dot(const vector<double> a, const double* b, int n)
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

extern int normv3(const double* a, double* b)
{
	double r;
	if ((r = norm(a, 3)) <= 0.0) { return 0; }
	b[0] = a[0] / r;
	b[1] = a[1] / r;
	b[2] = a[2] / r;
	return 1;
}

extern int normv3(vector<double> a, double* b)
{
	double r;
	if ((r = norm(a, 3)) <= 0.0) { return 0; }
	b[0] = a[0] / r;
	b[1] = a[1] / r;
	b[2] = a[2] / r;
	return 1;
}

extern void matcpy(double* A, const double* B, int n, int m)
{
	memcpy(A, B, sizeof(double) * n * m);
}

extern void matmul(const char* tr, int n, int k, int m, double alpha, 
	const double* A, const double* B, double beta, double* C)
{
	double d;
	int i, j, x, f = tr[0] == 'N' ? (tr[1] == 'N' ? 1 : 2) : (tr[1] == 'N' ? 3 : 4);

	for (i = 0; i < n; i++) for (j = 0; j < k; j++) {
		d = 0.0;
		switch (f) {
		case 1: for (x = 0; x < m; x++) d += A[i + x * n] * B[x + j * m]; break;
		case 2: for (x = 0; x < m; x++) d += A[i + x * n] * B[j + x * k]; break;
		case 3: for (x = 0; x < m; x++) d += A[x + i * m] * B[x + j * m]; break;
		case 4: for (x = 0; x < m; x++) d += A[x + i * m] * B[j + x * k]; break;
		}
		if (beta == 0.0) C[i + j * n] = alpha * d; else C[i + j * n] = alpha * d + beta * C[i + j * n];
	}
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

static double sign(double d, double d1)
{
	if		(d1 > 0) { return fabs(d); }
	else if (d1 < 0) { return 0 - fabs(d); }
	else			 { return 0; }
}

extern void cross3(const double* a, const double* b, double* c)
{
	c[0] = a[1] * b[2] - a[2] * b[1];
	c[1] = a[2] * b[0] - a[0] * b[2];
	c[2] = a[0] * b[1] - a[1] * b[0];
}

/*----------------------------------------------------------------------------
C
C       NAME	        ECLIPS (version Sep  2011)
C
C	PURPOSE 	DETECT ECLIPSING & YAW ROTATE ECLIP. SATELLITES
C                       (THE INPUT BODY-X UNIT VECTOR - SANTXYZ IS YAW-ROTATED
C                        BY PHI-YANGLE (THE ECL-NOMINAL) YAW ANGLE DIFFERENCE)
C
C       COPYRIGHT       GEODETIC SURVEY DIVISION, 2011.
C                       ALL RIGHTS RESERVED.
C                       ALL TERMS AND CONDITIONS APPLY AS DETAILED IN
C                       " TERMS AND CONDITIONS FOR SOFTWARE "
C
C       CONTACT         kouba@geod.nrcan.gc.ca
C
C       UPDATE HISTORY: Aug. 23, 2011:1996-2008 W. AVERAGES of JPL REPROCESSING
C                                YRATE SOLUTIONS FOR ALL II/IIA CODED IN DATA
C                                STATEMENT, THIS ENABLES REPROCESSING FROM 1996
C                       Sep 26, 2011: 1. Corrected bug causing Block IIF shadow
C                               CROSSING MANEVURE WITH 0.06 DEG/SEC RATE EVEN
C                               FOR BETADG > 8 DEG
C                                     2. CORRECTED/improved IIA RECOVERY logic
C
C	PARAMETERS	DESCRIPTION
C
C        IDIR		DIRECTION OF PROCESSING (1=FORWARD, -1=BACKWARD)
C        IPRN           SV PRN NUMBER (.LE.32 FOR GPS, .GT.32 FOR GLONASS)
C        TTAG           OBSERVATION EPOCH TIME TAG (EG SEC OF GPS WEEK)
C        SVBCOS         SV BETA ANGLE (BETWEEN SV AND SUN RADIUS) COSINE			对应论文中的E
C        ANOON          SV BETA ANGLE LIMIT (DEG) FOR A NOON TURN MANEVURE
C        ANIGHT         SV BETA ANGLE LIMIT (DEG) FOR A NIGHT SHADOW CROSSING
C        NECLIPS        NUMBER OF ECLIPSING FOR THE PRN SATELLITE
C        ECLSTM         ECLIPSE START TIME(EG SEC OF GPS WEEK)
C        ECLETM         ECLIPSE END TIME  ( "         "      )
C        IECLIPS        SV ECLIPSING (0=NO,1, 2=YES(1=night, 2=noon))
C        PI             = PI=3.1415926536D0
C        XSV(3)         SV X, Y, Z (m)(ITRF)
C        SANTXYZ        BODY-X UNIT VECTOR (ITRF)
C                       WARNING: THE IIA BODY-X ORIENTATION EXPECTED FOR THE IIR
C                        THE  BODY-X REVERSED FOR IIR (SEE BELOW) & RETURNED
C        VSVC           SV INERTIAL VELOCIRY VECTOR IN ITRF
C        BETA           90 DEG + THE SUN ANGLE WITH ORBITAL PLANE(IN RAD)
C        IBLK           SV BLOCK  1=I, 2=II, 3=IIA, IIR=(4, 5) IIF=6
C
C        INTERNAL PARAMETRS DESCRIPTION
C        YANGLE         THE NOMINAL YAW ANGLE
C        PHI            THE ECLIPSING YAW ANGLE
C
C *********************************************************************
*/
static int eclips_(int IPRN, double SVBCOS, double ANIGHT, double BETA, double TTAG,
	double XSV[3], double SANTXYZ[3], double VSVC[3], int IBLK)
{
	int i, j, ii;
	int IECLIPS;
	int IDIR = 1;
	double    TWOHR, HALFHR;
	double    ANOON;
	double    CNOON, CNIGHT;
	double    DTR, DTTAG;
	double    MURATE, YANGLE, DET, BETADG, PHI = 0.0, SANTX, SANTY, v[3], r[3];
	double    YAWEND;
	double	  ECLSTM, ECLETM;
	int NOON, NIGHT;

	//  MAX YAW RATES OF CURRENT&PAST BLOCK II/IIA's,(AVER'D 1996-2008 JPL SOLUTIONS)  
	//  CHANGE IF REQUIRED OR INPUT IF ESTIMATED 
	//  PRN              01     02     03      04      05      06     07
	double YRATE[] = { .1211, .1339,  .123,  .1233,  .1180,  .1266, .1269,
		// 08     09     10     11      12      13      14     15
		.1033, .1278, .0978, 0.200,  0.199,  0.200, 0.0815, .1303,
		// PRN 16     17     18     19      20      21      22     23
		.0838, .1401, .1069,  .098,   .103, 0.1366,  .1025, .1140,
		// PRN 24     25     26     27      28      29      30     31
		.1089, .1001, .1227, .1194,  .1260,  .1228,  .1165, .0969,
		// PRN 32      33-64: GLONASS RATES (DILSSNER 2010)                            
		.1152,
		0.250, 0.250, 0.250, 0.250, 0.250, 0.250, 0.250, 0.250,
		0.250, 0.250, 0.250, 0.250, 0.250, 0.250, 0.250, 0.250,
		0.250, 0.250, 0.250, 0.250, 0.250, 0.250, 0.250, 0.250,
		0.250, 0.250, 0.250, 0.250, 0.250, 0.250, 0.250, 0.250
	};

	ECLSTM = ECLETM = -1e6;

	//  CHECK FOR BLOCK IIR AND FIX TO NOMINAL YAW RATE
	if (IPRN <= MAXPRNGPS && IBLK >= 4) YRATE[IPRN - 1] = 0.2;

	// THE NEW GPS BLK IIF YAW RATES ( DILSSNER (2010) INSIDE GNSS)
	if (IPRN <= MAXPRNGPS && IBLK >= 6) YRATE[IPRN - 1] = 0.11;

	IECLIPS = 0;

	TWOHR = 7200.0;
	HALFHR = 1800.0;
	DTR = D2R;

	// compute the noon beta angle limit (beta zero) FOR A NOON TURN from YRATEs
	// & THE ACTUAL SAT ORBIT ANGLE RATE (MURATE) (~0.00836 FOR GPS; ~ 0.00888 GLNS)
	MURATE = sqrt((pow(VSVC[1 - 1], 2) + pow(VSVC[2 - 1], 2) + pow(VSVC[3 - 1], 2)) /
		(pow(XSV[1 - 1], 2) + pow(XSV[2 - 1], 2) + pow(XSV[3 - 1], 2))
	) / DTR;
	ANOON = atan(MURATE / YRATE[IPRN - 1]) / DTR;

	CNOON = cos(ANOON * DTR);
	CNIGHT = cos(ANIGHT * DTR);

	//
	NOON = 0;
	NIGHT = 0;
	BETADG = BETA / DTR - 90.0;

	if (40 == IPRN && 14 == pppglob.ep[3]) {
		ii = 0;
	}

	//
	if (IPRN > MAXPRNGPS && fabs(BETADG) < ANOON) {
		// GLONASS NOON TURN MODE ACORDING TO DILSSNER 2010 
		YAWEND = 75.0;
		//  ITERATION FOR YAWEND OF THE GLONASS  NOON TURN

		for (j = 1; j <= 3; j++) {
			YAWEND = fabs(atan2(-tan(BETADG * DTR), sin(PI - DTR * MURATE * YAWEND / YRATE[IPRN - 1])) / DTR
				- atan2(-tan(BETADG * DTR), sin(PI + DTR * MURATE * YAWEND / YRATE[IPRN - 1])) / DTR
			) / 2.0;
		}

		// UPDATE ANOON, CNOON FOR NEW GLONASS NOON TURN LIMITS
		ANOON = MURATE * YAWEND / YRATE[IPRN - 1];
		CNOON = cos(ANOON * DTR);
	}

	// BLK IIR'S
	if (IBLK == 4 || IBLK == 5) {
		CNIGHT = cos((ANOON + 180.0) * DTR);
		for (j = 1; j <= 3; j++) {
			// BODY-X U VECTOR REVERSAL FOR IIR ONLY
			SANTXYZ[j - 1] = -SANTXYZ[j - 1];
		}
	}
	//
	if (SVBCOS < CNIGHT)
		NIGHT = 1;

	if (SVBCOS > CNOON)
		NOON = 1;

	//
	//     IF SV IN NIGHT SHADOW OR NOON TURN DURING FORWARD PASS
	//     STORE START AND END TIME OF YAW MANEUVRE (FOR THE BACKWARD RUN)
	//

	//acos: 0-pi
	// YAW ANLGE
	YANGLE = acos((SANTXYZ[1 - 1] * VSVC[1 - 1] + SANTXYZ[2 - 1] * VSVC[2 - 1] + SANTXYZ[3 - 1] * VSVC[3 - 1]) /
		sqrt(pow(VSVC[1 - 1], 2) + pow(VSVC[2 - 1], 2) + pow(VSVC[3 - 1], 2))
	) / DTR;

	// IIR YANGLE has the same sign as beta, II/IIA has the opposite sign
	if (BETADG < 0.0 && IBLK >= 4 && IBLK <= 5)
		YANGLE = -YANGLE;
	if (BETADG > 0.0 && IBLK != 4 && IBLK != 5)
		YANGLE = -YANGLE;

	//
	if ((NIGHT || NOON)) {
		DET = SQRT(pow(180.0 - acos(SVBCOS) / DTR, 2) - pow(BETADG, 2));
		PHI = PI / 2.0;
		// Check if already after a midnight or noon
		if (NIGHT) {
			if (IBLK == 4 || IBLK == 5) {
				if (fabs(YANGLE) > 90.0)	DET = -DET;
				if (DET != 0.0)				PHI = atan2(tan(BETADG * DTR), -sin(-DET * DTR)) / DTR;
			}
			else {
				// BLK IIA & GLONASS TOO !
				if (fabs(YANGLE) < 90.0)	DET = -DET;
				if (DET != 0.0)				PHI = atan2(-tan(BETADG * DTR), sin(-DET * DTR)) / DTR;
			}
		}
		if (NOON) {
			DET = SQRT(pow(acos(SVBCOS) * 180.0 / PI, 2) - pow(BETADG, 2));

			if (IBLK == 4 || IBLK == 5) {
				if (fabs(YANGLE) < 90.0)	DET = -DET;
				if (DET != 0.0)				PHI = atan2(tan(BETADG * DTR), -sin(PI - DET * DTR)) / DTR;
			}
			else {
				// BLK IIA & GLONASS !
				if (fabs(YANGLE) > 90.0)		DET = -DET;
				if (DET != 0.0)				PHI = atan2(-tan(BETADG * DTR), sin(PI - DET * DTR)) / DTR;
			}
		}



		// ONLY FORWARD
		//计算正午/午夜机动，地影恢复期时间段
		if (IDIR > 0) {
			//
			// INITIALIZE ECLIPSE START AND TIME TAG ARRAYS  
			//
			//1
			//if ( NECLIPS[IPRN-1] == 0 ) 
			{
				//NECLIPS[IPRN-1]=NECLIPS[IPRN-1]+1;
				ECLSTM = TTAG + DET / MURATE;
				// IIR MIDNIGHT/NOON TURN or II/IIA NOON TURN START
				// for IIR/GLONAS NIGHT (turn) only makes sense when BETADG < ANOON!
				// For IIA it gets here only when NOON is true and that happens  only when BETADG < ANOON!
				YAWEND = atan(MURATE / YRATE[IPRN - 1]) / DTR;

				if (((IBLK > 3 && IBLK <= 5) || NOON) && fabs(BETADG) < YAWEND) {
					// GLONASS
					if (IPRN > MAXPRNGPS) {
						// GLONASS NOON TURN MODE ACORDING TO DILSSNER ET AL 2010 
						ECLSTM = ECLSTM - ANOON / MURATE;
						ECLETM = ECLSTM + 2.0 * ANOON / MURATE;
					}
					else {
						// GPS IIA/IIR/IIF NOON OR IIR MIDNIGHT TURNs
						ECLSTM = ECLSTM - fabs(BETADG) * sqrt(ANOON / fabs(BETADG) - 1.0) / MURATE;
						ECLETM = ECLSTM + 2 * fabs(BETADG) * sqrt(ANOON / fabs(BETADG) - 1.0) / MURATE;
					}
				}

				// II/IIA SHADOW START & END TIMES
				if ((IBLK <= 3 || IBLK > 5) && NIGHT) {
					//if (ANIGHT<180)
					//	ANIGHT+=180;


					ECLSTM = ECLSTM - SQRT(pow(ANIGHT - 180.0, 2) - pow(BETADG, 2)) / MURATE;
					ECLETM = ECLSTM + 2.0 * SQRT(pow(ANIGHT - 180.0, 2) - pow(BETADG, 2)) / MURATE;
				}
				//
				// UPDATE SV COSINE AND TIME TAG ARRAYS
				// (TO BE USED DURING BACKWARDS RUN)
				//
				if ((NIGHT && SVBCOS < CNIGHT) || (NOON && SVBCOS > CNOON)) {
					DTTAG = fabs(TTAG - ECLSTM);
					//
					// ECLIPSE TIME IS MORE THAN 2 HOURS, THIS IS A NEW ECLIPSE!
					//
					if (DTTAG > TWOHR) {
						ECLSTM = TTAG + DET / MURATE;
						// IIR MIDNIGHT/NOON TURN  or II/IIA NOON TURN START
						// AND GLONASS NOON
						if ((IBLK > 3 && IBLK <= 5) || NOON) {
							// GLONASS

							if (IPRN > MAXPRNGPS) {
								// GLONASS NOON TURN MODE ACORDING TO DILSSNER ET AL 2010 
								ECLSTM = ECLSTM - ANOON / MURATE;
								ECLETM = ECLSTM + 2.0 * ANOON / MURATE;
							}
							else {
								// GPS TURNS ONLY
								ECLSTM = ECLSTM - fabs(BETADG) * sqrt(ANOON / fabs(BETADG) - 1.0) / MURATE;
								ECLSTM = ECLSTM + 2 * fabs(BETADG) * sqrt(ANOON / fabs(BETADG) - 1.0) / MURATE;
							}
						}
					}

					//     II/IIA SHADOW START & END TIMES
					//   & GLONASS & IIF AS WELL !
					if ((IBLK <= 3 || IBLK > 5) && NIGHT) {
						//if (ANIGHT<180)
						//	ANIGHT+=180;

						ECLSTM = ECLSTM - SQRT(pow(ANIGHT - 180.0, 2) - pow(BETADG, 2)) / MURATE;
						ECLSTM = ECLSTM + 2.0 * SQRT(pow(ANIGHT - 180.0, 2) - pow(BETADG, 2)) / MURATE;
					}
				}
			}
			//  END OF FORWARD LOOP (IDIR = 1)
		}
	}
	//
	//     BOTH FWD (IDIR= 1) OR BWD (IDIR=-1)
	//     SET ECLIPSE FLAG (1=NIGHT SHADOW, 2=NOON TURN) 
	//
	if (1) {
		// CHECK IF IPRN IS ECLIPSING AND WHICH SEQ NO (I)
		i = 0;


		for (j = 1; j <= 1; j++) {
			if (fabs(ECLETM + 1.0e6) <= 1.0e-8 && fabs(ECLSTM + 1.0e6) <= 1.0e-8)
				continue;

			if (TTAG >= ECLSTM && TTAG <= (ECLETM + HALFHR))
				i = j;
		}

		// CURRENTLY NOT ECLIPSING (i=0)
		if (0 == i) return IECLIPS;



		//判断此时时间是否在正午/午夜机动，地影恢复期
		if (TTAG >= ECLSTM && TTAG <= (ECLETM + HALFHR)) {
			// velocity & radius unit vectors V & R
			for (j = 1; j <= 3; j++) {
				v[j - 1] = VSVC[j - 1] / SQRT(pow(VSVC[1 - 1], 2) + pow(VSVC[2 - 1], 2) + pow(VSVC[3 - 1], 2));
				r[j - 1] = XSV[j - 1] / SQRT(pow(XSV[1 - 1], 2) + pow(XSV[2 - 1], 2) + pow(XSV[3 - 1], 2));
			}
			// ORBIT ANGLE MU AT ECLIPSE/TURN START
			DET = MURATE * (ECLETM - ECLSTM) / 2.0;


			//！！！！！！！！！！！！计算此时具体的航偏角PHI，将名义姿态绕航偏角旋转
			if (SVBCOS < 0) {
				// SHADOW CROSSING
				// BLK IIA/IIF SHADOW CROSSING
				if (IPRN <= MAXPRNGPS && (IBLK <= 3 || IBLK > 5)) {
					if (TTAG <= ECLETM) {
						// IIA NIGHT TURN
						if (IBLK <= 3)  PHI = atan2(-tan(BETADG * DTR), sin(-DET * DTR)) / DTR + sign(YRATE[IPRN - 1], 0.50) * (TTAG - ECLSTM);
						// IIF NIGHT TURN (DILSSNER  2010)
						if (IBLK > 5)   PHI = atan2(-tan(BETADG * DTR), sin(-DET * DTR)) / DTR + sign(0.060, BETADG) * (TTAG - ECLSTM);
					}
					else {
						// **** WARNING
						// IIA/IIF SHADOW EXIT RECOVERY: USING THE IIA DATA  DURING
						// THE IIA RECOVERY (UP TO 30 MIN) IS NOT RECOMMENDED!
						// **** WARNING
						// GPS IIA  AT SHADOW EXIT
						if (IBLK <= 3)  PHI = atan2(-tan(BETADG * DTR), sin(-DET * DTR)) / DTR + sign(YRATE[IPRN - 1], 0.50) * (ECLETM - ECLSTM);
						// GPS IIF AT SHADOW EXIT
						if (IBLK > 5)   PHI = atan2(-tan(BETADG * DTR), sin(-DET * DTR)) / DTR + sign(0.060, BETADG) * (ECLETM - ECLSTM);
						// YAWEND- HERE THE ACTUAL  YAW  AT THE SHADOW EXIT
						YAWEND = YANGLE - PHI;
						YAWEND = fmod(YAWEND, 360.0);
						if (fabs(YAWEND) > 180.0) YAWEND = YAWEND - 360.0 * YAWEND / fabs(YAWEND);
						PHI = PHI + sign(YRATE[IPRN - 1], YAWEND) * (TTAG - ECLETM);
						// SANTX- THE CURRENT ANGLE DIFF, CONSISTENT WITH YAWEND
						SANTX = YANGLE - PHI;
						SANTX = fmod(SANTX, 360.0);
						if (fabs(SANTX) > 180.0) SANTX = SANTX - 360.0 * SANTX / fabs(SANTX);
						// STOP! THE NOMINAL YAW (YANGLE) REACHED!
						if (fabs(SANTX) > fabs(YAWEND)) return IECLIPS;
						if (YAWEND != 0.0 && ((SANTX) / YAWEND) < 0.0) return IECLIPS;
						// SET PHI <-180,+180>
						PHI = fmod(PHI, 360.0);
						if (fabs(PHI) > 180.0) PHI = PHI - 360.0 * PHI / fabs(PHI);
					}
				}

				// GLONASS
				if (IPRN > MAXPRNGPS) {
					// GLONASS/GPS  NIGHT TURN (DILSSNER AT AL 2010 )
					if (TTAG > ECLETM) return IECLIPS;
					YAWEND = YRATE[IPRN - 1];
					PHI = atan2(-tan(BETADG * DTR), sin(-DET * DTR)) / DTR + sign(YAWEND, BETADG) * (TTAG - ECLSTM);
					// YAWEND -YAW ANGLE AT THE (GLONASS) SHADOW EXIT
					YAWEND = atan2(-tan(BETADG * DTR), sin(DET * DTR)) / DTR;

					if ((YAWEND / PHI) >= 1.0 || (PHI / YAWEND) < 0.0)
						PHI = YAWEND;
				}

				if (IPRN <= MAXPRNGPS && IBLK > 5)
					// GPS BLK IIF NIGHT YAW RATE(DILSSNER 2010):
					if (fabs(BETADG) > 8.0) return IECLIPS;

				if (IBLK > 3 && IBLK <= 5) {
					// BLK II R SHADOW (MIDNIGHT TURN) CROSSING
					PHI = atan2(tan(BETADG * DTR), -sin(-DET * DTR)) / DTR + sign(YRATE[IPRN - 1], BETADG) * (TTAG - ECLSTM);
					if ((PHI / YANGLE) >= 1.0 || (PHI / YANGLE) < 0.0) return IECLIPS;
				}
				//             write(*,*)"R",IPRN-32,TTAG,YANGLE, PHI,DET,
				//    & BETADG, ECLETM(IPRN,I),I
				IECLIPS = 1;
			}
			else {
				// NOON TURNS 
				PHI = atan2(-tan(BETADG * DTR), sin(PI - DET * DTR)) / DTR - sign(YRATE[IPRN - 1], BETADG) * (TTAG - ECLSTM);
				if (IBLK > 3 && IBLK <= 5) {
					// BLK IIR NOON TURNS ONLY
					PHI = atan2(tan(BETADG * DTR), -sin(PI - DET * DTR)) / DTR - sign(YRATE[IPRN - 1], BETADG) * (TTAG - ECLSTM);
					// IIR END TURN CHECK
					if ((YANGLE / PHI) >= 1.0 || (PHI / YANGLE) < 0.0) return IECLIPS;
				}
				else {
					// GLONASS END TURN CHECK
					if (IPRN > MAXPRNGPS && TTAG > ECLETM) return IECLIPS;
					// IIA OR IIF END TURN CHECK
					if (IPRN <= MAXPRNGPS && ((PHI / YANGLE) >= 1.0 || (PHI / YANGLE) < 0.0)) return IECLIPS;
				}
				//             write(*,*)"S",IPRN-32,TTAG,YANGLE, PHI,DET,
				//    & BETADG, ECLSTM(IPRN,I)
				IECLIPS = 2;
			}
			// ROTATE X-VECTOR TO ECLIPSING YAW ANGLE PHI 
			// ECLIPSING (II/IIA) NOT TO BE USED  A HALF HR AFTER SHADOW !
			SANTX = (cos((PHI - YANGLE) * DTR) * (v[2 - 1] - v[3 - 1] * r[2 - 1] / r[3 - 1]) - cos(PHI * DTR) * (SANTXYZ[2 - 1] - SANTXYZ[3 - 1] * r[2 - 1]) / r[3 - 1]) / (SANTXYZ[1 - 1] * v[2 - 1] - SANTXYZ[2 - 1] * v[1 - 1])
				+ ((SANTXYZ[2 - 1] * v[3 - 1] - SANTXYZ[3 - 1] * v[2 - 1]) * r[1 - 1] + (SANTXYZ[3 - 1] * v[1 - 1] - SANTXYZ[1 - 1] * v[3 - 1]) * r[2 - 1]) / r[3 - 1];
			SANTY = (cos(PHI * DTR) - (v[1 - 1] - v[3 - 1] * r[1 - 1] / r[3 - 1]) * SANTX) / (v[2 - 1] - v[3 - 1] * r[2 - 1] / r[3 - 1]);
			// THE BODY-X UNIT VECTOR ROTATED BY (PHI-YANGLE) RETURNED
			SANTXYZ[1 - 1] = SANTX;
			SANTXYZ[2 - 1] = SANTY;
			SANTXYZ[3 - 1] = (-r[1 - 1] * SANTX - r[2 - 1] * SANTY) / r[3 - 1];
		}
	}

	return IECLIPS;
}

extern int cal_Eclips(int sat, map<int, vector<double>>* rs, double* sunp, double TTAG,
	double SANTXYZ[3], const NavPack_t* navall)
{
	double SVBCOS, BETA = 0.0, eSunP[3], eSatP[3], eSatV[3], vec[3], ANIGHT, satv_[3];
	double angleLmt;
	const char* type;
	int IBLK = -1;		//IBLK SV BLOCK  1=I, 2=II, 3=IIA, IIR=(4, 5) IIF=6
	double rs_p[3] = { 0.0 };

	if (sat > MAXPRNGPS + MAXPRNGLO) { return 0; }

	type = (*navall).sat_pcv.at(sat).type;

	if (type) {
		if		(strstr(type, "BLOCK I      ")) { IBLK = 1; }
		else if (strstr(type, "BLOCK II     ")) { IBLK = 2; }
		else if (strstr(type, "BLOCK IIA    ")) { IBLK = 3; }
		else if (strstr(type, "BLOCK IIR-B  ")) { IBLK = 4; }
		else if (strstr(type, "BLOCK IIR-M  ")) { IBLK = 5; }
		else if (strstr(type, "BLOCK IIF    ")) { IBLK = 6; }
	}

	if (sat > MAXPRNGPS) { IBLK = 6; }

	satv_[0] = (*rs)[sat][3] - OMGE * (*rs)[sat][1];
	satv_[1] = (*rs)[sat][4] + OMGE * (*rs)[sat][0];
	satv_[2] = (*rs)[sat][5];

	normv3((*rs)[sat], eSatP);
	normv3(satv_, eSatV);
	normv3(sunp, eSunP);

	SVBCOS = dot(eSatP, eSunP, 3);

	cross3(eSatP, eSatV, vec);

	BETA = dot(vec, eSunP, 3);
	BETA = acos(BETA);
	BETA = -BETA + PI;

	angleLmt = 76.116;  //acos(RE_WGS84/26580000.0)*R2D;

	ANIGHT = 90 + angleLmt - 1.5;

	for (int i = 0; i < 3; i++) { rs_p[i] = (*rs)[sat][i]; }

	return eclips_(sat, SVBCOS, ANIGHT, BETA, TTAG, rs_p, SANTXYZ, satv_, IBLK);
}
