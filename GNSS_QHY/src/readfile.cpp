#include"readfile.h"

static GpsTime_t adjweek(GpsTime_t t, GpsTime_t t0) 
{
	/* 局部变量定义 =========================================== */
	double tt = timediff(t, t0);	// time difference t-t0 (s)
	/* ======================================================== */

	if (tt < -302400.0) { 
		return timeadd(t, 604800.0);
	}
	if (tt > 302400.0) {
		return timeadd(t, -604800.0);
	}

	return t;
}

static GpsTime_t adjday(GpsTime_t t, GpsTime_t t0) 
{
	/* 局部变量定义 =========================================== */
	double tt = timediff(t, t0);	// time difference t-t0 (s)
	/* ======================================================== */

	if (tt < -43200.0) { 
		return timeadd(t, 86400.0); 
	}
	if (tt > 43200.0) {
		return timeadd(t, -86400.0);
	}

	return t;
}

static int ura_index(double value)
{
	/* 局部变量定义 =========== */
	int i;		// ura index
	/* ======================== */

	for (i = 0; i < 15; i++) { 
		if (ura_eph[i] >= value) { break; }
	}
	return i;
}

static void sta_info(Station_t* sta, char* buff, char* label) 
{
	/* 局部变量定义 ================================= */
	int i, j;					// 循环遍历变量
	double del[3];				// 天线偏移量[3]
	/* ============================================== */
	
	if (strstr(label, "MARKER NAME")) {
		setstr(sta->name, buff, 60);
	}
	else if (strstr(label, "MARKER NUMBER")) {
		setstr(sta->marker, buff, 20);
	}
	else if (strstr(label, "REC # / TYPE / VERS")) {
		setstr(sta->recsno, buff, 20);
		setstr(sta->rectype, buff + 20, 20);
		setstr(sta->recver, buff + 40, 20);
	}
	else if (strstr(label, "ANT # / TYPE")) {
		setstr(sta->antsno, buff, 20);
		setstr(sta->antdes, buff + 20, 20);
	}
	else if (strstr(label, "APPROX POSITION XYZ")) {
		for (i = 0, j = 0; i < 3; i++, j += 14) {
			sta->pos[i] = str2num(buff, j, 14);
		}
	}
	else if (strstr(label, "ANTENNA: DELTA H/E/N")) {
		for (i = 0, j = 0; i < 3; i++, j += 14) {
			del[i] = str2num(buff, j, 14);
		}
		sta->del[2] = del[0]; /* h */
		sta->del[0] = del[1]; /* e */
		sta->del[1] = del[2]; /* n */
	}
}

static void saveslips(unsigned char slips[][NFREQ], map<int, ObsData_t>::iterator it) 
{
	int i;

	for (i = 0; i < NFREQ; i++) {
		if (it->second.LLI[i] & 1) { 
			slips[it->first - 1][i] |= 1; 
		}
	}
}

static void restslips(unsigned char slips[][NFREQ], map<int, ObsData_t>::iterator it) {
	int i;
	for (i = 0; i < NFREQ; i++) {
		if (slips[it->first - 1][i] & 1) { 
			it->second.LLI[i] |= 1; 
		}
		slips[it->first - 1][i] = 0;
	}
}

/* read rinex file ------------------------------------------------------------------------------------------------------*/

static void obsTypeV2(FILE* fp, double ver, char* buff, char tobs[][MAXOBSTYPE][4])
{
	/* 局部变量定义 ============================================= */
	int i, j;		// 循环遍历变量
	int n, nt;		// code总数/code计数
	char str[4];	// obs码类型
	/* ========================================================== */

	n = (int)str2num(buff, 0, 6);
	for (i = nt = 0, j = 10; i < n; i++, j += 6) {
		if (j > 58) {
			if (!fgets(buff, MAXRNXLEN, fp)) { break; }
			j = 10;
		}
		if (nt >= MAXOBSTYPE - 1) { continue; }
		if (ver <= 2.99) {
			setstr(str, buff + j, 2);
			convcode(ver, SYS_GPS, str, tobs[0][nt]);
			convcode(ver, SYS_GLO, str, tobs[1][nt]);
			convcode(ver, SYS_GAL, str, tobs[2][nt]);
			convcode(ver, SYS_BDS, str, tobs[3][nt]);
		}
		nt++;
	}
	*tobs[0][nt] = '\0';
}

static void obsTypeV3(FILE* fp, char* buff, char tobs[][MAXOBSTYPE][4]) {
	/* 局部变量定义 ============================================= */
	const char* defcodes[] = {
		"CWX   ",   /* GPS: L125___ */
		"CC    ",   /* GLO: L12____ */
		"X XXXX",   /* GAL: L1_5678 */
		"X  XX "    /* BDS: L1__67_ */
	};
	int i;			// 系统索引
	int j, k;		// 循环遍历变量
	int n = 0, nt;;	// code总数/code计数
	const char* p;	// 读文件指针
	/* ========================================================== */

	if (!(p = strchr(syscodes, buff[0]))) {
		n = (int)str2num(buff, 3, 3);

		if (n <= 0) {
			printf("read file error\n");
		}
		else if (n <= 13) {
			printf("   Invalid sat system: sys=%c, num=%2d\n", buff[0], n);
		}
		else if (n > 13) {
			printf("   Invalid sat system: sys=%c, num=%2d\n", buff[0], n);
			if (!fgets(buff, MAXRNXLEN, fp)) {
				printf("read file error\n");
			}
		}
		return;
	}

	i = (int)(p - syscodes);
	n = (int)str2num(buff, 3, 3);

	for (j = nt = 0, k = 7; j < n; j++, k += 4) {
		if (k > 58) {				// if k > 58, k = 7
			if (!fgets(buff, MAXRNXLEN, fp)) { break; }
			k = 7;
		}
		if (nt < MAXOBSTYPE - 1) {  // get tobs
			setstr(tobs[i][nt++], buff + k, 3);
		}
	}
	*tobs[i][nt] = '\0';

	/* change beidou B1 code: 3.02 draft -> 3.02 */
	if (i == 3) {
		for (j = 0; j < nt; j++) {
			if (tobs[i][j][1] == '2') {
				tobs[i][j][1] = '1';
			}
		}
	}
	/* if unknown code in ver.3, set default code */
	for (j = 0; j < nt; j++) {
		if (tobs[i][j][2]) { continue; }
		if (!(p = strchr(frqcodes, tobs[i][j][1]))) { continue; }
		tobs[i][j][2] = defcodes[i][(int)(p - frqcodes)];
	}
}

static void set_index(double ver, int sys, char tobs[MAXOBSTYPE][4], Sigind_t* ind, const char* opt)
{
	/* 局部变量定义 ========================================================= */
	const char* p;						// 系统首字母指针
	char str[8] = { 0 };				// 相位偏移相关选项[一般没用]
	const char* optstr = "";			// 相位偏移相关选项[一般没用]
	double shift;						// 相位偏移相关选项[一般没用]
	int i, j, n;						// 循环遍历变量
	int k0, k1, k2, k3;					// 临时位置变量
	/* ====================================================================== */

	/* 1.索引基本信息 */
	for (i = n = 0; *tobs[i]; i++, n++) {
		ind->code[i] = obs2code(tobs[i] + 1, ind->frq + i);
		ind->type[i] = (p = strchr(obscodes, tobs[i][0])) ? (unsigned char)(p - obscodes) : 0;
		ind->pri[i] = getcodepri(sys, ind->code[i], NULL);
		ind->pos[i] = -1;

		/* frequency index for beidou */
		if (sys == SYS_BDS) {
			if (ind->frq[i] == 5) ind->frq[i] = 2; /* B2 */
			else if (ind->frq[i] == 4) ind->frq[i] = 3; /* B3 */
		}
		/* frequency index for galileo, added by fzhou @ GFZ, 2017-04-10 */
		else if (sys == SYS_GAL) {
			if (ind->frq[i] == 3) ind->frq[i] = 2; /* E5a */
			else if (ind->frq[i] == 5) ind->frq[i] = 3; /* E5b */
		}
	}
	/* parse phase shift options [一般没用] */
	switch (sys) {
	case SYS_GPS: {optstr = "-GL%2s=%lf"; break; }
	case SYS_GLO: {optstr = "-RL%2s=%lf"; break; }
	case SYS_GAL: {optstr = "-EL%2s=%lf"; break; }
	case SYS_BDS: {optstr = "-CL%2s=%lf"; break; }
	}
	for (p = opt; p && (p = strchr(p, '-')); p++) {
		if (sscanf(p, optstr, str, &shift) < 2) { continue; }
		for (i = 0; i < n; i++) {
			if (strcmp(code2obs(ind->code[i], NULL), str)) { continue; }
			ind->shift[i] = shift;
			printf("phase shift: sys=%2d tobs=%s shift=%.3f\n", sys, tobs[i], shift);
		}
	}
	/* 2.assign index for highest priority code 根据优先级在对应pos[]处置数(0,1,2) */
	for (i = 0; i < NFREQ; i++) {		   // i遍历频率(3)
		k0 = k1 = k2 = k3 = -1;
		for (j = 0; j < n; j++) {		   // j遍历数据类型
			if (ind->type[j] == 0) {       // [0].伪距
				if (ind->frq[j] == i + 1 && ind->pri[j] && (k0<0 || ind->pri[j]>ind->pri[k0])) {
					k0 = j;
				}
			}
			else if (ind->type[j] == 1) {  // [1].相位
				if (ind->frq[j] == i + 1 && ind->pri[j] && (k1<0 || ind->pri[j]>ind->pri[k1])) {
					k1 = j;
				}
			}
			else if (ind->type[j] == 2) {  // [3].多普勒[一般没用]
				if (ind->frq[j] == i + 1 && ind->pri[j] && (k2<0 || ind->pri[j]>ind->pri[k2])) {
					k2 = j;
				}
			}
			else if (ind->type[j] == 3) {  // [3].信噪比[一般没用]
				if (ind->frq[j] == i + 1 && ind->pri[j] && (k3<0 || ind->pri[j]>ind->pri[k3])) {
					k3 = j;
				}
			}
		}
		if (k0 >= 0) { ind->pos[k0] = i; }
		if (k1 >= 0) { ind->pos[k1] = i; }
		if (k2 >= 0) { ind->pos[k2] = i; }
		if (k3 >= 0) { ind->pos[k3] = i; }
	}

	/* 测试代码，输出被排除的obs类型 */
	for (i = 0; i < n; i++) {
		if (!ind->code[i] || !ind->pri[i] || ind->pos[i] >= 0) { continue; }
		//printf("reject obs type: sys=%2d, obs=%s\n",sys,tobs[i]);
	}
	ind->n = n;
}

static int add_eph(NavSatData_t* sateph, const int sat, const NavData_t* eph)
{
	if (!eph) { return 0; }

	if (sateph->sat <= 0) { sateph->sat = sat; }

	sateph->naveph.insert(pair<GpsTime_t, NavData_t>(eph->toe, *eph));

	return 1;
}

static int add_geph(NavSatDataGlo_t* sateph, const int sat, const NavDataGlo_t* geph)
{
	if (!geph) { return 0; }

	if (sateph->sat <= 0) { sateph->sat = sat; }

	sateph->naveph.insert(pair<GpsTime_t, NavDataGlo_t>(geph->toe, *geph));

	return 1;
}

static void add_sat_eph(NavSatData_t* sateph, NavSatDataGlo_t* gsateph, NavPack_t* navall)
{
	int n = 0, m = 0;

	for (int i = 0; i < MAXSAT; i++) {
		if ((sateph + i)->sat > 0) {
			navall->eph.insert(pair<int, map<GpsTime_t, NavData_t>>((sateph + i)->sat, (sateph + i)->naveph));
			n += static_cast<int>((sateph + i)->naveph.size());
		}
		if ((gsateph + i)->sat > 0) {
			navall->geph.insert(pair<int, map<GpsTime_t, NavDataGlo_t>>((gsateph + i)->sat, (gsateph + i)->naveph));
			m += static_cast<int>((gsateph + i)->naveph.size());
		}
	}

	navall->n = n; navall->ng = m;
}

static int add_obs_data(ObsRecData_t* recdata, const ObsEphData_t* data)
{
	if (data->nsat <= 0) { return 0; }

	recdata->obseph.push_back(*data);
	recdata->neph++;

	return 1;
}

static void sort_obs(ObsRecData_t* recdata)
{
	/* 1. sorted by gps time in ascending order */
	//printf("sorting obs data...\n");
	std::sort(recdata->obseph.begin(), recdata->obseph.end(), compare_eph);

	/* 2. remove duplicated data */
	//printf("removing duplicated obs data...\n");
	for (auto tp = recdata->obseph.begin(); tp + 1 < recdata->obseph.end();) {
		if (tp->eph.time == (tp + 1)->eph.time &&
			tp->eph.sec == (tp + 1)->eph.sec) {
			tp = recdata->obseph.erase(tp);
		}
		else { tp++; }
	}
	recdata->neph = static_cast<int>(recdata->obseph.size());
	printf("   Duplicated data removed, epoch number=%4d\n", recdata->neph);
}

static int decode_eph(double ver, int sat, GpsTime_t toc, const double* data, NavData_t* eph) 
{
	/* 局部变量定义 ========================================================= */
	int sys;							// 系统
	/* 局部变量定义 ========================================================= */

	sys = satsys(sat, NULL);

	if (!(sys & (SYS_GPS | SYS_GAL | SYS_BDS))) {
		printf("ephemeris error: invalid satellite sat=%2d\n",sat);
		return 0;
	}

	eph->toc = toc;

	eph->f0 = data[0];
	eph->f1 = data[1];
	eph->f2 = data[2];

	eph->A    = SQR(data[10]); 
	eph->e    = data[8]; 
	eph->i0   = data[15]; 
	eph->OMG0 = data[13];
	eph->omg  = data[17];
	eph->M0   = data[6]; 
	eph->deln = data[5]; 
	eph->OMGd = data[18];
	eph->idot = data[19]; 
	eph->crc  = data[16]; eph->crs = data[4];
	eph->cuc  = data[7];  eph->cus = data[9]; 
	eph->cic  = data[12]; eph->cis = data[14];

	if (sys == SYS_GPS) {
		eph->iode = (int)data[3];
		eph->iodc = (int)data[26];
		eph->toes = data[11];
		eph->week = (int)data[21];
		eph->toe  = adjweek(gpst2time(eph->week, data[11]), toc);
		eph->ttr  = adjweek(gpst2time(eph->week, data[27]), toc);
		eph->code = (int)data[20];
		eph->svh  = (int)data[24];
		eph->sva  = ura_index(data[23]);
		eph->flag = (int)data[22];
		eph->tgd[0] = data[25];
		eph->fit = data[28];
	}
	else if (sys == SYS_GAL) {
		eph->iode = (int)data[3];
		eph->toes = data[11];
		eph->week = (int)data[21];
		eph->toe = adjweek(gpst2time(eph->week, data[11]), toc);
		eph->ttr = adjweek(gpst2time(eph->week, data[27]), toc);
		eph->code = (int)data[20];      /* data sources */
										/* bit 0 set: I/NAV E1-B */
										/* bit 1 set: F/NAV E5a-I */
										/* bit 2 set: F/NAV E5b-I */
										/* bit 8 set: af0-af2 toc are for E5a.E1 */
										/* bit 9 set: af0-af2 toc are for E5b.E1 */
		eph->svh = (int)data[24];       /* sv health */
										/* bit     0: E1B DVS */
										/* bit   1-2: E1B HS */
										/* bit     3: E5a DVS */
										/* bit   4-5: E5a HS */
										/* bit     6: E5b DVS */
										/* bit   7-8: E5b HS */
		eph->sva = ura_index(data[23]); /* ura (m->index) */
		eph->tgd[0] = data[25];			/* BGD E5a/E1 */
		eph->tgd[1] = data[26];			/* BGD E5b/E1 */
	}
	else if (sys == SYS_BDS) {
		eph->toc  = bdt2gpst(eph->toc);
		eph->iode = (int)data[3];
		eph->iodc = (int)data[28];
		eph->toes = data[11];
		eph->week = (int)data[21];
		eph->toe  = bdt2gpst(bdt2time(eph->week, data[11])); /* bdt -> gpst */
		eph->ttr  = bdt2gpst(bdt2time(eph->week, data[27])); /* bdt -> gpst */
		eph->toe  = adjweek(eph->toe, toc);
		eph->ttr  = adjweek(eph->ttr, toc);
		eph->svh  = (int)data[24];
		eph->sva  = ura_index(data[23]);
		eph->tgd[0] = data[25];			/* TGD1 B1/B3 */
		eph->tgd[1] = data[26];			/* TGD2 B2/B3 */
	}
	if (eph->iode < 0 || 1023 < eph->iode) {
		printf("rinex nav invalid: sat=%2d iode=%d\n", sat, eph->iode);
	}
	if (eph->iodc < 0 || 1023 < eph->iodc) {
		printf("rinex nav invalid: sat=%2d iodc=%d\n", sat, eph->iodc);
	}
	return 1;
}

static int decode_geph(double ver, int sat, GpsTime_t toc, double* data, NavDataGlo_t* geph) {
	GpsTime_t tof;
	double tow, tod;
	int week, dow;

	if (satsys(sat, NULL) != SYS_GLO) {
		printf("glonass ephemeris error: invalid satellite sat=%2d\n", sat);
		return 0;
	}

	/* toc rounded by 15 min in utc */
	tow = time2gpst(toc, &week);
	toc = gpst2time(week, floor((tow + 450.0) / 900.0) * 900);
	dow = (int)floor(tow / 86400.0);

	/* time of frame in utc */
	tod = ver <= 2.99 ? data[2] : fmod(data[2], 86400.0); /* tod (v.2), tow (v.3) in utc */
	tof = gpst2time(week, tod + dow * 86400.0);
	tof = adjday(tof, toc);

	geph->toe = utc2gpst(toc);   /* toc (gpst) */
	geph->tof = utc2gpst(tof);   /* tof (gpst) */

	/* iode = tb (7bit), tb =index of UTC+3H within current day */
	geph->iode = (int)(fmod(tow + 10800.0, 86400.0) / 900.0 + 0.5);

	geph->taun = -data[0];       /* -taun */
	geph->gamn =  data[1];       /* +gamman */

	geph->pos[0] = data[3] * 1E3; geph->pos[1] = data[7] * 1E3; geph->pos[2] = data[11] * 1E3;
	geph->vel[0] = data[4] * 1E3; geph->vel[1] = data[8] * 1E3; geph->vel[2] = data[12] * 1E3;
	geph->acc[0] = data[5] * 1E3; geph->acc[1] = data[9] * 1E3; geph->acc[2] = data[13] * 1E3;

	geph->svh = (int)data[6];
	geph->frq = (int)data[10];
	geph->age = (int)data[14];

	/* some receiver output >128 for minus frequency number */
	if (geph->frq > 128) { geph->frq -= 256; }

	if (geph->frq < MINFREQ_GLO || MAXFREQ_GLO < geph->frq) {
		printf("rinex gnav invalid freq: sat=%2d fn=%d\n", sat, geph->frq);
	}
	return 1;
}

static void decode_navh(char* buff, NavPack_t* nav) {
	/* 局部变量定义 ============================== */
	int i, j;					// 循环遍历变量
	char* label = buff + 60;	// 标签
	/* =========================================== */

	if (nav) {
		/* 1. [opt v.2] iono model parameters */
		if (strstr(label, "ION ALPHA")) {
			for (i = 0, j = 2; i < 4; i++, j += 12) { nav->ion_gps[i] = str2num(buff, j, 12); }
		}
		else if (strstr(label, "ION BETA")) {
			for (i = 0, j = 2; i < 4; i++, j += 12) { nav->ion_gps[i + 4] = str2num(buff, j, 12); }
		}
		/* 2. [opt v.2] gps utc parameters */
		else if (strstr(label, "DELTA-UTC: A0,A1,T,W")) {
			for (i = 0, j = 3; i < 2; i++, j += 19) { nav->utc_gps[i] = str2num(buff, j, 19); }
			for (; i < 4; i++, j += 9)				{ nav->utc_gps[i] = str2num(buff, j, 9); }
		}
		/* 3. [opt v.3] iono model parameters */
		else if (strstr(label, "IONOSPHERIC CORR")) {
			if (!strncmp(buff, "GPSA", 4)) {
				for (i = 0, j = 5; i < 4; i++, j += 12) { nav->ion_gps[i] = str2num(buff, j, 12); }
			}
			else if (!strncmp(buff, "GPSB", 4)) {
				for (i = 0, j = 5; i < 4; i++, j += 12) { nav->ion_gps[i + 4] = str2num(buff, j, 12); }
			}
			else if (!strncmp(buff, "GAL", 3)) {
				for (i = 0, j = 5; i < 4; i++, j += 12) { nav->ion_gal[i] = str2num(buff, j, 12); }
			}
			else if (!strncmp(buff, "BDSA", 4)) { /* v.3.02 */
				for (i = 0, j = 5; i < 4; i++, j += 12) { nav->ion_bds[i] = str2num(buff, j, 12); }
			}
			else if (!strncmp(buff, "BDSB", 4)) { /* v.3.02 */
				for (i = 0, j = 5; i < 4; i++, j += 12) { nav->ion_bds[i + 4] = str2num(buff, j, 12); }
			}
		}
		/* 4. [opt v.3] gps utc parameters */
		else if (strstr(label, "TIME SYSTEM CORR")) {
			if (!strncmp(buff, "GPUT", 4)) {
				nav->utc_gps[0] = str2num(buff, 5, 17);
				nav->utc_gps[1] = str2num(buff, 22, 16);
				nav->utc_gps[2] = str2num(buff, 38, 7);
				nav->utc_gps[3] = str2num(buff, 45, 5);
			}
			else if (!strncmp(buff, "GLUT", 4)) {
				nav->utc_glo[0] = str2num(buff, 5, 17);
				nav->utc_glo[1] = str2num(buff, 22, 16);
			}
			else if (!strncmp(buff, "GAUT", 4)) { /* v.3.02 */
				nav->utc_gal[0] = str2num(buff, 5, 17);
				nav->utc_gal[1] = str2num(buff, 22, 16);
				nav->utc_gal[2] = str2num(buff, 38, 7);
				nav->utc_gal[3] = str2num(buff, 45, 5);
			}
			else if (!strncmp(buff, "QZUT", 4)) { /* v.3.02 */
				return;
			}
			else if (!strncmp(buff, "BDUT", 4)) { /* v.3.02 */
				nav->utc_bds[0] = str2num(buff, 5, 17);
				nav->utc_bds[1] = str2num(buff, 22, 16);
				nav->utc_bds[2] = str2num(buff, 38, 7);
				nav->utc_bds[3] = str2num(buff, 45, 5);
			}
			else if (!strncmp(buff, "SBUT", 4)) { /* v.3.02 */
				nav->utc_bds[0] = str2num(buff, 5, 17);
				nav->utc_bds[1] = str2num(buff, 22, 16);
				nav->utc_bds[2] = str2num(buff, 38, 7);
				nav->utc_bds[3] = str2num(buff, 45, 5);
			}
		}
		/* 5. [opt] leap seconds */
		else if (strstr(label, "LEAP SECONDS")) {
			nav->leaps = (int)str2num(buff, 0, 6);
		}
	}
}

static void decode_obsh(FILE* fp, char* buff, double ver, int* tsys, char tobs[][MAXOBSTYPE][4], NavPack_t* navall, Station_t* sta) 
{
	/* 局部变量定义 ===================================================================== */
	/* default codes for unknown code */
	int i;							// 循环遍历变量
	int prn, fcn; 					// Code总数/Code索引/卫星PRN/GLONASS频道号
	char* label = buff + 60;		// 标签/obs码类型
	char* p;
	/* ================================================================================= */

	/* 1. station information */
	if (sta) { sta_info(sta, buff, label); }

	/* 2. obs type(ver 3.) */
	if (strstr(label, "SYS / # / OBS TYPES")) { obsTypeV3(fp, buff, tobs); }

	/* 3. obs type(ver 2.) */
	else if (strstr(label, "# / TYPES OF OBSERV")) { obsTypeV2(fp, ver, buff, tobs); }

	/* 4. time system */
	else if (strstr(label, "TIME OF FIRST OBS")) {
		if (!strncmp(buff + 48, "GPS", 3)) { *tsys = TSYS_GPS; }
		else if (!strncmp(buff + 48, "GLO", 3)) { *tsys = TSYS_UTC; }
		else if (!strncmp(buff + 48, "GAL", 3)) { *tsys = TSYS_GAL; }
		else if (!strncmp(buff + 48, "BDT", 3)) { *tsys = TSYS_BDS; } /* ver.3.02 */
	}

	/* 5. ignored information */
	if (strstr(label, "MARKER TYPE"));
	else if (strstr(label, "OBSERVER / AGENCY"));
	else if (strstr(label, "WAVELENGTH FACT L1/2")); /* opt ver.2 */
	else if (strstr(label, "ANTENNA: DELTA X/Y/Z")); /* opt ver.3 */
	else if (strstr(label, "ANTENNA: PHASECENTER")); /* opt ver.3 */
	else if (strstr(label, "ANTENNA: B.SIGHT XYZ")); /* opt ver.3 */
	else if (strstr(label, "ANTENNA: ZERODIR AZI")); /* opt ver.3 */
	else if (strstr(label, "ANTENNA: ZERODIR XYZ")); /* opt ver.3 */
	else if (strstr(label, "CENTER OF MASS: XYZ"));  /* opt ver.3 */
	else if (strstr(label, "SIGNAL STRENGTH UNIT")); /* opt ver.3 */
	else if (strstr(label, "SYS / DCBS APPLIED"));   /* opt ver.3 */
	else if (strstr(label, "SYS / PCVS APPLIED"));   /* opt ver.3 */
	else if (strstr(label, "SYS / SCALE FACTOR"));   /* opt ver.3 */
	else if (strstr(label, "SYS / PHASE SHIFTS"));   /* ver.3.01 */
	else if (strstr(label, "INTERVAL"));			 /* opt */
	else if (strstr(label, "TIME OF LAST OBS"));	 /* opt */
	else if (strstr(label, "RCV CLOCK OFFS APPL"));  /* opt */
	else if (strstr(label, "# OF SALTELLITES"));	 /* opt */
	else if (strstr(label, "PRN / # OF OBS"));		 /* opt */

	else if (strstr(label, "GLONASS SLOT / FRQ #")) { /* ver.3.02 GLONASS频道号 */
		if (navall) {
			for (i = 0, p = buff + 4; i < 8; i++, p += 8) {
				if (sscanf(p, "R%2d %2d", &prn, &fcn) < 2) { continue; }
				if (1 <= prn && prn <= MAXPRNGLO) { 
					navall->glo_fcn[prn - 1] = fcn + 8; 
				}
			}
		}
	}
	else if (strstr(label, "GLONASS COD/PHS/BIS")) { /* ver.3.02 */
		if (navall) {
			for (i = 0, p = buff; i < 4; i++, p += 13) {
				if		(strncmp(p + 1, "C1C", 3)) { navall->glo_cpbias[0] = str2num(p, 5, 8); }
				else if (strncmp(p + 1, "C1P", 3)) { navall->glo_cpbias[1] = str2num(p, 5, 8); }
				else if (strncmp(p + 1, "C2C", 3)) { navall->glo_cpbias[2] = str2num(p, 5, 8); }
				else if (strncmp(p + 1, "C2P", 3)) { navall->glo_cpbias[3] = str2num(p, 5, 8); }
			}
		}
	}
	else if (strstr(label, "LEAP SECONDS")) {		/* opt */
		if (navall) {
			navall->leaps = (int)str2num(buff, 0, 6);
		}
	}
}

static int decode_obsepoch(FILE* fp, char* buff, double ver, GpsTime_t* time, int* flag, int* sats) {
	int i, j, n;
	char satid[8] = "";

	/* ver.2 */
	if (ver <= 2.99) {
		if ((n = (int)str2num(buff, 29, 3)) <= 0) { return 0; }

		/* epoch flag: 3:new site,4:header info,5:external event */
		*flag = (int)str2num(buff, 28, 1);

		if (3 <= *flag && *flag <= 5) { return n; }

		if (str2time(buff, 0, 26, time)) {
			printf("rinex obs invalid epoch: epoch=%26.26s\n", buff);
			return 0;
		}
		for (i = 0, j = 32; i < n; i++, j += 3) {
			if (j >= 68) {
				if (!fgets(buff, MAXRNXLEN, fp)) break;
				j = 32;
			}
			if (i < MAXOBS) {
				strncpy(satid, buff + j, 3);
				sats[i] = satid2no(satid);
			}
		}
	}
	/* ver.3 */
	else {
		if ((n = (int)str2num(buff, 32, 3)) <= 0) { return 0; }

		*flag = (int)str2num(buff, 31, 1);

		if (3 <= *flag && *flag <= 5) { return n; }

		if (buff[0] != '>' || str2time(buff, 1, 28, time)) {
			//printf("rinex obs invalid epoch: epoch=%29.29s\n", buff);
			return 0;
		}
	}

	return n;
}

static int decode_obsdata(FILE* fp, char* buff, double ver, int mask, Sigind_t* index, ObsEphData_t* obs, int* sats) {
	/* 局部变量定义 ========================================================= */
	Sigind_t* ind;							// 信号索引
	double val[MAXOBSTYPE] = { 0 };			// 每行的值(伪距/相位)
	unsigned char lli[MAXOBSTYPE] = { 0 };	// 失锁指示符
	char satid[8] = "";						// 卫星ID(G01,G02...)
	int i, j, n, m;							// 循环遍历变量
	int k[16], l[16];						// L1/L2伪距信号索引(ver.2)
	int stat = 1, p[MAXOBSTYPE];			// 状态指示符/位置指示(freq)
	int sat = 0;
	ObsData_t satdata = { 0 };
	/* ====================================================================== */

	/* 1.ver.3 获取每行开始的卫星ID */
	if (ver > 2.99) {
		strncpy(satid, buff, 3);
		sat = (unsigned char)satid2no(satid);
	}
	if (!(satsys(sat, NULL) & mask)) { stat = 0; }

	/* 2.read obs data fields 根据卫星号，选择系统索引 */
	switch (satsys(sat, NULL)) {
	case SYS_GLO: {ind = index + 1; break; }
	case SYS_GAL: {ind = index + 2; break; }
	case SYS_BDS: {ind = index + 3; break; }
	default:	  {ind = index;	    break; }
	}
	/* 3.根据版本ver，获取每行观测值val和失锁指示符lli */
	for (i = 0, j = ver <= 2.99 ? 0 : 3; i < ind->n; i++, j += 16) {
		if (ver <= 2.99 && j >= 80) { /* ver.2 */
			if (!fgets(buff, MAXRNXLEN, fp)) { break; }
			j = 0;
		}
		if (stat) {
			val[i] = str2num(buff, j, 14) + ind->shift[i];
			lli[i] = (unsigned char)str2num(buff, j + 14, 1) & 3;
		}
	}
	if (!stat) { return 0; }
	
	/* 5.assign position in obs data 根据版本ver，设定信号所在位置 */
	for (i = n = m = 0; i < ind->n; i++) {
		p[i] = ver <= 2.11 ? ind->frq[i] - 1 : ind->pos[i];

		if (ind->type[i] == 0 && p[i] == 0) { k[n++] = i; } /* C1? index */
		if (ind->type[i] == 0 && p[i] == 1) { l[m++] = i; } /* C2? index */
	}
	if (ver <= 2.11) {
		/* if multiple codes (C1/P1,C2/P2), select higher priority */
		if (n >= 2) {
			if (val[k[0]] == 0.0 && val[k[1]] == 0.0) {
				p[k[0]] = -1; p[k[1]] = -1;
			}
			else if (val[k[0]] != 0.0 && val[k[1]] == 0.0) {
				p[k[0]] = 0; p[k[1]] = -1;
			}
			else if (val[k[0]] == 0.0 && val[k[1]] != 0.0) {
				p[k[0]] = -1; p[k[1]] = 0;
			}
			else if (ind->pri[k[1]] > ind->pri[k[0]]) {
				p[k[1]] = 0; p[k[0]] = -1;
			}
			else {
				p[k[0]] = 0; p[k[1]] = -1;
			}
		}
		if (m >= 2) {
			if (val[l[0]] == 0.0 && val[l[1]] == 0.0) {
				p[l[0]] = -1; p[l[1]] = -1;
			}
			else if (val[l[0]] != 0.0 && val[l[1]] == 0.0) {
				p[l[0]] = 1; p[l[1]] = -1;
			}
			else if (val[l[0]] == 0.0 && val[l[1]] != 0.0) {
				p[l[0]] = -1; p[l[1]] = 1;
			}
			else if (ind->pri[l[1]] > ind->pri[l[0]]) {
				p[l[1]] = 1; p[l[0]] = -1;
			}
			else {
				p[l[0]] = 1; p[l[1]] = -1;
			}
		}
	}
	/* 6.save obs data */
	j = 0;
	for (i = 0; i < ind->n; i++) {
		if (p[i] < 0 || val[i] == 0.0) { continue; }
		switch (ind->type[i]) {
		case 0: {	// 伪距
			satdata.   P[p[i]] = val[i];
			satdata.code[p[i]] = ind->code[i];
			satdata.type[p[i]] = (char*)code2obs(satdata.code[p[i]], &j);
			break;
		}
		case 1: {	// 相位
			satdata.   L[p[i]] = val[i];

			satdata. LLI[p[i]] = lli[i];
			break;
		}
		case 2: {break; }
		case 3: {break; }
		}
	}
	/* qhy add */
	if (ver > 2.99) {
		obs->obssat.insert(pair<int, ObsData_t>(sat, satdata));
		obs->nsat++;
	}
	else{
		sat = *(sats + obs->nsat);
		obs->obssat.insert(pair<int, ObsData_t>(sat, satdata));
		obs->nsat++;
	}

	return 1;
}

static int read_rnx_obsb(FILE* fp, double ver, char tobs[][MAXOBSTYPE][4], ObsEphData_t* data) {
	/* 局部变量定义 ===================================================================== */
	GpsTime_t time = { 0 };							// GPS时间变量
	Sigind_t index[4] = { {0} };					// 信号索引
	char buff[MAXRNXLEN];							// obs每行字符串
	int i = 0;										// 循环遍历变量
	int n = 0;										// 符合要求的卫星数
	int nsat = 0, sats[MAXOBS] = { 0 }, mask;		// 卫星数量/卫星号/系统筛选
	int flag = 0;									// event flag
	/* ================================================================================== */

	/* 1.set system mask 设置系统mask */
	mask = SYS_ALL;

	/* 2.set signal index设置索引index */
	set_index(ver, SYS_GPS, tobs[0], index, NULL);
	set_index(ver, SYS_GLO, tobs[1], index + 1, NULL);
	set_index(ver, SYS_GAL, tobs[2], index + 2, NULL);
	set_index(ver, SYS_BDS, tobs[3], index + 3, NULL);

	/* 3.read record 读文件体 */
	while (fgets(buff, MAXRNXLEN, fp)) {
		/* 3.1 decode obs epoch */
		if (i == 0) {
			if ((nsat = decode_obsepoch(fp, buff, ver, &time, &flag, sats)) <= 0) {
				continue;
			}
		}
		else if (flag <= 2 || flag == 6) {
			data->eph = time;
			/* 3.2 decode obs data */
			if (decode_obsdata(fp, buff, ver, mask, index, data, sats) && n < MAXOBS) {
				n++;
			}
		}
		if (++i > nsat) { return n; }
	}
	return -1;
}

static int read_rnx_obs(FILE* fp, double ver, int tsys, char tobs[][MAXOBSTYPE][4], vector<ObsRecData_t>* obsall) {
	/* 局部变量定义 ===================================================================== */
	ObsRecData_t recdata = { 0 };
	ObsEphData_t data = { 0 };						// 单个obs数据
	unsigned char slips[MAXSAT][NFREQ] = { {0} };	// 周跳标记
	int n; 											// obs数据个数
	int stat = 0;									// 时间标记符/状态标记符
	map<int, ObsData_t>::iterator it;
	GpsTime_t tmp = {0};
	/* ================================================================================== */

	recdata.rec = rec_num++;

	while ((n = read_rnx_obsb(fp, ver, tobs, &data)) >= 0 && stat >= 0) {
		/*for (it = data.obssat.begin(); it != data.obssat.end(); it++) {
			printf("Sat=%4d, P1=%14.3f, L1=%15.3f\n", it->first, it->second.P[0], it->second.L[0]);
		}
		printf("vol of map is %lu", data.obssat.size());
		system("pause");*/

		/* utc -> gpst */
		if (tsys == TSYS_UTC) { data.eph = utc2gpst(data.eph); }

		/* save cycle-slip */
		for (it = data.obssat.begin(); it != data.obssat.end(); it++) { 
			saveslips(slips, it); 
		}

		/* screen data by time */
		if (n > 0 && !screent(data.eph, tmp, tmp, 0.0)) { continue; }

		/* restore cycle-slip & save obs data */
		for (it = data.obssat.begin(); it != data.obssat.end(); it++) {
			restslips(slips, it);
		}

		stat = add_obs_data(&recdata, &data);

		data = { 0 };
	}

	sort_obs(&recdata);

	if (stat >= 0) { obsall->push_back(recdata); }

	return stat;
}

static int read_rnx_navb(FILE* fp, double ver, int sys, int* sat, int* type, NavData_t* eph, NavDataGlo_t* geph) {
	/* 局部变量定义 ========================================================= */
	GpsTime_t toc;					// 星历播发时间
	double data[64];				// 星历数据
	int i = 0, j;					// 循环遍历变量
	int prn, sp = 3, mask;			// 卫星PRN/卫星号/空格数/卫星系统
	char buff[MAXRNXLEN];			// 读入每行数据
	char id[8] = "", * p;			// 卫星ID/读文件指针
	/* ====================================================================== */

	/* set system mask */
	mask = SYS_ALL;

	while (fgets(buff, MAXRNXLEN, fp)) {
		if (i == 0) {	// 第一行(带toc那行)
			/* decode satellite field [ver.3 or GAL/QZS] */
			if (ver >= 3.0 || sys == SYS_GAL) {
				strncpy(id, buff, 3);	// 获取卫星ID(G01,G02...)
				*sat = satid2no(id);		// ID->sat
				sp  = 4;
				if (ver >= 3.0) { sys = satsys(*sat, NULL); }
			}
			else {
				prn = (int)str2num(buff, 0, 2);

				if (sys == SYS_GLO) {
					*sat = satno(SYS_GLO, prn);
				}
				else if (93 <= prn && prn <= 97) { /* extension */
				}
				else { *sat = satno(SYS_GPS, prn); }
			}
			/* decode toc field 解析发送时间 */
			if (str2time(buff + sp, 0, 19, &toc)) {
				printf("rinex nav toc error: %23.23s\n", buff);
				return 0;
			}
			/* decode data fields 解析每行数据 */
			for (j = 0, p = buff + sp + 19; j < 3; j++, p += 19) {
				data[i++] = str2num(p, 0, 19);
			}
		}
		else {	// 后几行
			/* decode data fields */
			for (j = 0, p = buff + sp; j < 4; j++, p += 19) {
				data[i++] = str2num(p, 0, 19);
			}
			/* decode GPS/GAL/BDS ephemeris 解码其它星历 */
			if (sys != SYS_GLO && i >= 31) {
				if (!(mask & sys)) { return 0; }
				*type = 0;
				return decode_eph(ver, *sat, toc, data, eph);
			}
			/* decode GLONASS ephemeris 解码GLONASS星历 */
			if (sys == SYS_GLO && i >= 15) {
				if (!(mask & sys)) { return 0; }
				*type = 1;
				return decode_geph(ver, *sat, toc, data, geph);
			}
		}
	}
	return -1;
}

static int read_rnx_nav(FILE* fp, double ver, int sys, NavPack_t* navall) {
	/* 局部变量定义 ========================================================= */
	NavData_t	  eph = { 0 };				// GPS星历变量结构体
	NavDataGlo_t geph = { 0 };				// GLO星历变量结构体
	NavSatData_t     sateph[MAXSAT] = { 0 };
	NavSatDataGlo_t gsateph[MAXSAT] = { 0 };
	int stat, type;							// 状态标记/类型标记
	int sat = 0;							// 卫星号
	map<int, map<GpsTime_t, NavData_t>>::iterator it;
	map<int, map<GpsTime_t, NavDataGlo_t>>::iterator it_g;
	/* ====================================================================== */

	if (!navall) { return 0; }

	/* read rinex navigation data body */
	while ((stat = read_rnx_navb(fp, ver, sys, &sat, &type, &eph, &geph)) >= 0) {
		/* add ephemeris to navigation data */
		if (stat) {
			switch (type) {
			case  1: { stat = add_geph(&gsateph[sat-1], sat, &geph); break; }
			default: { stat = add_eph(&sateph[sat-1], sat, &eph); break; }
			}
			if (!stat) { return 0; }
		}
	}
	add_sat_eph(sateph, gsateph, navall);

	for (it = navall->eph.begin(); it != navall->eph.end(); it++) {
		sat = it->first;
		navall->lam.insert(pair<int, vector<double>>(sat, vector<double>(NFREQ, 0.0)));
		for (int j = 0; j < NFREQ; j++) {
			navall->lam[sat][j] = sat_wavelen(sat, j, navall);
		}
	}

	for (it_g = navall->geph.begin(); it_g != navall->geph.end(); it_g++) {
		sat = it_g->first;
		navall->lam.insert(pair<int, vector<double>>(sat, vector<double>(NFREQ, 0.0)));
		for (int j = 0; j < NFREQ; j++) {
			navall->lam[sat][j] = sat_wavelen(sat, j, navall);
		}
	}

	return navall->n > 0 || navall->ng > 0;
}

static int read_rnxh(FILE* fp, double* ver, char* type, int* sys, int* tsys, char tobs[][MAXOBSTYPE][4], NavPack_t* navall, Station_t* sta) {
	/* 局部变量定义 ===================================================================== */
	char buff[MAXRNXLEN];				// 读文件字符串
	char* label = buff + 60;			// 每行文件头标记
	int i = 0;							// 读行数计数器（max = 1024）
	/* ================================================================================== */

	*ver = 2.10; *type = ' '; *sys = SYS_GPS; *tsys = TSYS_GPS;

	while (fgets(buff, MAXRNXLEN, fp)) {
		if		(strlen(buff) <= 60)				   { continue; }
		else if (strstr(label, "PGM / RUN BY / DATE")) { continue; }
		else if (strstr(label, "COMMENT"))			   { continue; }

		/* 1. set [ver, type, sys, tsys] */
		else if (strstr(label, "RINEX VERSION / TYPE")) {
			*ver = str2num(buff, 0, 9);
			*type = *(buff + 20);
			switch (*(buff + 40)) {
			case 'G': {*sys = SYS_GPS;  *tsys = TSYS_GPS; break; }
			case 'R': {*sys = SYS_GLO;  *tsys = TSYS_UTC; break; }
			case 'E': {*sys = SYS_GAL;  *tsys = TSYS_GAL; break; }
			case 'C': {*sys = SYS_BDS;  *tsys = TSYS_BDS; break; }
			case 'M': {*sys = SYS_NONE; *tsys = TSYS_GPS; break; }
			default: {printf("not supported satellite system: %c\n", *(buff + 40)); break; }
			}
			continue;
		}
		/* 2. decode rnx head */
		switch (*type) {
		case 'O': {decode_obsh(fp, buff, *ver, tsys, tobs, navall, sta); break; }
		case 'N': {decode_navh(buff, navall); break; }
		}

		if (strstr(label, "END OF HEADER"))    { return 1; }/* Right: read head finished */
		if (++i >= MAXPOSHEAD && *type == ' ') { break; }	/* Error: no rinex file      */
	}
	return 0;
}

static int read_rnxfp(FILE* fp, char* type, vector<ObsRecData_t>* obsall, NavPack_t* navall, Station_t* sta) {
	/* 局部变量定义 ============================================================== */
	double ver;										// RINEX版本
	int sys, tsys;									// 卫星系统/时间系统
	char tobs[NUMSYS][MAXOBSTYPE][4] = { {""} };	// Code类型
	/* =========================================================================== */

	/* 1. read rinex header */
	if (!read_rnxh(fp, &ver, type, &sys, &tsys, tobs, navall, sta)) { return 0; }

	/* 2. read rinex body */
	switch (*type) {
	case 'O': { return read_rnx_obs(fp, ver, tsys, tobs, obsall); }
	case 'N': { return read_rnx_nav(fp, ver, sys, navall); }
	}

	return 0;
}

extern int read_obsnav(GpsTime_t ts, GpsTime_t te, double ti, vector<const char*> infile, vector<ObsRecData_t>* obsall, NavPack_t* navall, Station_t* sta) {
	/* 局部变量定义 ===================================================================== */
	int nep;							// eph计数
	FILE* fp;							// 读文件指针
	char type = ' ';					// rinex 类型
	GpsTime_t first, last;				// obs中开始、结束历元gps时间
	char* p;
	/* ================================================================================== */

	for (int i = 0; i < infile.size(); i++) {
		p = (char*)strrchr(infile[i], '\\');

		if (obsall && navall) {
			printf(">> Rinex file %2d-th %s reading...\n", i + 1, p + 1);
		}
		if (!(fp = fopen(infile[i], "r"))) {
			printf("   ERROR: open RINEX file %s fail\n", p + 1);
			return 0;
		}
		/* 1.read rinex obs and nav file */
		nep = read_rnxfp(fp, &type, obsall, navall, sta);
		std::fclose(fp);
	}
	if (obsall && obsall->size() <= 0) {
		printf("   ERROR: no obs data!\n");
		return 0;
	}
	if (navall && navall->n <= 0 && navall->ng <= 0) {
		printf("   ERROR: no nav data!\n");
		return 0;
	}

	/* 2.set time span for progress display */
	if (ts.time == 0 || te.time == 0) {
		first = (*obsall)[0].obseph.begin()->eph;
		last  = (*obsall)[0].obseph.rbegin()->eph;
		
		if (first < last) {
			if (ts.time == 0) { ts = first; }
			if (te.time == 0) { te = last;  }
		}
	}

	return 1;
}

/* read ATX files -------------------------------------------------------------------------------------------------------*/

static int decode_atx(char* p, int n, double* v)
{
	int i;

	for (i = 0; i < n; i++) { v[i] = 0.0; }
	for (i = 0, p = strtok(p, " "); p && i < n; p = strtok(NULL, " ")) {
		v[i++] = atof(p) * 1E-3;
	}

	return i;
}

static int search_pcv(int sat, const char* type, GpsTime_t time, vector<PCV_t>* pcvs, PCV_t* pcv)
{
	char buff[MAXANT], * types[2], * p;
	int i, j, n = 0;

	/* 1.search satellite antenna */
	if (sat) {
		for (i = 0; i < pcvs->size(); i++) {
			if (pcvs->at(i).sat	== 0 && 
				pcvs->at(i).ts.time == 0.0 && 
				pcvs->at(i).te.time == 0.0) { 
				break; 
			}
			if (pcvs->at(i).sat != sat)											  { continue; }
			if (pcvs->at(i).ts.time != 0 && timediff(pcvs->at(i).ts, time) > 0.0) { continue; }
			if (pcvs->at(i).te.time != 0 && timediff(pcvs->at(i).te, time) < 0.0) { continue; }
			
			*pcv = pcvs->at(i);

			return 1;
		}
	}
	/* 2.search receiver antenna */
	else {
		strcpy(buff, type);
		for (p = strtok(buff, " "); p && n < 2; p = strtok(NULL, " ")) {
			types[n++] = p;
		}
		if (n <= 0) { return 0; }

		/* search receiver antenna with radome at first */
		for (i = 0; i < pcvs->size(); i++) {
			if (pcvs->at(i).sat > 0 ||
				pcvs->at(i).ts.time != 0.0 ||
				pcvs->at(i).te.time != 0.0) {
				continue;
			}

			for (j = 0; j < n; j++) {
				if (!strstr((*pcvs)[i].type, types[j])) {
					break;
				}
			}
			if (j >= n) { 
				(*pcv) = (*pcvs)[i]; 
				return 1;
			}
		}

		/* search receiver antenna without radome */
		for (i = 0; i < pcvs->size(); i++) {
			if (pcvs->at(i).sat > 0 ||
				pcvs->at(i).ts.time != 0.0 ||
				pcvs->at(i).te.time != 0.0) {
				continue;
			}
			
			if (strstr((*pcvs)[i].type, types[0]) != (*pcvs)[i].type) { continue; }
			
			(*pcv) = (*pcvs)[i];

			return 1;
		}
	}

	return 0;
}

extern int read_pcv(vector<const char*> atxfile, vector<PCV_t>* pcvs)
{
	/* 局部变量定义 ======================================================== */
	FILE* fp;								// Atx文件指针
	PCV_t pcv;								// pcv结构体
	double neu[3];							// [E、N、U]或[X、Y、Z]方向偏移量
	double col;								// 文件中pcv的列数
	int row;								// 文件中pcv的行数
	int j, k;								// 循环遍历变量
	int f, freq = 0;						// 频率
	int prn;								// 卫星PRN
	char buff[1024], csys;					// Atx字符串/卫星系统首字母(G,R,E,C)
	char* p;
	/* ===================================================================== */

	for (int i = 0; i < atxfile.size(); i++) {
		p = (char*)strrchr(atxfile[i], '\\');
		printf(">> ATX file %s reading...\n", p + 1);
		if (!(fp = fopen(atxfile[i], "r"))) {
			printf("   ERROR: open ATX file %s fail\n", p + 1);
			return 0;
		}

		while (fgets(buff, sizeof(buff), fp)) {
			if (strlen(buff) < 60 || strstr(buff + 60, "COMMENT")) { continue; }
			if (strstr(buff + 60, "START OF ANTENNA")) {
				pcv = { 0 };
			}
			if (strstr(buff + 60, "END OF ANTENNA")) {
				pcvs->push_back(pcv);
				//addpcv(&pcv, pcvs);
				continue;
			}
			// 天线类型、卫星码、卫星号
			if (strstr(buff + 60, "TYPE / SERIAL NO")) {
				strncpy(pcv.type, buff, 20);
				pcv.type[20] = '\0';
				strncpy(pcv.code, buff + 20, 20);
				pcv.code[20] = '\0';
				if (!(prn = (int)str2num(pcv.code, 1, 2))) {	// 如果无法解析卫星号，直接跳过后续
					continue;
				}
				if (!strncmp(pcv.code + 3, "        ", 8)) {
					pcv.sat = satid2no(pcv.code);
				}
			}
			// 开始、结束时间
			else if (strstr(buff + 60, "VALID FROM")) {
				if (!str2time(buff, 0, 43, &pcv.ts)) { continue; }
			}
			else if (strstr(buff + 60, "VALID UNTIL")) {
				if (!str2time(buff, 0, 43, &pcv.te)) { continue; }
			}
			// 方位角步长
			else if (strstr(buff + 60, "DAZI")) {
				pcv.dazi = str2num(buff, 2, 6);
				continue;
			}
			// 天顶角的起始、结束、步长
			else if (strstr(buff + 60, "ZEN1 / ZEN2 / DZEN")) {
				pcv.zen1 = str2num(buff, 2, 6);
				pcv.zen2 = str2num(buff, 8, 6);
				pcv.dzen = str2num(buff, 14, 6);
				continue;
			}

			else if (strstr(buff + 60, "START OF FREQUENCY")) {
				if (sscanf(buff + 4, "%d", &f) < 1) { continue; }	// 读频率数
				if (sscanf(buff + 3, "%c", &csys) < 1) { continue; }	// 读系统

				// 根据系统csys设置频率freq(G/R/C/E/J)
				if		(csys == 'G') { freq = f; }
				else if (csys == 'R') { freq = f + NFREQ; }
				else if (csys == 'C') { freq = f + 2 * NFREQ; }
				else if (csys == 'E') {
					if		(f == 1)  { freq = 1 + 3 * NFREQ; } // E1
					else if (f == 5)  { freq = 2 + 3 * NFREQ; } // E5
					else if (f == 6)  { freq = 3 + 3 * NFREQ; } // E6	
					else			  { freq = 0; }
				}
				else if (csys == 'J') {
					if		 (f < 5) { freq = f + 4 * NFREQ; }
					else if (f == 5) { freq = 3 + 4 * NFREQ; }
					else			 { freq = 0; }
				}
				else { freq = 0; }
			}
			else if (strstr(buff + 60, "END OF FREQUENCY")) { freq = 0; }

			/* 相位中心变化(PCO) ------------------------------------------------------
			 * 卫星                     | 平均天线相位中心到卫星质心的XYZ变化量(mm)
			 * 接收机                   | 平均天线相位中心到天线参考点ARP的ENU变化量(mm)
			 ----------------------------------------------------------------------- */
			else if (strstr(buff + 60, "NORTH / EAST / UP")) {
				if (decode_atx(buff, 3, neu) < 3) { continue; }
				if (freq < 1) { continue; }
				pcv.off[freq - 1][0] = neu[pcv.sat ? 0 : 1];	/* x or e */
				pcv.off[freq - 1][1] = neu[pcv.sat ? 1 : 0];	/* y or n */
				pcv.off[freq - 1][2] = neu[2];					/* z or u */
			}
			// 接收机PCV考虑随方位角的变化
			else if (strstr(buff, "NOAZI")) {
				if (freq < 1) { continue; }
				col = (pcv.zen2 - pcv.zen1) / pcv.dzen + 1;

				if (col != myRound(col) || col <= 1) {
					printf("   WARNING: zen in atx file error (d!=round(d)||d<1)!\n");
					continue;
				}

				// 相位中心变化(PCV)
				if (pcv.dazi == 0.0) {
					k = decode_atx(buff + 8, (int)col, pcv.var[freq - 1]);
					if (k <= 0) {
						printf("   ERROR: error in reading atx (k<=0)!\n");
						continue;
					}
					else if (k != (int)col) {
						printf("   ERROR: error in reading atx (k!=%d)!\n", (int)col);
						continue;
					}
				}
				else {
					row = (int)((360 - 0) / pcv.dazi) + 1;

					for (k = 0; k < row; k++) {
						fgets(buff, sizeof(buff), fp);

						j = decode_atx(buff + 8, (int)col, &pcv.var[freq - 1][k * (int)col]);
						if (j <= 0) {
							printf("   ERROR: error in reading atx (j<=0)!\n");
							continue;
						}
						else if (j != (int)col) {
							printf("   ERROR: error in reading atx (j!=%d)!\n", (int)col);
							continue;
						}
					}
				}
			}
		}
		std::fclose(fp);
	}
	return 1;
}

extern void set_pcv(GpsTime_t time, ProcOpt_t* popt, NavPack_t* navall, vector<PCV_t> *pcvs, const Station_t* sta)
{
	PCV_t pcv;
	double dt;
	int i, j, k = 0, sys;
	char id[8];
	int ret = 0;
	vector<double> pos = vector<double>(3, 0.0);
	vector<double> xyz = vector<double>(3, 0.0);
	vector<double> del = vector<double>(3, 0.0);
	vector<double> enu = vector<double>(3, 0.0);

	/* 1.set satellite antenna parameters */
	for (i = 0; i < MAXSAT; i++) {
		sys = satsys(i + 1, NULL);
		if (!(sys & popt->nav_sys))   { continue; }
		ret = search_pcv(i + 1, "", time, pcvs, &pcv);
		if (pcv.ts.time == (time_t)0) { continue; }
		satno2id(i + 1, id);
		navall->sat_pcv.insert(pair<int, PCV_t>(i + 1, pcv));

		// pcv列元素
		if (pcv.dzen == 0.0) { j = 10; }
		else { j = myRound((pcv.zen2 - pcv.zen1) / pcv.dzen); }
		// pcv行元素
		if		(sys == SYS_GPS) { k = 0; }
		else if (sys == SYS_GLO) { k = 0 + 1 * NFREQ; }
		else if (sys == SYS_BDS) { k = 0 + 2 * NFREQ; }
		else if (sys == SYS_GAL) { k = 0 + 3 * NFREQ; }

		// 计算pcv是否正常
		dt = norm(pcv.var[k], j);
		/*if (dt <= 0.0001) {
			printf("%s ATTENTION! PRELIMINARY PHASE CENTER CORRECTIONS!\n", id);
		}*/
	}
	/* 2.set receiver antenna parameters */
	for (i = 0; i < 1; i++) {
		if (!strcmp(navall->rec_ant, "")) { /* set by station parameters */
			strcpy(navall->rec_ant, sta[i].antdes);
			/* xyz */
			if (sta[i].deltype == 1) { 
				if (norm(sta[i].pos, 3) > 0.0) {
					for (j = 0; j < 3; j++) { 
						xyz[j] = sta[i].pos[j]; 
						del[j] = sta[i].del[j];
					}
					pos = ecef2pos(xyz);
					enu = ecef2enu(pos, del);
					for (j = 0; j < 3; j++) { navall->rec_del[j] = enu[j]; }
				}
			}
			/* enu */
			else {
				for (j = 0; j < 3; j++) { navall->rec_del[j] = sta[i].del[j]; }
			}
		}
		if (!(ret = search_pcv(0, navall->rec_ant, time, pcvs, &pcv))) {
			*navall->rec_ant = '\0';
			continue;
		}
		strcpy(navall->rec_ant, pcv.type);
		navall->rec_pcv = pcv;
	}
}

/* read DCB files -------------------------------------------------------------------------------------------------------*/

extern int read_dcb(vector<const char*> dcbfile, NavPack_t* navall)
{
	/* 局部变量定义 ========================================================= */
	FILE* fp;
	double cbias;
	int sat, type = 0;
	char buff[256];
	char tmp[7] = "\0", * p;
	string s = "";
	int fbad = 0;
	/* ====================================================================== */
	
	for (int i = 0; i < dcbfile.size(); i++) {
		p = (char*)strrchr(dcbfile[i], '\\');
		printf(">> DCB file %2d-th %s reading...\n", i + 1, p + 1);
		if (!(fp = fopen(dcbfile[i], "r"))) {
			printf("   ERROR: open %2d-th DCB file %s failed!\n", i + 1, p + 1);
			fbad = 1;
			continue;
		}

		while (fgets(buff, sizeof(buff), fp)) {
			if		(strstr(buff, "DIFFERENTIAL (P1-P2) CODE BIASES")) { type = 1; strncpy(tmp, buff + 14, 5); s = tmp; }
			else if (strstr(buff, "DIFFERENTIAL (P1-C1) CODE BIASES")) { type = 2; strncpy(tmp, buff + 14, 5); s = tmp; }
			else if (strstr(buff, "DIFFERENTIAL (P2-C2) CODE BIASES")) { type = 3; strncpy(tmp, buff + 14, 5); s = tmp; }

			if (!type) { continue; }

			if (!(sat = satid2no(buff)) || (cbias = str2num(buff, 26, 9)) == 0.0) { continue; }

			navall->dcb[sat].insert(pair<string, double>(s, cbias * 1E-9 * CLIGHT)); /* ns -> m */

			//printf("sat=%2d type=%s value=%6.3f\n", sat, s.c_str(), navall->dcb[sat].at(s));
		}
		std::fclose(fp);
	}

	if (fbad) { return 0; }
	else	  { return 1; }
}

extern int read_dcb_mgex(vector<const char*> mdcbfile, NavPack_t* navall, const GpsTime_t time)
{
	FILE* fp;
	double cbias;
	char buff[256], id[4];
	int sat, sys, type = 0, iy, doy1, doy2;
	GpsTime_t time1, time2;
	const char* fname;
	int fbad = 0;

	for (int i = 0; i < mdcbfile.size(); i++) {
		fname = strrchr(mdcbfile[i], '\\');
		printf(">> MDCB file %s reading...\n", fname + 1);
		if (!(fp = fopen(mdcbfile[i], "r"))) {
			printf("   ERROR: open MGEX DCB file %s failed!\n", fname + 1);
			fbad = 1;
			continue;
		}

		while (fgets(buff, sizeof(buff), fp)) {
			if (strstr(buff, "+BIAS/SOLUTION")) { type = 1; }
			if (strstr(buff, "-BIAS/SOLUTION")) { break; }

			if (!type) { continue; }

			if (strncmp(buff + 1, "DSB", 3) || strncmp(buff + 15, "    ", 4)) { continue; }

			iy = (int)str2num(buff, 35, 4); 
			doy1 = (int)str2num(buff, 40, 3); 
			doy2 = (int)str2num(buff, 55, 3);
			if (iy <= 50) { iy += 2000; }
			time1 = yrdoy2time(iy, doy1); 
			time2 = yrdoy2time(iy, doy2);

			if (!(timediff(time, time1) >= 0.0 && timediff(time, time2) < 0.0)) { continue; }

			strncpy(id, buff + 11, 3); 
			id[3] = '\0';
			sat = satid2no(id);
			sys = satsys(sat, NULL);

			if (sys == SYS_GPS) {
				if (!strncmp(buff + 25, "C1C  C5Q", 8)) {
					cbias = str2num(buff, 71, 21);
					navall->dcb[sat].insert(pair<string, double>("P1-P3", cbias * 1E-9 * CLIGHT));
					//nav->cbias[sat - 1][3] = cbias * 1E-9 * CLIGHT; /* ns -> m */
				}
			}
			else if (sys == SYS_BDS) {
				if (!strncmp(buff + 25, "C2I  C7I", 8)) {
					cbias = str2num(buff, 71, 21);
					navall->dcb[sat].insert(pair<string, double>("P1-P2", cbias * 1E-9 * CLIGHT));
					//nav->cbias[sat - 1][0] = cbias * 1E-9 * CLIGHT; /* ns -> m */
				}
				else if (!strncmp(buff + 25, "C2I  C6I", 8)) {
					cbias = str2num(buff, 71, 21);
					navall->dcb[sat].insert(pair<string, double>("P1-P3", cbias * 1E-9 * CLIGHT));
					//nav->cbias[sat - 1][3] = cbias * 1E-9 * CLIGHT; /* ns -> m */
				}
				else if (!strncmp(buff + 25, "C7I  C6I", 8)) {
					cbias = str2num(buff, 71, 21);
					navall->dcb[sat].insert(pair<string, double>("P2-P3", cbias * 1E-9 * CLIGHT));
					//nav->cbias[sat - 1][4] = cbias * 1E-9 * CLIGHT; /* ns -> m */
				}
			}
			else if (sys == SYS_GAL) {
				if (!strncmp(buff + 25, "C1X  C5X", 8)) {
					cbias = str2num(buff, 71, 21);
					navall->dcb[sat].insert(pair<string, double>("P1-P2", cbias * 1E-9 * CLIGHT));
					//nav->cbias[sat - 1][0] = cbias * 1E-9 * CLIGHT; /* ns -> m */
				}
				else if (!strncmp(buff + 25, "C1X  C7X", 8)) {
					cbias = str2num(buff, 71, 21);
					navall->dcb[sat].insert(pair<string, double>("P1-P3", cbias * 1E-9 * CLIGHT));
					//nav->cbias[sat - 1][3] = cbias * 1E-9 * CLIGHT; /* ns -> m */
				}
			}
		}
		std::fclose(fp);
	}

	if (fbad) { return 0; }
	else	  { return 1; }
}

/* read SP3 and CLK files -----------------------------------------------------------------------------------------------*/

static int read_sp3h(FILE* fp, GpsTime_t* time, char* type, double* bfact, char* tsys)
{
	/* 局部变量定义 ========================================================= */
	int i = 0;								// 第一行指示符
	int j, k = 0;							// 循环便利变量
	int ns = 0, sys, prn;					// 卫星数/卫星系统/PRN
	int row = 0, r;							// 行数/行数遍历变量
	char buff[1024];						// buffer
	double tmp[2] = { 0.0 };				// 临时存储变量
	/* ====================================================================== */

	while (fgets(buff, sizeof(buff), fp)) {
		if (i == 0) {
			*type = buff[2];
			if (str2time(buff, 3, 28, time)) {
				return 0;
			}
			i = 1;
		}
		else if (strstr(buff, "+") && (ns = (int)str2num(buff, 3, 3)) > 0) {
			row = ns / 17 - 1;

			for (j = 0; j < 17 && k < ns; j++) {
				sys = code2sys(buff[9 + 3 * j]);
				prn = (int)str2num(buff, 10 + 3 * j, 2);
				if (k < MAXSAT) { k++; }
			}
			for (r = 0; r < row; r++) {
				if (!fgets(buff, sizeof(buff), fp)) { break; }

				for (j = 0; j < 17 && k < ns; j++) {
					sys = code2sys(buff[9 + 3 * j]);
					prn = (int)str2num(buff, 10 + 3 * j, 2);
					if (k < MAXSAT) { k++; }
				}
			}
		}
		else if (strstr(buff, "%c M")) {
			strncpy(tsys, buff + 9, 3); tsys[3] = '\0';
		}
		else if (strstr(buff, "%f")) {
			tmp[0] = str2num(buff, 3, 10); tmp[1] = str2num(buff, 3, 10);
			if (tmp[0] > 0.0 || tmp[1] > 0.0) {
				bfact[0] = tmp[0]; bfact[1] = tmp[1];
			}
		}
		else if (strstr(buff, "/*") && strlen(buff) < 4) {
			break;
		}
	}

	return k;
}

static void read_sp3b(FILE* fp, char type, int ns, double* bfact, char* tsys, int index, NavPack_t* navall)
{
	PrecNav_t tmp = { 0 };
	GpsTime_t time = { 0 };
	double val, std, base;
	int i, j, sat = 0, sys, prn, ne = 0;
	char buff[1024];

	while (fgets(buff, sizeof(buff), fp)) {
		if (!strncmp(buff, "EOF", 3)) { break; }
		else if (strstr(buff, "*  ")) {
			tmp = { 0 };
			tmp.index = index;

			if (str2time(buff, 3, 28, &time)) { return; }
			if (!strcmp(tsys, "UTC")) {
				time = utc2gpst(time);
			}
		}
		else if (strstr(buff, "P")) {
			sys = buff[1] == ' ' ? SYS_GPS : code2sys(buff[1]);
			prn = (int)str2num(buff, 2, 2);

			if (!(sat = satno(sys, prn))) { continue; }

			for (j = 0; j < 4; j++) {
				val = str2num(buff, 4 + j * 14, 14);
				std = str2num(buff, 61 + j * 3, j < 3 ? 2 : 3);

				if (val != 0.0 && fabs(val - 999999.999999) >= 1E-6) {
					tmp.pos[j] = val * (j < 3 ? 1000.0 : 1E-6);
				}
				if ((base = bfact[j < 3 ? 0 : 1]) > 0.0 && std > 0.0) {
					tmp.std[j] = (float)(pow(base, std) * (j < 3 ? 1E-3 : 1E-12)); 
				}
			}
			navall->peph[sat].insert(pair<GpsTime_t, PrecNav_t>(time, tmp));
		}
		else if (strstr(buff, "V")) {
			sys = buff[1] == ' ' ? SYS_GPS : code2sys(buff[1]);
			prn = (int)str2num(buff, 2, 2);

			if (!(sat = satno(sys, prn))) { continue; }

			for (j = 0; j < 4; j++) {
				val = str2num(buff, 4 + j * 14, 14);
				std = str2num(buff, 61 + j * 3, j < 3 ? 2 : 3);

				if (val != 0.0 && fabs(val - 999999.999999) >= 1E-6) {
					tmp.vel[j] = val * (j < 3 ? 0.1 : 1E-10);
				}
				if ((base = bfact[j < 3 ? 0 : 1]) > 0.0 && std > 0.0) {
					tmp.vst[j] = (float)(pow(base, std) * (j < 3 ? 1E-7 : 1E-16));
				}
			}
			navall->peph[sat].insert(pair<GpsTime_t, PrecNav_t>(time, tmp));
		}
	}
	for (auto it = navall->peph.begin(); it != navall->peph.end(); it++) {
		navall->ne += static_cast<int>(it->second.size());
	}
}

static void read_sp3(const char* file, NavPack_t* navall)
{
	FILE* fp;
	GpsTime_t time = { 0 };
	double bfact[2] = { 0 };
	int ns, index = 0;
	char type = ' ', tsys[4] = "";
	const char* ext;
	const char* fname;

	if (!(ext = strrchr(file, '.'))) { return; }

	if (!strstr(ext + 1, "sp3") && !strstr(ext + 1, ".SP3") &&
		!strstr(ext + 1, "eph") && !strstr(ext + 1, ".EPH")) {
		return;
	}

	fname = strrchr(file, '\\');
	printf(">> SP3 file %s reading...\n", fname + 1);
	if (!(fp = fopen(file, "r"))) { 
		printf(">> ERROR: open SP3 file %s fail\n", fname + 1);
		return; 
	}

	if		(strstr(file, "cod") || strstr(file, "COD")) { index = 10; }
	else if (strstr(file, "igs") || strstr(file, "IGS")) { index = 9; }
	else if (strstr(file, "igr") || strstr(file, "IGR")) { index = 8; }
	else if (strstr(file, "gfz") || strstr(file, "GFZ")) { index = 7; }
	else if (strstr(file, "esa") || strstr(file, "ESA")) { index = 6; }
	else if (strstr(file, "iac") || strstr(file, "IAC")) { index = -1; }
	else												 { index = 0; }

	/* 1. read sp3 header */
	ns = read_sp3h(fp, &time, &type, bfact, tsys);
	/* 2. read sp3 body */
	read_sp3b(fp, type, ns, bfact, tsys, index, navall);

	std::fclose(fp);
}

static int read_clkh(FILE* fp, char* type)
{
	/* 局部变量定义 ========================================================= */
	char buff[1024];						// buffer
	/* ====================================================================== */

	while (fgets(buff, sizeof(buff), fp)) {
		if (strstr(buff + 60, "RINEX VERSION / TYPE")) {
			*type = buff[20];
		}
		else if (strstr(buff + 60, "END OF HEADER") && *type != ' ') {
			return 1;
		}
	}

	return 0;
}

static int read_clkb(FILE* fp, char type, int index, NavPack_t* navall)
{
	PrecClk_t pclk;
	GpsTime_t time;
	double data[2];
	int i, j, sat, mask;
	char buff[MAXRNXLEN], satid[8] = "";

	if (!navall) { return 0; }

	/* set system mask */
	mask = SYS_ALL;

	while (fgets(buff, sizeof(buff), fp)) {
		if (str2time(buff, 8, 26, &time)) { 
			pclk = { 0 };
			continue; 
		}

		strncpy(satid, buff + 3, 4);

		/* only read AS (satellite clock) record */
		if (strncmp(buff, "AS", 2) || !(sat = satid2no(satid))) { continue; }

		if (!(satsys(sat, NULL) & mask)) { continue; }

		for (i = 0, j = 40; i < 2; i++, j += 20) { 
			data[i] = str2num(buff, j, 19); 
		}
		
		pclk.index = index;
		pclk.clk = data[0];
		pclk.std = (float)data[1];

		navall->pclk[sat].insert(pair<GpsTime_t, PrecClk_t>(time, pclk));
		navall->nc++;
	}
	return navall->nc > 0;
}

static void read_clk(const char* file, NavPack_t* navall)
{
	FILE* fp;
	int stat, index = 0;
	char type = ' ';
	const char* ext;
	const char* fname;

	if (!(ext = strrchr(file, '.'))) { return ; }

	if (!strstr(ext + 1, "clk") && !strstr(ext + 1, ".clk") &&
		!strstr(ext + 1, "CLK") && !strstr(ext + 1, ".CLK")) {
		return;
	}

	fname = strrchr(file, '\\');
	printf(">> CLK file %s reading...\n", fname + 1);
	if (!(fp = fopen(file, "r"))) {
		printf(">> ERROR: open CLK file %s fail\n", fname + 1);
		return;
	}

	if		(strstr(file, "cod") || strstr(file, "COD")) { index = 10; }
	else if (strstr(file, "igs") || strstr(file, "IGS")) { index = 9; }
	else if (strstr(file, "igr") || strstr(file, "IGR")) { index = 8; }
	else if (strstr(file, "gfz") || strstr(file, "GFZ")) { index = 7; }
	else if (strstr(file, "esa") || strstr(file, "ESA")) { index = 6; }
	else if (strstr(file, "iac") || strstr(file, "IAC")) { index = -1; }
	else { index = 0; }

	/* 1. read clk header */
	stat = read_clkh(fp, &type);
	/* 2. read clk body */
	if (type == 'C') {
		read_clkb(fp, type, index, navall);
	}
	
	std::fclose(fp);
}

extern int read_prec_eph(vector<const char*> precfile, NavPack_t* navall)
{
	int i;

	navall->ne = navall->nc = 0;

	/* read precise ephemeris files */
	for (i = 0; i < precfile.size(); i++) { read_sp3(precfile[i], navall); }
	/* read precise clock files */
	for (i = 0; i < precfile.size(); i++) { read_clk(precfile[i], navall); }

	if (navall->ne == 0 || navall->nc == 0) { return 0; }
	else									{ return 1; }
}

/* read ERP files -------------------------------------------------------------------------------------------------------*/

extern int read_erp(vector<const char*> erpfile, NavPack_t* navall)
{
	FILE* fp;
	Erp_t erp;
	double v[14] = { 0 };
	char buff[256];
	const char* fname;
	int fbad = 0;

	for (int i = 0; i < erpfile.size(); i++) {
		fname = strrchr(erpfile[i], '\\');
		printf(">> ERP file %s reading...\n", fname + 1);
		if (!(fp = fopen(erpfile[i], "r"))) {
			printf("   ERROR: open ERP file %s fail\n", fname + 1);
			fbad = 1;
			continue;
		}

		while (fgets(buff, sizeof(buff), fp)) {
			if (sscanf(buff, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
				v, v + 1, v + 2, v + 3, v + 4, v + 5, v + 6, v + 7, v + 8, v + 9, v + 10, v + 11, v + 12, v + 13) < 5) {
				continue;
			}
			erp.mjd = v[0];
			erp.xp = v[1] * 1E-6 * AS2R;
			erp.yp = v[2] * 1E-6 * AS2R;
			erp.ut1_utc = v[3] * 1E-7;
			erp.lod = v[4] * 1E-7;
			erp.xpr = v[12] * 1E-6 * AS2R;
			erp.ypr = v[13] * 1E-6 * AS2R;

			navall->erp.push_back(erp);
		}
		std::fclose(fp);
	}
	std::sort(navall->erp.begin(), navall->erp.end(), compare_erp);

	if (fbad) { return 0; }
	else	  { return 1; }
}

/* read blq ocean tide loading parameters -------------------------------------------------------------------------------*/

extern int read_blq(vector<const char*> blqfile, const char* sta, vector<vector<double>>* otl)
{
	FILE* fp;
	char buff[256], staname[32] = "", name[32], * p;
	const char* fname;
	double v[11];
	int i, n;
	vector<double> tmp(vector<double>(11, 0.0));

	/* station name to upper case */
	sscanf(sta, "%16s", staname);
	for (p = staname; (*p = (char)toupper((int)(*p))); p++);

	for (int i = 0; i < blqfile.size(); i++) {
		fname = strrchr(blqfile[i], '\\');
		printf(">> BLQ file %s reading...\n", fname + 1);
		if (!(fp = fopen(blqfile[i], "r"))) {
			printf("   ERROR: open BLQ file %s fail\n", fname + 1);
			return 0;
		}

		if (otl->size() == 0) {
			for (int i = 0; i < 6; i++) {
				otl->push_back(tmp);
			}
		}

		while (fgets(buff, sizeof(buff), fp)) {
			if (!strncmp(buff, "$$", 2) || strlen(buff) < 2) { continue; }

			if (sscanf(buff + 2, "%16s", name) < 1) { continue; }
			for (p = name; (*p = (char)toupper((int)(*p))); p++);

			if (strcmp(name, staname)) { continue; }

			n = 0;
			/* read blq record */
			while (fgets(buff, sizeof(buff), fp)) {
				if (!strncmp(buff, "$$", 2)) { continue; }
				if (sscanf(buff, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
					v, v + 1, v + 2, v + 3, v + 4, v + 5, v + 6, v + 7, v + 8, v + 9, v + 10) < 11) {
					continue;
				}

				
				for (int j = 0; j < 11; j++) { (*otl)[n][j] = v[j]; }
				n++;

				if		(n == 6 && (*otl)[5][10] != 0.0) { std::fclose(fp); return 1; }
				else if (n == 6 && (*otl)[5][10] == 0.0) { break; }
			}
		}
		std::fclose(fp);
		printf("   WARNING: no ocean load parameters: sta = %s file = %s\n", sta, fname + 1);
	}

	return 0;
}