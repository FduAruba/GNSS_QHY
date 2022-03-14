#include"gnss.h"

/* output solution as the form of lat/lon/height, modified by fzhou @ GFZ, 2017-01-23 -----------------------------*/
static int outpos(unsigned char* buff, const char* s, const Sol_t* sol, const Solopt_t* sopt)
{
	const char* sep = "\t";
	char* p = (char*)buff;
	int i;
	double ep[6];
	GpsTime_t time;
	int week;
	double sow;
	vector<double>   rr = vector<double>(3, 0.0);
	vector<double>  pos = vector<double>(3, 0.0);
	vector<double> dxyz = vector<double>(3, 0.0);
	vector<double> denu = vector<double>(3, 0.0);

	ep[0] = str2num(s, 0, 4);
	ep[1] = str2num(s, 5, 2);
	ep[2] = str2num(s, 8, 2);
	ep[3] = str2num(s, 11, 2);
	ep[4] = str2num(s, 14, 2);
	ep[5] = str2num(s, 17, 2);
	time = epoch2time(ep);
	sow = time2gpst(time, &week);

	for (i = 0; i < 3; i++) { dxyz[i] = denu[i] = 0.0; }

	p += sprintf(p, "%04d%s%02d%s%02d%s%02d%s%02d%s%02d%s%4d%s%9.2f%s%14.4f%s%14.4f%s%14.4f%s%14.4f",
		(int)ep[0], sep, (int)ep[1], sep, (int)ep[2], sep, (int)ep[3], sep, (int)ep[4], sep, (int)ep[5], sep, week, sep, sow, sep,
		sol->rr[0], sep, sol->rr[1], sep, sol->rr[2], sep, sol->dtr[0] * CLIGHT);

	if ((*sol).rr_snx[0] == 0.0) {
		for (int i = 0; i < 3; i++) {
			rr[i] = sol->rr[i];
			denu[i] = 0;
		}
		pos = ecef2pos(rr);
	}
	else { 
		for (int i = 0; i < 3; i++) { 
			rr[i] = sol->rr_snx[i]; 
			dxyz[i] = sol->rr[i] - sol->rr_snx[i];
		}
		pos = ecef2pos(rr);
		denu = ecef2enu(pos, dxyz);
	}
	
	p += sprintf(p, "%s%8.4f%s%8.4f%s%8.4f%s%8.4f", sep, denu[0], sep, denu[1], sep, denu[2], sep, norm(denu, 3));
	p += sprintf(p, "\n");

	return p - (char*)buff;
}

/* output solution body --------------------------------------------------------
* output solution body to buffer
* args   : unsigned char *buff IO output buffer
*          sol_t  *sol      I   solution
*          solopt_t *opt    I   solution options
* return : number of output bytes
*-----------------------------------------------------------------------------*/
static int read_sol(unsigned char* buff, const Sol_t* sol, const ProcOpt_t* popt, const Solopt_t* sopt)
{
	GpsTime_t time_tmp;
	double sec_w;
	int week, digit;
	const char* sep = "\t";
	char t_str[64];
	unsigned char* p = buff;

	if ((*popt).pos_mode >= PMODE_PPP_KINEMA &&
		(*sol).stat == SOLQ_SINGLE) {
		p += sprintf((char*)p, "\n");
		return p - buff;
	}
	if ((*sol).stat <= SOLQ_NONE) {
		p += sprintf((char*)p, "\n");
		return p - buff;
	}

	digit = sopt->time_dig < 0 ? 0 : (sopt->time_dig > 20 ? 20 : sopt->time_dig);
	time_tmp = sol->time;

	if (sopt->time_sys >= TIMES_UTC) { time_tmp = gpst2utc(time_tmp); }

	if (sopt->time_fmt) { time2str(time_tmp, t_str, digit); }
	else {
		sec_w = time2gpst(time_tmp, &week);
		if (86400 * 7 - sec_w < 0.5 / pow(10.0, digit)) {
			week++;
			sec_w = 0.0;
		}
		sprintf(t_str, "%4d%s%*.*f", week, sep, 6 + (digit <= 0 ? 0 : digit + 1), digit, sec_w);
	}

	p += outpos(p, t_str, sol, sopt);

	return p - buff;
}

/* output solution body --------------------------------------------------------
* output solution body to file
* args   : FILE   *fp       I   output file pointer
*          sol_t  *sol      I   solution
*          double *rb       I   base station position {x,y,z} (ecef) (m)
*          solopt_t *opt    I   solution options
* return : none
*-----------------------------------------------------------------------------*/
extern void out_sol(FILE* fp, const Sol_t* sol, const ProcOpt_t* popt, const Solopt_t* sopt)
{
	unsigned char buff[50000] = { '\0' };
	int n;

	if ((n = read_sol(buff, sol, popt, sopt)) > 0) {
		if (n > 1) {
			fwrite(buff, n, 1, fp);
		}
	}
}