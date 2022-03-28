#include"gnss.h"

/* Global variables ------------------------------------------------*/
FileOpt_t fopt;
ProcOpt_t popt;
Solopt_t  sopt;

int rec_num = 1;			

void test()
{
	ifstream ifs;
	ofstream ofs;
	string buf;

	ofs.open("test.txt", ios::out);
	ifs.open("HD_2740.21o", ios::in);

	if (!ifs.is_open())
	{
		cout << "文件打开失败" << endl;
		return;
	}
	else
	{
		cout << "file is reading..." << endl;
		while (getline(ifs, buf))
		{
			ofs << buf << endl;
		}
	}

	cout << "file has read" << endl;
	ifs.close();
	ofs.close();
}

void test_matmul() {
	
	/*int a = 1, b = 0, c;
	try { c = a / b; }
	catch (exception& e) {
		cout << e.what() << endl;
	}*/
	
	
	vector<vector<double>> A = vector<vector<double>>(3, vector<double>(3, 0));
	vector<vector<double>> B = vector<vector<double>>(3, vector<double>(3, 0));
	vector<vector<double>> C = vector<vector<double>>(3, vector<double>(3, 0));
	int n = 0;
	for (int i = 0; i < A.size(); i++) {
		for (int j = 0; j < A[0].size(); j++) {
			A[i][j] = n++;
		}
	}
	for (int i = 0; i < B.size(); i++) {
		for (int j = 0; j < B[0].size(); j++) {
			B[i][j] = n++;
		}
	}
	printf("A=\n");
	for (int i = 0; i < A.size(); i++) {
		for (int j = 0; j < A[0].size(); j++) {
			printf("%04.2f ", A[i][j]);
		}
		printf("\n");
	}
	printf("B=\n");
	for (int i = 0; i < B.size(); i++) {
		for (int j = 0; j < B[0].size(); j++) {
			printf("%04.2f ", B[i][j]);
		}
		printf("\n");
	}

	matmul("NN", 3, -1, 3, 1.0, &A, &B, 0.0, &C);

	printf("C=\n");
	for (int i = 0; i < C.size(); i++) {
		for (int j = 0; j < C[0].size(); j++) {
			printf("%04.2f ", C[i][j]);
		}
		printf("\n");
	}
}

int main()
{
	long t1, t2;
	GpsTime_t tmp = { 0 };
	int ret;
	double err[5] = { 100.0,0.003,0.003,0.0,1.0 };

	popt.nav_sys = SYS_GPS;
	popt.pos_mode = PMODE_PPP_STATIC;
	popt.eph_opt = EPHOPT_PREC;
	popt.thres_GDOP = 30.0;
	popt.thres_PDOP = 20.0;
	popt.elmin = 15.0 * D2R;
	for (int i = 0; i < 5; i++) { popt.err[i] = err[i]; }
	for (int i = 0; i < MAXSAT; i++) { popt.exsats[i] = 0; }
	popt.flagMW = 1;
	popt.flagGF = 1;

	fopt.infile.push_back("E:\\CppCode\\GNSS_QHY\\data\\HD_2740.21p");
	fopt.infile.push_back("E:\\CppCode\\GNSS_QHY\\data\\HD_2740.21o");
	fopt.atxfile.push_back("E:\\CppCode\\GNSS_QHY\\data\\igs14.atx");
	fopt.dcbfile.push_back("E:\\CppCode\\GNSS_QHY\\data\\P1C12110.DCB");
	fopt.dcbfile.push_back("E:\\CppCode\\GNSS_QHY\\data\\P1P22110.DCB");
	fopt.dcbfile.push_back("E:\\CppCode\\GNSS_QHY\\data\\P2C22110.DCB");
	fopt.mdcbfile.push_back("E:\\CppCode\\GNSS_QHY\\data\\CAS0MGXRAP_20212740000_01D_01D_DCB.BSX");
	fopt.precfile.push_back("E:\\CppCode\\GNSS_QHY\\data\\wum21774.sp3");
	fopt.precfile.push_back("E:\\CppCode\\GNSS_QHY\\data\\wum21775.sp3");
	fopt.precfile.push_back("E:\\CppCode\\GNSS_QHY\\data\\wum21776.sp3");
	fopt.precfile.push_back("E:\\CppCode\\GNSS_QHY\\data\\wum21774.clk");
	fopt.precfile.push_back("E:\\CppCode\\GNSS_QHY\\data\\wum21775.clk");
	fopt.precfile.push_back("E:\\CppCode\\GNSS_QHY\\data\\wum21776.clk");
	fopt.erpfile.push_back("E:\\CppCode\\GNSS_QHY\\data\\igs21777.erp");
	fopt.blqfile.push_back("E:\\CppCode\\GNSS_QHY\\data\\FD_Station.blq");

	fopt.outfile.push_back("E:\\CppCode\\GNSS_QHY\\result\\solution.txt");

	fopt.dbgfile.push_back("E:\\CppCode\\GNSS_QHY\\result\\sele_sat.txt");
	fopt.dbgfile.push_back("E:\\CppCode\\GNSS_QHY\\result\\iteration.txt");
	fopt.dbgfile.push_back("E:\\CppCode\\GNSS_QHY\\result\\sat_ppp.txt");

	sopt.time_dig = 3;
	sopt.time_sys = TIMES_GPST;
	sopt.time_fmt = 1;

	t1 = clock();
	ret = posFDU(tmp, tmp, 0.0, &popt, &fopt, &sopt);
	t2 = clock();

	printf("\n* The total time for running the program: %6.3f seconds\n%c", (double)(t2 - t1) / CLOCKS_PER_SEC, '\0');

	//system("pause");
	return 0;
}