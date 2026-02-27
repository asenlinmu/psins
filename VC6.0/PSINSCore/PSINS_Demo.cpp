#include "PSINS.h"

//#define PSINSDemo 7

#ifdef  PSINSDemo

void Demo_CIIRV3(void);
void Demo_CMaxMin(void);
void Demo_CVAR(void);
void Demo_CVARn(void);
void Demo_CRAvar(void);
void Demo_CSINS_static(void);
void Demo_CAlignsv(void);
void Demo_CAlign_CSINS(void);
void Demo_CVCFileFind(void);

void main(void)
{
	switch(PSINSDemo)
	{
	case 1: Demo_CIIRV3(); break;
	case 2: Demo_CMaxMin(); break;
	case 3: Demo_CVAR(); break;
	case 4: Demo_CVARn(); break;
	case 5: Demo_CRAvar(); break;
	case 6: Demo_CSINS_static(); break;
	case 7: Demo_CAlignsv(); break;
	case 8: Demo_CAlign_CSINS(); break;
	case 11: Demo_CVCFileFind(); break;
	}
}

void Demo_CIIRV3(void)
{
	// use Matlab/fdatool to design the IIR filter coefficients
	double Num[] = {0.004824343357716,   0.019297373430865,   0.028946060146297,   0.019297373430865,   0.004824343357716},
		Den[] = {1.000000000000000,  -2.369513007182038,   2.313988414415880,  -1.054665405878568,   0.187379492368185};
	CFileRdWt::Dir(".\\Data\\");	CFileRdWt res("res.bin");
	CIIRV3 iir(Num, Den, sizeof(Num)/sizeof(double));
	for(int k=1; k<100; k++)
	{
		CVect3 x = randn(O31, I31);
		CVect3 y = iir.Update(x);
		res<<x<<y;
	}
}

void Demo_CMaxMin(void)
{
	CFileRdWt::Dir(".\\Data\\");	CFileRdWt res("res.bin");
	CMaxMin maxmin(20,10);
	for(int k=1; k<100; k++)
	{
		double x = randn(0.0, 1.0);
		int flag = maxmin.Update(x);
		res<<x<<maxmin.maxRes<<maxmin.minRes<<maxmin.maxpreRes<<maxmin.minpreRes;
	}
}

void Demo_CVAR(void)
{
	CFileRdWt::Dir(".\\Data\\");	CFileRdWt res("res.bin");
	CVAR var(20);
	for(int k=1; k<500; k++)
	{
		double x = randn(0.0, 1.0);
		double y = var.Update(x, true);
		res<<x<<(double)var.mean<<var.var;
	}
}

void Demo_CVARn(void)
{
	CFileRdWt::Dir(".\\Data\\");	CFileRdWt res("res.bin");
	CVARn var(20,1);
	for(int k=1; k<500; k++)
	{
		double x = randn(0.0, 1.0);
		double y = var.Update(x);
		res<<x<<(double)var.mx[0]<<var.stdx[0];
	}
}

void Demo_CRAvar(void)
{
	CFileRdWt::Dir(".\\Data\\");	CFileRdWt res("res.bin");
	CRAvar ravr(1);
	ravr.set(3.0, 10.0, 10.0, 0.1);
	for(int k=1; k<400; k++)
	{
		double x = randn(0.0, 1.0);  if(k==200) x=10;
		ravr.Update(x, 1.0);
		res<<x<<sqrt(ravr.R0[0]);
	}
}

void Demo_CSINS_static(void)
{
	CFileRdWt::Dir(".\\Data\\");	CFileRdWt fins("ins.bin");
	double ts=1, t;
	CVect3 att0=PRY(1,1.01,3), vn0=O31, pos0=LLH(34,0,0);
	CEarth eth;
	eth.Update(pos0);
	CVect3 wm[2], vm[2];
	CMat3 Cbn=~a2mat(att0);
	wm[0] = Cbn*eth.wnie*ts; wm[1] = wm[0];  // static IMU simuation
	vm[0] = -Cbn*eth.gn*ts; vm[1] = vm[0];
	CSINS sins(a2qua(att0)+CVect3(0,0,1)*glv.min, O31, pos0);
	for(t=0.0; t<24*3600.0; t+=2*ts)
	{
		sins.Update(wm, vm, 2, ts);  sins.pos.k = pos0.k;
		fins<<sins;
	}
}

void Demo_CAlignsv(void)
{
	CFileRdWt::Dir(".\\Data\\");
	CFileIMU6 fimu("lasergyro.imu"); CFileRdWt faln("aln.bin");
	CAlignsv aln(fimu.pos0, fimu.ts, 600, 200);
	for(double t=0.0; t<600.0; t+=fimu.ts)
	{
		if(!fimu.load(1)) break;
		aln.Update(fimu.pwm, fimu.pvm);
		faln<<q2att(aln.qnb)<<aln.tk;
	}
}

void Demo_CAlign_CSINS(void)
{
	CFileRdWt::Dir(".\\Data\\");
	CFileIMU6 fimu("lasergyro.imu"); CFileRdWt faln("aln.bin"), fins("ins.bin");
//	CAligni0 aln(pos0);
	CAlignkf aln(CSINS(fimu.att0,fimu.vn0,fimu.pos0));
	CSINS sins;
	int alnOK=0;
	for(double t=0.0; t<2000.0; t+=fimu.ts)
	{
		if(!fimu.load(1)) break;
		aln.Update(fimu.pwm, fimu.pvm, 1, fimu.ts);
		faln<<q2att(aln.qnb)<<t;
		if(t>400.0)
		{
			if(!alnOK) {
				alnOK=1;
				sins.Init(aln.qnb, O31, fimu.pos0, t);
			}
			else {
				sins.Update(fimu.pwm, fimu.pvm, 1, fimu.ts);
				fins<<sins;
			}
		}
	}
}

void Demo_CVCFileFind(void)
{
#ifdef PSINS_VC_AFX_HEADER
	CVCFileFind ffind(".\\PSINSCore\\", "*.cpp");
	while(1)
	{
		char *fname = ffind.FindNextFile();
		if(!fname) break;
		printf("%s\n", fname);
	}
#endif  // PSINS_VC_AFX_HEADER
}

#endif
