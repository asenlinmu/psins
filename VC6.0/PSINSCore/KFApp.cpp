#include "KFApp.h"

/***************************  class CKFApp  *********************************/
CKFApp::CKFApp(double ts):CSINSTDKF(15, 6)
{
//state: 0-2 phi; 3-5 dvn; 6-8 dpos; 9-11 eb; 12-14 db
//meas:  0-2 dvn; 3-5 dpos
}

void CKFApp::Init(const CSINS &sins0, int grade)
{
	CSINSTDKF::Init(sins0);
	Pmax.Set2(50.0*glv.deg,50.0*glv.deg,100.0*glv.deg,    500.0,500.0,500.0,    1.0e6/glv.Re,1.0e6/glv.Re,1.0e6, 
		5000.0*glv.dph,5000.0*glv.dph,5000.0*glv.dph,    10.0*glv.mg,10.0*glv.mg,10.0*glv.mg);
	Pmin.Set2(0.1*glv.min,0.1*glv.min,1.0*glv.min,    0.001,0.001,0.001,    .10/glv.Re,.10/glv.Re,0.1, 
		.10*glv.dph,.10*glv.dph,.10*glv.dph,    10.0*glv.ug,10.0*glv.ug,20.0*glv.ug);
	Pk.SetDiag2(10.0*glv.deg,10.0*glv.deg,5.0*glv.deg,    1.0,1.0,1.0,     10.0/glv.Re,10.0/glv.Re,10.0, 
		100.0*glv.dph,100.0*glv.dph,100.0*glv.dph,    3.0*glv.mg,3.0*glv.mg,10.0*glv.mg);
	Qt.Set2(.10*glv.dpsh,.10*glv.dpsh,.10*glv.dpsh,    1.0*glv.ugpsHz,1.0*glv.ugpsHz,1.0*glv.ugpsHz,    0.0,0.0,0.0,
		0.0*glv.dphpsh,0.0*glv.dphpsh,0.0*glv.dphpsh,    0.0*glv.ugpsh,0.0*glv.ugpsh,0.0*glv.ugpsh);
	Rt.Set2(0.5,0.5,0.5,   10.0/glv.Re,10.0/glv.Re,10.0);
	Rmax = Rt*100;  Rmin = Rt*0.01;  Rb = 0.6;
	FBTau.Set(1.0,1.0,1.0,     1.0,1.0,1.0,     1.0,1.0,1.0,    1.0,1.0,1.0,    1.0,1.0,1.0);
}

void CKFApp::SetMeas(const CVect3 &pgps, const CVect3 &vgps)
{
	if(!IsZero(pgps))
	{
		*(CVect3*)&Zk.dd[3] = sins.pos - pgps;
		SetMeasFlag(000070);
	}
	if(!IsZero(vgps))
	{
		*(CVect3*)&Zk.dd[0] = sins.vn - vgps;
		SetMeasFlag(000007);
	}
}

int CKFApp::Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts)
{
	int res=TDUpdate(pwm, pvm, nSamples, ts, 5);
	return res;
}

