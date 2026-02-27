#include ".\PSINSCore\kfapp.h"
#define FRQ	125
#define TS	(1.0/FRQ)

void main(void)
{
	CFileRdWt::Dir("D:\\ygm2020\\PSINSÍøÕ¾\\¹ßµ¼Êý¾Ý\\", "D:\\psins201210\\VC6.0\\Data\\");
	CFileRdWt fins("ins.bin"), fkf("kf.bin");
	CFileRdSr fimu("mimuattgps.bin",-16);  // download from: http://www.psins.org.cn/newsinfo/958984.html
	DataSensor *pDS=(DataSensor*)fimu.buff, *pDS0=&fimu.DS0;

	CKFApp kf(TS);
	kf.Init(CSINS(pDS0->att, pDS0->gpsvn, pDS0->gpspos, pDS0->t));

	for(int i=0; i<5000*FRQ; i++)
	{
		if(!fimu.load(1)) break;
		kf.Update(&pDS->wm, &pDS->vm, 1, TS);
		if(pDS->gpspos.i>0.1)
		{
			kf.SetMeas(pDS->gpspos, pDS->gpsvn);
		}

		if(i%5==0||pDS->gpspos.i>0.1)
		{
			fins << kf.sins << pDS->att;
			fkf << kf;
		}

		console_disp(i, FRQ, 100);
	}
}
