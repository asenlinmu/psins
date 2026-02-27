/* KFApp c++ hearder file KFApp.h */
/*
	By     : Yan Gongmin @ NWPU
	Date   : 2020-12-09
	From   : College of Automation, 
	         Northwestern Polytechnical University, 
			 Xi'an 710072, China
*/

#ifndef _KFAPP_H
#define _KFAPP_H

#include "PSINS.h"

class CKFApp:public CSINSTDKF
{
public:

	CKFApp(double ts);
	virtual void Init(const CSINS &sins0, int grade=-1);
	virtual void SetMeas(void) {};
	virtual void SetMeas(const CVect3 &pgps, const CVect3 &vgps);
	int Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts);
};

typedef struct {
	CVect3 wm, vm, att;
	double t;
	CVect3 gpsvn, gpspos;
} DataSensor;

#ifdef PSINS_IO_FILE
class CFileRdSr:public CFileRdWt  // CFile Read Sensor
{
public:
	DataSensor *pDS, DS0;
	CFileRdSr(const char *fname0, int columns0=0):CFileRdWt(fname0, columns0) {
		pDS = (DataSensor*)buff;
		load(1);             // get the first record line
		memcpy(&DS0, pDS, sizeof(DataSensor));
		fseek(f, sizeof(DataSensor), SEEK_CUR);
	};
	int load(int lines=1, BOOL txtDelComma=1) {
		if(!CFileRdWt::load(lines)) return 0;
		return 1;
	};
};
#endif

#endif

