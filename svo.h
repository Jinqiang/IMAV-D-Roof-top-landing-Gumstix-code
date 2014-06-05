#ifndef SVO_H_
#define SVO_H_

#include "thread.h"
#include "uav.h"

#define RATIO1		0.9391
#define RATIO2		0.8819
#define RATIO3		0.5335

#define MAX_SVO	128				//storage size for SVO data
#define MAX_SIG	128
//SVO

struct SVO_CONFIG {
	char devPort[32];
	short flag;
	int baudrate;
};

struct SVORAWDATA {
	unsigned short elevator;
	unsigned short throttle;
	unsigned short aileron;
	unsigned short auxiliary;
	unsigned short rudder;
	unsigned short sv6;
};

struct SVODATA {
	double aileron;
	double elevator;
	double auxiliary;
	double rudder;
	double throttle;
	double sv6;				//additional signal to determine mode of operation (automatic/manual)
};

struct HELICOPTERRUDDER {
	double aileron;
	double elevator;
	double auxiliary;
	double rudder;
	double throttle;
	double triger;
	double laser;
};

class clsSVO : public clsThread {
public:
	clsSVO();
	virtual ~clsSVO();
	void Init();

public:
	virtual BOOL InitThread();
	virtual int EveryRun();
	virtual void ExitThread();


protected:
	COMMAND m_cmd;
	pthread_mutex_t m_mtxCmd;
	BOOL m_bTrimvalue;		// indicate if trimvalue training is in progress
	double m_trimT0;

public:
	void PutCommand(COMMAND *pCmd);
	void GetCommand(COMMAND *pCmd);
	BOOL ProcessCommand(COMMAND *pCmd);

protected:
	int m_nsSVO;				//handle of device

	int m_nSVO;				//size of stored data
	double		m_tSVO[MAX_SVO];
	SVORAWDATA	m_svoRaw[MAX_SVO];
	SVODATA		m_svo[MAX_SVO];

	pthread_mutex_t m_mtxSVO;

	double		m_tSVO0;
	SVORAWDATA	m_svoRaw0;
	SVODATA		m_svo0,m_svo1;
	SVORAWDATA	m_svoManualTrimRaw;
	double		m_tRequest;				//the time when Command data
	double		m_tRetrieve;				//the time when get data
	double 		m_tTrimvalue;		//the time when get the updated trimvalue

	HELICOPTERRUDDER m_svoEqu;
	HELICOPTERRUDDER m_svoSet;

	double m_limit;				//damping limit

	int m_nTrimCount;
	HELICOPTERRUDDER m_avgTrimvalue;
	SVODATA m_manualTrim;
	BOOL m_bManualTrimFlag;

	SVO_CONFIG m_svoConfig;

	int m_trim_aileron, m_trim_elevator, m_trim_throttle, m_trim_rudder, m_trim_auxiliary;

public:
	double GetSVOTime() { return m_tSVO0; }
	SVODATA &GetSVOData() { return m_svo0; }
	SVODATA &GetSVOData1() { return m_svo1; }
	SVORAWDATA &GetSVODataRaw() { return m_svoRaw0; }
	SVORAWDATA &GetManualTrimRawData() { return m_svoManualTrimRaw; }
	void SetManualTrimRawData(SVORAWDATA svoraw);
	BOOL GetManualTrimFlag() { return m_bManualTrimFlag; }
	void SetManualTrimFlag() { m_bManualTrimFlag = TRUE; }
	void ResetManualTrimFlag() { m_bManualTrimFlag = FALSE; }

	void SetSvoConfig(char *devPort, short size, short flag, int baudrate);
	SVO_CONFIG GetSvoConfig() const {return m_svoConfig;}

	BOOL GetData(SVORAWDATA *pRaw);
	void WriteCommand();
	void WriteCommand1();
	void SetRudder(HELICOPTERRUDDER *pRudder);
	void SetRudder_FeiLion(HELICOPTERRUDDER *pRudder);

	void SetCamera(double camera);

	void Translate_HeLion(SVORAWDATA *pRaw, SVODATA *pSVO);
	void Translate_GremLion(SVORAWDATA *pRaw, SVODATA *pSVO);
	void Translate_GremLion(SVORAWDATA *pSVORAW, SVODATA *pSVO, SVORAWDATA *pSVOTrim, BOOL bManualTrim);

	BOOL ValidateTrimvalue();
	BOOL CalcNewTrim();		// calculate the new trim value
	void SetTrimvalue();
	void SetLimit(double limit) { m_limit = limit; }
	double GetTrimvalueTime()	{ return m_tTrimvalue; }
	HELICOPTERRUDDER &GetTrimvalue()	{ return m_svoEqu; }

	friend class clsDLG;
};

#endif				//SVO_H_
