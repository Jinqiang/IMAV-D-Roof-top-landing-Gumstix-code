#ifndef DLG_H_
#define DLG_H_

#include "daq.h"
#include "svo.h"
#include "im6.h"
#include "im7.h"
#include "im9.h"
#include "ctl.h"
#include "main.h"
#include "laser.h"

//DLG
#if (_DEBUG)
#define COUNTLOG_IM6    _DEBUG_COUNT_1				//to test the dtl thread every loop
#define COUNTLOG_IM7    _DEBUG_COUNT_1				//to test the dtl thread every loop
#define COUNTLOG_DAQ    _DEBUG_COUNT_1
#define COUNTLOG_SVO	_DEBUG_COUNT_1
#define COUNTLOG_SIG	_DEBUG_COUNT_1
#define COUNTLOG_CTL	_DEBUG_COUNT_1
#define COUNTLOG_TC		_DEBUG_COUNT_1
#else
#define COUNTLOG_IM6	50
#define COUNTLOG_IM7	50
#define COUNTLOG_DAQ	50
#define COUNTLOG_SVO	50
#define COUNTLOG_SIG	50
#define COUNTLOG_CTL	50
#define COUNTLOG_TC	50
#endif

#if (_DEBUG)
#define COUNT_DLG	_DEBUG_COUNT_1
#else
#define COUNT_DLG	50
#endif

//DLG
#pragma pack(push, 1)
struct DLGHEADER {
	short header;
	char code;				//type of data
	double time;				//time of data
};

#pragma pack(pop)

class clsDLG : public clsThread {
public:
	clsDLG();
	~clsDLG();

public:
	virtual BOOL InitThread();
	virtual int EveryRun();
	virtual void ExitThread();

protected:
	int			m_nIM6;
	double		m_tIM6[MAX_IM6PACK];
	IM6RAWPACK	m_im6Raw[MAX_IM6PACK];
	IM6PACK		m_im6[MAX_IM6PACK];

	int			m_nIM7;
	double		m_tIM7[MAX_IM7PACK];
	IM7RAWPACK	m_im7Raw[MAX_IM7PACK];
	IM7PACK		m_im7[MAX_IM7PACK];

	int 		m_nState;
	double 		m_tState[MAX_STATE];
	UAVSTATE 	m_state[MAX_STATE];

	int			m_nVisionState;
	double		m_tVisionState[MAX_STATE];
//	VISIONSTATE	m_visionState[MAX_STATE];

	int			m_nDAQ;
	double		m_tDAQ[MAX_DAQ];
	DAQRAWDATA	m_daqRaw[MAX_DAQ];
	DAQDATA		m_daq[MAX_DAQ];

	int			m_nURG;
	double 		m_tURG[MAX_IM9PACK];
	long		laser_ranges[MAX_IM9PACK][MAX_URG_PT];
	double		laser_height[MAX_IM9PACK];
	double		laser_front[MAX_IM9PACK];
	double		laser_left[MAX_IM9PACK];
	double 		laser_phi[MAX_IM9PACK];
	double 		laser_tht[MAX_IM9PACK];
	double 		laser_psi[MAX_IM9PACK];

	int			m_nCTL;
	double		m_tCTL[MAX_CTL];
	CONTROLSTATE m_ctl[MAX_CTL];				//inner loop signal - {u,v,w,p,q,r,ug,vg,wg}

	int			m_nSIG;
	double		m_tSIG[MAX_SIG];
	HELICOPTERRUDDER m_sig[MAX_SIG];

	int 		m_nSVO;
	double 		m_tSVO[MAX_SVO];
	SVORAWDATA	m_svoRaw[MAX_SVO];
	SVODATA		m_svo[MAX_SVO];

	int m_nTC;
	double		m_tTC[MAX_TC];
	TIMECOST	m_tc[MAX_TC];

	char m_szFile[256];
	char m_szFileb[256];
	char m_szFilea[256];

	BOOL m_bBlackBox;
	FILE *m_pfLog, *m_pfLogb;
	FILE *m_pfLoga; 	// file pointer to laser.txt
};


#endif				//DLG_H_
