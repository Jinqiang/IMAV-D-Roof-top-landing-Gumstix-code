/*
 * main.h
 *  Created on: Mar 15, 2011
 *
 * this is the header file for clsMain. clsMain is the class managing all task threads
 */

#ifndef MAIN_H_
#define MAIN_H_

//main and global
#define PRIORITY_IM6	9
#define PRIORITY_IM7	9
#define PRIORITY_IM8	9
#define PRIORITY_IM9	9
#define PRIORITY_DAQ	9
#define PRIORITY_CTL	9
#define PRIORITY_SVO	9
#define PRIORITY_CMM	8
#define PRIORITY_DLG	7
#define PRIORITY_USER	9
#define PRIORITY_CAM	9
#define PRIORITY_SERVER	8
#define PRIORITY_CLIENT	8
#define PRIORITY_COOP	8
#define PRIORITY_NET	8
#define PRIORITY_URG	8
#define PRIORITY_PTU    8

#define THREADFLAG_IM6			0x0001
#define THREADFLAG_IM7			0x0002
#define THREADFLAG_DAQ			0x0004
#define THREADFLAG_CTL			0x0008

#define THREADFLAG_SVO			0x0010
#define THREADFLAG_CMM			0x0020
#define THREADFLAG_DLG			0x0040
#define THREADFLAG_CAM			0x0080

#define THREADFLAG_SERVER		0x0100
#define THREADFLAG_CLIENT		0x0200
#define THREADFLAG_COOP			0x0400
#define THREADFLAG_NET			0x0800

#define THREADFLAG_IM8			0x1000
#define THREADFLAG_IM9			0x2000
#define THREADFLAG_URG			0x4000
#define THREADFLAG_PTU          0x8000

#define THREADFLAG_ALL			0xffff


//struct to store time consumption of every thread
struct TIMECOST {
	double tIMU;
	double tDAQ;
	double tCAM;
	double tCTL;
	double tSVO;
	double tCMM;
	double tDLG;

//	double tSVR;
//	double tCLT;
	double tMain;
};

class clsMain {
public:
	clsMain();
	~clsMain();

	void Init();

protected:
	int m_chMain;
	int m_nCount;

	int m_fThread;

protected:
	int m_nTC;
	double m_tTC[MAX_TC];
	TIMECOST m_tc[MAX_TC];

	pthread_mutex_t m_mtxTC;

public:
	void Run();
	BOOL MAGTest();

	BOOL ProcessCommand(COMMAND *pCmd);

	friend class clsDLG;
};


#endif /* MAIN_H_ */
