#ifndef DAQ_H_
#define DAQ_H_

#include "dscud.h"
#include "thread.h"
#include "uav.h"

#define SNA_PACKET_SIZE	6
#define BUFFERSIZE_SNA	1024

//DAQ
#pragma pack(push, 1)
struct DAQRAWDATA {
	short   level;
	short   height;
//	short   Elevator_Svo;
	BOOL bRPM;				//the flag to indicate whether RPM data is available
	unsigned char  status;
	unsigned short  num;
	uint64_t	cycleElapse;				//the clock cycle elapsed
};
#define HEIGHT_CORRECTION 0

struct DAQDATA {
	double level;
	double height;
//	double elevator;
//	BOOL bRPM;
//	unsigned char status;
	double rpm;
};

#pragma pack(pop)

class clsDAQ : public clsThread {
public:
	clsDAQ();
	virtual ~clsDAQ();

public:
	virtual BOOL InitThread();
	virtual int EveryRun();
	virtual void ExitThread();

protected:
	int m_nDAQ;
	double m_tDAQ[MAX_DAQ];				//time(cycle) for each pack of data
	DAQRAWDATA m_daqRaw[MAX_DAQ];
	DAQDATA m_daq[MAX_DAQ];

	double m_tDAQ0;
	DAQRAWDATA m_daqRaw0;
	DAQDATA m_daq0;

	pthread_mutex_t m_mtxDAQ;

	DSCB m_dscb;
	DSCCB m_dsccb;
	DSCADSCAN m_dscadscan;
	DSCCR m_dsccr;

	uint64_t m_cycle;
	unsigned short m_num;

protected:
	BOOL first_time;
	int m_nsSNA;
//	pthread_mutex_t m_mtxSNA;

	//to store received packs

	/*
	double m_tSNA[MAX_SNA];
	int m_sna[MAX_SNA];
	int m_nSNA;
*/
	int m_sna;

	char m_buffer[BUFFERSIZE_SNA];
	int m_nBuffer;

protected:
	static void Translate(DAQRAWDATA *pRaw, DAQDATA *pDAQ);

public:
	double GetDAQTime() { return m_tDAQ0; }
	DAQDATA &GetDAQData() { return m_daq0; }

	friend class clsDLG;
};


#endif				//DAQ_H_
