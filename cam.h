/*
 * cam.h
 *
 *  Created on: Mar 15, 2011
 * this is the header file for camera function
 */

#ifndef CAM_H_
#define CAM_H_

#include "thread.h"
#include "uav.h"
#include <stdio.h>
#include <stdlib.h>

//CAM
#define CAM_BAUDRATE    115200
#define COUNT_UAVSTATE_CAM	5
#define MASTERMIND 11

#pragma pack(push, 1)

#define MAX_CAMBUFFER		4096
#define DETECT_LOCK		1
struct TARGETINFO {
	short code;
	short DetectState;
	double   time;
	// pose of the target related to the world coordinate
	double ga, gb, gc;	//angles of the target
	//double gp, gq, gr;  //angular velocity of the target
    double gx, gy, gz;	//position or (longitude, latitude, altitude) of the target
	//double gu, gv, gw;	//velocity	of the target

	// pose of the target related to the body coordinate
	double ta, tb, tc;	//angles of the target
	//double tp, tq, tr;  //angular velocity of the target
    double tx, ty, tz;	//position or (longitude, latitude, altitude) of the target
	//double tu, tv, tw;	//velocity	of the target
};

struct TARGETSTATE {
	// pose of the target related to the world coordinate
	double ga, gb, gc;	//angles of the target
	//double gp, gq, gr;  //angular velocity of the target
    double gx, gy, gz;	//position or (longitude, latitude, altitude) of the target
	//double gu, gv, gw;	//velocity	of the target

	// pose of the target related to the body coordinate
	double ta, tb, tc;	//angles of the target
	//double tp, tq, tr;  //angular velocity of the target
    double tx, ty, tz;	//position or (longitude, latitude, altitude) of the target
	//double tu, tv, tw;	//velocity	of the target
};

struct CAMTELEINFO
{
	double detectState;		// detect status: true, false
	double ta, tb, tc;		// target angle
	double tx, ty, tz;		// target position w.r.t uav body frame
	double tmp1, tmp2, tmp3;
};

#pragma pack(pop)

#define MAX_CAMINFO		128

class clsCAM : public clsThread {
public:
	clsCAM();
	virtual ~clsCAM();

protected:
	COMMAND m_cmd;
	pthread_mutex_t m_mtxCmd;

public:
	void PutCommand(COMMAND *pCmd);
	void GetCommand(COMMAND *pCmd);

	BOOL ProcessCommand(COMMAND *pCommand);
public:
	BOOL Init();
	virtual BOOL InitThread();
	virtual int EveryRun();
	virtual void ExitThread();

	BOOL StartInputThread(int priority);
	static void *InputThread(void *pParameter);
	void Input();

	BOOL ReadCommand(COMMAND *pCmd);
protected:
	FILE *m_pfCAM;
	int m_nsCAM;
	char m_buffer[MAX_CAMBUFFER];
	int m_nBuffer;
	int m_nCountCAMRD;
//	BOOL ReadCamInfo(CAMINFO *pInfo);

//	int SearchPackage(char *pBuffer, int len);
protected:
	CAMTELEINFO m_camTeleInfo;
	double m_tInfo0;

	double m_tInfo[MAX_CAMINFO];
	int m_nInfo;
	double m_tInfo2;    //latest time chosed from m_tInfo[]

	pthread_mutex_t m_mtxCAM;

	void MakeTelegraph(TELEGRAPH *pTele, short code, double time, void *pData, int nDataSize);
	void MakeCAMTeleInfo(const TARGETINFO &targetInfo, const TARGETSTATE &targetState);

protected:
	double m_tTargetState0;  //the current time
	TARGETINFO  m_targetInfo0;
	TARGETSTATE m_targetState0;  //the current state of tracking and attacking
	void Update();       //calcualte the state of target
	void Estimator();    //esimste the velocity of target in the world coordinate

protected:
	short m_DetectState;    //detect the target: true, didn't detect the target: false
	void GetInfo();

public:
	short IsDetected() { return m_DetectState; }

public:
	TARGETSTATE &GetState() { return m_targetState0; }
	TARGETINFO &GetTargetInfo()    { return m_targetInfo0; }  //get the latest time and info
//	double GetStateTime() { return m_tTargetState0; }
	double GetStateTime() { return m_targetInfo0.time; }
	CAMTELEINFO &GetCAMTeleInfo()	{ return m_camTeleInfo; }

/* added by LPD for 2013-UAVGP 2013-08-06 */
protected:
	DATASTRUCT_VISION2GUMSTIX m_dataFromVision;
	DATASTRUCT_VISION2GUMSTIX m_targetInfoUpdate;
	double tStart;
	double m_releaseTargetFinalPhase;
	BOOL m_bVisionFinalPhaseFlag;

public:
	DATASTRUCT_VISION2GUMSTIX GetVisionTargetInfo();  //return relative position in SHIP frame;
	void SetReleaseFinalPhaseStatus() { m_releaseTargetFinalPhase = 1; }
	void ResetReleaseFinalPhaseStatus() { m_releaseTargetFinalPhase = 0; }

	BOOL GetVisionFinalPhase() { return m_bVisionFinalPhaseFlag; }
	void SetVisionFinalPhase() { m_bVisionFinalPhaseFlag = true; }
	void ResetVisionFinalPhase() { m_bVisionFinalPhaseFlag = false; }
	/* end of LPD's part */
};


#endif /* CAM_H_ */
