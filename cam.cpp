/*
 * cam.cpp
 *
 *  Created on: Mar 15, 2011

 */

#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>

#include "uav.h"
#include "cam.h"
#include "state.h"
#include "ctl.h"
#include "laser.h"

extern EQUILIBRIUM _equ_Hover;
extern clsState _state;
extern clsCTL _ctl;
extern clsURG _urg;
extern double Psimeasureall[4];
/*
 * clsCAM
 */


clsCAM::clsCAM()
{
	m_nsCAM = -1;

	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutex_init(&m_mtxCAM, &attr);
}

clsCAM::~clsCAM()
{
}

BOOL clsCAM::Init()
{
	m_nsCAM = open("/dev/serusb3", O_RDWR|O_NONBLOCK);

	if (m_nsCAM == -1) {
		printf("[CAM] open serusb3 failed!\n");
		return FALSE;
	}

	termios termCAM;
    tcgetattr(m_nsCAM, &termCAM);

	cfsetispeed(&termCAM, CAM_BAUDRATE);				//input and output baudrate
	cfsetospeed(&termCAM, CAM_BAUDRATE);

	termCAM.c_cflag &= ~(CS5|CS6|CS7|HUPCL|IHFLOW|OHFLOW|PARENB|CSTOPB|CSIZE); // Note: must do ~CSIZE before CS8
	termCAM.c_cflag |= (CS8 | CLOCAL | CREAD);
	termCAM.c_iflag = 0; // Raw mode for input
	termCAM.c_oflag = 0; // Raw mode for output

	tcflush(m_nsCAM, TCIOFLUSH);
	tcsetattr(m_nsCAM, TCSANOW, &termCAM);

	return TRUE;
}

BOOL clsCAM::InitThread()
{
	m_nBuffer = 0;
	m_tInfo0 = -1;

	m_nInfo=0;

	m_DetectState = 0;
	::memset(&m_targetInfo0, 0, sizeof(TARGETINFO));
	::memset(&m_targetState0, 0, sizeof(TARGETSTATE));
	::memset(&m_dataFromVision, 0, sizeof(DATASTRUCT_VISION2GUMSTIX));
	::memset(&m_targetInfoUpdate, 0, sizeof(DATASTRUCT_VISION2GUMSTIX));

	int index;

	tStart = 0;

	m_releaseTargetFinalPhase = 0;
	m_bVisionFinalPhaseFlag = false;

	printf("[CAM] start\n");

	return TRUE;
}

void *clsCAM::InputThread(void *pParameter)
{
	clsCAM *pCAM = (clsCAM *)pParameter;

	pCAM->Input();

	return NULL;
}

BOOL clsCAM::StartInputThread(int priority)
{
    pthread_attr_t attribute;
    pthread_attr_init(&attribute);
    pthread_attr_setdetachstate(&attribute, PTHREAD_CREATE_DETACHED);
    pthread_attr_setinheritsched(&attribute, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attribute, SCHED_RR);

    sched_param_t param;
    pthread_attr_getschedparam(&attribute, &param);
    param.sched_priority = priority;
    pthread_attr_setschedparam(&attribute, &param);

	pthread_t thread;
    pthread_create(&thread, &attribute, &clsCAM::InputThread, this);

    return TRUE;
}

void clsCAM::Input()
{
	printf("[CAM] start (input thread)\n");

	COMMAND cmd; cmd.code = -1; //initialize

	//struct TARGETSTATE targetState;

	m_nCountCAMRD=0;

	while (1) {
		if (ReadCommand(&cmd)) {
			switch (cmd.code){
			case DATA_VISION2GUMSTIX:{
				memcpy(&m_dataFromVision, cmd.parameter, sizeof(DATASTRUCT_VISION2GUMSTIX));

//				for (int i = 0; i< 10; i++) printf("%d ", m_dataFromVision.flags[i]);
//				printf("%.2f ", m_dataFromVision.masterMind_time);
//				for(int i = 0; i< 3; i++) printf("%.2f ", m_dataFromVision.cameraFrame_dvec[i]);
//				for(int i = 0; i< 3; i++) printf("%.2f ", m_dataFromVision.nedFrame_dvec[i]);
//				printf("\n");

				////			for gimble case;
				double abc[3] = {(-1.0)*_state.GetState().a, (-1.0)*_state.GetState().b, 0};
				double r_t_c[3] = {m_dataFromVision.cameraFrame_dvec[0], m_dataFromVision.cameraFrame_dvec[1], m_dataFromVision.cameraFrame_dvec[2]};

				double camera_body[3] = {0};
				B2G(abc, r_t_c, camera_body);
				camera_body[0] -= 0.13;
				camera_body[1] += 0;
				camera_body[2] -= 0.2;

				//printf("[Cam Target Camera-BodyFrame] %d %d %.2f %.2f %.2f\n", m_dataFromVision.flags[0], m_dataFromVision.flags[1], camera_body[0], camera_body[1], camera_body[2]);

				for(int i = 0 ; i < 10; i++) m_targetInfoUpdate.flags[i] = m_dataFromVision.flags[i];

				double abc_ned[3] = {_state.GetState().a, _state.GetState().b, _state.GetState().c};
				double delta_body[3] = {camera_body[0], camera_body[1], camera_body[2]};
				double delta_ned[3] = {0};
				B2G(abc_ned, delta_body, delta_ned);

				// in ned frame
				m_targetInfoUpdate.nedFrame_dvec[0] = delta_ned[0];
				m_targetInfoUpdate.nedFrame_dvec[1] = delta_ned[1];
				m_targetInfoUpdate.nedFrame_dvec[2] = delta_ned[2];
				if(m_nCount%50 == 0){
					printf("[Cam Target Camera-NEDFrame] %d %d %.2f %.2f %.2f\n", m_targetInfoUpdate.flags[0], m_targetInfoUpdate.flags[1], m_targetInfoUpdate.nedFrame_dvec[0], m_targetInfoUpdate.nedFrame_dvec[1], m_targetInfoUpdate.nedFrame_dvec[2]);
				}
				break;
				}
			}
		}
		m_nCountCAMRD++;
		usleep(100000);
	}
}

BOOL clsCAM::ReadCommand(COMMAND *pCmd)
{
	int nRead = read(m_nsCAM, m_buffer+m_nBuffer, MAX_CAMBUFFER-m_nBuffer);	//get data as many as it can

	if (nRead < 0) return FALSE;
	if (/*nRead != -1*/ nRead > 0){
		//printf("[CAM] ReadCommand, read byte %d, buffer size %d\n", nRead, m_nBuffer);
//		printf("tElapse %.2f: ", ::GetTime());
//		for (int i= 0; i<nRead; i++)
//		printf("%02x ",  (unsigned char)m_buffer[m_nBuffer + i]);
//		printf("\n");
	}
	m_nBuffer += nRead;

#if ( _DEBUG & DEBUGFLAG_CAM )
	if(m_nCountCAMRD%_DEBUG_COUNT_CAMRD ==0){
	if(nRead!=0){
		printf("\n");
		printf("[CAM] ReadCommand, read byte %d, buffer size %d\n", nRead, m_nBuffer);
//     	char *pdChar = m_buffer;
//    	printf("[CAM] ReadCommand Buffer=");
//    	while(pdChar< m_buffer+nRead){
//    		printf("%4x", *(unsigned char*)pdChar);  pdChar++;
//   		}
//    	printf("\n");
	}
	}
#endif

	COMMAND cmd = {0};

	char *pChar = m_buffer;
	char *pCharMax = m_buffer+m_nBuffer-1;
	enum { BEGIN, HEADER, SIZE, PACKAGE, CHECK } state = BEGIN;

	char package[MAXSIZE_PACKAGE];
	int nPackageSize = 0;

	char *pNextTelegraph = NULL;

	while (pChar <= pCharMax) {
		if (state == BEGIN) {
			if ( *pChar == 0x80|MASTERMIND) {
				state = HEADER;
			}
			pChar ++; continue;
		}

		if (state == HEADER) {
			if (*pChar == 0x80|5) state = SIZE;
			else  state = BEGIN;
			pChar ++; continue;
		}

		if (state == SIZE) {
			if (pChar + 1 > pCharMax) break;
			nPackageSize = GETWORD(pChar);
			if (nPackageSize < 1 || nPackageSize > MAXSIZE_PACKAGE) {
				state = BEGIN;
				continue;
			}
			else {
				state = PACKAGE;
				pChar += 2;
				continue;
			}
		}

		if (state == PACKAGE) {
			if (pChar + nPackageSize + 1 > pCharMax) break;

			unsigned short sum = CheckSum(pChar, nPackageSize);
			unsigned short check = GETWORD(pChar+nPackageSize);

//#if (_DEBUG & DEBUGFLAG_CAM)
//		printf("[CAM] Readcommand PACKAGE: sum=%d, check=%d\n", sum, check);
//#endif
			if (sum != check) {
				//printf("[CAM] Checksum error\n");
				state = BEGIN; continue;
			}

			::memcpy(package, pChar, nPackageSize);		// copy the correct package

			pNextTelegraph = pChar + nPackageSize + 2;

			pChar=pNextTelegraph;
		}
	}

	//if (pNextTelegraph == NULL) return FALSE;
	if ( (pNextTelegraph==NULL)) {//didn't find a completed data package
//		if (state!=PACKAGE) {// the state is not PACKAGE
//			m_nBuffer=0;
//		}
//		printf("pNextTelegraph NULL!\n");
		return FALSE;
	}

	if (pNextTelegraph > pCharMax) {
//		printf("pNextTelegraph > pCharMax!\n");
		m_nBuffer = 0;
	}
	else {
		m_nBuffer = pCharMax - pNextTelegraph + 1; //unfinished telegraph copy to buffer for prcessing in next cycle
		::memcpy(m_buffer, pNextTelegraph, m_nBuffer);
	}

	cmd.code = GETWORD(package);

//	m_DetectState = GETWORD(package+2);
	::memcpy(cmd.parameter, package+10, sizeof(DATASTRUCT_VISION2GUMSTIX) );
	*pCmd = cmd;

	return TRUE;
}

void clsCAM::Update()
{
//	UAVSTATE &state = _state.GetState();

	/*double l, m, n, h;

	h = state.z;

	m_tTARGETState0   = m_tInfo2;
	m_TARGETState0.tq = atan(-1*n/sqrt(l*l+m*m));
	m_TARGETState0.tr = atan(m/l);
	*/
}

void clsCAM::GetInfo()
{
	double tInfo = 0;
	int index, tIndex=0;

	for (index=0; index< MAX_CAMINFO; index++) {
		if ( m_tInfo[index]>tInfo ) {
			tIndex = index;
			tInfo  = m_tInfo[index];
		}
	}
}

void clsCAM::PutCommand(COMMAND *pCmd)
{
	pthread_mutex_lock(&m_mtxCmd);

	m_cmd = *pCmd;

	pthread_mutex_unlock(&m_mtxCmd);
}

void clsCAM::GetCommand(COMMAND *pCmd)
{
	pthread_mutex_lock(&m_mtxCmd);

	if (m_cmd.code == 0) pCmd->code = 0;
	else {
		*pCmd = m_cmd;
		m_cmd.code = 0;				//clear
	}

	pthread_mutex_unlock(&m_mtxCmd);
}

BOOL clsCAM::ProcessCommand(COMMAND *pCommand)
{
	COMMAND &cmd = *pCommand;
//	char *paraCmd = cmd.parameter;

//	int &behavior = m_behavior.behavior;
//	char *paraBeh = m_behavior.parameter;

	BOOL bProcess = TRUE;
//	_state.EnableVision();
	switch (cmd.code) {
	case COMMAND_CAMVBS:
//		behavior = BEHAVIOR_TAKEOFF;
		_state.EnableVision();
		break;
	default:
		bProcess = FALSE;
		break;
	}
	return bProcess;
}

void clsCAM::MakeTelegraph(TELEGRAPH *pTele, short code, double time, void *pData, int nDataSize)
{
	char *pBuffer = pTele->content;

	pBuffer[0] = 0x87;				//_nHelicopter 6 or 7, 0x56 for helion, 0x57 for shelion
	pBuffer[1] = 0x93;
	
	switch(code) {
		case COMMAND_CAMRUN:
		case COMMAND_CAMSTOP:
		case COMMAND_CAMQUIT:
		case COMMAND_CAMTHOLD: {
//			(short &)pBuffer[2] = nDataSize+2;
//			(short &)pBuffer[4] = code;
//			(double &)pBuffer[6] = *(double*)pData;
//			(unsigned short &)pBuffer[6+nDataSize] = CheckSum(pBuffer+4, 2+nDataSize);
//			pTele->size = nDataSize + 8;
			PUTWORD(pBuffer+2, nDataSize+2);
			PUTWORD(pBuffer+4, code);
			::memcpy(pBuffer+6, pData, nDataSize);

			unsigned short sum  = CheckSum(pBuffer+4, 2+nDataSize);
			PUTWORD(pBuffer+14+nDataSize, sum);

			pTele->size = nDataSize + 8;
			break; }
		case DATA_GUMSTIX2VISION: {
//			(short &)pBuffer[2] = nDataSize+10;
//			(short &)pBuffer[4] = code;
//			(double &)pBuffer[6]= time;
//			::memcpy(pBuffer+14, pData, nDataSize);
//			(unsigned short &)pBuffer[14+nDataSize] = CheckSum(pBuffer+4, 10+nDataSize);

			PUTWORD(pBuffer+2, nDataSize+10);
			PUTWORD(pBuffer+4, code);
			PUTDOUBLE(pBuffer+6, time);

			::memcpy(pBuffer+14, pData, nDataSize);

			unsigned short sum  = CheckSum(pBuffer+4, 10+nDataSize);
			PUTWORD(pBuffer+14+nDataSize, sum);

			pTele->size = nDataSize + 16;
			break;}
	}

#if(_DEBUG  & DEBUGFLAG_CAMWR )
	switch(code) {
		case COMMAND_CAMRUN:
		case COMMAND_CAMSTOP:
		case COMMAND_CAMQUIT:
		case COMMAND_CAMTHOLD:
			printf("[CAM] Tele: code=%3d, parameter=%7.3f\n",(short &)pBuffer[4], (double &)pBuffer[6]);
			break;
		case COMMAND_CAMUAVPOSE:
			printf("[CAM] Tele: code=%3d, time=%7.3f\n",(short &)pBuffer[4], (double &)pBuffer[6]);
			printf("[CAM] Tele: data=");

			char *pChar;
			for (pChar=&pBuffer[14]; pChar<pBuffer+nDataSize+14; pChar+=sizeof(double) ) {
				printf("%6.2f", *(double *)pChar);
			}
			printf("\n");

			printf("[CAM] Tele (binary):");
			for (pChar=pBuffer; pChar<pBuffer+nDataSize+16; pChar++ ) {
				printf("%4x", *(unsigned char*)pChar);
			}
			printf("\n");
			break;
	}
#endif

}
int clsCAM::EveryRun()
{
	UAVSTATE &state = _state.GetState();
	double t = ::GetTime();

	DATASTRUCT_GUMSTIX2VISION temp_dataGumstix2Vision;
	memset(&temp_dataGumstix2Vision, 0, sizeof(DATASTRUCT_GUMSTIX2VISION));

	temp_dataGumstix2Vision.landingFinishFlag = _ctl.GetLandingFinishFlag();
	temp_dataGumstix2Vision.gumstixTime = ::GetTime();
	temp_dataGumstix2Vision.x = state.x; temp_dataGumstix2Vision.y = state.y; temp_dataGumstix2Vision.z = state.z;
	temp_dataGumstix2Vision.ug = state.ug; temp_dataGumstix2Vision.vg = state.vg; temp_dataGumstix2Vision.wg = state.wg;
	temp_dataGumstix2Vision.acx = state.acx; temp_dataGumstix2Vision.acy = state.acy; temp_dataGumstix2Vision.acz = state.acz;
	temp_dataGumstix2Vision.a = state.a; temp_dataGumstix2Vision.b = state.b; temp_dataGumstix2Vision.c = state.c;
	temp_dataGumstix2Vision.u = state.u; temp_dataGumstix2Vision.v = state.v; temp_dataGumstix2Vision.w = state.w;
	temp_dataGumstix2Vision.latitude = state.latitude*180.0/PI; temp_dataGumstix2Vision.longitude = state.longitude*180.0/PI;

	TELEGRAPH tele;

	//write the pose of the UAV
	if ( m_nCount% 2 == 0  )  {
		MakeTelegraph(&tele, DATA_GUMSTIX2VISION, t, &temp_dataGumstix2Vision, sizeof(DATASTRUCT_GUMSTIX2VISION));
		int nWrite=write(m_nsCAM, tele.content, tele.size);
		//printf("[CAM] Sent %d bytes to vision cpu\n", nWrite);
	}
	return TRUE;
}

void clsCAM::ExitThread()
{
//	if (m_pfCAM != NULL) ::fclose(m_pfCAM);
	printf("[CAM] quit\n");
}

DATASTRUCT_VISION2GUMSTIX clsCAM::GetVisionTargetInfo() {  //return relative position in NED frame;
	m_targetInfoUpdate.nedFrame_dvec[2] = m_targetInfoUpdate.nedFrame_dvec[2]*(-1);

//	m_targetInfoUpdate.flags[0] = true;
//	m_targetInfoUpdate.nedFrame_dvec[0] = 0;
//	m_targetInfoUpdate.nedFrame_dvec[1] = 0.02;

	return m_targetInfoUpdate;
}
/*
 * clsVision
 */
