//cmm.cpp
//this is the implementation file for class clsCMM, which is responsible for the wireless modem communication with the ground station
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <stdio.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <netinet/in.h>

#include "uav.h"
#include "laser.h"
#include "cmm.h"
#include "state.h"
#include "coop.h"
#include "daq.h"
#include "svo.h"
#include "net.h"
#include "ctl.h"

#include "im6.h"
#include "im7.h"
#include "im8.h"

extern clsState _state;
extern clsURG _urg;

BOOL clsCMM::Open()
{
	m_nsCMM = open("/dev/ser4", O_RDWR |O_NONBLOCK);

//	m_nsCMM = open("/dev/serusb1", O_RDWR | O_NONBLOCK);
	if (m_nsCMM == -1) { m_nsCMM = 0; return FALSE; }

	printf("[CMM] wireless modem (/dev/serusb1) successfully opened\n");

	termios termCMM;
    tcgetattr(m_nsCMM, &termCMM);

	cfsetispeed(&termCMM, CMM_BAUDRATE);				//input and output baudrate
	cfsetospeed(&termCMM, CMM_BAUDRATE);

	termCMM.c_cflag = CS8 | CLOCAL | CREAD;
//	termCMM.c_iflag = IGNBRK |IGNCR |IGNPAR;

	tcsetattr(m_nsCMM, TCSANOW, &termCMM);
	tcflush(m_nsCMM, TCIOFLUSH);

//	char devPort[] = "\/dev\/ser4";
//	SetCmmConfig(devPort, sizeof(devPort), 1, CMM_BAUDRATE);

	return TRUE;
}

void clsCMM::Close()
{
	close(m_nsCMM);
}

void clsCMM::MakeTelegraph(TELEGRAPH *pTele, short code, double time, const void *pData, int nDataSize)
{
	char *pBuffer = pTele->content;

	pBuffer[0] = 0x80 | _HELICOPTER;
	pBuffer[1] = 0x80 | ID_STATION;				//ground station

//	(short &)pBuffer[2] = nDataSize+10;
//	(short &)pBuffer[4] = code;
//	(double &)pBuffer[6] = time;

	PUTWORD(pBuffer+2, nDataSize+10);
	PUTWORD(pBuffer+4, code);
	PUTDOUBLE(pBuffer+6, time);

	::memcpy(pBuffer+14, pData, nDataSize);

	unsigned short sum  = CheckSum(pBuffer+4, 10+nDataSize);
	PUTWORD(pBuffer+14+nDataSize, sum);

	pTele->size = nDataSize + 16;
}

/*void clsCMM::MakeTelegraph(TELEGRAPH *pTele, short code, double time, void *pData, int nDataSize)
{
	//telegraph format
	// from, to, size, (COMMAND_DATA, datatype, time, data), checksum				//bracketed is the package
	// 1      1    2        2            2       8     var    2
	char *pBuffer = pTele->content;

//	(double &)pBuffer[6] = time;
	pBuffer[0] = _HELICOPTER;				//_nHelicopter 6 or 7, 0x56 for helion, 0x57 for shelion
	pBuffer[1] = ID_STATION;				//ground station

	(short &)pBuffer[2] = nDataSize+12;
	(short &)pBuffer[4] = COMMAND_DATA;
	(short &)pBuffer[6] = code;				//code - data type

	double* pTime = &time;
	(double &)pBuffer[8] = time;
	::memcpy(pBuffer+16, pData, nDataSize);
	(unsigned short &)pBuffer[16+nDataSize] = CheckSum(pBuffer+4, 12+nDataSize);

	pTele->size = nDataSize + 18;
}
*/
BOOL clsCMM::InitThread()
{
	m_nBuffer = 0;
	m_cmd.code = 0;
	m_bVicon = FALSE;
	m_nMessage = 0;

	printf("[CMM] Start\n");

	return TRUE;
}

void clsCMM::SendAllMessages()
{
	pthread_mutex_lock(&m_mtxMessage);

	for (int i=0; i<=m_nMessage-1; i++) {
		SendMessage(m_szMessage[i]);
	}
	m_nMessage = 0;

	pthread_mutex_unlock(&m_mtxMessage);
}

int clsCMM::EveryRun()
{
	TELEGRAPH tele;

/*	if (m_nCount % LEADER_UPDATE != 0)
		return TRUE;

	double tCoop = _coop.GetCoopTime();
	if ( tCoop > 0 && _coop.GetCoopStart() )
	{
		UAVSTATE& state = _state.GetState();
		RPTSTATE& RPTState = _state.GetRPTState();


		double t = ::GetTime();
		double ldStatus[11] = {tPath t, state.x, state.y, state.z, state.c, state.ug, state.vg, state.wg, state.p, state.q, state.r};
		double flRef[4];
		int cmdPara4Fl[4] = {0};
		double actionPara4Fl[9] = {flRef[0], flRef[1], flRef[2], flRef[3], 0, 0, 0, 0, 0};
	//	MakeCoopPkt(LEADERFORMATION_UPDATE, cmdPara4Fl, actionPara4Fl);
		_coop.MakeCoopPkt(LEADERFORMATION_UPDATE, NULL, ldStatus);

		COOP_PKT coopedPkt = _coop.FetchCoopedPkt();
		MakeTelegraph(&tele, DATA_COOP, tCoop, &coopedPkt, sizeof(COOP_PKT));
		write(m_nsCMM, tele.content, tele.size);
	}*/

//	if (m_nCount % COUNT_CMM != 0 || (m_nCount == 0)) return TRUE;
	if (m_nCount % 5 != 0 ) return TRUE;
//	double t;
//	IM6PACK pack6;

	SendAllMessages();

/*	BOOL bCMMGP6 = _im6.GetLastPack(DATA_GP6, &t, &pack6);
	if (bCMMGP6) {
		MakeTelegraph(&tele, DATA_GP6, t, &pack6, sizeof(IM6PACK));
		write(m_nsCMM, tele.content, tele.size);
	}*/

/*    BOOL bCMMAH6 = _im6.GetLastPack(DATA_AH6, &t, &pack6);
	if (bCMMAH6) {
		MakeTelegraph(&tele, DATA_AH6, t, &pack6, sizeof(IM6PACK));
        write(m_nsCMM, tele.content, tele.size);
    }*/

/*	BOOL bCMMSC6 = _im6.GetLastPack(DATA_SC6, &t, &pack6);
    if (bCMMSC6) {
		MakeTelegraph(&tele, DATA_SC6, t, &pack6, sizeof(IM6PACK));
        write(m_nsCMM, tele.content, tele.size);
    }*/

/*
	IM7PACK pack7;
	BOOL bCMMGP7 = _im7.GetLastPack(DATA_GP7, &t, &pack7);
	if (bCMMGP7) {
		MakeTelegraph(&tele, DATA_GP7, t, &pack7, sizeof(IM7PACK));
		write(m_nsCMM, tele.content, tele.size);
	}

	BOOL bCMMSC7 = _im7.GetLastPack(DATA_SC7, &t, &pack7);
    if (bCMMSC7) {
		MakeTelegraph(&tele, DATA_SC7, t, &pack7, sizeof(IM7PACK));
        write(m_nsCMM, tele.content, tele.size);
    }*/

	double tState = _state.GetStateTime();
	UAVSTATE &state = _state.GetState();

	BOOL bCMMState = tState > 0;
    if (bCMMState) {
		MakeTelegraph(&tele, DATA_STATE, tState, &state, sizeof(UAVSTATE));
		SendTelegraph(&tele);
    }

	double tDAQ = _daq.GetDAQTime();
	BOOL bCMMDAQ = tDAQ > 0;
	if (bCMMDAQ) {
		MakeTelegraph(&tele, DATA_DAQ, tDAQ, &_daq.GetDAQData(), sizeof(DAQDATA));
		SendTelegraph(&tele);
    }

	double tSVO = _svo.GetSVOTime();
	BOOL bCMMSVO = tSVO > 0;
	if (bCMMSVO) {
		MakeTelegraph(&tele, DATA_SVO, tSVO, &_svo.GetSVOData(), sizeof(SVODATA));
		SendTelegraph(&tele);
	}

	double tEQU = _svo.GetTrimvalueTime();
	BOOL bCMMEQU = tEQU > 0;
	if(bCMMEQU)
	{
		MakeTelegraph(&tele, DATA_EQU, tEQU, &_svo.GetTrimvalue(), sizeof(HELICOPTERRUDDER));
		SendTelegraph(&tele);
	}

	double tSIG = _state.GetSIGTime();
	BOOL bCMMSIG = tSIG > 0;
	if (bCMMSIG) {
		MakeTelegraph(&tele, DATA_SIG, tSIG, &_state.GetSIG(), sizeof(HELICOPTERRUDDER));
		SendTelegraph(&tele);
	}

	double tURG = _urg.getURGTime();
	if(tURG>0){
		MakeTelegraph(&tele, DATA_URG, tURG, &_urg.getLaserData(), sizeof(LASERRAWDATA));
		SendTelegraph(&tele);
	}

/*	double tURG = _urg.getURGTime();
		if(tURG>0){
			MakeTelegraph(&tele, DATA_URG_4, tURG, &_urg.getLaserData4(), sizeof(LASERRAWDATA4));
			SendTelegraph(&tele);
		}*/

/*	double tCoop1 = _coop.GetCoopTime();
	if ( tCoop1 > 0 && !_coop.GetCoopStart() )
	{
		COOP_PKT coopedPkt = _coop.FetchCoopedPkt();
		MakeTelegraph(&tele, DATA_COOP, tCoop1, &coopedPkt, sizeof(COOP_PKT));
		write(m_nsCMM, tele.content, tele.size);
	}*/
	double tCoop = _coop.GetCoopTime();
	if ( tCoop > 0 )
	{
		COOPSTATE coopstate = _coop.GetCoopState();
		MakeTelegraph(&tele, DATA_COOP, tCoop, &coopstate, sizeof(COOPSTATE));
		SendTelegraph(&tele);
	}

	// send configuration data
/*	IMU_CONFIG imuConfig = _im8.GetIMUConfig();
	MakeTelegraph(&tele, DATA_IMU_CONFIG, 0, &imuConfig, sizeof(IMU_CONFIG));
	SendTelegraph(&tele);

	LASER_CONFIG laserConfig = _urg.GetLaserConfig();
	printf("Laser configuration: %s, %d\n", laserConfig.laserDevice, laserConfig.flag);
	MakeTelegraph(&tele, DATA_LASER_CONFIG, 0, &laserConfig, sizeof(LASER_CONFIG));
	SendTelegraph(&tele);

	SVO_CONFIG svoConfig = _svo.GetSvoConfig();
	MakeTelegraph(&tele, DATA_SVO_CONFIG, 0, &svoConfig, sizeof(SVO_CONFIG));
	SendTelegraph(&tele);*/

/*	MakeTelegraph(&tele, DATA_CAM_CONFIG, 0, &svoConfig, sizeof(SVO_CONFIG));
	SendTelegraph(&tele);*/

//	CAM_CONFIG camConfig = _cam.GetCamConfig();
//	printf("Cam config: %s, %d, %d\n", camConfig.devPort, camConfig.flag, camConfig.baudrate);
//	MakeTelegraph(&tele, DATA_CAM_CONFIG, 0, &camConfig, sizeof(CAM_CONFIG));
//	SendTelegraph(&tele);

//	CMM_CONFIG cmmConfig = GetCmmConfig();
//	MakeTelegraph(&tele, DATA_CMM_CONFIG, 0, &cmmConfig, sizeof(CMM_CONFIG));

/*	TCP_CONFIG tcpConfig = GetTcpConfig();
	printf("TCP configuration: %s, %d\n", tcpConfig.ipaddress, tcpConfig.portNum);
	MakeTelegraph(&tele, DATA_TCP_CONFIG, 0, &tcpConfig, sizeof(TCP_CONFIG));
	SendTelegraph(&tele)*/;

#if (_DEBUG & DEBUGFLAG_CMM)
	char szCMM[256];
	strcpy(szCMM, "[CMM] Send data,");
//	if (bCMMGP6) strcat(szCMM, " GP6");
//	if (bCMMAH6) strcat(szCMM, " AH6");
//	if (bCMMSC6) strcat(szCMM, " SC6");
	if (bCMMState) strcat(szCMM, " STATE");
	if (bCMMDAQ) strcat(szCMM, " DAQ");
	if (bCMMSVO) strcat(szCMM, " SVO");
	if (bCMMEQU) strcat(szCMM, " EQU ");
	if (bCMMSIG) strcat(szCMM, " SIG");
	printf("%s\n", szCMM);
#endif
	return TRUE;
}

void clsCMM::ExitThread()
{
	printf("[CMM] quit\n");
}

void *clsCMM::InputThread(void *pParameter)
{
	clsCMM *pCMM = (clsCMM *)pParameter;

	pCMM->Input();

	return NULL;
}

void clsCMM::Input()
{
	printf("[CMM] Input start\n");

	ADDRESSEDPACKAGE package;
	while (1) {
		int nToRead = MAX_CMMBUFFER-m_nBuffer;
		int nRead = read(m_nsCMM, m_szBuffer+m_nBuffer, nToRead);

		if (nRead < 0) {
			nRead = 0;				//read returns -1 on occassion of no byte read
			usleep(20000);
			continue;
		}

		m_nBuffer += nRead;

#if (_DEBUG & DEBUGFLAG_CMM)
		printf("[CMM] ReadCommand, read byte %d, buffer size %d\n", nRead, m_nBuffer);
#endif

		if (ParseBuffer(m_szBuffer, m_nBuffer, &package))	{
			ProcessPackage(&package);		//analysis buffer to extrace telegraph package from it
		}

		usleep(20000);
	}
}

void clsCMM::SendMessage(const char *psz)
{
	TELEGRAPH tele;

	MakeTelegraph(&tele, DATA_MESSAGE, GetTime(), psz, ::strlen(psz)+1);

	if (m_nsCMM != 0) {
		int nWrite = write(m_nsCMM, tele.content, tele.size);
		unsigned char *buffer = (unsigned char *)tele.content;
		printf("[CMM] %d bytes sent, data %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", nWrite,
				buffer[0], buffer[1], buffer[2], buffer[3], buffer[4],
				buffer[5], buffer[6], buffer[7], buffer[8], buffer[9]);
	}
//	else if (m_socket != 0) SendViaNet("255.255.255.255", tele.content, tele.size);
	else if (m_socket != 0) SendViaNet(m_gcsIPAddr, /*GCS_IPADDR,*/ tele.content, tele.size);
//	if (m_socket != 0) SendViaNet("192.168.109.1", tele.content, tele.size);

	printf("[CMM] Message sent - %s\n", psz);
}

void clsCMM::SendRawMessage(char *message) {
	if (m_nsCMM != 0) {
		write(m_nsCMM, message, strlen(message));
	}
}

void clsCMM::SendPackage(ADDRESSEDPACKAGE *pPackage)
{
	TELEGRAPH tele;
	MakeTelegraph(pPackage, &tele);

	SendTelegraph(&tele);
}

void clsCMM::SendTelegraph(TELEGRAPH *pTele)
{
	if (m_nsCMM != 0) {
		int nWrite = write(m_nsCMM, pTele->content, pTele->size);
		printf("[CMM] sent bytes %d\n", nWrite);
	}
	else if (m_socket != 0) {
//		SendViaNet("192.168.1.99", /*m_gcsIPAddr,*/ /*GCS_IPADDR,*/ pTele->content, pTele->size);
//		SendViaNet("192.168.1.98", /*m_gcsIPAddr,*/ /*GCS_IPADDR,*/ pTele->content, pTele->size);
		SendViaNet(m_gcsIPAddr, pTele->content, pTele->size);
	}
}

void clsCMM::MakeTelegraph(ADDRESSEDPACKAGE *pPackage, TELEGRAPH *pTele)
{
	int size = pPackage->package.size;

	pTele->size = size+6;

	unsigned short sum = CheckSum(pPackage->package.content, pPackage->package.size);

	char *content = pTele->content;
	content[0] = 0x80 | pPackage->from;				//ground station
	content[1] = 0x80 | pPackage->to;

	PUTWORD(content, size);
//	(short &)content[2] = size;				//size

	::memcpy(content+4, pPackage->package.content, size);				//package
	PUTWORD(content+4+size, sum);
//	(unsigned short &)content[4+size] = sum;				//check
}

int clsCMM::SendViaNet(const char *host, char *buffer, int nBuffer)
{
	struct sockaddr_in to;

//	memset( &toSockaddr, 0, sizeof(toSockaddr) );
	to.sin_family = AF_INET;
	to.sin_port = htons(NETPORT_BROADCAST);

	to.sin_addr.s_addr = ::inet_addr(host);
	memset(to.sin_zero, '\0', sizeof(to.sin_zero));

	int nSend = sendto(m_socket, (void *)buffer, nBuffer, 0, (struct sockaddr *)&to, sizeof(to));

	//printf("sent %d bytes\n", nSend);
	return nSend;
}


BOOL clsCMM::StartInputThread(int priority)
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

    pthread_create(&m_idInputThread, &attribute, &clsCMM::InputThread, this);

    return TRUE;
}

clsCMM::clsCMM()
{
	m_nsCMM = 0;

	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutex_init(&m_mtxCmd, &attr);

	pthread_mutex_init(&m_mtxMessage, &attr);
}

clsCMM::~clsCMM()
{
	pthread_mutex_destroy(&m_mtxCmd);
}

void clsCMM::PutMessage(char *pszMessage)
{
	pthread_mutex_lock(&m_mtxMessage);
	if (m_nMessage < MAX_MESSAGE)
		::strcpy(m_szMessage[m_nMessage++], pszMessage);
	pthread_mutex_unlock(&m_mtxMessage);
}

void clsCMM::GetCommand(COMMAND *pCmd)
{
	pthread_mutex_lock(&m_mtxCmd);

	if (m_cmd.code == 0) pCmd->code = 0;
	else {
		*pCmd = m_cmd;
		m_cmd.code = 0;				//clear
	}

	pthread_mutex_unlock(&m_mtxCmd);
}

void clsCMM::PutCommand(COMMAND *pCmd)
{
	pthread_mutex_lock(&m_mtxCmd);

	m_cmd = *pCmd;

	pthread_mutex_unlock(&m_mtxCmd);
}

BOOL clsCMM::StartListenThread(int priority)
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

    pthread_t id;
    pthread_create(&id, &attribute, clsCMM::ListenThread, this);

    return TRUE;
}

void *clsCMM::ListenThread(void *pParameter)
{
	clsCMM *pCMM = (clsCMM *)pParameter;

	pCMM->Listen();

	return NULL;
}

void clsCMM::Listen()
{
	//begin receive data
	struct sockaddr_in from;
	socklen_t fromlen = sizeof(from);

	ADDRESSEDPACKAGE package;

	printf("[CMM] Net started\n");
	while (1) {
		int nRecv = ::recvfrom(m_socket, m_bufferNet, MAXSIZE_TELEGRAPH-m_nBufferNet, 0, (struct sockaddr *)&from, &fromlen);
		if (nRecv == -1) {
			printf("[CMM] socket disconnected.\n");
			break;
		}
//		printf("[CMM] %d bytes received from %s\n", nRecv, inet_ntoa(from.sin_addr));
		m_nBufferNet += nRecv;

/*		for (int i=0; i<m_nBufferNet; i++) {
			printf("%02x ", (unsigned char)m_bufferNet[i]);
		}
		printf("\n");*/
		if (ParseBuffer(m_bufferNet, m_nBufferNet, &package)) {
			ProcessPackage(&package);		//analysis buffer to extrace telegraph package from it
		}
//		usleep(20000);
		//so far only process packages from ground station
	}

	::close(m_socket);
}

BOOL clsCMM::ParseBuffer(char *pBuffer, int &nBuffer, ADDRESSEDPACKAGE *pPackage)
{
	//ParseBuffer analysis if there is a package in buffer,
	//if yes, extract it to pPackage and reset the pBuffer and nBuffer and return true,
	//otherwise do nothing and return false

	char *pChar = pBuffer;
	char *pCharMax = pBuffer + nBuffer - 1;

	enum { BEGIN, HEADER, SIZE, PACKAGE, CHECK } state = BEGIN;

	char from, to;
	short size;

	//pointers to identify packages in buffer
	//pThisPackage point to the first package in buffer, pNextPackage point the the next package
	//if there is no package in buffer, pThisPackage is null, if there is package in buffer, but not completed, then pThisPackage point to the package, but pNextPackage is null
	//if buffer contains a complete package, then pThisPackage point to the package and pNextPackage point to the first char after this package
	char *pThisPackage = NULL;
	char *pNextPackage = NULL;

	while (pChar <= pCharMax) {

		if (state == BEGIN) {
			if ((*pChar >= 0 && *pChar <= 99) || *pChar & 0x80) {
				from = pChar[0];
				pThisPackage = pChar;
				if (from & 0x80) from &= ~0x80;				//eliminate the leading 1, new protocol
				state = HEADER;				//from
			}
			pChar++; continue;
		}

		if (state == HEADER) {
			if ((*pChar >= 0 && *pChar <= 99) || *pChar & 0x80) {
				to = pChar[0];
				to &= ~0x80;				//eliminate the leading 1
				state = SIZE;
			}
			else {
				pThisPackage = NULL;
				state = BEGIN;
			}

			pChar ++; continue;
		}

		if (state == SIZE) {
			if (pChar + 1 > pCharMax) break;					//non-complete break

			size = (short &)pChar[0];
			if (size < 1 || size > MAXSIZE_PACKAGE) {				//improper package, reset
				pThisPackage = NULL;
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
			if (pChar + size + 1 > pCharMax) break;				//non-complete break;

			unsigned short sum = CheckSum(pChar, size);
			unsigned short check = GETWORD(pChar+size);

			if (sum != check) {				//improper package, reset
				pThisPackage = NULL;
				state = BEGIN;
				continue;
			}

			//success
			pPackage->from = from;
			pPackage->to = to;
			pPackage->package.size = size;
			::memcpy(pPackage->package.content, pChar, size);

			pNextPackage = pChar + size + 2;
			break;
		}
	}

	if (pThisPackage == NULL) {				//no package in buffer, clear buffer
		nBuffer = 0;
		return FALSE;
	}

	if (pNextPackage == NULL) return FALSE;				//package not complete, also return false

	if (pNextPackage > pCharMax)
		nBuffer = 0;				//buffer exactly end at this package, clear buffer
	else {				//otherwise, shift left content to the head of buffer (clear this package)
		nBuffer = pCharMax - pNextPackage + 1;
		::memcpy(pBuffer, pNextPackage, nBuffer);
	}

	return TRUE;
}

void clsCMM::ProcessPackage(ADDRESSEDPACKAGE *pPackage)
{
	if (pPackage->to != _HELICOPTER && pPackage->to != ID_ALL) return;				//drop packages not for this helicopter

	int nFrom = pPackage->from;

	char *package = pPackage->package.content;

	COMMAND cmd;
	cmd.code = (short &)package[0];

	if ( GetCommandString(cmd.code) != NULL ) {
		printf("[CMM] Command received %d(%s) from %d\n", cmd.code, GetCommandString(cmd.code), pPackage->from);

		char string[256];
		sprintf(string, "command received %d(%s) from %d", cmd.code, GetCommandString(cmd.code), pPackage->from);
		SendMessage(string);
	}

	switch (cmd.code) {
	case COMMAND_QUIT:
	case COMMAND_MAGTEST:
	case COMMAND_MAGSET:
	case COMMAND_HOVER:
	case COMMAND_FILTER:
	case COMMAND_TAKEOFF:
	case COMMAND_LAND:
	case COMMAND_ENGINEUP:
	case COMMAND_ENGINEDOWN:
	case COMMAND_EMERGENCY:
	case COMMAND_EMERGENCYGROUND:
	case COMMAND_NOTIFY:
	case COMMAND_NONOTIFY:
	case COMMAND_LEADERUPDATE:
	case COMMAND_COMPETITION:
		break;

	case COMMAND_LIFT:
	case COMMAND_DESCEND:
	case COMMAND_ENGINE:
		COPYDOUBLE(cmd.parameter, package+2);
		break;

	case COMMAND_HEADTO:
	case COMMAND_COORDINATE:
		::memcpy(cmd.parameter, package+2, 3*sizeof(double));
		break;

	case COMMAND_RUN:
	case COMMAND_PARA:
	case COMMAND_TEST:				//parameter is an integer
	case COMMAND_GPATH:
//		(int &)cmd.parameter[0] = (int &)package[2];
		COPYLONG(cmd.parameter, package+2);
		break;

	case COMMAND_FORMATION:		// parameter is the path number of the leader
//	case COMMAND_ATTACK:		// for later cooperative behavior usage
		(COOP_PKT &)cmd.parameter[0] = (COOP_PKT &)package[2]; //extract coop_pkt from CMM
		_net.ToggleNetStart();
		break;

	case COMMAND_STOPFORMATION:
		_coop.ResetConnectFlag();
		break;

	case COMMAND_PATH:
//		(int &)cmd.parameter[0] = (int &)package[2];				//path id
//		(int &)cmd.parameter[4] = (int &)package[6];			//tracking option
		COPYLONG(cmd.parameter, package+2);
		COPYLONG(cmd.parameter+4, package+6);
		break;

	case COMMAND_PATHA: {
		double nPoint;
//		COPYWORD(&nPoint, package+2);
		COPYDOUBLE(&nPoint, package+2);
		int npoint = (int)nPoint;
		LOCATION *pPoint = (LOCATION *)(package+10);

		::memcpy(cmd.parameter, package+2, sizeof(double));
		::memcpy(cmd.parameter+8, package+10, npoint*sizeof(LOCATION));

/*		//generate dynamic path
		LOCATION pos0;
		_state.GetCoordination(&pos0.longitude, &pos0.latitude, &pos0.altitude);
		_pathTmp.CreatePath(pPoint, nPoint, &_state.GetState(), &pos0);

		//set command to path tracking
		cmd.code = COMMAND_PATH;
		PUTLONG(cmd.parameter, -1);
		PUTLONG(cmd.parameter + 4, PATHTRACKING_FIXED);*/
		break;
	}

	case COMMAND_TRACK:
		::memcpy(cmd.parameter, package+2, 2*sizeof(int)+3*sizeof(double));
		break;

	case COMMAND_DYNAMICPATH:
	case COMMAND_DYNAMICPATHRESET:
		::memcpy(cmd.parameter, package+2, 5*sizeof(double));				//t,x,y,z,c
		break;

	case COMMAND_HFLY:				//parameter (u,v,h,r)
	case COMMAND_FLY:				//parameter (u,v,w,r)
	case COMMAND_HOLD:				//(x,y,z,c)
	case COMMAND_HOLDPI:
	case COMMAND_CFLY:				//(u,v,w,r)
	case COMMAND_TAKEOFFA:			//(x,y,z,c)
	case COMMAND_LANDA:				//(x,y,z,c)
		::memcpy(cmd.parameter, package+2, 4*sizeof(double));
		break;

	case COMMAND_CHIRP:
		::memcpy(cmd.parameter, package+2, sizeof(int)+4*sizeof(double));
		break;

	case COMMAND_PLAN:
//		(int &)cmd.parameter[0] = (int &)package[2];
		COPYLONG(cmd.parameter, package+2);
		printf("[CMM] Command plan, parameter %d\n", (int)GETLONG(package+2));
		break;

	case COMMAND_CAMRUN: break;
	case COMMAND_CAMSTOP: break;
	case COMMAND_CAMQUIT: break;
	case COMMAND_CAMTRACK:	break;

	case COMMAND_CAMVBS:
//		_state.EnableVision();
	case COMMAND_CAMTHOLD:
	    ::memcpy(cmd.parameter, package+2, sizeof(double));
		break;

	case COMMAND_COOP_PKT:	// coop pkts, futher decoding will be used in the clsCoop
		::memcpy(cmd.parameter, package+2, sizeof(COOP_PKT));
		break;

	case COMMAND_MODE:
		COPYLONG(cmd.parameter, package+2);
		printf("[CMM] Command mode, parameter %d\n", (int)GETLONG(package+2));
		break;

	case DATA_VICON: {
		SetViconFlag();
		_state.SetbMeasurementUpdate();
//		AccountCMMRecord(nFrom);
		VICON_DATA viconData;
		int nObjLen = GETLONG(package+12);
//		viconData.x = GETDOUBLE(package+16+nObjLen); viconData.y = GETDOUBLE(package+16+nObjLen+8); viconData.z = GETDOUBLE(package+16+nObjLen+16);
//		viconData.a = GETDOUBLE(package+16+nObjLen+24); viconData.b = GETDOUBLE(package+16+nObjLen+32); viconData.c = GETDOUBLE(package+16+nObjLen+40);
		memcpy(&viconData, package+16+nObjLen, sizeof(VICON_DATA));
		CollectViconData(viconData);
		return;
		break;
	}
	default:
		break;				//for unknown command only code is transfered
	}

	PutCommand(&cmd);
}

BOOL clsCMM::InitSocket()
{
	m_socket = socket(AF_INET, SOCK_DGRAM, 0);
	if ( m_socket == -1 ) {
		printf("[CMM] Error in creating socket.\n");
		return FALSE;
	}

	BOOL bBroadcast = TRUE;
	::setsockopt(m_socket, SOL_SOCKET, SO_BROADCAST, &bBroadcast, sizeof(BOOL));				//for broadcasting

	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
//	in_addr inaddr;
//	inet_aton("192.168.2.3", &inaddr);
//	addr.sin_addr.s_addr = inaddr.s_addr;

	addr.sin_port = htons(NETPORT_BROADCAST);
	memset( addr.sin_zero, '\0', sizeof(addr.sin_zero) );

	if ( bind(m_socket, (sockaddr *)&addr, sizeof(addr)) == -1 ) {
		printf("[CMM] Bind error\n");
		return FALSE;
	}

	char fileName[] = "quadlion.txt";
	if ( !GetGCSIPAddr(fileName, m_gcsIPAddr) ) {
		printf("[CMM] GCS IP address cannot be found!\n");
		return FALSE;
	}

	//init buffer
	m_nBufferNet = 0;

/*	char ipaddr[] = "192.168.10.1";
	SetTcpConfig(ipaddr, sizeof(ipaddr), NETPORT_BROADCAST, 2, 1);*/

	return TRUE;
}

BOOL clsCMM::GetGCSIPAddr(char *fileName, char *gcsIPAddr)
{
	FILE *pFile = fopen(fileName, "r");
	if (pFile == NULL) {
		printf("GCS IP File cannot be found!\n");
		return FALSE;
	}

	char linebuf[128];
	while ( fgets(linebuf, 128, pFile) != NULL ) {
		if (strstr(linebuf, "GCS_IP_ADDR = ") != NULL) {
//			fputs(linebuf, stdout);
			if (sscanf(linebuf, "GCS_IP_ADDR = %s", gcsIPAddr) == 1)
				printf("GCS IP addr is %s\n", gcsIPAddr);
		}
	}
	fclose(pFile);

	return TRUE;
}
void clsCMM::CollectViconData(VICON_DATA viconData)
{
	m_viconData.x = viconData.x/1000.0; m_viconData.y = viconData.y/1000.0; m_viconData.z = viconData.z/1000.0;
	m_viconData.a = viconData.a; m_viconData.b = viconData.b; m_viconData.c = viconData.c;
	//printf("Vicon data: x %.3f, y %.3f, z %.3f, a %.3f, b %.3f, c %.3f \n",
	//		m_viconData.x, m_viconData.y, m_viconData.z, m_viconData.a, m_viconData.b, m_viconData.c);
}

void clsCMM::SetCmmConfig(char *devPort, short size, short flag, int baudrate)
{
	memcpy(m_cmmConfig.devPort, devPort, size);
	m_cmmConfig.flag = flag;
	m_cmmConfig.baudrate = baudrate;
}

void clsCMM::SetTcpConfig(char *ipaddr, short size, int portNum, short protocol, short flag)
{
	memcpy(m_tcpConfig.ipaddress, ipaddr, size);
	m_tcpConfig.portNum = portNum;
	m_tcpConfig.protocol = protocol;
	m_tcpConfig.flag = flag;
}
