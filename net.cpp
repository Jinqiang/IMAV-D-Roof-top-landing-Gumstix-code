/*
 * net.cpp
 *
 *  Created on: Mar 15, 2011
 */
#include <stdio.h>
#include <pthread.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream.h>

#include "uav.h"
#include "net.h"
#include "coop.h"
#include "state.h"

extern clsState _state;

clsNET::clsNET()
{
	m_idListenThread = 0;

	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutex_init(&m_mtxClient, &attr);
}
clsNET::~clsNET()
{
	pthread_mutex_destroy(&m_mtxClient);
}

void clsNET::MakeTelegraph(TELEGRAPH *pTele, short code, double time, void *pData, int nDataSize)
{
	//telegraph format
	// from, to, size, (COMMAND_DATA, datatype, time, data), checksum				//bracketed is the package
	// 1      1    2        2            2       8     var    2
	char *pBuffer = pTele->content;

//	(double &)pBuffer[6] = time;
	pBuffer[0] = _HELICOPTER;				//_nHelicopter 6 or 7, 0x56 for helion, 0x57 for shelion
	pBuffer[1] = 5;				//ground station

//	(short &)pBuffer[2] = nDataSize+12;
//	(short &)pBuffer[4] = COMMAND_DATA;
//	(short &)pBuffer[6] = code;				//code - data type

	PUTWORD(pBuffer+2, nDataSize+12);
	PUTWORD(pBuffer+4, COMMAND_DATA);
	PUTWORD(pBuffer+6, code);

//	double* pTime = &time;
	(double &)pBuffer[8] = time;
	::memcpy(pBuffer+16, pData, nDataSize);

	unsigned short sum = CheckSum(pBuffer+4, 12+nDataSize);
	PUTWORD(pBuffer+16+nDataSize, sum);

	pTele->size = nDataSize + 18;
}

BOOL clsNET::InitThread()
{
	PEERADDRESS peerAddr;
	GetHostAddr(_HELICOPTER, peerAddr);
	printf("[NET] start, ip:%s\n", peerAddr.ip);
	return TRUE;
}

BOOL clsNET::Init()
{
	m_clientSocket = socket(AF_INET, SOCK_DGRAM, 0);
	if (m_clientSocket == -1) {
		printf("[NET] Error in creating a client socket.\n");
		return FALSE;
	}

	m_socket = socket(AF_INET, SOCK_DGRAM, 0);
	if ( m_socket == -1 ) {
		printf("[NET] Error in creating a server socket.\n");
		return FALSE;
	}

/*	BOOL bBroadcast = TRUE;
	::setsockopt(m_socket, SOL_SOCKET, SO_BROADCAST, &bBroadcast, sizeof(BOOL));				//for broadcasting
*/
	//	gethostname(hostname, HOSTLEN);		/* where am I? */
	//	ServerMakeInternetAddr(hostname, PORTNUM, &saddr);

	PEERADDRESS peerAddr;
	GetHostAddr(_HELICOPTER, peerAddr);
	m_addr.sin_family = AF_INET;
	m_addr.sin_port = htons(NETPORT_BROADCAST);
//	m_addr.sin_port = htons(peerAddr.rxport);
	in_addr inaddr;
	inet_aton(peerAddr.ip, &inaddr);
//	m_addr.sin_addr.s_addr = INADDR_ANY;
	m_addr.sin_addr.s_addr = inaddr.s_addr;
//	clientSockaddr.sin_addr.s_addr = inet_aton( PEER_HOST2, &(clientSockaddr.sin_addr) );
	memset( m_addr.sin_zero, '\0', sizeof(m_addr.sin_zero) );

	if ( bind(m_socket, (struct sockaddr *)&m_addr, sizeof(m_addr)) == -1 )	{
		perror("[NET] bind");
		return FALSE;
	}

	m_addr.sin_family = AF_INET;
//	m_addr.sin_port = htons(/*NET_PORT*/ PEER_HOST1_PORT_REQUEST);
	m_addr.sin_port = htons(peerAddr.txport);
	m_addr.sin_addr.s_addr = INADDR_ANY;
//	clientSockaddr.sin_addr.s_addr = inet_aton( PEER_HOST2, &(clientSockaddr.sin_addr) );
	memset( m_addr.sin_zero, '\0', sizeof(m_addr.sin_zero) );
	if ( bind(m_clientSocket, (struct sockaddr *)&m_addr, sizeof(m_addr)) == -1 )	{
		perror("[NET] bind");
		return FALSE;
	}

	return TRUE;
}

int clsNET::EveryRun()
{
	if (_coop.GetRole() == FOLLOWER && _coop.GetCoopTime()>0 ) {
//		if ( !CheckNetStart() ) return TRUE;
//		if ( 0 != m_nCount % 5  /*COUNT_CLIENT*/  ) return TRUE;	// follower sends pkt spontaneously

//		COOP_PKT coopedPkt = _coop.FetchCoopedPkt();
	//	TELEGRAPH tele;
	//	MakeTelegraph(&tele, DATA_STATE, ::GetTime(), &coopedPkt, sizeof(COOP_PKT));
	//	ClientSendTo( PEER_HOST1, PEER_HOST1_PORT_LISTEN, /*m_sendBuf*/ tele.szTele, tele.nSize);

//		ClientSendTo( PEER_HOST2, PEER_HOST2_PORT_LISTEN, (char *)&coopedPkt, sizeof(COOP_PKT));
		return TRUE;
	}

	else if (_coop.GetRole() == LEADER && _coop.GetCoopTime()>0 && _coop.GetConnectFlag()) {
//		if ( !CheckNetStart() ) return TRUE; // no use of NET sending

		if ( 0 != m_nCount % 5 /*COUNT_CLIENT*/ ) return TRUE;

			UAVSTATE& state = _state.GetState();
//			RPTSTATE& RPTState = _state.GetRPTState();

			double t = ::GetTime();
			double ldStatus[11] = {/*tPath*/ t, state.x, state.y, state.z, state.c, state.ug, state.vg, state.wg, state.p, state.q, state.r};
//			double flRef[4];
//			int cmdPara4Fl[4] = {0};
//			double actionPara4Fl[9] = {flRef[0], flRef[1], flRef[2], flRef[3], 0, 0, 0, 0, 0};
		//	MakeCoopPkt(LEADERFORMATION_UPDATE, cmdPara4Fl, actionPara4Fl);
			_coop.MakeCoopPkt(LEADERFORMATION_UPDATE, NULL, ldStatus);

			COOP_PKT coopedPkt = _coop.FetchCoopedPkt();
			PEERADDRESS peerAddr;
			_net.FindDestPeers(_HELICOPTER, peerAddr);
			_net.ClientSendTo(peerAddr.ip /*PEER_HOST2*/, peerAddr.rxport /*PEER_HOST2_PORT_LISTEN*/, (char *)&coopedPkt, sizeof(COOP_PKT));
	//		cout<<"TX "<<coopedPkt.npkt<<" "<</*coopedPkt.actionParas[0]*/ ::GetTime() <<endl;
			return TRUE;
	}
	return TRUE;
}

void clsNET::ExitThread()
{
	printf("[NET] quit\n");
}

void clsNET::FindDestPeers(int nfrom, PEERADDRESS& peerAddr)
{
	if (nfrom == 6) {	// HeLion
		peerAddr.ip = (char *)PEER_HOST2;
//		::memcpy(toip, PEER_HOST2, sizeof(PEER_HOST2));
		peerAddr.rxport = PEER_HOST_PORT_LISTEN;
	}
	else if (nfrom == 7) {	// from SheLIon
		peerAddr.ip = (char *)PEER_HOST1;
//		::memcpy(toip, PEER_HOST1, sizeof(PEER_HOST1));
		peerAddr.rxport = PEER_HOST_PORT_LISTEN;
	}
}

void clsNET::GetHostAddr(int nfrom, PEERADDRESS& peerAddr)
{
	if (nfrom == 6) {
		peerAddr.ip = (char *)PEER_HOST1;
		peerAddr.rxport = PEER_HOST_PORT_LISTEN;
		peerAddr.txport = PEER_HOST_PORT_REQUEST;
	}
	else if (nfrom == 7) {
		peerAddr.ip = (char *)PEER_HOST2;
		peerAddr.rxport = PEER_HOST_PORT_LISTEN;
		peerAddr.txport = PEER_HOST_PORT_REQUEST;
	}
	else if (nfrom == 11) {				//gumstix2
		peerAddr.ip = (char *)PEER_HOST11;
		peerAddr.rxport = PEER_HOST_PORT_LISTEN;
		peerAddr.txport = PEER_HOST_PORT_REQUEST;
	}
}

int clsNET::ClientSendTo(const char *hostIP, int toPort, char *sendBuf, int nBuffer)
{
	int sentBytes;
	struct sockaddr_in toSockaddr;

//	memset( &toSockaddr, 0, sizeof(toSockaddr) );
	toSockaddr.sin_family = AF_INET;
	toSockaddr.sin_port = htons(toPort);

		toSockaddr.sin_addr.s_addr = ::inet_addr(/*"172.20.73.102"*/ hostIP);
		memset(toSockaddr.sin_zero, '\0', sizeof(toSockaddr.sin_zero) );

		sentBytes = sendto( /*m_socket*/ m_clientSocket, (void *)sendBuf, nBuffer/*sizeof(UAVSTATE)*/, 0, \
							(struct sockaddr *)&toSockaddr, sizeof(toSockaddr) );

		if ( sentBytes == -1 ) {
			perror("[NET] sendto");
			return FALSE;
		}

		if (sentBytes == sizeof(COOP_PKT)) {
			COOP_PKT coopedPkt = _coop.FetchCoopedPkt();
//			cout<<"TX "<<coopedPkt.npkt<<" "<</*coopedPkt.actionParas[0]*/ ::GetTime() <<endl;
		}
//	inet_aton( hostIP, &(toSockaddr.sin_addr) );
//	toSockaddr.sin_addr.s_addr = ::inet_addr("255.255.255.255");

	return TRUE;
}

BOOL clsNET::StartListenThread(int priority)
{
    pthread_attr_t attribute;
    pthread_attr_init(&attribute);
    pthread_attr_setdetachstate(&attribute, PTHREAD_CREATE_DETACHED);
    pthread_attr_setinheritsched(&attribute, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attribute, SCHED_RR);

//    attribute.__param.__sched_priority = priority;
//    attribute.param.sched_priority = priority;

    sched_param_t param;
    pthread_attr_getschedparam(&attribute, &param);
    param.sched_priority = priority;
    pthread_attr_setschedparam(&attribute, &param);

    pthread_create(&m_idListenThread, &attribute, &clsNET::ListenThread, this);

    return TRUE;
}

void *clsNET::ListenThread(void *pParameter)
{
	clsNET *pNET = (clsNET *)pParameter;
	pNET->Listen();
	return NULL;
}

void clsNET::Listen()
{
	/*
	m_socket = ::socket(AF_INET, SOCK_DGRAM, 0);

	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons(NET_PORT);

	if (::bind(m_socket, (sockaddr *)&addr, sizeof(addr)) == -1) return;
*///already initialized in clsNET::Init();

	//begin receive data
	struct sockaddr_in from;
	socklen_t fromlen;

	ADDRESSEDPACKAGE package;

	while (1) {
		int nRecv = ::recvfrom(m_socket, m_buffer, MAXSIZE_TELEGRAPH-m_nBuffer, 0, (struct sockaddr *)&from, &fromlen);
		if (nRecv == -1) {
			printf("[NET] socket disconnected.\n");
			break;
		}
		printf("[NET] %d bytes received from %s\n", nRecv, inet_ntoa(from.sin_addr));
		m_nBuffer += nRecv;

		if (ParseBuffer(m_buffer, m_nBuffer, &package))	{
			ProcessPackage(&package);		//analysis buffer to extrace telegraph package from it
		}
		//so far only process packages from ground station
	}
}

BOOL clsNET::ParseBuffer(char *pBuffer, int nBuffer, ADDRESSEDPACKAGE *pPackage)
{
	//ParseBuffer analysis if there is a package in buffer,
	//if yes, extract it to pPackage and reset the pBuffer and nBuffer and return true,
	//otherwise do nothing and return false

	char *pChar = pBuffer;
	char *pCharMax = pBuffer + nBuffer - 1;

	enum { BEGIN, HEADER, SIZE, PACKAGE, CHECK } state = BEGIN;

	char from = _HELICOPTER, to = 0;
	short size;

	BOOL bPackage = FALSE;					//determine if there is a package in buffer
	char *pNextPackage = NULL;

	while (pChar <= pCharMax) {

		if (state == BEGIN) {
			if (*pChar >= 0 && *pChar <= 99) {
				from = pChar[0];
				state = HEADER;				//from
			}
			pChar++; continue;
		}

		if (state == HEADER) {
			if (*pChar >= 0 && *pChar <= 99) {
				to = pChar[0];
				state = SIZE;
			}
			else state = BEGIN;

			pChar ++; continue;
		}

		if (state == SIZE) {
			if (pChar + 1 > pCharMax) break;

//			pCharSize = pChar;				//record the position of pointer to size
//			size = (short &)pChar[0];
			size = GETWORD(pChar);
			if (size < 1 || size > MAXSIZE_PACKAGE) { state = BEGIN; continue; }
			else { state = PACKAGE; pChar += 2; continue; }
		}

		if (state == PACKAGE) {
			if (pChar + size + 1 > pCharMax) break;

			unsigned short sum = CheckSum(pChar, size);
//			unsigned short check = *(unsigned short *)(pChar+size);
			unsigned short check;
			::memcpy(&check, pChar+size, 2);

			if (sum != check) { state = BEGIN; continue; }

			pPackage->from = from;
			pPackage->to = to;
			pPackage->package.size = size;
			::memcpy(pPackage->package.content, pChar, size);

			bPackage = TRUE;
			pNextPackage = pChar + size + 2;
			break;
		}
	}

	if (pChar > pCharMax) nBuffer = 0;
	if (!bPackage) return FALSE;

	if (pNextPackage > pCharMax) nBuffer = 0;
	else {
		nBuffer = pCharMax - pNextPackage + 1;
		::memcpy(pBuffer, pNextPackage, nBuffer);
	}

	return TRUE;
}

void clsNET::ProcessPackage(ADDRESSEDPACKAGE *pPackage)
{
//	short from = pPackage->from;
//	if (from != ID_STATION) return;				//so far only process packages from ground station

	COMMAND cmd;
	::memcpy(&cmd, pPackage->package.content, pPackage->package.size);
//	char *para = cmd.parameter;

//	_ctl.PutCommand(&cmd);
}
