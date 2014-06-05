//this file is to implement the function of IMU - NAV420
//created on 15 Mar 2011

#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>

#include "uav.h"
#include "im6.h"
#include "state.h"

extern clsState _state;

clsIM6::clsIM6()
{
	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutex_init(&m_mtxIM6, &attr);

	m_xIM6 = 0.01639;
	m_yIM6 = -0.001933;
	m_zIM6 = 0.1299;

//	m_xAntenna = -1.2;
	m_xAntenna = -0.725;
	m_yAntenna = 0;
	m_zAntenna = -0.13;
}

clsIM6::~clsIM6()
{
	pthread_mutex_destroy(&m_mtxIM6);
}

BOOL clsIM6::InitThread()
{
    //initial setting
    m_nsIM6= ::open("/dev/ser1", O_RDWR /* | O_NONBLOCK */);
	if (m_nsIM6 == -1) { printf("[IM6] Open IMU serial port (/dev/ser1) failed!\n"); return FALSE; }

    // Setup the COM port
    termios term;
    tcgetattr(m_nsIM6, &term);

    cfsetispeed(&term, IM6_BAUDRATE);				//input and output baudrate
    cfsetospeed(&term, IM6_BAUDRATE);

    term.c_cflag = CS8 | CLOCAL | CREAD;				//communication flags
//    term.c_cflag &= ~CSTOPB;
//  term.c_iflag = IGNBRK | IGNCR | IGNPAR;

	tcsetattr(m_nsIM6, TCSANOW, &term);

	IM6RAWPACK packraw[MAX_IM6PACK];

#ifdef IM6_MODE_POLL
	SetRate(0);
	usleep(15000);
//	SetType('A');
//	usleep(15000);
	Poll('N');
	usleep(15000);
	GetPack(packraw, MAX_IM6PACK);				//filter the first package which is often invalid
	Poll('N');
	usleep(10000);
#else
	SetRate(0);
	usleep(15000);
//	SetType('A');
//	usleep(15000);
	Poll('N');
	usleep(15000);
	GetPack(packraw, MAX_IM6PACK);				//filter the first package which is often invalid
	Poll('N');
	usleep(10000);
	// now start the continuous mode
//	SetType('A');
/*	SetType('N');
	SetRate(100);*/
#endif

	SetType('N');
	SetRate(100);

	//initialize variables
	m_tRetrieve = m_tRequest = -1;

	m_nIM6 = 0;
	m_nLost = 0;

	m_tGP6 = -1;
	m_tAH6 = -1;
	m_tSC6 = -1;

	m_nBuffer = 0;

	m_bit = 0;

	printf("[IM6] Start\n");

	return TRUE;
}

int clsIM6::EveryRun()
{
//	cout<<"[IM6] "<<m_nCount<<endl;
//	while(1)
//		;
//	usleep(500000);
//	usleep(500000);
//	usleep(500000);
//	usleep(500000);
	IM6RAWPACK packraw[MAX_IM6PACK];
//	cout<<::GetTime()<<" ";
	int nGet = GetPack(packraw, MAX_IM6PACK);
//	cout<<::GetTime()<<endl;

#if (_DEBUG & DEBUGFLAG_IM6)
if (m_nCount % _DEBUG_COUNT == 0) {
//	printf("[IM6] %d packs get\n", nGet);
}
#endif

	IM6RAWPACK *pGP6 = NULL;
	IM6RAWPACK *pAH6 = NULL;
	IM6RAWPACK *pSC6 = NULL;

	for (int i=0; i<=nGet-1; i++)		// select the last(most updated) packet to record
	{
		IM6RAWPACK &raw = packraw[i];
		if (raw.gp6.type == 'N') pGP6 = &raw;
		else if (raw.gp6.type == 'A') pAH6 = &raw;
		else pSC6 = &raw;
	}

	//calculate the most approximate time of the pack
//	m_tRetrieve = m_tRequest + 0.005;
//	m_tRetrieve = ::GetTime();
	double tGP6, tAH6, tSC6;

#ifdef IM6_MODE_POLL
	tGP6 = tAH6 = tSC6 = m_tRetrieve;
#else
	tGP6 = tAH6 = tSC6 = GetTime();
//	tGP6 = m_tRetrieve;
#endif

#ifdef IMU_EXTRAPOLATION
//	static unsigned int lostPkt = 0;
	IM6PACK A0Lost;
	if (nGet==0)	// no valid packets rcved in this time
	{
//		lostPkt++;
//		cout<<"Lost packets: "<<lostPkt<<endl;

		double timeLostpkt = ::GetTime();

		if(m_nIM6>2)
		{
			A0Lost.ah6.a = (m_im6[m_nIM6-2].ah6.a + m_im6[m_nIM6-1].ah6.a)/2;
			A0Lost.ah6.b = (m_im6[m_nIM6-2].ah6.b + m_im6[m_nIM6-1].ah6.b)/2;
			A0Lost.ah6.c = (m_im6[m_nIM6-2].ah6.c + m_im6[m_nIM6-1].ah6.c)/2;
			A0Lost.ah6.p = (m_im6[m_nIM6-2].ah6.p + m_im6[m_nIM6-1].ah6.p)/2;
			A0Lost.ah6.q = (m_im6[m_nIM6-2].ah6.q + m_im6[m_nIM6-1].ah6.q)/2;
			A0Lost.ah6.r = (m_im6[m_nIM6-2].ah6.r + m_im6[m_nIM6-1].ah6.r)/2;
			A0Lost.ah6.acx = (m_im6[m_nIM6-2].ah6.acx + m_im6[m_nIM6-1].ah6.acx)/2;
			A0Lost.ah6.acy = (m_im6[m_nIM6-2].ah6.acy + m_im6[m_nIM6-1].ah6.acy)/2;
			A0Lost.ah6.acz = (m_im6[m_nIM6-2].ah6.acz + m_im6[m_nIM6-1].ah6.acz)/2;
			A0Lost.ah6.magx = (m_im6[m_nIM6-2].ah6.magx + m_im6[m_nIM6-1].ah6.magx)/2;
			A0Lost.ah6.magy = (m_im6[m_nIM6-2].ah6.magy + m_im6[m_nIM6-1].ah6.magy)/2;
			A0Lost.ah6.magz = (m_im6[m_nIM6-2].ah6.magz + m_im6[m_nIM6-1].ah6.magz)/2;
		}
		else
			A0Lost = m_im6[m_nIM6];

		if (_state.GetSimulationType() == 0)
			_state.Update(timeLostpkt, &A0Lost);		// clsState's states follows the IMU's data

		pthread_mutex_lock(&m_mtxIM6);
		if (m_nIM6 != MAX_IM6PACK) {
			m_tIM6[m_nIM6] = timeLostpkt;
			m_im6Raw[m_nIM6] = m_im6Raw[0];
			m_im6[m_nIM6++] = A0Lost;
		}
		pthread_mutex_unlock(&m_mtxIM6);
	}
#endif

#if (_DEBUG & DEBUGFLAG_IM6)
if (m_nCount % _DEBUG_COUNT == 0) {
//	printf("[IM6] pGPS %d, pAH6 %d, pSC6 %d\n", (int)pGP6, (int)pAH6, (int)pSC6);
}
#endif

	pthread_mutex_lock(&m_mtxIM6);				//begin to storage data

	//retrieve the newest pack of SC6, AH6 and GP6, and update
	if (pSC6 != NULL) PushPack(tSC6, pSC6);
	if (pAH6 != NULL) PushPack(tAH6, pAH6);
	if (pGP6 != NULL) PushPack(tGP6, pGP6);

	pthread_mutex_unlock(&m_mtxIM6);
/*
#ifdef IM6_MODE_POLL
//	char type = m_nCount%COUNT_GP6 == 0 ? 'N': 'A';
//	Poll(type);
//	Poll('A');
	Poll('N');
#else
	if (m_nCount%COUNT_GP6 == 0) Poll('N');
#endif
*/
	return TRUE;
}

void clsIM6::ExitThread()
{
	::close(m_nsIM6);
	printf("[IM6] quit\n");
}

void clsIM6::SetType(char chType)
{
	char bufGP6[] = {0x55,0x55,0x53,0x46,0x01,0x00,0x03,0x00,0x4E,0x00,0xEB};
	char bufAH6[] = {0x55,0x55,0x53,0x46,0x01,0x00,0x03,0x00,0x41,0x00,0xDE};
	char bufSC6[] = {0x55,0x55,0x53,0x46,0x01,0x00,0x03,0x00,0x53,0x00,0xF0};

	char *pBuffer = NULL;
	if (chType == 'N') pBuffer = bufGP6;
	else if (chType == 'A') pBuffer = bufAH6;
	else pBuffer = bufSC6;

	::write(m_nsIM6, pBuffer, 11);
}

void clsIM6::SetRate(int nRate)
{
	char buf0Hz[]	= {0x55,0x55,0x53,0x46,0x01,0x00,0x01,0x00,0x00,0x00,0x9B};
	char buf100Hz[]	= {0x55,0x55,0x53,0x46,0x01,0x00,0x01,0x00,0x01,0x00,0x9C};
	char buf50Hz[]	= {0x55,0x55,0x53,0x46,0x01,0x00,0x01,0x00,0x02,0x00,0x9D};
	char buf25Hz[]	= {0x55,0x55,0x53,0x46,0x01,0x00,0x01,0x00,0x04,0x00,0x9F};
	char buf20Hz[]	= {0x55,0x55,0x53,0x46,0x01,0x00,0x01,0x00,0x05,0x00,0xA0};

	char *pBuffer = NULL;

	if (nRate == 0) pBuffer = buf0Hz;
	else if (nRate == 100) pBuffer = buf100Hz;
	else if (nRate == 50) pBuffer = buf50Hz;
	else if (nRate == 25) pBuffer = buf25Hz;
	else if (nRate == 20) pBuffer = buf20Hz;
	else return;

	write(m_nsIM6, pBuffer, 11);
}

void clsIM6::Poll(char chType)
{
    char bufGP6[] = {0x55,0x55,0x47,0x50,0x4E,0x00,0xE5};
	char bufAH6[] = {0x55,0x55,0x47,0x50,0x41,0x00,0xD8};
	char bufSC6[] = {0x55,0x55,0x47,0x50,0x53,0x00,0xEA};

	char *pBuffer = NULL;
	if (chType == 'N') pBuffer = bufGP6;
	else if (chType == 'A') pBuffer = bufAH6;
	else pBuffer = bufSC6;

    ::write(m_nsIM6, pBuffer, 7);
	m_tRequest = GetTime();
//	cout<<m_tRequest<<endl;
}

void clsIM6::PushPack(double tPack, IM6RAWPACK *pPackRaw)
{
	IM6PACK pack;
	Translate(pPackRaw, &pack);

#if (_DEBUG & DEBUGFLAG_IM6)
if (m_nCount % _DEBUG_COUNT == 0) {
	GP6PACK &gp6 = pack.gp6;
	AH6PACK &ah6 = pack.ah6;
	SC6PACK &sc6 = pack.sc6;

	printf("[IM6] PushPack, time %.3f, pack %c\n", tPack, pPackRaw->gp6.type);

	switch (pack.gp6.type) {
	case DATA_GP6:
		printf("[IM6] GP6 data, longtitude %.3f, latitude %.3f, altitude %.3f\n",
			gp6.longitude, gp6.latitude, gp6.altitude);
		break;
	case DATA_AH6:
		printf("[IM6] AH6 data, roll angle %.3f, pitch angle %.3f, heading angle %.3f\n",
			ah6.a, ah6.b, ah6.c);
		break;
	case DATA_SC6:
		printf("[IM6] SC6 data, acx %.3f, acy %.3f, acz %.3f\n",
			sc6.acx, sc6.acy, sc6.acz);
		break;
	}
}
#endif

	switch (pack.gp6.type) {
	case DATA_GP6:
		m_tGP6 = tPack;
		m_gp6Raw = *pPackRaw;
		m_gp6 = pack;
		break;
	case DATA_AH6:
		m_tAH6 = tPack;
		m_ah6Raw = *pPackRaw;
		m_ah6 = pack;
		break;
	case DATA_SC6:
		m_tSC6 = tPack;
		m_sc6Raw = *pPackRaw;
		m_sc6 = pack;
		break;
	}

	if (_state.GetSimulationType() == 0)
		_state.Update(tPack, &pack);		// clsState's states follows the IMU's data

	pthread_mutex_lock(&m_mtxIM6);
	if (m_nIM6 != MAX_IM6PACK) {
		m_tIM6[m_nIM6] = tPack;
		m_im6Raw[m_nIM6] = *pPackRaw;
		m_im6[m_nIM6++] = pack;
	}
	pthread_mutex_unlock(&m_mtxIM6);
}

int clsIM6::GetPack(IM6RAWPACK raw[], int nMaxPack)			//return the type of got pack
{
//	cout<<::GetTime()<<endl;
	int nRead = read(m_nsIM6, m_szBuffer+m_nBuffer, MAX_IM6BUFFER);				//get data as many as it can
//	int nRead = read(m_nsIM6, m_szBuffer+m_nBuffer, 50);				//get data as many as it can

	if (nRead > 0)
		m_nBuffer += nRead;

#if (_DEBUG & DEBUGFLAG_IM6)
if (m_nCount % _DEBUG_COUNT == 0)
	{
		printf("[IM6] IM6GetPack, bytes read %d, new length %d\n", nRead, m_nBuffer);

		for (int i=0; i<=m_nBuffer-1; i++) {
			printf("%02x ", (unsigned char)m_szBuffer[i]);
		}
		printf("\n");

}
#endif

	int nPack = 0;
	int iNext = -1;
	int nSize = -1;
	for (int i = 0; i <= m_nBuffer-1; )
	{
		if (m_szBuffer[i] != 0x55)
			{ i++; continue; }
		if (i == m_nBuffer-1)
			{ iNext = i; break; }
		if (m_szBuffer[i+1] != 0x55)
			{ i += 2; continue; }
		if (i == m_nBuffer-2)
			{ iNext = i; break; }

		char chPack = m_szBuffer[i+2];
		if (chPack == 'N') nSize = 37;
		else if (chPack == 'A' || chPack == 'S') nSize = 35;
		else
			{ i += 3; continue;
//				cout<<"Type not correct"<<endl;
			}

		if (i+nSize > m_nBuffer)
			{ iNext = i; break;
//				cout<<"Not complete packet"<<endl;
			}

		unsigned short nSum = 0;
		for (int j=2; j<=nSize-3; j++) nSum += (unsigned char)m_szBuffer[i+j];
		unsigned short nCheck = m_szBuffer[i+nSize-2] << 8 | (unsigned char)m_szBuffer[i+nSize-1];
//		unsigned short nCheck = (unsigned short)(m_szBuffer[i+nSize-2] << 8 | m_szBuffer[i+nSize-1]);

#if (_DEBUG & DEBUGFLAG_IM6)
if (m_nCount % _DEBUG_COUNT == 0) {
		printf("[IM6] IM6GetPack, pack type '%c', check ok, sum %d, check %d\n", chPack, nCheck, nSum);
}
#endif

		if (nSum != nCheck)
		{
			i += 3;
/*			printf("[IM6] IM6GetPack, bytes read %d, new length %d\n", nRead, m_nBuffer);
			for (int i=0; i<=m_nBuffer-1; i++) {
				printf("%02x ", (unsigned char)m_szBuffer[i]);
			}
			printf("\n");*/
//			cout<<"checksum error"<<endl;
			continue;
		}

		ExtractPackFromBuffer(m_szBuffer+i, raw+nPack);
		if (++nPack == nMaxPack) {
			iNext = i + nSize;
			if (iNext > m_nBuffer - 1) iNext = -1;
			break;
		}

		i += nSize;
	}

	if (nPack == 0)
	{
		m_nLost++;
//		char str[20] = {0};
//		sprintf(str, "Lost packet %d\n", m_nLost);
//		_cmm.PutMessage(str);

/*		printf("[IM6] IM6GetPack, bytes read %d, new length %d\n", nRead, m_nBuffer);
		for (int i=0; i<=m_nBuffer-1; i++) {
			printf("%02x ", (unsigned char)m_szBuffer[i]);
		}
		printf("\n");*/
	}

	if (iNext != -1) {
		m_nBuffer -= iNext;
		memcpy(m_szBuffer, m_szBuffer+iNext, m_nBuffer);
	}
	else m_nBuffer = 0;

#if (_DEBUG & DEBUGFLAG_IM6)
if (m_nCount % _DEBUG_COUNT == 0) {
/*		printf("[IM6] IM6GetPack, buffer left %d\n", m_nBuffer);

		for (int i=0; i<=m_nBuffer-1; i++) {
			printf("%02x ", (unsigned char)m_szBuffer[i]);
		}
		printf("\n");
		*/
}
#endif

	return nPack;
}

void clsIM6::ExtractPackFromBuffer(char *pBuffer, IM6RAWPACK *pPack)
{
	int nSize = 0;
	char *p = pBuffer;
	char ch;

	switch (pBuffer[2]) {
	case 'N':
		nSize = 37;
		for (int i = 3; i <= 19; i += 2)
			{ ch = p[i]; p[i]= p[i+1]; p[i+1] = ch; }

		ch = p[21];p[21]=p[24]; p[24]= ch;
		ch = p[22];p[22]=p[23]; p[23]= ch;

		ch = p[25];p[25]=p[28]; p[28]= ch;
		ch = p[26];p[26]=p[27]; p[27]= ch;

	//go on two_byte change
		for (int i = 29; i <= 35; i += 2)
			{ ch = p[i]; p[i]= p[i+1]; p[i+1] = ch; }
		break;

	case 'A':
	case 'S':
		nSize = 35;
		for (int i=3; i<=33; i+=2)
			{ ch=p[i]; p[i]= p[i+1]; p[i+1]=ch; }
		break;
	}

	memcpy(pPack, pBuffer, nSize);
}

void clsIM6::Translate(IM6RAWPACK *pPackRaw, IM6PACK *pPack)
{
	unsigned short b16 = 0x8000;
	unsigned long b32 = 0x80000000;

	char type = pPackRaw->gp6.type;

	switch (type) {
	case 'N': {				//GP6 data
		GP6RAWPACK &gp6raw = pPackRaw->gp6;
		GP6PACK &gp6 = pPack->gp6;

		gp6.type = DATA_GP6;
		gp6.a = (double)gp6raw.a*PI/b16;
		gp6.b = (double)gp6raw.b*PI/b16;
		gp6.c = (double)gp6raw.c*PI/b16;

		gp6.p = (double)gp6raw.p*PI*(630/180)/b16;
		gp6.q = (double)gp6raw.q*PI*(630/180)/b16;
		gp6.r = (double)gp6raw.r*PI*(630/180)/b16;

		gp6.u = (double)gp6raw.u*256/b16;
		gp6.v = (double)gp6raw.v*256/b16;
		gp6.w = (double)gp6raw.w*256/b16;
		gp6.w /= 2;				//according to the change of altitude measurement

		gp6.longitude = (double)gp6raw.longitude*PI/b32;
		gp6.latitude = (double)gp6raw.latitude*PI/b32;
		gp6.altitude = (double)gp6raw.altitude*8192/b16;

		BOOL gpsStatus;
		if (gp6raw.bit & GPSSTATUS_UNLOCK)
			{
				gpsStatus = FALSE;
//				printf("GPS unlocked.\n");
			}
		else
			gpsStatus = TRUE;
		break;
	}
	case 'A': {
		AH6RAWPACK &ah6raw = pPackRaw->ah6;
		AH6PACK &ah6 = pPack->ah6;

		ah6.type = DATA_AH6;

		ah6.a = (double)ah6raw.a*PI/b16;
		ah6.b = (double)ah6raw.b*PI/b16;
		ah6.c = (double)ah6raw.c*PI/b16;

		ah6.p = (double)ah6raw.p*PI*(630/180)/b16;
		ah6.q = (double)ah6raw.q*PI*(630/180)/b16;
		ah6.r = (double)ah6raw.r*PI*(630/180)/b16;

		ah6.acx = (double)ah6raw.acx*10/b16*_gravity;
		ah6.acy = (double)ah6raw.acy*10/b16*_gravity;
		ah6.acz = (double)ah6raw.acz*10/b16*_gravity;

		ah6.magx = (double)ah6raw.magx/b16;
		ah6.magy = (double)ah6raw.magy/b16;
		ah6.magz = (double)ah6raw.magz/b16;

		ah6.tmp = (double)ah6raw.tmp*100/b16;

		break;
	}
	case 'S':
	default: {
		SC6RAWPACK &sc6raw = pPackRaw->sc6;
		SC6PACK &sc6 = pPack->sc6;

		sc6.type = DATA_SC6;

		sc6.acx = (double)sc6raw.acx*10/b16*_gravity;
		sc6.acy = (double)sc6raw.acy*10/b16*_gravity;
		sc6.acz = (double)sc6raw.acz*10/b16*_gravity;

		sc6.p = (double)sc6raw.p*PI*(630/180)/b16;
		sc6.q = (double)sc6raw.q*PI*(630/180)/b16;
		sc6.r = (double)sc6raw.r*PI*(630/180)/b16;

		sc6.magx = (double)sc6raw.magx/b16;
		sc6.magy = (double)sc6raw.magy/b16;
		sc6.magz = (double)sc6raw.magz/b16;

		sc6.tmpx = (double)sc6raw.tmpx*100/b16;
		sc6.tmpy = (double)sc6raw.tmpy*100/b16;
		sc6.tmpz = (double)sc6raw.tmpz*100/b16;

		sc6.tmpBoard = (double)sc6raw.tmpBoard*100/b16;

		break;
	}
	}
}

BOOL clsIM6::GetLastPack(int nType, double *pTime, IM6PACK *pPack)
{
	if (nType == DATA_GP6) {
		if (m_tGP6 < 0) return FALSE;
		else {
			*pTime = m_tGP6;
			*pPack = m_gp6;
			return TRUE;
		}
	}

	if (nType == DATA_AH6) {
		if (m_tAH6 < 0) return FALSE;
		else {
			*pTime = m_tAH6;
			*pPack = m_ah6;
			return TRUE;
		}
	}

	if (nType == DATA_SC6) {
		if (m_tSC6 < 0) return FALSE;
		else {
			*pTime = m_tSC6;
			*pPack = m_sc6;
			return TRUE;
		}
	}

	return FALSE;
}

BOOL clsIM6::GetLastPack(int nType, double *pTime, IM6RAWPACK *pRaw)
{
	if (nType == DATA_GP6RAW) {
		if (m_tGP6 < 0) return FALSE;
		else {
			*pTime = m_tGP6;
			*pRaw = m_gp6Raw;
			return TRUE;
		}
	}

	if (nType == DATA_AH6RAW) {
		if (m_tAH6 < 0) return FALSE;
		else {
			*pTime = m_tAH6;
			*pRaw = m_ah6Raw;
			return TRUE;
		}
	}

	if (nType == DATA_SC6RAW) {
		if (m_tSC6 < 0) return FALSE;
		else {
			*pTime = m_tSC6;
			*pRaw = m_sc6Raw;
			return TRUE;
		}
	}

	return FALSE;
}
