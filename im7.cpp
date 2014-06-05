//im7.cpp
//implementation file for reading data from IMU - NAV440

#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

#include "uav.h"
#include "im7.h"
#include "state.h"

extern clsState _state;
/*
 * NAV440
 */
clsIM7::clsIM7()
{
	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutex_init(&m_mtxIM7, &attr);

	m_xIM7 = 0.01639;
	m_yIM7 = -0.001933;
	m_zIM7 = 0.1299;

	m_xAntenna = -1.2;
	m_yAntenna = 0;
	m_zAntenna = -0.13;
}

clsIM7::~clsIM7()
{
	pthread_mutex_destroy(&m_mtxIM7);
}

BOOL clsIM7::InitThread()
{
    //initial setting
	FD_ZERO(&rfd);
	m_timeOut.tv_sec = 0;
	m_timeOut.tv_usec = IMU_TIMEOUT_READ;

    m_nsIM7= ::open("/dev/ser1", O_RDWR /*| O_NONBLOCK*/ );
	if (m_nsIM7 == -1) { printf("[IM7] Open IMU serial port (/dev/ser1) failed!\n"); return FALSE; }

    // Setup the COM port
    termios term;
    tcgetattr(m_nsIM7, &term);

    cfsetispeed(&term, IM7_BAUDRATE);				//input and output baudrate
    cfsetospeed(&term, IM7_BAUDRATE);

    term.c_cflag = CS8 | CLOCAL | CREAD;				//communication flags
//    term.c_iflag &= ~IXOFF;
//    term.c_iflag = /*IGNBRK | IGNCR |*/ IGNPAR;

	tcsetattr(m_nsIM7, TCSANOW, &term);

	tcflush(m_nsIM7, TCIOFLUSH);
//	IM7RAWPACK packraw[MAX_IM7PACK];

#ifdef IM7_MODE_POLL
	SetRate(0);
	usleep(15000);
//	SetType('A');
//	usleep(15000);
//	Poll('N');
	Poll('A');	// previous poll N

	usleep(15000);
//	GetPack(packraw, MAX_IM6PACK);				//filter the first package which is often invalid

//	Poll('A');	// previous poll N
//	usleep(10000);
#else
	Poll('N');
	usleep(15000);
//	SetType('A');
#endif
	SetType('N');
//	SetType('A');
	SetRate(100);

	//initialize variables
	m_tRetrieve = m_tRequest = -1;

	m_nIM7 = 0;
	m_nLost = 0;

	m_tGP7 = -1;
	m_tAH7 = -1;
	m_tSC7 = -1;

	m_nBuffer = 0;

	m_bit = 0;

    printf("[IM7] Start\n");		// after filtering the 1st packet, then start real imu data capture
	return TRUE;
}

int clsIM7::EveryRun()
{
	IM7RAWPACK packraw[MAX_IM7PACK];
	int nGet = GetPack(packraw, MAX_IM7PACK);	// # of imu packets get, 'N' or 'A' or 'S'

	IM7RAWPACK *pGP7 = NULL;
	IM7RAWPACK *pAH7 = NULL;
	IM7RAWPACK *pSC7 = NULL;

	for (int i=0; i<=nGet-1; i++)
	{
		IM7RAWPACK &raw = packraw[i];
//		if ( raw.gp7.type == 0x304e )	// N0
		if ( raw.gp7.type == 0x314e )	// N1
			pGP7 = &raw;
		else if ( raw.gp7.type == 0x3041 )
			pAH7 = &raw;
		else
			pSC7 = &raw;
	}

	//calculate the most approximate time of the pack
	m_tRetrieve = m_tRequest + 0.005;	// ???

	double tGP7, tAH7, tSC7;

#ifdef IM6_MODE_POLL
	tGP7 = tAH7 = tSC7 = m_tRetrieve;
#else
	tGP7 = tAH7 = tSC7 = GetTime();
#endif


#if (_DEBUG & DEBUGFLAG_IM7)
if (m_nCount % _DEBUG_COUNT == 0) {
//	printf("[IM7] pGPS %d, pAH7 %d, pSC7 %d\n", (int)pGP7, (int)pAH7, (int)pSC7);
}
#endif

	pthread_mutex_lock(&m_mtxIM7);				//begin to storage data

	//retrieve the newest pack of SC7, AH7 and GP7, and update
	if (pSC7 != NULL)
		PushPack(tSC7, pSC7);
	if (pAH7 != NULL)
		PushPack(tAH7, pAH7);
	if (pGP7 != NULL)
	{
		PushPack(tGP7, pGP7);
	}

	pthread_mutex_unlock(&m_mtxIM7);

//#ifdef IM7_MODE_POLL
/*	char type = m_nCount%COUNT_GP7 == 0 ? 'N': 'A';
	Poll(type);*/
//	Poll('A');
//#else
//	if (m_nCount%COUNT_GP7 == 0) Poll('N');
//#endif

//	char sentBuf2[] = {0x55,0x55,0x47,0x50,0x02,0x4e,0x30,0x84,0xb9}; // get 4e30(N0) 440 packet
//	int nWrite = ::write(m_nsIM6, sentBuf2, 9);
	return TRUE;
}

void clsIM7::ExitThread()
{
	::close(m_nsIM7);
	printf("[IM7] quit\n");
}

BOOL clsIM7::GetLastPack(int nType, double* pTime, IM7PACK* pPack)
{
	if (nType == DATA_GP7) {
		if (m_tGP7 < 0) return FALSE;
		else {
			*pTime = m_tGP7;
			*pPack = m_gp7;
			return TRUE;
		}
	}

	if (nType == DATA_AH7) {
		if (m_tAH7 < 0) return FALSE;
		else {
			*pTime = m_tAH7;
			*pPack = m_ah7;
			return TRUE;
		}
	}

	if (nType == DATA_SC7) {
		if (m_tSC7 < 0) return FALSE;
		else {
			*pTime = m_tSC7;
			*pPack = m_sc7;
			return TRUE;
		}
	}
	return FALSE;
}

BOOL clsIM7::GetLastPack(int nType, double *pTime, IM7RAWPACK *pRaw)
{
	if (nType == DATA_GP7RAW) {
		if (m_tGP7 < 0) return FALSE;
		else {
			*pTime = m_tGP7;
			*pRaw = m_gp7Raw;
			return TRUE;
		}
	}

	if (nType == DATA_AH7RAW) {
		if (m_tAH7 < 0) return FALSE;
		else {
			*pTime = m_tAH7;
			*pRaw = m_ah7Raw;
			return TRUE;
		}
	}

	if (nType == DATA_SC7RAW) {
		if (m_tSC7 < 0) return FALSE;
		else {
			*pTime = m_tSC7;
			*pRaw = m_sc7Raw;
			return TRUE;
		}
	}
	return FALSE;
}
void clsIM7::SetType(char chType)
{
//	char bufGP6[] = {0x55,0x55,0x53,0x46,0x01,0x00,0x03,0x00,0x41,0x00,0xDE};
//	char bufAH6[] = {0x55,0x55,0x53,0x46,0x01,0x00,0x03,0x00,0x41,0x00,0xDE};
//	char bufSC6[] = {0x55,0x55,0x53,0x46,0x01,0x00,0x03,0x00,0x41,0x00,0xDE};

//	char bufGP7[] = {0x55,0x55,0x53,0x46,0x05,0x01,0x00,0x03,0x4E,0x30,0x36,0x71};	// N0
	char bufGP7[] = {0x55,0x55,0x53,0x46,0x05,0x01,0x00,0x03,0x4E,0x31,0x26,0x50};	// N1
	char bufAH7[] = {0x55,0x55,0x53,0x46,0x05,0x01,0x00,0x03,0x41,0x30,0x26,0x4F};	// A0
	char bufSC7[] = {0x55,0x55,0x53,0x46,0x05,0x01,0x00,0x03,0x53,0x30,0x43,0x5E};	// S0

	char *pBuffer = NULL;
	if (chType == 'N') pBuffer = bufGP7;
	else if (chType == 'A') pBuffer = bufAH7;
	else pBuffer = bufSC7;

	::write(m_nsIM7, pBuffer, 12);
}

void clsIM7::SetRate(int nRate)
{
	// NAV420 continuous mode configuration
/*	char buf0Hz[]	= {0x55,0x55,0x53,0x46,0x01,0x00,0x01,0x00,0x00,0x00,0x9B};
	char buf100Hz[]	= {0x55,0x55,0x53,0x46,0x01,0x00,0x01,0x00,0x01,0x00,0x9C};
	char buf50Hz[]	= {0x55,0x55,0x53,0x46,0x01,0x00,0x01,0x00,0x02,0x00,0x9D};
	char buf25Hz[]	= {0x55,0x55,0x53,0x46,0x01,0x00,0x01,0x00,0x04,0x00,0x9F};
	char buf20Hz[]	= {0x55,0x55,0x53,0x46,0x01,0x00,0x01,0x00,0x05,0x00,0xA0};*/

	// NAV440 continuous mode configuration
	char buf0Hz[] 	= {0x55,0x55,0x53,0x46,0x05,0x01,0x00,0x01,0x00,0x00,0x40,0x81};
	char buf100Hz[] = {0x55,0x55,0x53,0x46,0x05,0x01,0x00,0x01,0x00,0x01,0x50,0xA0};
	char buf50Hz[]	= {0x55,0x55,0x53,0x46,0x05,0x01,0x00,0x01,0x00,0x02,0x60,0xC3};

	char *pBuffer = NULL;

	if (nRate == 0) pBuffer = buf0Hz;
	else if (nRate == 100) pBuffer = buf100Hz;
	else if (nRate == 50) pBuffer = buf50Hz;
//	else if (nRate == 25) pBuffer = buf25Hz;
//	else if (nRate == 20) pBuffer = buf20Hz;
	else return;

	write(m_nsIM7, pBuffer, 12);
}

void clsIM7::Poll(char chType)
{
//	char bufGP6[] = {0x55,0x55,0x47,0x50,0x4E,0x00,0xE5};	// previous NAV420
//	char bufGP7[] = {0x55,0x55,0x47,0x50,0x02,0x4e,0x30,0x84,0xb9}; 	// N0
	char bufGP7[] = {0x55,0x55,0x47,0x50,0x02,0x4e,0x31,0x94,0x98}; 	// N1
	char bufAH7[] = {0x55,0x55,0x47,0x50,0x02,0x41,0x30,0x94,0x87};		// A0
	char bufSC7[] = {0x55,0x55,0x47,0x50,0x02,0x53,0x30,0xf1,0x96};		// S0

	char *pBuffer = NULL;
	if (chType == 'N')
		pBuffer = bufGP7;
	else if (chType == 'A')
		pBuffer = bufAH7;
	else
		pBuffer = bufSC7;

	m_tRequest = GetTime();
}

void clsIM7::PushPack(double tPack, IM7RAWPACK *pPackRaw)
{
	IM7PACK pack;
	Translate(pPackRaw, &pack);

#if (_DEBUG & DEBUGFLAG_IM7)
if (m_nCount % _DEBUG_COUNT == 0) {
	GP7PACK &gp7 = pack.gp7;
	AH7PACK &ah7 = pack.ah7;
	SC7PACK &sc7 = pack.sc7;

	printf("[IM7] PushPack, time %.3f, pack %c\n", tPack, pPackRaw->gp7.type);

	switch (pack.gp7.type) {
	case DATA_GP7:
		printf("[IM7] GP7 data, longtitude %.3f, latitude %.3f, altitude %.3f\n",
			gp7.longitude, gp7.latitude, gp7.altitude);
		break;
	case DATA_AH7:
		printf("[IM7] AH7 data, roll angle %.3f, pitch angle %.3f, heading angle %.3f\n",
			ah7.a, ah7.b, ah7.c);
		break;
	case DATA_SC7:
		printf("[IM7] SC7 data, acx %.3f, acy %.3f, acz %.3f\n",
			sc7.acx, sc7.acy, sc7.acz);
		break;
	}
}
#endif

	switch (pack.gp7.type) {
	case DATA_GP7:
		m_tGP7 = tPack;
		m_gp7Raw = *pPackRaw;
		m_gp7 = pack;
		break;
	case DATA_AH7:
		m_tAH7 = tPack;
		m_ah7Raw = *pPackRaw;
		m_ah7 = pack;
		break;
	case DATA_SC7:
		m_tSC7 = tPack;
		m_sc7Raw = *pPackRaw;
		m_sc7 = pack;
		break;
	}

	if (_state.GetSimulationType() == 0)
		_state.Update(tPack, &pack);

	pthread_mutex_lock(&m_mtxIM7);
	if (m_nIM7 != MAX_IM7PACK) {
		m_tIM7[m_nIM7] = tPack;
		m_im7Raw[m_nIM7] = *pPackRaw;
		m_im7[m_nIM7++] = pack;
	}
	pthread_mutex_unlock(&m_mtxIM7);
}

int clsIM7::GetPack(IM7RAWPACK raw[], int nMaxPack)
{
	int nRead = read(m_nsIM7, m_szBuffer+m_nBuffer, MAX_IM7BUFFER /*74*/);		//get data as many as it can
//	printf("Byte read: %d\n", nRead);
	if (nRead > 0)
		m_nBuffer += nRead;

#if (_DEBUG & DEBUGFLAG_IM7)
if (m_nCount % _DEBUG_COUNT == 0)
{
		printf("[IM6] IM7GetPack, bytes read %d, new length %d\n", nRead, m_nBuffer);
		for (int i=0; i<=m_nBuffer-1; i++)
		{
			printf("%02x ", (unsigned char)m_szBuffer[i]);
		}
		printf("\n");
}
#endif

	int nPack = 0;
	int iNext = -1;
	int nSize = -1;
	for (int i = 0; i <= m_nBuffer /*nRead*/-1; )
	{
		if (m_szBuffer[i] != 0x55)
			{ i++;
			continue; }

		if (i == m_nBuffer /*nRead*/-1)
			{ iNext = i;
			break; }

		if (m_szBuffer[i+1] != 0x55)
			{ i += 2;
			continue; }

		if (i == m_nBuffer /*nRead*/-2)
			{ iNext = i;
			break; }

		char chPackType1 = m_szBuffer[i+2];
		char chPackType2 = m_szBuffer[i+3];
		if ( (chPackType1==0x53 || chPackType1==0x41) && (chPackType2==0x30) )
			nSize = 37;

/*		else if ( (chPackType1==0x4e) && (chPackType2==0x30) )	// N0
			nSize = 39;*/

		else if ( (chPackType1==0x4e) && (chPackType2==0x31) )	// N1
			nSize = 49;
		
		else if ( (chPackType1 == 0x15) && (chPackType2 == 0x15) ) {
//			cout<<"NAK packet."<<endl;
		}
		else
			{
//			i += 4;
			i++;
			continue;
			}

		if (i+nSize > m_nBuffer /*nRead*/)
			{ iNext = i;
			break; }

		unsigned short crcCalc = CheckCRC(m_szBuffer+i+2, nSize-4);
		unsigned short crcCheck = m_szBuffer[i+nSize-2] << 8 | (unsigned char)m_szBuffer[i+nSize-1];

		if (crcCalc != crcCheck)
		{
			i += 4;
//			cout<<"checksum error"<<endl;
			continue;
		}
		ExtractPackFromBuffer(m_szBuffer+i, raw+nPack);
		if (++nPack == nMaxPack) {
			iNext = i + nSize;
			if (iNext > m_nBuffer/*nRead*/ - 1)
				iNext = -1;
			break;
		}

		i += nSize;			// now i indicates the next imu packet
	}	//end of for loop

	if (nPack==0)
	{
		m_nLost++;
//		char str[20] = {0};
//		sprintf(str, "Lost packet %d\n", m_nLost);
//		_cmm.PutMessage(str);
//
//		printf("[IM6] IM6GetPack, bytes read %d, new length %d\n", nRead, m_nBuffer);
//		for (int i=0; i<=m_nBuffer /*nRead*/-1; i++)
//		{
//			printf("%02x ", (unsigned char)m_szBuffer[i]);
//		}
//		printf("\n");
//		cout<<"0 pkts"<<endl;
	}

	if (iNext != -1) {
		m_nBuffer -= iNext;
		memcpy(m_szBuffer, m_szBuffer+iNext, m_nBuffer);
	}
	else
		m_nBuffer = 0;

#if (_DEBUG & DEBUGFLAG_IM7)
if (m_nCount % _DEBUG_COUNT == 0)
{
		printf("[IM7] IM7GetPack, buffer left %d\n", m_nBuffer);
		for (int i=0; i<=m_nBuffer-1; i++)
		{
			printf("%02x ", (unsigned char)m_szBuffer[i]);
		}
		printf("\n");
}
#endif


	return nPack;
}

void clsIM7::ExtractPackFromBuffer(char* pBuffer, IM7RAWPACK* pPack)
{
	int nSize = 0;
	char *p = pBuffer;
	char ch;

	switch (pBuffer[2]) {
	case 'N':
/*		nSize = 39;
		for (int i = 5; i <= 21; i += 2)
			{ ch = p[i]; p[i]= p[i+1]; p[i+1] = ch; }

		ch = p[23];p[23]=p[26]; p[26]= ch;		/// GPS byte order rotation
		ch = p[24];p[24]=p[25]; p[25]= ch;

		ch = p[27];p[27]=p[30]; p[30]= ch;
		ch = p[28];p[28]=p[29]; p[29]= ch;

	//go on two_byte change
		for (int i = 31; i <= 35; i += 2)
			{ ch = p[i]; p[i]= p[i+1]; p[i+1] = ch; }*/
	
		nSize = 49;
		for (int i = 5; i <= 27; i += 2)
			{ ch = p[i]; p[i]= p[i+1]; p[i+1] = ch; }

		ch = p[29];p[29]=p[32]; p[32]= ch;		/// GPS byte order rotation
		ch = p[30];p[30]=p[31]; p[31]= ch;

		ch = p[33];p[33]=p[36]; p[36]= ch;
		ch = p[34];p[34]=p[35]; p[35]= ch;

	//go on two_byte change
		for (int i = 37; i <= 41; i += 2)
			{ ch = p[i]; p[i]= p[i+1]; p[i+1] = ch; }
		break;

	case 'A':
	case 'S':
		nSize = 37;
		for (int i=5; i<=33; i+=2)
			{ ch=p[i]; p[i]= p[i+1]; p[i+1]=ch; }
		break;
	}

	memcpy(pPack, pBuffer, nSize);		/// store big-endian? raw packet
}

void clsIM7::Translate(IM7RAWPACK *pPackRaw, IM7PACK *pPack)
{
	unsigned short b16 = 0x8000;		// 2^15
	unsigned long b32 = 0x80000000;		// 2^31

	char type = pPackRaw->gp7.type;

	switch (type) {
	case 'N': {				//GP7 data
		GP7RAWPACK &gp7raw = pPackRaw->gp7;
		GP7PACK &gp7 = pPack->gp7;

		gp7.type = DATA_GP7;
		gp7.a = (double)gp7raw.a*PI/b16;
		gp7.b = (double)gp7raw.b*PI/b16;
		gp7.c = (double)gp7raw.c*PI/b16;
		
		gp7.p = (double)gp7raw.p*(630/180)*PI/b16;
		gp7.q = (double)gp7raw.q*(630/180)*PI/b16;
		gp7.r = (double)gp7raw.r*(630/180)*PI/b16;

		gp7.acx = (double)gp7raw.acx*10/b16*_gravity;
		gp7.acy = (double)gp7raw.acy*10/b16*_gravity;
		gp7.acz = (double)gp7raw.acz*10/b16*_gravity;
		
		gp7.u = (double)gp7raw.u*256/b16;
		gp7.v = (double)gp7raw.v*256/b16;
		gp7.w = (double)gp7raw.w*256/b16;
		gp7.w /= 2;				//according to the change of altitude measurement

		gp7.longitude = (double)gp7raw.longitude*PI/b32;
		gp7.latitude = (double)gp7raw.latitude*PI/b32;
		gp7.altitude = (double)gp7raw.altitude*8192/b16;

		break;
	}
	case 'A': {
		AH7RAWPACK &ah7raw = pPackRaw->ah7;
		AH7PACK &ah7 = pPack->ah7;

		ah7.type = DATA_AH7;

		ah7.a = (double)ah7raw.a*PI/b16;
		ah7.b = (double)ah7raw.b*PI/b16;
		ah7.c = (double)ah7raw.c*PI/b16;

		ah7.p = (double)ah7raw.p*PI*(630/180)/b16;
		ah7.q = (double)ah7raw.q*PI*(630/180)/b16;
		ah7.r = (double)ah7raw.r*PI*(630/180)/b16;

		ah7.acx = (double)ah7raw.acx*10/b16*_gravity;
		ah7.acy = (double)ah7raw.acy*10/b16*_gravity;
		ah7.acz = (double)ah7raw.acz*10/b16*_gravity;

		ah7.magx = (double)ah7raw.magx/b16;
		ah7.magy = (double)ah7raw.magy/b16;
		ah7.magz = (double)ah7raw.magz/b16;

		ah7.tmp = (double)ah7raw.tmp*100/b16;

		break;
	}
	case 'S':
	default: {
		SC7RAWPACK &sc7raw = pPackRaw->sc7;
		SC7PACK &sc7 = pPack->sc7;

		sc7.type = DATA_SC7;

		sc7.acx = (double)sc7raw.acx*10/b16*_gravity;
		sc7.acy = (double)sc7raw.acy*10/b16*_gravity;
		sc7.acz = (double)sc7raw.acz*10/b16*_gravity;

		sc7.p = (double)sc7raw.p*PI*(630/180)/b16;
		sc7.q = (double)sc7raw.q*PI*(630/180)/b16;
		sc7.r = (double)sc7raw.r*PI*(630/180)/b16;

		sc7.magx = (double)sc7raw.magx/b16;
		sc7.magy = (double)sc7raw.magy/b16;
		sc7.magz = (double)sc7raw.magz/b16;

		sc7.tmpx = (double)sc7raw.tmpx*100/b16;
		sc7.tmpy = (double)sc7raw.tmpy*100/b16;
		sc7.tmpz = (double)sc7raw.tmpz*100/b16;

		sc7.tmpBoard = (double)sc7raw.tmpBoard*100/b16;

		break;
	}
	}
};
