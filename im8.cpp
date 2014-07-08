//im8.cpp
//implementation file for reading data from IMU - IG500N

#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <iostream.h>

#include "uav.h"
#include "im8.h"
#include "state.h"

extern clsState _state;
/*
 * IG500N
 */
clsIM8::clsIM8()
{
	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutex_init(&m_mtxIM8, &attr);

//	m_xIM8 = 0.01639;
//	m_yIM8 = -0.001933;
//	m_zIM8 = 0.1299;

	m_xIM8 = 0;
	m_yIM8 = 0;
	m_zIM8 = 0;

	m_xAntenna = 0; //-1.2;
	m_yAntenna = 0;
	m_zAntenna = 0; //-0.13;
}

clsIM8::~clsIM8()
{
	pthread_mutex_destroy(&m_mtxIM8);
}

BOOL clsIM8::InitThread()
{
    //initial setting
	FD_ZERO(&rfd);
	m_timeOut.tv_sec = 0;
	m_timeOut.tv_usec = IMU_TIMEOUT_READ;

    m_nsIM8= ::open("/dev/ser3", O_RDWR /*| O_NONBLOCK*/ );
	if (m_nsIM8 == -1) {
			printf("[IM8] Open IMU serial port (/dev/ser3) failed!\n");
			return FALSE;
	}

    // Setup the COM port
    termios term;
    tcgetattr(m_nsIM8, &term);

    cfsetispeed(&term, IM8_BAUDRATE);				//input and output baudrate
    cfsetospeed(&term, IM8_BAUDRATE);

    term.c_cflag = CS8 | CLOCAL | CREAD;				//communication flags
//    term.c_iflag &= ~IXOFF;
//    term.c_iflag = /*IGNBRK | IGNCR |*/ IGNPAR;

	tcsetattr(m_nsIM8, TCSANOW, &term);

	tcflush(m_nsIM8, TCIOFLUSH);
//	IM8RAWPACK packraw[MAX_IM8PACK];

/*	Poll('N');
	usleep(15000);

	SetType('N');
	SetRate(100);*/

	//initialize variables
	m_tRetrieve = m_tRequest = -1;

	m_nIM8 = 0;
	m_nLost = 0;

	m_tGP8 = -1;
	m_tAH8 = -1;
	m_tSC8 = -1;

	m_nBuffer = 0;

	m_bit = 0;

	SetIMUConfig(IMU_IG500N, TRUE, IM8_BAUDRATE);
    printf("[IM8] Start\n");		// after filtering the 1st packet, then start real imu data capture
	return TRUE;
}

void clsIM8::SetIMUConfig(short imuID, short flag, int baudrate)
{
	m_imuConfig.imuID = imuID;
	m_imuConfig.flag = flag;
	m_imuConfig.baudrate = baudrate;
}

int clsIM8::EveryRun()
{
	int nMode = _state.GetSimulationType();
	if (nMode == SIMULATIONTYPE_MODEL)
		return TRUE;	//return if in simulation mode

	IM8RAWPACK packraw[MAX_IM8PACK];
	GetPack(packraw, MAX_IM8PACK);
	Translate();

//	if ( m_nCount % 50 == 0 ) {
//		cout<<"size of float "<<sizeof(float)<<endl;
//		cout<<"a "<<m_gp8.gp8.a*180/PI<<" b "<<m_gp8.gp8.b*180/PI<<" c "<<m_gp8.gp8.c*180/PI<<endl;
//		cout<<"p "<<m_gp8.gp8.p*180/PI<<" q "<<m_gp8.gp8.q*180/PI<<" r "<<m_gp8.gp8.r*180/PI<<endl;
//		cout<<"acx "<<m_gp8.gp8.acx<<" acy "<<m_gp8.gp8.acy<<" acz "<<m_gp8.gp8.acz<<endl;
//
//		cout<<"Number of satallites: "<<(int)(m_gp8.gp8.nGPS)<<endl;
//		cout<<"GPS fixed: "<<(m_gp8.gp8.gpsinfo & 0x03)<<endl;
//	}

	double tPack = ::GetTime();
	_state.Update(tPack, /*&pack*/ &m_gp8);

	pthread_mutex_lock(&m_mtxIM8);
	if (m_nIM8 != MAX_IM8PACK) {
		m_tIM8[m_nIM8] = tPack;
//		m_im8Raw[m_nIM8] = *pPackRaw;
		m_im8[m_nIM8++] = /*pack*/ m_gp8;
	}
	pthread_mutex_unlock(&m_mtxIM8);

	return TRUE;
}

void clsIM8::ExitThread()
{
	::close(m_nsIM8);
	printf("[IM8] quit\n");
}

BOOL clsIM8::GetLastPack(int nType, double* pTime, IM8PACK* pPack)
{
	if (nType == DATA_GP8) {
		if (m_tGP8 < 0) return FALSE;
		else {
			*pTime = m_tGP8;
			*pPack = m_gp8;
			return TRUE;
		}
	}

/*	if (nType == DATA_AH8) {
		if (m_tAH8 < 0) return FALSE;
		else {
			*pTime = m_tAH8;
			*pPack = m_ah8;
			return TRUE;
		}
	}*/

	if (nType == DATA_SC8) {
		if (m_tSC8 < 0) return FALSE;
		else {
			*pTime = m_tSC8;
			*pPack = m_sc8;
			return TRUE;
		}
	}
	return FALSE;
}

BOOL clsIM8::GetLastPack(int nType, double *pTime, IM8RAWPACK *pRaw)
{
	if (nType == DATA_GP8RAW) {
		if (m_tGP8 < 0) return FALSE;
		else {
			*pTime = m_tGP8;
			*pRaw = m_gp8Raw;
			return TRUE;
		}
	}

/*	if (nType == DATA_AH8RAW) {
		if (m_tAH8 < 0) return FALSE;
		else {
			*pTime = m_tAH8;
			*pRaw = m_ah8Raw;
			return TRUE;
		}
	}*/

	if (nType == DATA_SC8RAW) {
		if (m_tSC8 < 0) return FALSE;
		else {
			*pTime = m_tSC8;
			*pRaw = m_sc8Raw;
			return TRUE;
		}
	}
	return FALSE;
}

void clsIM8::SetType(char chType)
{
//	char bufGP6[] = {0x55,0x55,0x53,0x46,0x01,0x00,0x03,0x00,0x41,0x00,0xDE};
//	char bufAH6[] = {0x55,0x55,0x53,0x46,0x01,0x00,0x03,0x00,0x41,0x00,0xDE};
//	char bufSC6[] = {0x55,0x55,0x53,0x46,0x01,0x00,0x03,0x00,0x41,0x00,0xDE};

//	char bufGP8[] = {0x55,0x55,0x53,0x46,0x05,0x01,0x00,0x03,0x4E,0x30,0x36,0x71};	// N0
	char bufGP8[] = {0x55,0x55,0x53,0x46,0x05,0x01,0x00,0x03,0x4E,0x31,0x26,0x50};	// N1
	char bufAH8[] = {0x55,0x55,0x53,0x46,0x05,0x01,0x00,0x03,0x41,0x30,0x26,0x4F};	// A0
	char bufSC8[] = {0x55,0x55,0x53,0x46,0x05,0x01,0x00,0x03,0x53,0x30,0x43,0x5E};	// S0

	char *pBuffer = NULL;
	if (chType == 'N') pBuffer = bufGP8;
	else if (chType == 'A') pBuffer = bufAH8;
	else pBuffer = bufSC8;

	::write(m_nsIM8, pBuffer, 12);
}

void clsIM8::SetRate(int nRate)
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

	write(m_nsIM8, pBuffer, 12);
}

void clsIM8::Poll(char chType)
{
//	char bufGP6[] = {0x55,0x55,0x47,0x50,0x4E,0x00,0xE5};	// previous NAV420
//	char bufGP8[] = {0x55,0x55,0x47,0x50,0x02,0x4e,0x30,0x84,0xb9}; 	// N0
	char bufGP8[] = {0x55,0x55,0x47,0x50,0x02,0x4e,0x31,0x94,0x98}; 	// N1
	char bufAH8[] = {0x55,0x55,0x47,0x50,0x02,0x41,0x30,0x94,0x87};		// A0
	char bufSC8[] = {0x55,0x55,0x47,0x50,0x02,0x53,0x30,0xf1,0x96};		// S0

	char *pBuffer = NULL;
	if (chType == 'N')
		pBuffer = bufGP8;
	else if (chType == 'A')
		pBuffer = bufAH8;
	else
		pBuffer = bufSC8;

	m_tRequest = GetTime();
}

void clsIM8::PushPack(double tPack, IM8RAWPACK *pPackRaw)
{
	IM8PACK pack;
	Translate(pPackRaw, &pack);

//#if (_DEBUG & DEBUGFLAG_IM8)
if (m_nCount % _DEBUG_COUNT == 0) {
	GP8PACK &gp8 = pack.gp8;
//	AH8PACK &ah8 = pack.ah8;
//	SC8PACK &sc8 = pack.sc8;

//	printf("[IM8] PushPack, time %.3f, pack %c\n", tPack, pPackRaw->gp8.type);

	switch (pack.gp8.type) {
	case DATA_GP8:
		printf("[IM8] GP8 data, roll angle %.3f, pitch angle %.3f, heading angle %.3f\n",
			gp8.a, gp8.b, gp8.c);
//		printf("[IM8] GP8 data, longtitude %.3f, latitude %.3f, altitude %.3f\n",
//			gp8.longitude, gp8.latitude, gp8.altitude);
		break;
/*	case DATA_AH8:
		printf("[IM8] AH8 data, roll angle %.3f, pitch angle %.3f, heading angle %.3f\n",
			ah8.a, ah8.b, ah8.c);
		break;
	case DATA_SC8:
		printf("[IM8] SC8 data, acx %.3f, acy %.3f, acz %.3f\n",
			sc8.acx, sc8.acy, sc8.acz);
		break; */
	}
}
//#endif

	switch (pack.gp8.type) {
	case DATA_GP8:
		m_tGP8 = tPack;
		m_gp8Raw = *pPackRaw;
		m_gp8 = pack;
		break;
/*	case DATA_AH8:
		m_tAH8 = tPack;
		m_ah8Raw = *pPackRaw;
		m_ah8 = pack;
		break;*/
	case DATA_SC8:
		m_tSC8 = tPack;
		m_sc8Raw = *pPackRaw;
		m_sc8 = pack;
		break;
	}

	if (_state.GetSimulationType() == 0)
		_state.Update(tPack, &pack);

	pthread_mutex_lock(&m_mtxIM8);
	if (m_nIM8 != MAX_IM8PACK) {
		m_tIM8[m_nIM8] = tPack;
		m_im8Raw[m_nIM8] = *pPackRaw;
		m_im8[m_nIM8++] = pack;
	}
	pthread_mutex_unlock(&m_mtxIM8);
}

int clsIM8::GetPack(IM8RAWPACK raw[], int nMaxPack)
{
	int nRead = read(m_nsIM8, m_szBuffer+m_nBuffer, MAX_IM8BUFFER /*74*/);		//get data as many as it can
//	printf("Byte read: %d\n", nRead);
	if (nRead > 0)
		m_nBuffer += nRead;

#if (_DEBUG & DEBUGFLAG_IM8)
if (m_nCount % _DEBUG_COUNT == 0)
{
		printf("[IM6] IM8GetPack, bytes read %d, new length %d\n", nRead, m_nBuffer);
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
		if ( (unsigned char)m_szBuffer[i] != 0xff ) {
			i++;
			continue;
		}
		if (i == m_nBuffer-1) {
			iNext = i;
			break;
		}

		if (m_szBuffer[i+1] != 0x02) {
			i += 2;
			continue;
		}
		if (i == m_nBuffer /*nRead*/-2) {
			iNext = i;
			break;
		}

		if ( (unsigned char)m_szBuffer[i+2] != 0x90 ) {
			i += 3;
			continue;
		}
		if (i == m_nBuffer /*nRead*/-3)	{
			iNext = i;
			break;
		}

		nSize = (unsigned short)((unsigned char)m_szBuffer[i+3] | (unsigned char)m_szBuffer[i+4]);
//		cout<<"packet size: "<<nSize<<endl;

		if (i+nSize > m_nBuffer) {
			iNext = i;
			break;
		}

		if ( (unsigned char)m_szBuffer[i+nSize+7] != 0x03 ) {		// end of pkt: 0x03
			i += 4;
			continue;
		}

//		unsigned short crcCalc = CheckCRC(m_szBuffer+i+2, nSize-4);
//		unsigned short crcCheck = m_szBuffer[i+nSize-2] << 8 | (unsigned char)m_szBuffer[i+nSize-1];
		unsigned short crcCalc = CheckCRC1((const unsigned char *)(m_szBuffer+i+2), nSize+3);
		unsigned short crcCheck = (unsigned short)((unsigned char)m_szBuffer[i+nSize+5] << 8 | (unsigned char)m_szBuffer[i+nSize+6]);

		if (crcCalc != crcCheck) {
			i += 4;
//			cout<<"checksum error"<<endl;
			continue;
		}
/*		else {
			cout<<"checksum correct"<<endl;
		}*/

//		ExtractPackFromBuffer(m_szBuffer+i+5, raw+nPack, nSize);	// only copy data payload
		memcpy(m_szDataBuf, m_szBuffer+i+5, nSize);
		if (++nPack == nMaxPack) {
			cout<<"nMaxPack reached"<<endl;
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
		//cout<<"0 pkts"<<endl;
	}

	if (iNext != -1) {
		m_nBuffer -= iNext;
//		printf("memory copy\n");
//		cout<<"iNext: "<<iNext<<" m_nBuffer "<<m_nBuffer<<endl;
		memcpy(m_szBuffer, m_szBuffer+iNext, m_nBuffer);
	}
	else {	// no data left in the buffer to process
		m_nBuffer = 0;
//		printf("No data left\n");
	}


#if (_DEBUG & DEBUGFLAG_IM8)
if (m_nCount % _DEBUG_COUNT == 0)
{
		printf("[IM8] IM8GetPack, buffer left %d\n", m_nBuffer);
		for (int i=0; i<=m_nBuffer-1; i++)
		{
			printf("%02x ", (unsigned char)m_szBuffer[i]);
		}
		printf("\n");
}
#endif


	return nPack;
}

void clsIM8::ExtractPackFromBuffer(char* pBuffer, IM8RAWPACK* pPack, int nSize)
{
	int n = nSize;
	char *p = pBuffer;

/*	for (int i=0; i<n; i++) {
		printf("IM8RAWPACK: %02x ", pPack[i]);
	}
	printf("\n");*/
}

void clsIM8::Translate()
{
	m_gp8.gp8.type = DATA_GP8;
	m_gp8.gp8.a = (double)(*(float *)m_szDataBuf);
	m_gp8.gp8.b = (double)(*(float *)(m_szDataBuf+4));
	m_gp8.gp8.c = (double)(*(float *)(m_szDataBuf+8));

	m_gp8.gp8.p = (double)(*(float *)(m_szDataBuf+12));
	m_gp8.gp8.q = (double)(*(float *)(m_szDataBuf+16));
	m_gp8.gp8.r = (double)(*(float *)(m_szDataBuf+20));

	m_gp8.gp8.acx = (double)(*(float *)(m_szDataBuf+24));
	m_gp8.gp8.acy = (double)(*(float *)(m_szDataBuf+28));
	m_gp8.gp8.acz = (double)(*(float *)(m_szDataBuf+32));

//	m_gp8.gp8.gpstime = *(uint32_t *)(m_szDataBuf+36);
	m_gp8.gp8.gpstime = GETLONG(m_szDataBuf+36);
	m_gp8.gp8.gpsinfo = *(uint8_t *)(m_szDataBuf+40);
	m_gp8.gp8.nGPS = *(uint8_t *)(m_szDataBuf+41);

	m_gp8.gp8.pressure = GETLONG(m_szDataBuf + 42);

	m_gp8.gp8.latitude = GETDOUBLE(m_szDataBuf+46) * PI/180;
	m_gp8.gp8.longitude = GETDOUBLE(m_szDataBuf+54) * PI/180;
	m_gp8.gp8.altitude = GETDOUBLE(m_szDataBuf+62);

//	m_gp8.gp8.u = (double)(*(float *)(m_szDataBuf+60+6));
//	m_gp8.gp8.v = (double)(*(float *)(m_szDataBuf+64+6));
//	m_gp8.gp8.w = (double)(*(float *)(m_szDataBuf+68+6));

	m_gp8.gp8.u = (double)(GETFLOAT(m_szDataBuf+70));
	m_gp8.gp8.v = (double)(GETFLOAT(m_szDataBuf+74));
	m_gp8.gp8.w = (double)(GETFLOAT(m_szDataBuf+78));
}

void clsIM8::Translate(IM8RAWPACK *pPackRaw, IM8PACK *pPack)
{
	GP8RAWPACK &gp8raw = pPackRaw->gp8;
	GP8PACK &gp8 = pPack->gp8;

	gp8.type = DATA_GP8;
	gp8.a = (double)((float)gp8raw.a);
	gp8.b = (double)((float)gp8raw.b);
	gp8.c = (double)((float)gp8raw.c);

	gp8.p = (double)((float)gp8raw.p);
	gp8.q = (double)((float)gp8raw.q);
	gp8.r = (double)((float)gp8raw.r);

	gp8.acx = (double)((float)gp8raw.acx);
	gp8.acy = (double)((float)gp8raw.acy);
	gp8.acz = (double)((float)gp8raw.acz);

	gp8.u = (double)((float)gp8raw.u);
	gp8.v = (double)((float)gp8raw.v);
	gp8.w = (double)((float)gp8raw.w);

	gp8.longitude = (double)gp8raw.longitude;
	gp8.latitude = (double)gp8raw.latitude;
	gp8.altitude = (double)gp8raw.altitude;

};
