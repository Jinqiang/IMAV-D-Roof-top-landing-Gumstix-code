//im8.h
//this is the header file for function of IMU - IG500N

#ifndef IM8_H_
#define IM8_H_

#include <sys/select.h>

#include "thread.h"

//IM8
#define MAX_IM8PACK		128				//storage size for packages
//#define MAX_STATE		128				//storage size for uav state

#define IM8_BAUDRATE  /*57600*/ 115200

//#define IMU_EXTRAPOLATION
#define FLAG_GPSCHECK   0xC240

#define MAX_IM8BUFFER	4096				//this flag specify the size of buffer for receiving IM6 data

#if (_DEBUG && _DEBUG_RATE != 0)				//debug and slow
#define COUNT_GP8 		2				//gp6, ah6, sc6 in turn
#define RATIO_FILTER	0				//no filter
#else
#define COUNT_GP8 		10		// previous 10
#define RATIO_FILTER	0.636
#endif

//#define IM6_MODE_POLL
#define IMU_TIMEOUT_READ	200000

/*
 * IG500N packet format
 */
#pragma pack(push, 1)

struct IMU_CONFIG {
	short imuID;
	short flag;
	int baudrate;
};

struct SC8RAWPACK {
	int16_t  header;
	unsigned short type;
	unsigned char length;

	int16_t  acx;
	int16_t  acy;
	int16_t  acz;

    int16_t  p;
	int16_t  q;
	int16_t  r;

	int16_t  magx;
	int16_t  magy;
	int16_t  magz;

	int16_t  tmpx;
	int16_t  tmpy;
	int16_t  tmpz;

	int16_t  tmpBoard;
	unsigned short  itow;
	unsigned short  bit;
	unsigned short  crcCheck;
};

struct SC8PACK {
	unsigned char  type;

	double  acx;
	double  acy;
	double  acz;

    double  p;
	double  q;
	double  r;

	double  magx;
	double  magy;
	double  magz;

	double  tmpx;
	double  tmpy;
	double  tmpz;

	double  tmpBoard;
};

struct  AH8RAWPACK {
	int16_t  header;
	unsigned short  type;
	unsigned char length;

	int16_t  a;
	int16_t  b;
	int16_t  c;

	int16_t  p;
	int16_t  q;
	int16_t  r;

	int16_t  acx;
	int16_t  acy;
	int16_t  acz;

	int16_t  magx;
	int16_t  magy;
	int16_t  magz;

	int16_t  tmp;
	unsigned short  itow;
	unsigned short  bit;
	unsigned short  crcCheck;
};

struct  AH8PACK {
	unsigned char  type;

	double  a;
	double  b;
	double  c;

	double  p;
	double  q;
	double  r;

	double  acx;
	double  acy;
	double  acz;

	double  magx;
	double  magy;
	double  magz;

	double  tmp;
};
/*
struct GP8RAWPACK {
	int16_t  header;
	unsigned short type;
	unsigned char length;

	int16_t  a;
	int16_t  b;
	int16_t  c;

	int16_t  p;
	int16_t  q;
	int16_t  r;

	int16_t  u;
	int16_t  v;
	int16_t  w;

	int32_t  longitude;
	int32_t  latitude;
	int16_t  altitude;

	unsigned short  itow;
	unsigned short  bit;
	unsigned short  crcCheck;
};

struct GP8PACK {
	unsigned char type;

	double  a;
	double  b;
	double  c;

	double  p;
	double  q;
	double  r;

	double  u;
	double  v;
	double  w;

	double  longitude;
	double  latitude;
	double  altitude;
};*/

struct GP8RAWPACK {
//	int16_t  header;
//	unsigned short type;
//	unsigned char length;

	int32_t  a;
	int32_t  b;
	int32_t  c;

	int32_t  p;
	int32_t  q;
	int32_t  r;

	int32_t  acx;
	int32_t  acy;
	int32_t  acz;
	
	int32_t  u;
	int32_t  v;
	int32_t  w;

	int64_t  longitude;
	int64_t  latitude;
	int64_t  altitude;

//	int16_t  xtmp;
	
//	unsigned int  itow;
//	unsigned short  bit;
//	unsigned short  crcCheck;
};

struct GP8PACK {
	unsigned char type;

	double  a;
	double  b;
	double  c;

	double  p;
	double  q;
	double  r;

	double  acx;
	double  acy;
	double  acz;
	
	double  u;
	double  v;
	double  w;

	double  longitude;
	double  latitude;
	double  altitude;

	uint32_t gpstime;
	uint8_t gpsinfo;
	uint8_t nGPS;

	uint32_t pressure;
};

union IM8RAWPACK {
//	SC8RAWPACK		sc8;
//	AH8RAWPACK		ah8;
	GP8RAWPACK		gp8;
};

union IM8PACK {
//	SC8PACK		sc8;
//	AH8PACK		ah8;
	GP8PACK		gp8;
};

#pragma pack(pop)

/*
 * IG500N
 */
class clsIM8 : public clsThread {
public:
	clsIM8();
	virtual ~clsIM8();

protected:
	int m_nsIM8;
	fd_set rfd;		//read fd
	timeval m_timeOut;

protected:
	int m_nLost;	// # of lost packets
	//to store received packs
	double m_tIM8[MAX_IM8PACK];
	IM8RAWPACK m_im8Raw[MAX_IM8PACK];
	IM8PACK m_im8[MAX_IM8PACK];
	int m_nIM8;				//number of pack

	//to store last gp8 pack
	double m_tGP8;
	IM8RAWPACK m_gp8Raw;
	IM8PACK m_gp8;

	//to store last sc8 pack
	double m_tSC8;
	IM8RAWPACK m_sc8Raw;
	IM8PACK m_sc8;

	//to store last ah8 pack
	double m_tAH8;
	IM8RAWPACK m_ah8Raw;
	IM8PACK m_ah8;

	pthread_mutex_t m_mtxIM8;

	double m_tRequest;
	double m_tRetrieve;

	short m_bit;				//this word set after each read of a pack of data

	char m_szBuffer[MAX_IM8BUFFER];
	int m_nBuffer;
	char m_szDataBuf[1024];	// current IG500N data payload should not exceed 1024 Bytes

	double m_xIM8;
	double m_yIM8;
	double m_zIM8;				//position of IM8

	double m_xAntenna;
	double m_yAntenna;
	double m_zAntenna;

	IMU_CONFIG m_imuConfig;

public:
	void SetIMUConfig(short imuID, short flag, int baudrate);
	void SetType(char chType);
	void SetRate(int nRate);
	void SetBaudRate(int nBaudRate);

	void Poll(char chType);

	void WriteCommand(int nType);

	int GetPack(IM8RAWPACK raw[], int nMaxPack);

	void ExtractPackFromBuffer(char *pBuffer, IM8RAWPACK *pPack, int nSize);	/// put the data rcved in pBuffer to pPack

	void PushPack(double tPack, IM8RAWPACK *pPack);
	void Translate(IM8RAWPACK *pRaw, IM8PACK *pPack);
	void Translate();

	BOOL GetLastPack(int nType, double *pTime, IM8PACK *pPack);
	BOOL GetLastPack(int nType, double *pTime, IM8RAWPACK *pRaw);

	void GetIM8Position(double pos[3]) { pos[0] = m_xIM8; pos[1] = m_yIM8; pos[2] = m_zIM8; }
	void GetAntennaPosition(double pos[3]) { pos[0] = m_xAntenna; pos[1] = m_yAntenna; pos[2] = m_zAntenna; }
	IMU_CONFIG GetIMUConfig() const {return m_imuConfig;}

public:
	virtual BOOL InitThread();
	virtual int EveryRun();
	virtual void ExitThread();

	friend class clsDLG;
};


#endif				//IM8_H_

