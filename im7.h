//im7.h
//this is the header file for function of IMU - NAV440

#ifndef IM7_H_
#define IM7_H_

#include <sys/select.h>

#include "thread.h"

//IM7
#define MAX_IM7PACK		128				//storage size for packages
//#define MAX_STATE		128				//storage size for uav state

#define IM7_BAUDRATE  57600

//#define IMU_EXTRAPOLATION
#define FLAG_GPSCHECK   0xC240

#define MAX_IM7BUFFER	4096				//this flag specify the size of buffer for receiving IM6 data

#if (_DEBUG && _DEBUG_RATE != 0)				//debug and slow
#define COUNT_GP7 		2				//gp6, ah6, sc6 in turn
#define RATIO_FILTER	0				//no filter
#else
#define COUNT_GP7 		10		// previous 10
#define RATIO_FILTER	0.636
#endif

//#define IM6_MODE_POLL
#define IMU_TIMEOUT_READ	200000

/*
 * NAV440 packet format
 */
#pragma pack(push, 1)
struct SC7RAWPACK {
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

struct SC7PACK {
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

struct  AH7RAWPACK {
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

struct  AH7PACK {
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
struct GP7RAWPACK {
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

struct GP7PACK {
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

struct GP7RAWPACK {
	int16_t  header;
	unsigned short type;
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
	
	int16_t  u;
	int16_t  v;
	int16_t  w;

	int32_t  longitude;
	int32_t  latitude;
	int16_t  altitude;

	int16_t  xtmp;
	
	unsigned int  itow;
	unsigned short  bit;
	unsigned short  crcCheck;
};

struct GP7PACK {
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
};

union IM7RAWPACK {
	SC7RAWPACK		sc7;
	AH7RAWPACK		ah7;
	GP7RAWPACK		gp7;
};

union IM7PACK {
	SC7PACK		sc7;
	AH7PACK		ah7;
	GP7PACK		gp7;
};

#pragma pack(pop)

/*
 * NAV440
 */
class clsIM7 : public clsThread {
public:
	clsIM7();
	virtual ~clsIM7();

protected:
	int m_nsIM7;
	fd_set rfd;		//read fd
	timeval m_timeOut;

protected:
	int m_nLost;	// # of lost packets
	//to store received packs
	double m_tIM7[MAX_IM7PACK];
	IM7RAWPACK m_im7Raw[MAX_IM7PACK];
	IM7PACK m_im7[MAX_IM7PACK];
	int m_nIM7;				//number of pack

	//to store last gp7 pack
	double m_tGP7;
	IM7RAWPACK m_gp7Raw;
	IM7PACK m_gp7;

	//to store last sc7 pack
	double m_tSC7;
	IM7RAWPACK m_sc7Raw;
	IM7PACK m_sc7;

	//to store last ah7 pack
	double m_tAH7;
	IM7RAWPACK m_ah7Raw;
	IM7PACK m_ah7;

	pthread_mutex_t m_mtxIM7;

	double m_tRequest;
	double m_tRetrieve;

	short m_bit;				//this word set after each read of a pack of data

	char m_szBuffer[MAX_IM7BUFFER];
	int m_nBuffer;

	double m_xIM7;
	double m_yIM7;
	double m_zIM7;				//position of IM7

	double m_xAntenna;
	double m_yAntenna;
	double m_zAntenna;

public:
	void SetType(char chType);
	void SetRate(int nRate);
	void SetBaudRate(int nBaudRate);

	void Poll(char chType);

	void WriteCommand(int nType);

	int GetPack(IM7RAWPACK raw[], int nMaxPack);

	void ExtractPackFromBuffer(char *pBuffer, IM7RAWPACK *pPack);	/// put the data rcved in pBuffer to pPack

	void PushPack(double tPack, IM7RAWPACK *pPack);
	void Translate(IM7RAWPACK *pRaw, IM7PACK *pPack);

	BOOL GetLastPack(int nType, double *pTime, IM7PACK *pPack);
	BOOL GetLastPack(int nType, double *pTime, IM7RAWPACK *pRaw);

	void GetIM7Position(double pos[3]) { pos[0] = m_xIM7; pos[1] = m_yIM7; pos[2] = m_zIM7; }
	void GetAntennaPosition(double pos[3]) { pos[0] = m_xAntenna; pos[1] = m_yAntenna; pos[2] = m_zAntenna; }

public:
	virtual BOOL InitThread();
	virtual int EveryRun();
	virtual void ExitThread();

	friend class clsDLG;
};


#endif				//IM7_H_

