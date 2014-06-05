#ifndef IM6_H_
#define IM6_H_

#include "thread.h"

//IM6
#define MAX_IM6PACK	128				//storage size for packages

#define IM6_BAUDRATE 57600
//#define IM6_BAUDRATE 38400
//#define IM6_BAUDRATE 19200

#define FLAG_GPSCHECK   0xC240

#define MAX_IM6BUFFER	4096				//this flag specify the size of buffer for receiving IM6 data

#if (_DEBUG && _DEBUG_RATE != 0)				//debug and slow
#define COUNT_GP6 		2				//gp6, ah6, sc6 inturn
#define RATIO_FILTER	0				//no filter
#else
#define COUNT_GP6 		10
#define RATIO_FILTER	0.636
#endif

//#define IMU_EXTRAPOLATION		// extrapolate the IMU data in case of 0 correct packet rcved
//#define IM6_MODE_POLL
#define GPSSTATUS_UNLOCK	0x0200

//IM6
/*
 * NAV420
 */
#pragma pack(push, 1)
struct SC6RAWPACK {
	int16_t  header;
	unsigned char  type;

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
	int16_t  itow;
	int16_t  bit;
	int16_t  check;
};

struct SC6PACK {
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

struct  AH6RAWPACK {
	int16_t  header;
	unsigned char  type;

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
	int16_t  itow;
	int16_t  bit;
	int16_t  check;
};

struct  AH6PACK {
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

struct GP6RAWPACK {
	int16_t  header;
	unsigned char type;

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

	int16_t  itow;
	int16_t  bit;
	int16_t  check;
};

struct GP6PACK {
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
};

union IM6RAWPACK {
	SC6RAWPACK		sc6;
	AH6RAWPACK		ah6;
	GP6RAWPACK		gp6;
};

union IM6PACK {
	SC6PACK		sc6;
	AH6PACK		ah6;
	GP6PACK		gp6;
};

#pragma pack(pop)

class clsIM6 : public clsThread {
public:
	clsIM6();
	virtual ~clsIM6();

protected:
	int m_nsIM6;

protected:
	int m_nLost;	// # of lost pkts
	//to store received packs
	double m_tIM6[MAX_IM6PACK];
	IM6RAWPACK m_im6Raw[MAX_IM6PACK];
	IM6PACK m_im6[MAX_IM6PACK];
	int m_nIM6;				//number of pack

	//to store last gp6 pack
	double m_tGP6;
	IM6RAWPACK m_gp6Raw;
	IM6PACK m_gp6;

	//to store last sc6 pack
	double m_tSC6;
	IM6RAWPACK m_sc6Raw;
	IM6PACK m_sc6;

	//to store last ah6 pack
	double m_tAH6;
	IM6RAWPACK m_ah6Raw;
	IM6PACK m_ah6;

	pthread_mutex_t m_mtxIM6;

	double m_tRequest;
	double m_tRetrieve;

	short m_bit;				//this word set after each read of a pack of data

	char m_szBuffer[MAX_IM6BUFFER];
	int m_nBuffer;

	double m_xIM6;
	double m_yIM6;
	double m_zIM6;				//position of IM6

	double m_xAntenna;
	double m_yAntenna;
	double m_zAntenna;

public:
	void SetType(char chType);
	void SetRate(int nRate);
	void SetBaudRate(int nBaudRate);

	void Poll(char chType);

	void WriteCommand(int nType);

//	int GetLostPktNum() const	{ return m_nLost; }

	int GetPack(IM6RAWPACK raw[], int nMaxPack);

	void ExtractPackFromBuffer(char *pBuffer, IM6RAWPACK *pPack);

	void PushPack(double tPack, IM6RAWPACK *pPack);
	void Translate(IM6RAWPACK *pRaw, IM6PACK *pPack);

	BOOL GetLastPack(int nType, double *pTime, IM6PACK *pPack);
	BOOL GetLastPack(int nType, double *pTime, IM6RAWPACK *pRaw);

	void GetIM6Position(double pos[3]) { pos[0] = m_xIM6; pos[1] = m_yIM6; pos[2] = m_zIM6; }
	void GetAntennaPosition(double pos[3]) { pos[0] = m_xAntenna; pos[1] = m_yAntenna; pos[2] = m_zAntenna; }

public:
	virtual BOOL InitThread();
	virtual int EveryRun();
	virtual void ExitThread();

	friend class clsDLG;
};

#endif /* IM6_H_ */
