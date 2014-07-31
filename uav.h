//uav.h
//header file for onboard program
//2011-03-15 revised
//this is a public header file for the uav project, in which common parameters and operations all defined, which is referred by other modules

#ifndef UAV_H_
#define UAV_H_

#include <sys/neutrino.h>
#include <inttypes.h>
#include <string.h>

#include "matrix.h"
#include "parser.h"
//#include "ctl.h"

#define _VERSION 2752

//#define _GUMSTIX				//for babylion, gumstix chips are used

//debug flags
#define DEBUGFLAG_MAIN		0x0001
#define DEBUGFLAG_IM6		0x0002
#define DEBUGFLAG_SVO		0x0004
#define DEBUGFLAG_DAQ		0x0008
#define DEBUGFLAG_CTL		0x0010
#define DEBUGFLAG_CMM		0x0020
#define DEBUGFLAG_DLG		0x0040
#define DEBUGFLAG_IM7		0x0080
#define DEBUGFLAG_SERVER 	0x0100
#define DEBUGFLAG_CLIENT 	0x0200

// hardware list ID
#define IMU_NAV420		1
#define IMU_NAV440		2
#define IMU_IG500N		3

#define DEBUGFLAG_SIMULATION		0x1000

//#define _DEBUG		0			//debug flag, 0 for no debug
//#define _DEBUG		DEBUGFLAG_MAIN
//#define _DEBUG		DEBUGFLAG_DLG | DEBUGFLAG_MAIN
//#define _DEBUG		(DEBUGFLAG_CTL | DEBUGFLAG_MODEL)
//#define _DEBUG		DEBUGFLAG_CMM
//#define _DEBUG		DEBUGFLAG_CTL
//#define _DEBUG		(DEBUGFLAG_CTL | DEBUGFLAG_CMM)
//#define _DEBUG		(DEBUGFLAG_SIMULATION | DEBUGFLAG_IM6)
//#define _DEBUG		(DEBUGFLAG_MAIN | DEBUGFLAG_IM7 | DEBUGFLAG_DAQ | DEBUGFLAG_DLG)
//#define _DEBUG		(DEBUGFLAG_IM7 | DEBUGFLAG_DLG)
//#define _DEBUG		DEBUGFLAG_IM7
//#define _DEBUG		DEBUGFLAG_IM6
//#define _DEBUG		(DEBUGFLAG_IM6 | DEBUGFLAG_CTL)
//#define _DEBUG		(DEBUGFLAG_IM6 | DEBUGFLAG_DLG | DEBUGFLAG_CMM)
//#define _DEBUG		(DEBUGFLAG_IM6 | DEBUGFLAG_CTL | DEBUGFLAG_DLG | DEBUGFLAG_CMM)
//#define _DEBUG		(DEBUGFLAG_SVO | DEBUGFLAG_DLG)
//#define _DEBUG		DEBUGFLAG_SVO
//#define _DEBUG		DEBUGFLAG_SERVER | DEBUGFLAG_CLIENT
//#define _DEBUG		DEBUGFLAG_SERVER
//#define _DEBUG		DEBUGFLAG_CLIENT

#define _DEBUG_RATE		0				//0 for fast, 20ms /turn, 1 for slow, 1 s /turn
//#define _DEBUG_RATE	1

#if (_DEBUG_RATE == 0)
#define _DEBUG_PERIOD	20
#define _DEBUG_COUNT_1	50
#else
#define _DEBUG_PERIOD	1000
#define _DEBUG_COUNT_1	1
#endif

#define _DEBUG_COUNT	_DEBUG_COUNT_1				//number of periods for every printf

#ifndef BOOL
#define BOOL int
#define TRUE 1
#define FALSE 0
#endif

#ifndef BYTE
#define BYTE	unsigned char
#endif

#ifndef WORD
#define WORD	unsigned short
#endif

#ifndef DWORD
#define DWORD	unsigned long
#endif

#define BYTE0(a)	(BYTE)(0x00ff & (a))
#define BYTE1(a)	(BYTE)(0x00ff & ((a)>>8))
#define BYTE2(a)	(BYTE)(0x00ff & ((a)>>16))
#define BYTE3(a)	(BYTE)(0x00ff & ((a)>>24))

#define WORD0(a)	(WORD)(0x0000ffff & (a))
#define WORD1(a)	(WORD)(0x0000ffff & ((a)>>16))

#define LOBYTE(a)	(BYTE)(0x00ff & (a))
#define HIBYTE(a)	(BYTE)((WORD)(a) >> 8)

#define LOWORD(a)	WORD0(a)
#define HIWORD(a)	WORD1(a)

#define MAKEWORD(lo, hi)	((WORD)(BYTE)(lo) | (WORD)(hi) << 8)
#define MAKELONG(lo, hi)	((DWORD)(WORD)(lo) | (DWORD)(hi) << 16)
#define MAKELONG4(a,b,c,d)	((DWORD)(BYTE)(a) | (DWORD)(BYTE)(b) << 8 | (DWORD)(BYTE)(c) << 16 | (DWORD)(BYTE)(d) << 24)

#ifdef _GUMSTIX
#define _ARM
#endif

#ifndef _ARM
#define GETWORD(buf)	(WORD &)(buf)[0]
#define GETLONG(buf)	(DWORD &)(buf)[0]
#define GETDOUBLE(buf)	(double &)(buf)[0]

inline void COPYWORD(void *buf, void *src) { *(WORD *)buf = *(WORD *)src; }
inline void COPYLONG(void *buf, void *src) { *(DWORD *)buf = *(DWORD *)src; }
inline void COPYDOUBLE(void *buf, void *src) { *(double *)buf = *(double *)src; }

inline void PUTWORD(void *buf, WORD a) { *(WORD *)buf = a; }
inline void PUTLONG(void *buf, DWORD a) { *(DWORD *)buf = a; }
inline void PUTDOUBLE(void *buf, double a) { *(double *)buf = a; }

#else				//arm code regulation, avoid alignment confliction
#define GETWORD(buf)	MAKEWORD((buf)[0], (buf)[1])				//get word from char buffer
#define GETLONG(buf)	MAKELONG4((buf)[0], (buf)[1], (buf)[2], (buf)[3])
#define GETDOUBLE(buf)	GETDOUBLEBYTES(buf)
#define GETFLOAT(buf)	GETFLOATBYTES(buf)

double GETDOUBLEBYTES(void *buf);
float GETFLOATBYTES(void *buf);

inline void COPYLONG(void *buf, void *src) {
	((BYTE *)buf)[0] = ((BYTE *)src)[0];
	((BYTE *)buf)[1] = ((BYTE *)src)[1];
	((BYTE *)buf)[2] = ((BYTE *)src)[2];
	((BYTE *)buf)[3] = ((BYTE *)src)[3];
}

inline void COPYWORD(void *buf, void *src) {
	((BYTE *)buf)[0] = ((BYTE *)src)[0];
	((BYTE *)buf)[1] = ((BYTE *)src)[1];
}

inline void COPYDOUBLE(void *buf, void *src) {
	memcpy(buf, src, sizeof(double));
}

inline void PUTWORD(void *buf, WORD a) { ((BYTE *)buf)[0] = LOBYTE(a); ((BYTE *)buf)[1] = HIBYTE(a); }
inline void PUTLONG(void *buf, DWORD a) {
	((BYTE *)buf)[0] = BYTE0(a);
	((BYTE *)buf)[1] = BYTE1(a);
	((BYTE *)buf)[2] = BYTE2(a);
	((BYTE *)buf)[3] = BYTE3(a);
}

void PUTDOUBLEBYTES(void *buf, double a);
inline void PUTDOUBLE(void *buf, double a) { PUTDOUBLEBYTES(buf, a); }
#endif

#ifndef PI
#define PI	3.1415926535897932384626433832795
#endif

#define EPS 2.71828182845905

#define ZERO	1e-8

#define VALUE_VACANT	1e20
#define MAXVALUE		1e15
//VALUE_VACANT is large than MAXVALUE to define a vacant value
//it is decided on the expression "if (value>MAXVALUE) { vacant case }



#define MIN_STATE_U		-5
#define MAX_STATE_U		15
#define MIN_STATE_V		-8
#define MAX_STATE_V		8
#define MIN_STATE_W		-5
#define MAX_STATE_W		5

#define MIN_STATE_ACX	-2
#define MAX_STATE_ACX	2
#define MIN_STATE_ACY	-2
#define MAX_STATE_ACY	2
#define MIN_STATE_ACW	-1.5
#define MAX_STATE_ACW	1.5

#define MIN_STATE_R		(-PI/4)				//8 second 1 round
#define MAX_STATE_R		(PI/4)

#define MIN_STATE_X		-10000				//200m
#define MAX_STATE_X		10000				//200m
#define MIN_STATE_Y		-10000				//200m
#define MAX_STATE_Y		10000				//200m
#define MIN_STATE_Z		-200
#define MAX_STATE_Z		200

#define MIN_STATE_AS	-0.04
#define MAX_STATE_AS	0.04
#define MIN_STATE_BS	-0.04
#define MAX_STATE_BS	0.04
#define MIN_STATE_RFB	-0.6
#define MAX_STATE_RFB	0.8

#define MIN_TRIM_ERROR  -15
#define MAX_TRIM_ERROR	15

#define STRLEN_COMMAND  64

#define MILISECOND		1000000				//one milisecond in nanoseconds

#if (_DEBUG)
#define PERIOD_MAIN		_DEBUG_PERIOD			//1 second per loop
#else
#define PERIOD_MAIN		20				//period for one circle of tasks processing in milisecond
#endif

#define MAX_TC	128

#define MAX_FILENAME		64

//supposed maxmum processing period in milisecond for each task
#define PERIOD_IM6		2
#define PERIOD_IM7		2
#define PERIOD_IM8		3
#define PERIOD_IM9		2
#define PERIOD_DAQ		1
#define PERIOD_CAM		1
#define PERIOD_CTL		1
#define PERIOD_SVO		2
#define PERIOD_CMM		2
#define PERIOD_DLG		5
#define PERIOD_SERVER	2	// TODO
#define PERIOD_CLIENT	2	// TODO
#define PERIOD_COOP		1
#define PERIOD_NET		2
#define PERIOD_URG		6
#define PERIOD_PTU 		1

//DATACODE, char
#define DATA_VERSION	0x91				//version of onboard program
#define DATA_INFO		0x92				//data information relative with program version

#define DATA_GP6RAW		0x02
#define DATA_AH6RAW		0x03
#define DATA_SC6RAW		0x04
#define DATA_DAQRAW		0x05
#define DATA_SVORAW		0x06
#define DATA_GP7RAW		0x07
#define DATA_AH7RAW		0x08
#define DATA_SC7RAW		0x1C
#define DATA_GP8RAW		0x1D
#define DATA_SC8RAW		0x1E

#define DATA_GP6		0x12
#define DATA_AH6		0x13
#define DATA_SC6		0x14
#define DATA_DAQ		0x15
#define DATA_SVO		0x16
#define DATA_SIG		0x17
#define DATA_CTL		0x18
#define DATA_GP7		0x19
#define DATA_AH7		0x1A
#define DATA_SC7		0x1B
#define DATA_URG        0x1C
#define DATA_URG_4      0x1D


#define DATA_GP8		0x22
#define DATA_SC8		0x23

#define DATA_MESSAGE	0x09				//for message transfer between helicopter and station
#define DATA_STATE		0x21
#define DATA_EQU		0x22

#define DATA_TC			0x23
#define DATA_COOP		0x27
#define DATA_VICON		220

#define DATA_IMU_CONFIG	0x28
#define DATA_LASER_CONFIG 0x29
#define DATA_CAM_CONFIG 0x2A
#define DATA_SVO_CONFIG 0x2B
#define DATA_CMM_CONFIG 0x2C
#define DATA_TCP_CONFIG 0x2D

#define DATA_GUMSTIX2VISION 221
#define DATA_VISION2GUMSTIX 0x24

//DAQ
#define MAX_DAQ	128				//storage size

#if (_DEBUG)
#define COUNT_DAQ (2*_DEBUG_COUNT_1)
#else
#define COUNT_DAQ 100
#endif

//#define VIRTUAL_LEADER
#define DIST_FD		0
#define DIST_LD		-10
#define DIST_HD		0
//SVO
#define SVO_BAUDRATE 115200


#define SVO_WEIGHT1		0.7154
#define SVO_WEIGHT2		0.2846
// for trimvalue: validate svo data till #COUNT_GETTRIM
#define COUNT_GETTRIM	250		// validate number of svo data for trimvalue calculation
#define THRESHOLDHIGH_U		1
#define THRESHOLDLOW_U		-1
#define THRESHOLDHIGH_V		1
#define THRESHOLDLOW_V		-1
#define THRESHOLDHIGH_W		1
#define THRESHOLDLOW_W		-1
#define THRESHOLDHIGH_P		1
#define THRESHOLDLOW_P		-1
#define THRESHOLDHIGH_Q		1
#define THRESHOLDLOW_Q		-1
#define THRESHOLDHIGH_R		1
#define THRESHOLDLOW_R		-1


#define COMMAND_RUN			1
#define COMMAND_QUIT		2
#define COMMAND_MAGTEST		3
#define COMMAND_MAGSET		4
#define COMMAND_ENGINE		5
#define COMMAND_HOVER		6
#define COMMAND_LIFT		7
#define COMMAND_DESCEND		8
#define COMMAND_HEADTO		9
#define COMMAND_TURN		10
#define COMMAND_HFLY		16
#define COMMAND_CFLY		17				//fly with velocity feedback
#define COMMAND_PATH		18				//select tracking path (index)
#define COMMAND_TRACK		19				//tracking target
#define COMMAND_CHIRP		20
#define COMMAND_FLY			21				//fly(u,v,w,r)
#define COMMAND_HOLD		22
#define COMMAND_HOLDPI		99		// the same as hold, but with PI as outloop control law

#define COMMAND_PLAN		23

#define COMMAND_TAKEOFF		24
#define COMMAND_LAND		25

#define COMMAND_ENGINEUP	26
#define COMMAND_ENGINEDOWN	27

#define COMMAND_EMERGENCY	28
#define COMMAND_EMERGENCYGROUND	29

#define COMMAND_TAKEOFFA	30
#define COMMAND_LANDA		31

#define COMMAND_COORDINATE	33

#define COMMAND_GPATH		34

#define COMMAND_DYNAMICPATHRESET	41
#define COMMAND_DYNAMICPATH			42

#define COMMAND_GPSORIGIN			43
#define COMMAND_LEADERUPDATE		44

#define COMMAND_FORMATION			45		// broadcast: command all the uavs to the desired position for formation
#define COMMAND_STARTFORMATION		46		// broadcast: start executing formation
#define COMMAND_STOPFORMATION 		48
#define COMMAND_FILTER		51
#define COMMAND_PARA		52

#define COMMAND_PATHA		53

#define COMMAND_TEST		61

#define COMMAND_NOTIFY		77
#define COMMAND_NONOTIFY	78

#define COMMAND_GETTRIM		80		// for get pilot's svos' trim values

#define COMMAND_MODE	88
#define COMMAND_GOTO	89

#define COMMAND_COOP_PKT		85
#define COMMAND_DATA		201
#define COMMAND_MESSAGE		202

/***Added by Peidong for Kangli's Competition
 */
#define COMMAND_COMPETITION 200


// Added by Lin Feng
#define COMMAND_CAMRUN     92
#define COMMAND_CAMSTOP    93
#define COMMAND_CAMTHOLD   94
#define COMMAND_CAMQUIT    95
#define COMMAND_CAMUAVPOSE 96
#define COMMAND_CAMTRACK	97
#define COMMAND_CAMVBS		98		// vision based stabilization
#define COMMAND_TARGETSTATE_TOYCAR		81
#define COMMAND_TARGETSTATE_LANDPAD		82
#define COMMAND_OPTICALFLOW	99

#define MAXSIZE_PACKAGE		5000 //512				//package = code + parameter, must bigger than MAXSIZE_PARAMETER+1
#define MAXSIZE_TELEGRAPH	MAXSIZE_PACKAGE+6				//telegraph = 0x55 + 0x55 + size + package + check
#define MAX_CMMBUFFER		2*MAXSIZE_TELEGRAPH

#define MAXSIZE_PARAMETER	512 /*128*/

#define STRLEN_MESSAGE	256

#define LEADER_UPDATE	10

#define ID_STATION				5
#define ID_HELION				6
#define ID_SHELION				7
#define ID_HENGLION				8
#define ID_QUADLION				14
#define ID_FEILION				10
#define ID_GREMLION				11
#define ID_ALL					1

#if (_DEBUG)
#define COUNT_CMM _DEBUG_COUNT_1
#else
#define COUNT_CMM 50
#endif

//User
#define MAX_SIZE_USERCOMMAND	8

//function define
#define INPI(angle)		(angle -= ::floor((angle+PI)/(2*PI))*2*PI)

//declared classes
class clsPath;
class clsTmpPath;

	class clsIM6;
	class clsIM7;
	class clsIM8;
	class clsIM9;
//	class clsURG;
	class clsDAQ;
	class clsCAM;
	class clsCTL;
	class clsSVO;
//	class clsCMM;
	class clsDLG;
	class clsNET;
	class clsCoop;
//	class clsURG;

//class clsState;
//struct UAVSTATE;

class clsMain;
class clsUser;

//class clsParser;

#pragma pack(push, 1)
struct FORMATIONSTATE
{
	double t, x, y, z, c;				//position and yaw, for relative-position formation
	double longitude, latitude, altitude;	// for GPS-position formation
};

#pragma pack(pop)

inline double range(double x, double min, double max) {	return x > max ? max : x < min ? min : x; }

inline BOOL IsVacant(double value) { return (value >= MAXVALUE); }

struct PACKAGE {
	short size;
	char content[MAXSIZE_TELEGRAPH];
};

struct ADDRESSEDPACKAGE {
	short from, to;
	PACKAGE package;
};

struct COMMAND {
	short code;
	char parameter[MAXSIZE_PARAMETER];
};

struct TELEGRAPH {
	int size;
	char content[MAXSIZE_TELEGRAPH];
};

#define EVENT_BEHAVIOREND	1
#define MAXSIZE_EVENTINFO	128
struct EVENT {
	int code;
	char info[MAXSIZE_EVENTINFO];
};

struct UAVSTATE {
	double x, y, z;				//position or (longitude, latitude, altitude)
	double u, v, w;				//velocity
	double a, b, c;				//attitude
	double p, q, r;				//rotating

	double acx, acy, acz;				//accelerate along x,y,z axis
	double acp, acq, acr;

	double ug, vg, wg;
	double longitude, latitude, altitude;

	double as, bs, rfb;
};

struct EQUILIBRIUM {
	double u, v, w;
	double p, q, r;
	double a, b, c;

	double as, bs, rfb;

	double ea, ee, eu, er, et;
};

struct DATASTRUCT_GUMSTIX2VISION{
	int landingFinishFlag; int dummy;
	double gumstixTime;
	double x, y, z;				//position or (longitude, latitude, altitude)
	double ug,vg,wg;				//velocity
	double acx, acy, acz;		// acceleration in x,y,z
	double a, b, c;				//attitude
	double u, v, w;
	double latitude, longitude;
};

struct DATASTRUCT_VISION2GUMSTIX{
	int flags[10];
	double masterMind_time;
	double cameraFrame_dvec[3]; // in camera frame
	double nedFrame_dvec[3];
};



//extern clsState _state;
extern clsIM6 _im6;
extern clsIM7 _im7;
extern clsIM8 _im8;
extern clsIM9 _im9;
//extern clsURG _urg;
extern clsDAQ _daq;
extern clsCAM _cam;
extern clsCTL _ctl;
extern clsSVO _svo;
//extern clsCMM _cmm;
extern clsDLG _dlg;
extern clsMain _main;
extern clsUser _user;
extern clsParser _parser;				//the parser is used to load system parameters
extern clsCoop _coop;
extern clsNET	_net;

extern int _HELICOPTER;
extern double _gravity;
extern double _radius;

//extern EQUILIBRIUM	_equ_Hover;

#define MAX_PATH			20				//max number of paths

extern clsPath _path[MAX_PATH];				//ten pathes
extern int _nPath;

extern clsTmpPath _pathTmp;				//for dynamically created path

unsigned short CheckSum(char *pChar, int nSize);				//check sum of a package
unsigned short CheckCRC(char *pBuffer, int nLen);
unsigned short CheckCRC1(const unsigned char *pBuffer, int nLen);

inline double min(double x, double y) { return x <= y ? x : y; }
inline double max(double x, double y) { return x >= y ? x : y; }

extern uint64_t _cycle0;                               //initial clock cycle as the application start
extern uint64_t _cps;
//uint64_t _cps = SYSPAGE_ENTRY(qtime)->cycles_per_sec;                           //cycles in one second
//uint64_t _cpms = (uint64_t)(0.001*_cps);
inline double GetTime() { return (double)(ClockCycles()-_cycle0)/_cps; }

//Calculation funcitons
inline void G2B(double abc[3], double xg[3], double xf[3]) {
	double Mfg[3][3];

	clsMetric::AttitudeToTransformMatrix(abc, Mfg, NULL);
	clsMetric::X(Mfg, xg, xf);
}

inline void B2G(double abc[3], double xf[3], double xg[3]) {
	double Mgf[3][3];
	clsMetric::AttitudeToTransformMatrix(abc, NULL, Mgf);
	clsMetric::X(Mgf, xf, xg);
}

inline void N2G(double abc0[3], double n[3], double g[3]) {
	double Mgn[3][3];
	clsMetric::AttitudeToTransformMatrix(abc0, Mgn, NULL);
	clsMetric::X(Mgn, n, g);
}

inline void G2N(double abc[3], double g[3], double n[3]) {
	double Mng[3][3];
	clsMetric::AttitudeToTransformMatrix(abc, NULL, Mng);
	clsMetric::X(Mng, g, n);
}

void EulerNext(double abc[3], double pqr[3], double dt);
void EulerEstimate(double abc[3], double ac[3], double mg[3]);
double EulerEstimateYawangle(double ab[2], double mg[3]);

const char *GetCommandString(short code);
const char *GetBehaviorString(int nBehavior);

#endif
