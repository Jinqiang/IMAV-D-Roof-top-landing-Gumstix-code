/**************************************************************************
uav.cpp
the implementation file of onboard software of helicopter
2005-07-15
***************************************************************************/ 

#include <sys/syspage.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>

#include "uav.h"

#include "im6.h"
#include "im7.h"
#include "im8.h"
#include "im9.h"
#include "daq.h"
#include "cam.h"
#include "ctl.h"
#include "svo.h"
#include "cmm.h"
#include "net.h"
#include "state.h"
#include "coop.h"
#include "dlg.h"
#include "main.h"
#include "parser.h"
#include "user.h"
#include "laser.h"
#include "ptu.h"

// Global variables
int _HELICOPTER = 9;

double _gravity = 9.781;	// gravitational constant
double _radius = 6.3781e6;	// radius of earth

EQUILIBRIUM _equ_Hover = {
	0, 0, 0,		// u, v, w
	0, 0, 0,		// p, q, r
	0, 0, 0,		// a, b, c
	0, 0, 0,		// as,bs,rfb
	-0.066, -0.068, -0.1722, 0.00, 0.075 //ea, ee, eu, er, et
};

clsState _state;
clsIM6 _im6;
clsIM7 _im7;
clsIM8 _im8;
clsIM9 _im9;
clsDAQ _daq;
clsCAM _cam;
clsCTL _ctl;
clsSVO _svo;
clsCMM _cmm;
clsDLG _dlg;
clsNET	_net;
clsURG _urg;
clsCoop _coop;
clsPTU _ptu;

clsMain _main;
clsUser _user;
clsParser _parser; // parser is used to load system parameters

//global
uint64_t _cycle0; // initial clock cycle as the application start
uint64_t _cps = SYSPAGE_ENTRY(qtime)->cycles_per_sec; //cycles in one second
uint64_t _cpms = (uint64_t)(0.001*_cps);

//inline double GetTime() { return (double)(ClockCycles()-_cycle0)/_cps; }

//path
clsPath _path[MAX_PATH]; // 20 paths
int _nPath = 0;

clsTmpPath _pathTmp;				//for dynamically created path

struct COMMANDENTRY {
	short code;
	const char *pszName;
};

static COMMANDENTRY _cmdentry[] = {
	{ COMMAND_RUN,			"run"		},
	{ COMMAND_QUIT,			"quit"		},
	{ COMMAND_MAGTEST,		"magtest"	},
	{ COMMAND_MAGSET,		"magset"	},
	{ COMMAND_HOVER,		"hover"		},
	{ COMMAND_LIFT,			"lift"		},
	{ COMMAND_DESCEND,		"descend"	},
	{ COMMAND_HEADTO,		"headto"	},
	{ COMMAND_HFLY,			"hfly" 		},
	{ COMMAND_PATH,			"path"		},
	{ COMMAND_GPATH,		"gpath"		},
	{ COMMAND_TEST,			"test"		},
	{ COMMAND_CHIRP,		"chirp"		},
	{ COMMAND_FLY,			"fly"		},
	{ COMMAND_HOLD,			"hold"		},
	{ COMMAND_PLAN,			"plan"		},
	{ COMMAND_NOTIFY,		"notify"	},
	{ COMMAND_NONOTIFY,		"nonotify"	},
	{ COMMAND_FILTER,		"filter"	},
	{ COMMAND_PARA,			"para"		},
	{ COMMAND_TRACK,		"track"		},
	{ COMMAND_CFLY,			"cfly"		},
	{ COMMAND_ENGINE,		"engine"	},
	{ COMMAND_TAKEOFF,		"takeoff"	},
	{ COMMAND_LAND,			"land"		},
	{ COMMAND_ENGINEUP,		"engineup"	},
	{ COMMAND_ENGINEDOWN,	"enginedown"},
	{ COMMAND_EMERGENCY,	"emergency"	},
	{ COMMAND_EMERGENCYGROUND,	"emergencyground"	},
	{ COMMAND_TAKEOFFA,		"takeoff-a"	},
	{ COMMAND_LANDA,		"land-a"		},
	{ COMMAND_PATHA,		"path-a"		},
	{ COMMAND_DYNAMICPATHRESET,		"dynamicpath(reset)"	},
	{ COMMAND_DYNAMICPATH,	"dynamicpath"	},
	{ COMMAND_GETTRIM,		"gettrim"	},
	{ COMMAND_HOLDPI,		"holdpi"	},
	{ COMMAND_COORDINATE,	"coordinate" },
	{ COMMAND_FORMATION,	"formation"	},
	{ COMMAND_STOPFORMATION,"stopformation" },
	{ COMMAND_COOP_PKT,		"coopRelay" },
	{ COMMAND_CAMRUN,       "camrun"    },
	{ COMMAND_CAMSTOP,      "camstop"   }, 
	{ COMMAND_CAMTHOLD,     "camthold"  }, 
	{ COMMAND_CAMQUIT,      "camquit"   }, 	
	{ COMMAND_CAMTRACK,		"camtrack"	},
	{ COMMAND_CAMVBS,		"camvbs"	},
	{ COMMAND_MODE,			"mode"		},
	{ COMMAND_COMPETITION,  "competition"},
	{ 0, NULL }				//last entry
};

const char *GetCommandString(short code)
{
	int i;
	for (i=0; ; i++) {
		if (code == _cmdentry[i].code || _cmdentry[i].code == 0) {
			break;
		}
	}
	
	return _cmdentry[i].pszName;
}

void EulerEstimate(double att[3], double ac[3], double mg[3])
//att - attitude, ac - acceleration (including gravity), mg - magnetic field
{
	att[0] = ::atan2(-ac[1], -ac[2]);

	double acyz = ::sqrt(ac[1]*ac[1]+ac[2]*ac[2]);
	att[1] = ::atan2(ac[0], acyz);
	att[2] = ::atan2(-mg[1], mg[0]);
}

double EulerEstimateYawangle(double ab[2], double mg[3])
{
	double mx = -1.6209;
	double my = 0.2895;
	double mz = 0.0523;

//	double mx = 0.4293;
//	double my = 0;
//	double mz = -0.1161;

	double sina = ::sin(ab[0]); double sinb = ::sin(ab[1]);
	double cosa = ::cos(ab[0]); double cosb = ::cos(ab[1]);

	double mxf = mg[0]; double myf = mg[1];

	double a1 = mx*cosb; double b1 = my*cosb;
	double c1 = mxf+mz*sinb;

	double a2 = mx*sina*sinb+my*cosa;
	double b2 = -mx*cosa+my*sina*sinb;
	double c2 = myf-mz*sina*cosb;

	double sinc = (a2*c1-a1*c2)/(a2*b1-a1*b2);
	double cosc = (b2*c1-b1*c2)/(a1*b2-a2*b1);

	double c = atan2(sinc, cosc);

	return c;
}

void EulerNextOde4(double att[3], double rot[3], double dt)
{
	double _Mfg[3][3];	clsMatrix Mfg(3, 3, (double *)_Mfg, TRUE);
	clsMetric::AttitudeToTransformMatrix(att, _Mfg, NULL);

	double p = rot[0]; double q = rot[1]; double r = rot[2];

	double _Mom[3][3] = { { 0, -r, q 		}, { r, 0, -p 		}, { -q, p, 0 } };
	clsMatrix Mom(3, 3, (double *)_Mom, TRUE);

	double _Mom2[3][3]; clsMatrix Mom2(3, 3, (double *)_Mom2, TRUE);
	clsMatrix::X(Mom, Mom, Mom2);				//squre Mom

	double _Mom3[3][3]; clsMatrix Mom3(3, 3, (double *)_Mom3, TRUE);
	clsMatrix::X(Mom2, Mom, Mom3);

	double _Mom4[4][4]; clsMatrix Mom4(3, 3, (double *)_Mom4, TRUE);
	clsMatrix::X(Mom3, Mom, Mom4);

	//expm(Mom) = I-Mom*dt+(Mom*dt)^2/2-(Mom*dt)^3/6+(Mom*dt)^4/24
	Mom *= -dt;
	Mom2 *= dt*dt/2;
	Mom3 *= -dt*dt*dt/6;
	Mom4 *= dt*dt*dt*dt/24;

	Mom += Mom2; Mom += Mom3; Mom += Mom4;
	Mom[0][0] += 1; Mom[1][1] += 1; Mom[2][2] += 1;

	//Mfg = exp(Mom*dt)*Mfg, using fourth order Tayler series
	double _New[3][3]; clsMatrix New(3, 3, (double *)_New, TRUE);

	clsMatrix::X(Mom, Mfg, New);

	//get next attitude after dt
	att[0] = ::atan2(_New[1][2], _New[2][2]);
	
	if (-New[0][2] >= 1) att[1] = PI/2;
	else if (-New[0][2] <= -1) att[1] = -PI/2;
	else att[1] = ::asin(-_New[0][2]);

	att[2] = ::atan2(_New[0][1], _New[0][0]);
}

void EulerNext(double att[3], double rot[3], double dt)
{
	double _Mfg[3][3];	clsMatrix Mfg(3, 3, (double *)_Mfg, TRUE);
	clsMetric::AttitudeToTransformMatrix(att, _Mfg, NULL);

	double p = rot[0]; double q = rot[1]; double r = rot[2];

//	double _Mom[3][3] = { { 0, -r, q 		}, { r, 0, -p 		}, { -q, p, 0 } };
//	clsMatrix Mom(3, 3, (double *)_Mom, TRUE);

	//Mom = I-Mom*dt
//	double _Mom[3][3] = { { 1, r*dt, -q*dt 		}, { -r*dt, 1, p*dt 		}, { q*dt, -p*dt, 1 } };

	double dt2 = dt*dt/2;
	double _Mom[3][3] = {				//second order estimation
		{ 1-(q*q+r*r)*dt2,	r*dt+p*q*dt2, 		-q*dt+p*r*dt2			},
		{ -r*dt+p*q*dt2,	1-(p*p+r*r)*dt2,	p*dt+q*r*dt2			},
		{ q*dt+p*r*dt2,		-p*dt+q*r*dt2,		1-(p*p+q*q)*dt2	}
	};
	clsMatrix Mom(3, 3, (double *)_Mom, TRUE);

	//Mfg = exp(Mom*dt)*Mfg, using Tayler series approximation
	double _New[3][3]; clsMatrix New(3, 3, (double *)_New, TRUE);

	clsMatrix::X(Mom, Mfg, New);

	//get next attitude after dt
	att[0] = ::atan2(_New[1][2], _New[2][2]);
	
	if (-New[0][2] >= 1) att[1] = PI/2;
	else if (-New[0][2] <= -1) att[1] = -PI/2;
	else att[1] = ::asin(-_New[0][2]);

	att[2] = ::atan2(_New[0][1], _New[0][0]);
}

unsigned short CheckSum(char *pBuffer, int nLen)
{
	unsigned short sum = 0;

	for (int i=0; i<=nLen-1; i++) {
		sum += (unsigned char)pBuffer[i];
	}

	return sum;
}

unsigned short CheckCRC(char *pBuffer, int nLen)
{
	unsigned short crc = 0x1d0f;
	for (int i=0; i<nLen; i++)
	{
		crc ^= pBuffer[i] << 8;	
		
		for (int j=0; j<8; j++)
		{
			if (crc & 0x8000)
				crc = (crc << 1) ^ 0x1021;
			else
				crc = crc << 1;
		}	
	}
	
	return crc;
}

unsigned short CheckCRC1(const unsigned char *pBuffer, int nLen)
{
	unsigned short poly = 0x8408;
	unsigned short crc = 0;
	unsigned short carry;

	for (int i=0; i<nLen; i++) {
		crc ^= pBuffer[i];
		for (int j=0; j<8; j++) {
			carry = crc & 1;
			crc = crc / 2;
			if (carry) crc ^= poly;
		}
	}
	return crc;
}

void PUTDOUBLEBYTES(void *buf, double a)
{
	BYTE *pbuf = (BYTE *)buf;
	BYTE *p = (BYTE *)&a;
	pbuf[0] = p[0]; pbuf[1] = p[1];
	pbuf[2] = p[2]; pbuf[3] = p[3];
	pbuf[4] = p[4]; pbuf[5] = p[5];
	pbuf[6] = p[6]; pbuf[7] = p[7];
}

double GETDOUBLEBYTES(void *buf)
{
	BYTE *pbuf = (BYTE *)buf;
	double a;
	BYTE *p = (BYTE *)&a;

	p[0] = pbuf[0]; p[1] = pbuf[1];
	p[2] = pbuf[2]; p[3] = pbuf[3];
	p[4] = pbuf[4]; p[5] = pbuf[5];
	p[6] = pbuf[6]; p[7] = pbuf[7];

	return a;
}

float GETFLOATBYTES(void *buf)
{
	BYTE *pbuf = (BYTE *)buf;
	float a;
	BYTE *p = (BYTE *)&a;

	p[0] = pbuf[0]; p[1] = pbuf[1];
	p[2] = pbuf[2]; p[3] = pbuf[3];

	return a;
}

struct BEHAVIORSTRINGENTRY {
	int behavior;
	const char *name;
};
BEHAVIORSTRINGENTRY _behaviorstring[] = {
	{ BEHAVIOR_HOLD,	"hold"		},
	{ BEHAVIOR_HEADTO,	"headto"	},
	{ BEHAVIOR_HFLY,	"hfly"		},
	{ BEHAVIOR_PATH,	"path"		},
	{ BEHAVIOR_TEST,	"test"		},
	{ BEHAVIOR_CHIRP,	"chirp"		},
	{ BEHAVIOR_FLY,		"fly"		},
	{ BEHAVIOR_HOLD,	"hold"		},
	{ BEHAVIOR_HOLDPI,	"holdpi"	},
	{ BEHAVIOR_EMERGENCY,	"emergency"	},
	{ BEHAVIOR_AIM,		"aim"		},
	{ BEHAVIOR_CFLY,	"cfly"		},
	{ BEHAVIOR_EMERGENCYGROUND,	"emergencyground"	},
	{ BEHAVIOR_ENGINE,	"engine"	},
	{ BEHAVIOR_ENGINEUP,	"engineup"	},
	{ BEHAVIOR_ENGINEDOWN,	"enginedown"	},
	{ BEHAVIOR_TAKEOFF,	"takeoff"	},
	{ BEHAVIOR_LAND,	"land"		},
	{ BEHAVIOR_AUXILIARYDOWN,	"auxiliarydown"	},
	{ BEHAVIOR_PATHA,	"patha"		},
	{ BEHAVIOR_VELTRACK,	"veltrack"	},
	{ BEHAVIOR_FORMATION,	"formation"	},
	{ BEHAVIOR_DECOUPLEHOVER,	"decouplehover"	},
	{ BEHAVIOR_CAMTRACK,	"camtrack"	},
	{ BEHAVIOR_SEMIAUTO,		"semiauto"	},
	{ 0,	NULL }
};

const char *GetBehaviorString(int nBehavior)
{
	for (int i=0; _behaviorstring[i].behavior != 0; i++) {
		if (_behaviorstring[i].behavior == nBehavior) return _behaviorstring[i].name;
	}
	
	return "unknown";
}
