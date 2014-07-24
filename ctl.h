#ifndef CTL_H_
#define CTL_H_

#include "matrix.h"
//#include "state.h"
#include "svo.h"
#include "uav.h"

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

extern EQUILIBRIUM _equ_Hover;

#ifndef BOOL
#define BOOL	int
#define TRUE	1
#define FALSE	0
#endif

#define CYCLE_TIME_IN_SECONDS					0.02
#define NUMBER_OF_DOFS							4

//CTL
#define NEW_OUTERLOOP
#define INNERLOOP_LQR				1
#define INNERLOOP_GAINSCHEDULE		2
#define INNERLOOP_DECOUPLE			3
#define INNERLOOP_CNF				4
#define INNERLOOP_RPT				5

#define BEHAVIOR_ADD			0x80000000				//flag for behavior adding

#define BEHAVIOR_HEADTO			4				//head to (x,y,z)
#define BEHAVIOR_HFLY			11				//height keeping fly (u,v,r,h)
#define BEHAVIOR_PATH			12				//path tracking
#define BEHAVIOR_TEST			13				//test
#define BEHAVIOR_CHIRP			14				//chirp signal
#define BEHAVIOR_FLY			15				//fly (u,v,w,r)
#define BEHAVIOR_HOLD			16				//hold (x,y,z,c)
#define BEHAVIOR_HOLDPI			17
#define BEHAVIOR_EMERGENCY		18				//emergency
#define BEHAVIOR_AIM			20				//tracking
#define BEHAVIOR_CFLY			21				//cfly(u,v,w,r)
#define BEHAVIOR_EMERGENCYGROUND	23			//emergency ground
#define BEHAVIOR_ENGINE			24				//engine, defaultly low
#define BEHAVIOR_ENGINEUP		26				//engine up
#define BEHAVIOR_ENGINEDOWN		27				//engine down
#define BEHAVIOR_TAKEOFF		28				//takeoff
#define BEHAVIOR_LAND			29				//land
#define BEHAVIOR_AUXILIARYDOWN	32				//auxiliary down, descending near ground
#define BEHAVIOR_PATHA			33				//patha, more complex path tracking for formation

#define BEHAVIOR_VELTRACK		36			// velocity tracking
#define BEHAVIOR_FORMATION		35

#define BEHAVIOR_DECOUPLEHOVER	19

#define BEHAVIOR_CAMTRACK		34				// camtrack, control uav to keep constant distance with target

#define BEHAVIOR_SEMIAUTO		37

#define CONTROLFLAG_A1		0x00000001				//inner-loop Gain Scheduling, u = F*x
#define CONTROLFLAG_A2		0x00000002				//inner-loop RPT
#define CONTROLFLAG_A3		0x00000004				//inner-loop CNF, u = F*x + G*v + cnf part
#define CONTROLFLAG_A4		0x00000008				//servo driving test, only for behavior_h13
#define CONTROLFLAG_A5		0x00000010				//inner-loop decouple, u = F*x + G*v + cnf part (decoupled)
#define CONTROLFLAG_A6		0x00000020				//inner-loop LQR, u=F*x+G*v (originally A2)
#define CONTROLFLAG_A7		0x00000040				//reserved
#define CONTROLFLAG_A8		0x00000080				//engine up and down
#define CONTROLFLAG_A9		0x00000100				//direct servo position
#define CONTROLFLAG_B1		0x00001000				//reserved
#define CONTROLFLAG_B2		0x00002000				//outer-loop (original B5)
#define CONTROLFLAG_B3		0x00004000				//reserved
#define CONTROLFLAG_B4		0x00008000				//height keeping
#define CONTROLFLAG_B5		0x00010000				//outer-loop trajctory tracking (@)
#define CONTROLFLAG_B6		0x00020000				//chirp signal
#define CONTROLFLAG_OUTERLOOP_QUADLION		0x00040000				// outerloop for QuadLion
#define CONTROLFLAG_B8		0x00080000				//reserved
#define CONTROLFLAG_B9		0x00100000				//head to
#define CONTROLFLAG_B10		0x00200000				//reserved
#define CONTROLFLAG_B11		0x00400000				//outer-loop for aiming control
#define CONTROLFLAG_B12		0x00800000				//outer-loop for velocity feedback control
#define CONTROLFLAG_B13		0x01000000				//outer-loop for heading yawing control
#define CONTROLFLAG_B14		0x02000000				//outer-loop for virtual leader-follower
#define CONTROLFLAG_INNERLOOP_QUADLION		0x04000000				//PI outerloop control, almost same as B5
#define CONTROLFLAG_B16		0x08000000				// cooperative


#define CHIRPADD_OUTERLOOP		1
#define CHIRPADD_CONTROLSIGNAL	2

#define MAX_CTL	128


#define XERRINT_MIN		-24
#define XERRINT_MAX		24
#define YERRINT_MIN		-24
#define YERRINT_MAX		24
#define ZERRINT_MIN		-24
#define ZERRINT_MAX		24
#define CERRINT_MIN		-0.1745		// -10 deg
#define CERRINT_MAX		0.1745		// 10 deg


//#define THROTTLE_LOW		0.8				//engine low, clutch on
#define THROTTLE_LOW		0.92				//engine low, clutch on
#define THROTTLE_ENTRY		0.67			//governer entry
#define THROTTLE_ENTER		0.6				//a point in the governer entry
#define THROTTLE_HIGH		0.075			//governer managed, used for in-air control

#define THROTTLE_SHUTDOWN	-0.75
#define LAND_HEIGHT		0.25		// 0.25m to land
#define MAX_THROTTLE	0.85

#define AUXILIARY_LOW		0.1				//no lifting
#define AUXILIARY_HOVER		-0.227

#define A8MODE_ENGINEUP			1
#define A8MODE_ENGINEDOWN		2
#define A8MODE_AUXILIARYDOWN	3

#define A9FLAG_AILERON		0x0001
#define A9FLAG_ELEVATOR		0x0002
#define A9FLAG_AUXILIARY	0x0004
#define A9FLAG_RUDDER		0x0008
#define A9FLAG_THROTTLE		0x0010
#define A9FLAG_ALL			0xffff

#define MODE_ALLAUTO	1	// all manual channels to semi-auto
#define MODE_SEMIAUTO	2	// all manual channels except throttle to semi-auto
#define MODE_NAVIGATION	3	// purely auto control and navigation

#define MANUAL_DEADZON	0.03
#define MAX_LONSPEED		20
#define MAX_LATSPEED		20
#define MAX_VERSPEED		10
#define MAX_YAWRATE			PI
#define HEADINGERR_DEADZONE 5*PI/180
#define GET_TRIMVALUE	1		// test(1) command
#define RESET_TRIMVALUE 2		// test(2) command
#define WP1_TRACK		3		// waypoint1 tracking
#define WP2_TRACK		4		// waypoint2 tracking
#define WP3_TRACK		5		// waypoint3 tracking
#define UAVFORGE_DST	6

#define SAFMC2014_THROTTLE_LOW 0

#define MAXSIZE_BEHAVIORPARAMETER	128

struct QROTOR_REF    // current input to the inner loop
{
	double p_x_r;
	double p_y_r;
	double p_z_r;
	double v_x_r;
	double v_y_r;
	double v_z_r;
	double agx_r;
	double agy_r;
	double agz_r;
	double psi_r;
	double r_r;
};

struct BEHAVIOR {
	int behavior;
	char parameter[MAXSIZE_BEHAVIORPARAMETER];
};

// NED GPS location
struct LOCATION {
	double latitude;
	double longitude;
	double altitude;
};

// Heading frame location
struct HLOCATION {
	double x;
	double y;
	double z;
};

struct CONTROLSTATE {
	int nBehavior;
	int fControl;

	double u,v,w,r;				//outerloop setting
	double vChirp[4];

	char reserve[128-8*sizeof(double)-2*sizeof(int)];
};				//control setting for inner-loop control


//struct COMMAND;
//struct EQUILIBRIUM;

#define MAX_PLAN	10				//maximum ten plans

class clsPath;

class clsPlan {
public:
	clsPlan();
	virtual ~clsPlan();

protected:
	UAVSTATE m_state;
	int m_planID;

public:
	void SetState(UAVSTATE *pState) { m_state = *pState; }

protected:
	BEHAVIOR m_behavior;

public:
	void GetBehavior(BEHAVIOR *pBehavior) { *pBehavior = m_behavior; }
	int GetPlanID() const { return m_planID; }
	void SetPlanID(int nPlan) { m_planID = nPlan; }

public:
	virtual void Reset() {};
	virtual int Run() = 0;
};

class clsPlan1 : public clsPlan {				//fixed plan
public:
	clsPlan1();
	~clsPlan1();

protected:
	enum { READY, ENGINEUP, TAKEOFF, PATH, LANDING, ENGINEDOWN, END } m_mode;

	double m_pos0[4];

	int m_nPath;				//1,2,3,... index of path

public:
	void Reset();
	int Run();

protected:
	double m_kx, m_ky, m_kz, m_kc;

public:
	void SetKxy(double kx, double ky) { m_kx = kx; m_ky = ky; }
	void SetKz(double kz) { m_kz = kz; }

	void SetPath(int nPath) { m_nPath = nPath; }
};

class clsTakeoffPlan : public clsPlan {
protected:
	double m_x, m_y, m_z, m_c;

public:
	void SetTarget(double x, double y, double z, double c) { m_x = x; m_y = y; m_z = z; m_c = c; }

protected:
	enum { READY, ENGINEUP, ASCEND, HOLD } m_mode;

public:
	void Reset();
	int Run();
};

class clsLandPlan : public clsPlan {
protected:
	double m_x, m_y, m_z, m_c;

public:
	void SetTarget(double x, double y, double z, double c) { m_x = x; m_y = y; m_z = z; m_c = c; }

protected:
	enum { START, DESCEND, ENGINEDOWN, GROUND } m_mode;

public:
	void Reset();
	int Run();
};

/*
 * cls2014SAFMCPlan :: Task management for 2014 SAFMC by LPD, 2014 - March - 05
 */
class cls2014SAFMCPlan : public clsPlan {
protected:
	BOOL m_bTaskFinish;
	enum {READY, ENGINE_UP, TAKEING_OFF, TRANSITION_1, PATH_1, VISION_INITIALIZATION, VISION_GUIDANCE, TASK, ENGINE_UP_AFTER_TASK, TRANSITION_2, PATH_2, LANDING, ENGINE_DOWN, GROUND, HOLD} m_mode;

public:
	void Reset();
	int Run();
};


struct UVWR {
	double u, v, w, r;				//the reference signal for inner-loop control
};

struct SMOOTHPATH {
	double vCruise;
	double acc;
	int nPoints;
	int curPoint;
//	int ntest;
	char waypoints[512];
};

class clsCTL : public clsThread {

	/* Functions and variables defined for 2014 SAFMC by LPD, 2014-March-05*/
private:
	BOOL m_bTakeOffFlag;
	BOOL m_bTransition_1;
	BOOL m_bPath_1;
	BOOL m_bVisionInitialization;
	BOOL m_bVisionGuidance;
	BOOL m_bTaskComplete;
	BOOL m_bTransition2;
	BOOL m_bPath2;
	BOOL m_bLandingFlag;
	double B5_wg0;
	bool m_bSAFMCPathTotalTimeGetted;
	bool m_bSAFMCtargetDropped;
	clsPath *m_pLoadedTextPath;
public:
	void SetSAFMCTargetDropped() {m_bSAFMCtargetDropped = true;}
	void ResetSAFMCTargetDropped() {m_bSAFMCtargetDropped = false;}
	bool GetSAFMCTargetDropped() {return m_bSAFMCtargetDropped;}

	void SetIntegratorFlag() { m_bIntegrator = TRUE; }
	void ResetIntegratorFlag() { m_bIntegrator = FALSE; A2_equ.et = _equ_Hover.et;}
	BOOL GetIntegratorFlag() const { return m_bIntegrator; }

	BOOL GetTakeOffFlag() { return m_bTakeOffFlag;}
	void SetTakeOffFlag() { m_bTakeOffFlag = TRUE; }
	void ResetTakeOffFlag() { m_bTakeOffFlag = FALSE; }

	BOOL GetTransition1Flag() {return m_bTransition_1;}
	void SetTransition1Flag() { m_bTransition_1 = true;}
	void ResetTransition1Flag() { m_bTransition_1 = false;}

	bool GetPath1Flag() {return m_bPath_1; }
	void SetPath1Flag() { m_bPath_1 = true; }
	void ResetPath1Flag() { m_bPath_1 = false; }

	bool GetVisionInitializationFlag() {return m_bVisionInitialization; }
	void SetVisionInitializationFlag() { m_bVisionInitialization = true; }
	void ResetVisionInitializationFlag() { m_bVisionInitialization = false; }

	bool GetVisionGuidanceFlag() {return m_bVisionGuidance; }
	void SetVisionGuidanceFlag() { m_bVisionGuidance = true; }
	void ResetVisionGuidanceFlag() { m_bVisionGuidance = false; }

	bool GetTaskFlag() { return m_bTaskComplete;}
	void SetTaskFlag() { m_bTaskComplete = true; }
	void ResetTaskFlag() { m_bTaskComplete = false; }

	bool GetTransition2Flag() {return m_bTransition2; }
	void SetTransition2Flag() { m_bTransition2 = true; }
	void ResetTransition2Flag() { m_bTransition2 = false; }

	bool GetPath2Flag() {return m_bPath2; }
	void SetPath2Flag() { m_bPath2 = true; }
	void ResetPath2Flag() { m_bPath2 = false; }

	bool GetLandingFlag() {return m_bLandingFlag; }
	void SetLandingFlag() { m_bLandingFlag = true; }
	void ResetLandingFlag() { m_bLandingFlag = false; }

	void ConstructTakeOffPath(UAVSTATE state, double height, /*final height should be negative*/ double pnr[4], double vnr[3], double anr[3]);
	void ConstructTransition1PathRef(double outerRefPos[4], double outerRefVel[3], double outerRefAcc[3]);
	void ConstructPath1Ref(double outerRefPos[4], double outerRefVel[3], double outerRefAcc[3]);
	void ConstructVisionInitializationPathref(double outerRefPos[4], double outerRefVel[3], double outerRefAcc[3]);
	void ConstructVisionGuidancePathRef(double outerRefPos[4], double outerRefVel[3], double outerRefAcc[3]);
	void ConstructTransition2PathRef(double outerRefPos[4], double outerRefVel[3], double outerRefAcc[3]);
	void ConstructPath2Ref(double outerRefPos[4], double outerRefVel[3], double outerRefAcc[3]);
	void ConstructLandingPath(UAVSTATE state, double pnr[4], double vnr[3], double anr[3]);
	void ConstructLandingPath2(UAVSTATE state, double pnr[4], double vnr[3], double anr[3]);
	double GetPathStartWg() {return B5_wg0;}
	/* End of functions defined for 2014 SAFMC by LPD, 2014-March-05*/
	double ReflexxesPathPlanning(UAVSTATE state, QROTOR_REF ref, double pnr[4], double vnr[3], double anr[3]);
public:
	clsCTL();
	virtual ~clsCTL();

	void Init();

public:
	virtual BOOL InitThread();
	virtual int EveryRun();
	virtual void ExitThread();

protected:
	COMMAND m_cmd;
	pthread_mutex_t m_mtxCmd;

public:
	void PutCommand(COMMAND *pCmd);
	void GetCommand(COMMAND *pCmd);
    BOOL SAMFC;
	BOOL ProcessCommand(COMMAND *pCommand);


protected:
	clsPlan *m_pPlan;

	clsPlan1 m_plan1;

	clsTakeoffPlan m_planTakeoff;
	clsLandPlan m_planLand;
	cls2014SAFMCPlan m_plan2014SAFMC;
	clsPath* m_pFlPath, *m_pLdPath;
public:
	void SetPlan(int nPlan);
	int start;
	double tLand;
	double targetx;
	double targety;
	double t_start;
	int ysdd;
	int xy_control;
	int xsdd;
	int z_control;
	int zsdd;
	int SetTrim;

	UAVSTATE  init_state;
	UAVSTATE  take_off_state;
	UAVSTATE  isdfa;
protected:
	double m_tNotify;				//this is used to kept the record of notify command, which is used for datalink check
	BOOL m_bNotify;

	BOOL m_bGPSDeny;
	BOOL m_bUseRefInOuter;
protected:
	//m_behavior to recieve the behavior command, m_behaviorCurrent to indicate the behavior currently being excuted
	BEHAVIOR m_behavior;

	BEHAVIOR m_behaviorCurrent;

	int m_fControl;

	HELICOPTERRUDDER m_sig;				//output control signal
	double m_camera;				//camera pitch angle
	BOOL m_bCamTrack;
	double m_dRudderFactor;

	double m_curX, m_curY, m_curZ;

protected:
	double m_innerloop;				//decide which block is used to implement the inner-loop control
									//so far options include lqr, cnf and decouple
	int m_nPath;	// the path# currently is tracking

	double m_cc; 		// ground frame heading reference for inner loop

	EQUILIBRIUM		m_equ;				//equilibrium for all controls

	//A1 parameter
	void A1_Lookup(double velb[3], EQUILIBRIUM &equ, clsMatrix &F, clsMatrix &G);
	void A1_Lookup(double velb[3], EQUILIBRIUM& equ);

//	EQUILIBRIUM A1_equ0;
//	double _A1_F0[4][11]; clsMatrix A1_F0;
//	double _A1_G0[4][4]; clsMatrix A1_G0;
//
//	EQUILIBRIUM A1_equ;
//
//	double _A1_F[4][11]; clsMatrix A1_F;
//	double _A1_G[4][4]; clsMatrix A1_G;

	//A2 parameter
	EQUILIBRIUM A2_equ;

	// Added by Wang Fei
	EQUILIBRIUM Ave_equ;
	int equ_count;

	double _A2_F[4][11];
	clsMatrix A2_F;

	double _A2_G[4][4];
	clsMatrix A2_G;

	clsPath *A2_pVel;
	BOOL A2_bEnd, A2_bbEnd;
	double A2_t0;

	double _A2_F_RPT[4][9];
	clsMatrix A2_F_RPT;
	double _A2_G_RPT[4][4];
	clsMatrix A2_G_RPT;

	double _A2_F_GREMLION[4][11];
	clsMatrix A2_F_GREMLION;
	double _A2_G_GREMLION[4][4];
	clsMatrix A2_G_GREMLION;

	double _A2_F_GREMLION_NEW[3][8];
	clsMatrix A2_F_GREMLION_NEW;
	double _A2_G_GREMLION_NEW[3][3];
	clsMatrix A2_G_GREMLION_NEW;
	double _A2_TRANS_GREMLION[3][3];
	clsMatrix A2_TRANS_GREMLION;
	//A3 parameter
	EQUILIBRIUM A3_equ;

	double _A3_F[4][11];
	clsMatrix A3_F;

	double _A3_G[4][4];
	clsMatrix A3_G;

	double _A3_N[4][11];
	clsMatrix A3_N;

	double _A3_Ge[11][4];
	clsMatrix A3_Ge;

	double _A3_be[4];
	clsVector A3_be;
	double _A3_al[4];
	clsVector A3_al;

	BOOL A3_bCut;

	//A4 parameter
	EQUILIBRIUM A4_equ;

	double A4_t0;
	double A4_nTest;

	//A5 parameter
	EQUILIBRIUM A5_equ;

	double _A5_F1[2][8]; clsMatrix A5_F1;
	double _A5_G1[2][2]; clsMatrix A5_G1;
	double _A5_K1[2][2]; clsMatrix A5_K1;
	double _A5_L1[2][8]; clsMatrix A5_L1;
	double _A5_M1[2][2]; clsMatrix A5_M1;

	double A5_al1, A5_al2;
	double A5_be1, A5_be2;

	double _A5_P1[2][2]; clsMatrix A5_P1;
	double _A5_Q1[2][2]; clsMatrix A5_Q1;

	double _A5_F4[2][4]; clsMatrix A5_F4;
	double _A5_G4[2][2]; clsMatrix A5_G4;
	double _A5_K4[2][4]; clsMatrix A5_K4;
	double _A5_M4[4][2]; clsMatrix A5_M4;

	double A5_al4;
	double A5_be4;

	//inner-loop control parameter
	double A1A2A3_u;
	double A1A2A3_v;
	double A1A2A3_w;
	double A1A2A3_r;

	double A5_u, A5_v, A5_w, A5_c;

	//A6 parameter
	double _A6_F[4][11];
	clsMatrix A6_F;

	double _A6_G[4][4];
	clsMatrix A6_G;

	//A7 parameter
	EQUILIBRIUM A7_equ;

	double _A7_F[4][11];	clsMatrix A7_F;
	double _A7_G[4][4];		clsMatrix A7_G;

	double _A7_Fr[4];		clsVector A7_Fr;
	double A7_Gr;

	double _A7_Nr[4];		clsVector A7_Nr;
	double _A7_Ger[4];		clsVector A7_Ger;

	double A7_al, A7_be;

	double A7_t0;
	double A7_bOpen;

	double A7_r;
	double A7_er;				//open loop setting

	//A8 parameter
	int A8_mode;

	double A8_t0;
	HELICOPTERRUDDER A8_sig0;

	BOOL A8_bEnd;

	//A9 parameter
	int A9_flag;
	double A9_ea, A9_ee, A9_eu, A9_er, A9_et;

	//outerloop control parameter B1-B9

	//b2
	double B2_t0;
	double B2_x, B2_y, B2_z, B2_c;				//for hover

	double B2_t;

	clsPath *B2_pPath;				//path to track
	int B2_mode;				//path tracking mode
	
	double B2_x0, B2_y0, B2_z0, B2_c0;				//initial position for path tracking
	double B2_kx, B2_ky, B2_kz, B2_kc;				//feedback gains

	double B2_dxi, B2_dyi, B2_dzi, B2_dci;
	double B2_kxi, B2_kyi, B2_kzi, B2_kci;

	BOOL B2_bEnd;

	//b2
	double B4_z;
	double B4_kz;

	//b5 variables
	double B5_t0;
	double B5_semiPsi, B5_semiPsic, B5_PsiErr;
	double B5_t1, B5_t2,B5_t3, B5_t4;;

	double B5_x, B5_y, B5_z, B5_c;					//for hover
	double B5_xref, B5_yref, B5_zref, B5_cref;

	clsPath *B5_pPath, *B5_pPath2;				//B5_pPath2 for additionally addon path, may used in formation
	double B5_tPath;		// the time passed during the path tracking
	int B5_outerloopMode;

	double B5_x0, B5_y0, B5_z0, B5_c0;				//for path tracking, initial position and heading angle
	double B5_kx, B5_ky, B5_kz, B5_kc;				//control parameters
	double B5_adjustc;					// adjustment in ground frame heading tracking purpose

	double B5_t;
	double B5_dxi, B5_dyi, B5_dzi, B5_dci;				//control parameters
	double B5_kxi, B5_kyi, B5_kzi, B5_kci;				//control parameters

	BOOL B5_bEnd;
	int B5_mode;

	double _B5_F[3][6];
	clsMatrix B5_F;
	double _B5_G[3][9];
	clsMatrix B5_G;

	double _B5_CMDGEN_A[2][2]; clsMatrix B5_CMDGEN_A;
	double _B5_CMDGEN_B[2];	clsVector B5_CMDGEN_B;
	double B5_vax[2], B5_vay[2], B5_vaz[2];
	double B5_vac;

	double _B5_F_GREMLION_RPT[3][6]; clsMatrix B5_F_GREMLION_RPT;
	double _B5_G_GREMLION_RPT[3][9]; clsMatrix B5_G_GREMLION_RPT;

	double _B5_Aorg_GREMLION[2][2]; clsMatrix B5_Aorg_GREMLION;
	double _B5_Borg_GREMLION[2]; clsVector B5_Borg_GREMLION;

	double _B5_Aorg2_GREMLION[2][2]; clsMatrix B5_Aorg2_GREMLION;
	double _B5_Aorg3_GREMLION[2][2]; clsMatrix B5_Aorg3_GREMLION;

	double _B5_F_GREMLION[3][6];
	clsMatrix B5_F_GREMLION;
	double _B5_G_GREMLION[3][3];
	clsMatrix B5_G_GREMLION;
	double B5_FVn;

	double B5_pnr[4], B5_vnr[3], B5_anr[3];	// p, v, a ref in NED for GremLion
	BOOL B5_bSemi1stFlag;
	BOOL B6_bChirp;				//if chirp signal is on
	double B6_t0;
	double B6_vChirp[4];
	int B6_add;

	double _OUTER_P_QUADLION[4][7]; clsMatrix OUTER_P_QUADLION;
	double _OUTER_D_QUADLION[4][7]; clsMatrix OUTER_D_QUADLION;
	double _Fxy[5]; clsVector Fxy;
	double _Fz[5]; clsVector Fz;
	double _Fc[3]; clsVector Fc;
	double dc_ail2phi, dc_ele2tht, dc_thr2w, dc_rud2r;
	double damping_u, damping_v, ratio_u, ratio_v;
	double m_xerrint, m_yerrint, m_zerrint, m_cerrint;
	BOOL m_bIntegrator;

	int B6_channel;
	double B6_a;
	double B6_om1;
	double B6_om2;
	double B6_T;

	double B7_x, B7_y, B7_z, B7_c;
	double B7_kx;
	double B7_ky;
	double B7_kz;
	double B7_kc;

	double B8_c;
	double B8_kc;

	double B9_x, B9_y, B9_z;
	double B9_kx;
	double B9_ky;
	double B9_kz;
	double B9_kc;
	double B9_zc;
	double B9_lockNED[3];	// camtrack locked position in NED
	double B9_xLock, B9_yLock, B9_zLock;	// camtrack locked position in body frame

	double B10_t;

	double B10_x, B10_y, B10_z;
	double B10_kx, B10_ky, B10_kz;

	double B10_dxi, B10_dyi, B10_dzi;
	double B10_kxi, B10_kyi, B10_kzi;

	clsPath *B11_pPath;
	int B11_mode;

	double B11_t0;
	double B11_x0, B11_y0, B11_z0;
	double B11_c0;

	double B11_xt, B11_yt, B11_zt;

	double B11_kx, B11_ky, B11_kz, B11_kc;

	double B12_u, B12_v, B12_w, B12_r;
	double B12_ku, B12_kv, B12_kw, B12_kr;

	double B13_t0;

	double B13_x, B13_y, B13_z, B13_c;

	clsPath *B13_pPath;
	int B13_mode;

	double B13_x0, B13_y0, B13_z0, B13_c0;

	double B13_kx, B13_ky, B13_kz;

	double B13_t;
	double B13_dxi, B13_dyi, B13_dzi;				//control parameters
	double B13_kxi, B13_kyi, B13_kzi;				//control parameters

	BOOL B13_bEnd;				//to judge if the path is ended

//B14 virtual leader-follower
	double B14_t0;

	clsPath *B14_pPath;

	double B14_x0, B14_y0, B14_z0, B14_c0;				//for path tracking, initial position and heading angle
	double B14_kx, B14_ky, B14_kz, B14_kc;				//control parameters

	double B14_t;
	double B14_dxi, B14_dyi, B14_dzi, B14_dci;				//control parameters
	double B14_kxi, B14_kyi, B14_kzi, B14_kci;				//control parameters

// B15: B5 added with PI control
	double B15_t0;

	double B15_x, B15_y, B15_z, B15_c;					//for hover

	clsPath *B15_pPath, *B15_pPath2;				//B5_pPath2 for additionally addon path, may used in formation
	int B15_mode;

	double B15_x0, B15_y0, B15_z0, B15_c0;				//for path tracking, initial position and heading angle
	double B15_kx, B15_ky, B15_kz, B15_kc;				//control parameters

	double B15_t;
	double B15_dxi, B15_dyi, B15_dzi, B15_dci;				//control parameters
	double B15_kxi, B15_kyi, B15_kzi, B15_kci;				//control parameters
	BOOL B15_bEnd;

	double B_acxr_ub, B_acyr_vb, B_aczr_wb, B_cr,B_tr,B_lr;
	double laser_angle;
	double _B_psi[3]; 
	double _pva[3][3]; 
	double _F_F[3][3]; clsMatrix F_F;
	double _F_H[1][3]; clsMatrix F_H;
	double _F_H2[2][3]; clsMatrix F_H2;
	double _F_P[3][3]; clsMatrix F_P;
	double _F_P2[3][3]; clsMatrix F_P2;
	double _F_R[1][1]; clsMatrix F_R;
	double _F_R2[2][2]; clsMatrix F_R2;
	double _F_Q[3][3]; clsMatrix F_Q;
	double _F_K[3][1]; clsMatrix F_K;
	double _F_K2[3][2]; clsMatrix F_K2;
	BOOL m_bFK;		// flag for formation kalman
	double m_ldHeading;
	int m_nHeading;
	
	int m_nLandCnt;
	int m_nThrottleMode;
	int m_nThrottleCase;
	BOOL m_bThrottleBypass, m_bThrMode3, m_bThrMode1;
	BOOL m_bLandCmd, m_bLandFlag;


	double _A1_A[11][11]; clsMatrix A1_A;
	double _A1_B[11][6];  clsMatrix A1_B;
	double _A1_C[3][11];  clsMatrix A1_C;
	double _A1_G[3][3];   clsMatrix A1_G;
	double _A1_K[11][3];  clsMatrix A1_K;
	double _A1_x[11];
	double _A1_x_raw[11];

	double A1_ref_a;
	double A1_ref_b;
	double A1_ref_c;
	double A1_throttle;
	EQUILIBRIUM A1_equ;

	//B3 parameter
	double _B3_A[6][6]; clsMatrix B3_A;
	double _B3_B[6][3]; clsMatrix B3_B;
	double _B3_C[3][6]; clsMatrix B3_C;
	double _B3_G[3][9]; clsMatrix B3_G;
	double _B3_K[6][9]; clsMatrix B3_K;
	double _B3_x[6];

	double B2_xref, B2_yref, B2_zref, B2_cref;

	double B2_z_P, B2_z_I, B2_z_D;
	double B2_kx_P, B2_kx_D, B2_kx_I, B2_x_I, B2_c_I;
	double B2_ky_P, B2_ky_D, B2_ky_I, B2_y_I;
	double B2_kz_P, B2_kz_I, B2_kz_D;
	double B2_kc_P, B2_kc_I, B2_kc_D;

	double ref_x, ref_y, ref_z;
	double ref_u, ref_v, ref_w;
	double ref_acx, ref_acy, ref_acz;

	BOOL m_bAutoPath;



	SMOOTHPATH m_smoothPath;
	BOOL m_bPathSmooth, m_bIndoorPath, num1, num2;
	int m_waypointIndex;
protected:
    ReflexxesAPI				*RML;
    RMLPositionInputParameters	*IP;
    RMLPositionOutputParameters	*OP;
    RMLPositionFlags			Flags;
    bool m_ReflexxesInitialized;
public:
	double m_abcRef[3];

protected:
	void A1();				//u = Fx, x = xreal - xref
	void A2();				// H-infinity
	void A2_GremLion();
	void A2_GremLion_New();
	void A2_HeLion();
	void A3();				//CNF
	void A4();				//servo test
	void A5();				//decouple cnf & chip signal
	void A6();				//LQR: u = Fx + Gv; v = (u,v,w,r)
	void A6_GremLion();
	void A6_HeLion();
	void A7();				// Emergency control for FeiLion
	void A8();				//for yaw control
	void A9();				//steady control, fixed input
	void B1();				//
	void B2();				//
	void B3();				//CNF outerloop, with A3
	void B4();				//height keeping, with A2, A1 and A3
	void B5();				//RPT: robust perfect tracking, with A2
	void B5_GremLion();
	void B5_GremLion_SemiAuto();
	void B5_GremLion_SemiAuto_RPT();
	void B5_GremLion_AllAuto();
	void B5_HeLion();
	void B6();				//chirp signal adding
	void B7();				//hold
	void B8();				//yaw angle hold
	void B9();				//head to
	void B10();				//outer loop for decoupled control
	void B11();				//outer-loop for tracking
	void B12();				//outer-loop for cfly
	void B13();				//path-tracking based on A5 (decoupled control)
	void B14();				//yunben's virtual leader
	void B15();				//NEW: PI outloop control law
	void B16();				// cooperative
	void C1();				// FeiLion follow wall
	void C2();				// FeiLion hover at corner

	void Outerloop_QuadLion();
	void Innerloop_QuadLion();

	void SetBehavior(BEHAVIOR *pBeh);				//uniform setbehavior

	void SetBehavior(int nBehavior);				//nBehavior == BEHAVIOR_H1(hover)
	void SetBehavior(int nBehavior, int nTest);		//BEHAVIOR_H13, test
	void SetBehavior(int nBehavior, double d1);				//BEHAVIOR_H2(lift), H3(disecnd)
	void SetBehavior(int nBehavior, double d1, double d2, double d3);				//BEHAVIOR_H4, H7, H8, H9
	void SetBehavior(int nBehavior, double d1, double d2, double d3, double d4);				//BEHAVIOR_H5, H10, H11, H15, H16

	void SetBehavior(int nBehavior, int channel, double a, double T, double om1, double om2);				//BEHAVIOR_H14, chirp
	void SetBehavior(int nBehavior, int nPath, int nMode);				//BEHAVIOR_PATH
	void SetBehavior(int nBehavior, double x, double y, double z, double c, int nPath, int nPath2);				//BEHAVIOR_PATH
	void SetBehavior(int nBehavior, int nPath, int nMode, double xt, double yt, double zt);				//BEHAVIOR_AIM
	void SetBehavior(int nBehavior, double pos1[4], double pos2[4]);				//BEHAVIOR_TAKEOFFX and BEHAVIOR_LANDX

	void AddBehavior(BEHAVIOR *pBehavior);
	void AddBehavior(int nBehavior);
	void AddBehavior(int nBehavior, double para1);

	void AutoPathGeneration();
	void ManualReferenceGeneration();
	void GCSRefGeneration();
	void GremLionThrottleControl(double& velz);

public:
	void B2Para(double kx, double ky, double kz, double kc);
	void B2Parai(double kxi, double kyi, double kzi, double kci);
	void B5Para(unsigned int nChoice);
	void B14CalculateVirtualFollower(double pcl[4], double pcf[4]);				//pcl, position and heading angle of leader, pcf, for follower
//	void CreateSmoothPath(LOCATION *pPath, double vCruise, double acc, int nPath, UAVSTATE *from, LOCATION *pPos0);

public:
	BOOL isGPSDenied() { return m_bGPSDeny; }
	void SetGPSDeny() { m_bGPSDeny = TRUE; }
	void ResetGPSDeny() { m_bGPSDeny = FALSE; }

	BOOL isRefUseInOuter() { return m_bUseRefInOuter; }
	void SetRefUseInOuter() { m_bUseRefInOuter = TRUE; }
	void ResetRefUseInOuter() { m_bUseRefInOuter = FALSE; }

public:
	void SetObserver0();
	void SetObserver1();
	void SetObserver2();

public:
	void FormationKalman(double pos[4], double vel[3], double acc[3], double psider[2]);
public:
	clsPath *GetPath(int nPath);

public:
	int GetMode() const { return B5_outerloopMode; }
	void SetMode(int mode) { B5_outerloopMode = mode; }
	void SetSemiPsi(double psi) { B5_semiPsi = psi; }
	double GetSemiPsi()	{ return B5_semiPsi; }
	BOOL GetThrottleByPassFlag() { return m_bThrottleBypass; }

	void SetPathSmooth() { m_bPathSmooth = TRUE; }
	BOOL IsPathSmooth() { return m_bPathSmooth? TRUE: FALSE; }
	void SetIndoorPath() { m_bIndoorPath = TRUE; }
	void ResetIndoorPath() { m_bIndoorPath = FALSE; }
	BOOL IsIndoorPath() { return m_bIndoorPath? TRUE: FALSE; }
	void ResetPathSmooth() { m_bPathSmooth = FALSE; }
	double GetPathStartTime() { return B5_t0; }
protected:
	int m_nCTL;
	double m_tCTL[MAX_CTL];
	CONTROLSTATE m_ctl[MAX_CTL];				//inner loop signal - {u,v,w,p,q,r,ug,vg,wg}

	pthread_mutex_t m_mtxCTL;

	friend class clsDLG;
	friend class clsCoop;
	friend class clsTmpPath;
};

//path
#define PATHTYPE_DYNAMIC		1
#define PATHTYPE_STATIC			2
#define PATHTYPE_DYNAMICX		3

#define PATHTRACKING_FIXED		0x0001
#define PATHTRACKING_ADDON		0x0002
#define PATHTRACKING_REPEAT		0x0004				//repeat while path is end

#define PATHTYPE_CIRCLE		7
#define PATHTYPE_CIRCLE8	8

struct LINEPATH {
	double ps[4];				//start state, x0, y0, z0, c0
	double pe[4];				//ending state, x1, y1, z3, c1
	double dt;				//delta t
	double v;
	double T;				//v or T is available to indicate average velocity or time
};

class clsPath {
public:
	clsPath();

	clsMatrix m_path;

public:
	BOOL IsEmpty() { return m_path.GetP() == NULL; }

public:
	BOOL LoadPath(char *pszFile);

public:
	UAVSTATE init_state;
	int start_control;
    BOOL SAMFC;
    double tLand;
    void SetSAMFC(){SAMFC = TRUE;}
    void GetPositionVelocity(double time, double pos[4], double vel[4],BOOL bRepeat);
	void GetPositionVelocity1(double time, double pos[4], double vel[4], double acc[3],BOOL bRepeat);
	void GetPosVelAcc(double time, double pos[4], double vel[3], double acc[3]);
	double GetMappedRefTimeOnNorm(double tPath, double ldPos[3]);
	double GetEndTime();

	clsMatrix &GetMatrix() { return m_path; }
};

#define MAX_TMPPATHSIZE		10000

class clsTmpPath : public clsPath {
public:
	clsTmpPath();
	~clsTmpPath();

protected:
//	double m_data[MAX_TMPPATHSIZE][5];
	double m_data[MAX_TMPPATHSIZE][11];

public:
	void CreatePath(LINEPATH *pPath);
	void CreateFixedPath(clsPath *pPath, double pos0[4]);	//create fixed path based on relative path and initial position
	void CreatePath(clsMatrix &mtrx);
	void CreatePath(double mtrx[][5], int m);
	void CreatePath(LOCATION *pPath, int nPath, UAVSTATE *from, LOCATION *pPos0);
	void CreatePath(LOCATION *loc, UAVSTATE *from, LOCATION *loc0);
	void CreateHPath(double hpos[3], int nPath, UAVSTATE *cur, double vel_2d);

	void CreatePath1(LOCATION *loc, UAVSTATE *from, LOCATION *loc0);
	void CreatePath2(LOCATION *loc, UAVSTATE *from, LOCATION *loc0);

	void CreateTakeoffPath(double pos0[4], double pose[4]);
	void CreateLandingPath(double poss[4], double pos0[4]);

	void CreateHoldingPath(double pos[4]);

	void CreateReturnPath(double pos[4]);

	void AddPathPoint(double t, double pos[4]);
	void AddPathPointRPT(double t, double pos[4], double vel[3], double acc[3]);
	void CreateFromGPSPath(clsPath *pPath, double coor[2], double pos[4]);

	void CreateSmoothPath(SMOOTHPATH *pSmoothPath, UAVSTATE *from, LOCATION *pPos0, \
			double outerRefPos[4], double outerRefVel[3], double outerRefAcc[3]);


};


#endif				//CTL_H_
