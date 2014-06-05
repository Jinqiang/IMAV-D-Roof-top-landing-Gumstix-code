//state.h
//this is the header file for helicopter state update and model simulation
#ifndef STATE_H_
#define STATE_H_

#include "uav.h"
#include "cam.h"
#include "svo.h"
#include "im6.h"
#include "im7.h"
#include "im8.h"
#include "im9.h"
#include "ctl.h"
//#include "vision.h"

class clsVision;
//simulation
#define SIMULATIONTYPE_MODEL			1
#define SIMULATIONTYPE_STATE			2
#define SIMULATIONTYPE_DATA				3

//#define _SIMULATIONPERTURBANCE				//simulate the perturbance caused by body trembling and other factor like wind
#define _SIMULATIONODE4				//flag of Runge-Kutta algorithm in simulation

//defines for nonlinear model
#define NONLINEAR_MODEL
#define NONLINEARMODEL_PARAS 	15
#define CONTROL_PARAS			4
#define WIND_PARAS				3

#define MAX_STATE		128				//storage size for uav state

struct NONLINEAR_STATE
{
	double x, y, z;
	double phi, theta, psi;
	double u, v, w;
	double p, q, r;
	double beta_1c, beta_1s, r_f;

	double ug, vg, wg;
	double longitude, latitude, altitude;
};


//NONLINEAR_STATE operator = (NONLINEAR_STATE nonX1);
NONLINEAR_STATE operator + (NONLINEAR_STATE nonX1, NONLINEAR_STATE nonX2);
NONLINEAR_STATE operator * (NONLINEAR_STATE nonX1, double coef);

//filter class is defined to filter position signals of y ans z
//one filter for one channel
class clsFilter {
public:
	clsFilter();

public:
	void Reset() { t = -1; }
	void Update(double tState, double xm, double um);
	double GetPositionEstimation() { return xmfv; }
	double GetVelocityEstimation() { return ue; }

//parameters
protected:
	double Reta;
	double Reps[2][2];

protected:
	double t;				//time
	double xi;				//integration of velocity(measurement)

	double ee;				//error estimation, error = xi - xm
	double bee;				//bias estimation

	double P[2][2];			//covariance matrices
	double Q[2][2];

	double ue;				//velocity estimation (velocity output)

	double xei;				//integration of estimated velocity (ve)
	double xeif;			//filter xei

	double xmf;				//filter xm (measurement)

	double xif;				//filter xi;

	double xmfv;			//verified xmf (position output)
};

struct RPTSTATE
{
	double ug, vg, wg;
	double xg, yg, zg;
	double xn, yn, zn;
};

class clsState {
public:
	clsState();
	~clsState();

	void Init();

public:
	//clsVision m_visionFilter;
protected:
	int m_nCount;				//counter

	//the current state after filtering
	double m_tState0;
	UAVSTATE m_state0;

	//first updated
	double m_tState1;				//corresponding time
	UAVSTATE m_state1;				//state used for filter

	clsFilter m_fx;				//filter for x direction
	clsFilter m_fy;				//filter for y direction
	clsFilter m_fz;				//filter for z direction

//	clsVision m_visionFilter;	// kalman filter for m_state0
	//flag for filter
	BOOL m_bFilter;
	BOOL m_bVision;
	BOOL m_bVisionMeasureUpdate, m_bVisionInterval;
	BOOL m_bStateFromVision;
	CAMTELEINFO m_camInfo;
	double X_n_phi_UAV[4];
	double X_n_theta_UAV[4];
protected:
	double _m_Kalman_A_WF[6][6]; 	clsMatrix m_Kalman_A_WF;
	double _m_Kalman_B_WF[6][3]; 	clsMatrix m_Kalman_B_WF;
	double _m_Kalman_C_WF[3][6]; 	clsMatrix m_Kalman_C_WF;
	double _m_Kalman_Q_WF[3][3]; 	clsMatrix m_Kalman_Q_WF;
	double _m_Kalman_R_WF[3][3]; 	clsMatrix m_Kalman_R_WF;
	double _m_Kalman_P_WF[6][6]; 	clsMatrix m_Kalman_P_WF;
	double _m_BQBt_WF[6][6];		clsMatrix m_BQBt_WF;
	double _m_xVision_WF[6];		clsVector m_xVision_WF;

	double _m_Kalman_A_ZAxis[2][2]; clsMatrix m_Kalman_A_ZAxis;
	double _m_Kalman_B_ZAxis[2][1]; clsMatrix m_Kalman_B_ZAxis;
	double _m_Kalman_C_ZAxis[1][2]; clsMatrix m_Kalman_C_ZAxis;
	double _m_Kalman_Q_ZAxis[1][1]; clsMatrix m_Kalman_Q_ZAxis;
	double _m_Kalman_R_ZAxis[1][1]; clsMatrix m_Kalman_R_ZAxis;
	double _m_Kalman_P_ZAxis[2][2]; clsMatrix m_Kalman_P_ZAxis;
	double _m_BQBt_ZAxis[2][2];		clsMatrix m_BQBt_ZAxis;
	double _m_x_ZAxis[2];		clsVector m_x_ZAxis;

public:
	void EnableFilter(BOOL bFilter = TRUE) {
		m_bFilter = bFilter;
		if (!m_bFilter) {
			m_fx.Reset(); m_fy.Reset(); m_fz.Reset();
		}
	}
	BOOL ToggleFilter() { EnableFilter(!m_bFilter); return m_bFilter; }
	BOOL ToggleVision()	{ EnableVision(!m_bVision); return m_bVision; }

	void EnableVision(BOOL bVision = TRUE)
	{
		m_bVision = bVision;
//		if (!m_bVision)
//			m_visionFilter.SetbFirst();
//			m_visionFilter.Init();
	}
	BOOL GetVisionIntervalFlag() { return m_bVisionInterval; }
	void SetVisionIntervalFlag() { m_bVisionInterval = TRUE; }
	void ResetVisionIntervalFlag() { m_bVisionInterval = FALSE; }
	void SetVisionMeasurementUpdateFlag() { m_bVisionMeasureUpdate = TRUE; }
	BOOL GetVisionMeasurementUpdateFlag() const { return m_bVisionMeasureUpdate; }
	void ResetVisionMeasurementUpdateFlag()	{ m_bVisionMeasureUpdate = FALSE; }

	BOOL GetStateFromVisionFlag() const { return m_bStateFromVision; }
	void SetStateFromVisionFlag() { m_bStateFromVision = TRUE; }

protected:
	//to store state
	double m_tState[MAX_STATE];
	UAVSTATE m_state[MAX_STATE];
	int m_nState;

	// to store vision estimation state
	double m_tVisionState[MAX_STATE];
	//VISIONSTATE m_visionState[MAX_STATE];
	int m_nVisionState;

	pthread_mutex_t m_mtxState, m_mtxVisionState;

	double m_tSIG0;
	HELICOPTERRUDDER m_sig0;

	double m_camera;				//deflection angle of the camera (laser gun)
	//CAM_PANTILT m_camPanTilt;

	int m_nSIG;
	double m_tSIG[MAX_SIG];
	HELICOPTERRUDDER m_sig[MAX_SIG];

	pthread_mutex_t m_mtxSIG;

	EVENT m_event;

public:
	void SetEvent(int code) { m_event.code = code; }
	void SetEvent(int code, int para) { m_event.code = code; (int &)m_event.info[0] = para; }
	void ClearEvent() { m_event.code = 0; }
	EVENT &GetEvent() { return m_event; }

protected:
	BOOL m_bCoordinated;
	double m_longitude0, m_latitude0, m_altitude0, m_pressure0;
	double m_leaderLon0, m_leaderLat0, m_leaderAlt0;	// leader's gps origin position
	BOOL m_bGPSSent;
	BOOL m_bEulerInited;

protected:
	double m_c0;
//	double m_gvel[3], m_gpos[3];
	RPTSTATE m_RPTState, m_RPTState0;
	BOOL m_bMeasureUpdate;
	UAVSTATE m_KalmanState;

public:
	void Setc0(double c) { m_c0 = c; }
	double Getc0()	{ return m_c0; }
	void SetRPTState0();		// set the RPT state: Pg, Vg given the c0 when cmd is issued
	void SetRPTState();
	UAVSTATE &GetState() { return m_state0; }
	RPTSTATE &GetRPTState() { return m_RPTState; }
	RPTSTATE &GetRPTState0()	{ return m_RPTState0; }

	double GetStateTime() { return m_tState0; }
	UAVSTATE &GetStateHeightFusion();	// incorporate the height fusion (from GPS,Ultrasonic) into new UAV state

	UAVSTATE &GetState1() { return m_state1; }				//unfiltered

	void Update(double tPack, IM6PACK *pPack);
	void Update(double tPack, IM7PACK *pPack);
	void Update(double tPack, IM8PACK *pPack);

	void Update1(double tPack, IM6PACK *pPack);
	void Update1(double tPack, IM7PACK *pPack);
	void Update1(double tPack, IM8PACK *pPack);

	void Update(double tPack, IM9PACK *pPack);
	void Update1(double tPack, IM9PACK *pPack);

	void Filter();				//filter processing after update1
//	BOOL VisionFusion();		// data fusion with vision output
	void Coordinate(double longitude, double latitude, double altitude) {
		m_longitude0 = longitude; m_latitude0 = latitude; m_altitude0 = altitude;
	}

	//calculate the distance vector relative to the most current position;
	void CalculateRelativePositionViaGPS(double latitude, double longitude, double deltaPose[2]){
		deltaPose[0] = (latitude-m_state0.latitude)*_radius;
		deltaPose[1] = (longitude-m_state0.longitude)*_radius * cos(latitude);
	}
	void ResetCoordinate(double longitude, double latitude) {
		m_longitude0 = longitude; m_latitude0 = latitude;
	}
	void SetInitPressure(uint32_t pressure0) { m_pressure0 = pressure0; }
	double CalculateHeightViaBaro(double pressure);

	BOOL KalmanTimeUpdate(UAVSTATE &uavstate, const UAVSTATE KalmanState);
	BOOL KalmanMeasurementUpdate(UAVSTATE &uavstate, const UAVSTATE KalmanState);
	BOOL KalmanTimeUpdateZAxis(UAVSTATE &uavstate, const UAVSTATE KalmanState);
	BOOL KalmanMeasurementUpdateZAxis(UAVSTATE &uavstate, const UAVSTATE KalmanState);

	void SetbMeasurementUpdate() { m_bMeasureUpdate = TRUE; }
	void GetCoordination(double *plongitude, double *platitude) {
		*plongitude = m_longitude0; *platitude = m_latitude0;
	}

	void GetCoordination(double *plongitude, double *platitude, double *paltitude) {
		*plongitude = m_longitude0; *platitude = m_latitude0; *paltitude = m_altitude0;
	}

	double GetSIGTime() { return m_tSIG0; }
	HELICOPTERRUDDER &GetSIG() { return m_sig0; }
	void GetSIG(HELICOPTERRUDDER *pSIG) { *pSIG = m_sig0; }

	void UpdateSIG(HELICOPTERRUDDER *pSIG);
	/*
	void SetCameraPanTilt(CAM_PANTILT camPanTilt) { m_camPanTilt = camPanTilt; }
	CAM_PANTILT &GetCamPanTilt() { return m_camPanTilt; }
	void SetCamera(double camera) { m_camera = camera; }
	*/
	double GetCamera() { return m_camera; }

	BOOL Emergency();
	static BOOL Valid(UAVSTATE *pState);				//check the validity of helicopter state

protected: /* parameters for 15 order nonlinear model */
	double rho;            // air density, kg/m^3
	double g;            // gravitational acceleration, m/s^2

	double m;     		// total mass of helicopter, kg
	double OMG_mr;		// angular speed of main rotor, rad/s
	double R_mr;		// effective radius of main rotor, m
	double C_la_mr;		// lift curve slope of main rotor, rad^-1
	int n_mr;			// number of blades
	double c_mr;     	// chord length of blades, m
	double K_col;		// collective pitch gain
	double theta0_col;	// collective pitch offset of main rotor, rad

	double S_fx;          // effective longitudinal drag area, m^2
	double S_fy;          // effective lateral drag area, m^2
	double S_fz;          // effective vertical drag area, m^2

	double H_mr;         // vertical position of main rotor above CG, m
	double K_beta;      // spring constant of main blade, N*m/rad
	double C_d0;          // drag coefficient of main blades

	double D_hf;        // longitudinal position of horizontal fin behind CG, m
//	%H_hf    = 0.197;       // vertical position of horizontal fin above CG, m
	double S_hf;        // effective area of horizontal fin, m^2
	double C_la_hf;        // lift curve slope of horizontal fin, rad^-1

	double R_tr;     // effective radius of tail rotor, m
	double D_tr;     // longtiudinal position of tail rotor behind CG, m
	double H_tr;     // vertical position of tail rotor above CG, m
	int n_tr;         // number of blades
	double c_tr;     // chord length of tail rotor, m
	double C_la_tr;    // lift curve slope, rad^-1
	double theta0_ped;     // triming offset of pedal pitch, rad
	double K_ped;         // gain from servo defelction to pedal pitch

	double D_vf;       // longitudinal posiotn of vertical fin behind CG, m
	double H_vf;       // vertical position of vertical fin above CG, m
	double lmd_vf;            // not exposed to tail rotor wake
	double C_la_vf;       // lift curve slope of the vertical fin, rad^-1
	double S_vf;       // effective area of vertical fin, m^2

	double A_lon;         // linkage gain from elevator servo to blade's pitch
	double C_lon;         // linkage gain from elevator servo to flybar's pitch
	double B_lat;         // linkage gain from aileron servo to blade's pitch
	double D_lat;         // linkage gain from aileron servo to flybar's pitch
	double K_sb;            // contribution from flybar teetering to blade pitch

	double e_mr;       // flapping hinge offset of main rotor
	double I_beta_mr;      // moment of inertia for flapping of main rotor, kg*m^2
	double r_sb;      // inner radius of flybar, m
	double R_sb;      // outer radius of flybar, m
	double c_sb;      // chord lenght of flybar, m
	double I_beta_sb;      // moment of inertia for flapping of flybar, kg*m^2
	double C_la_sb;     // lift curve slope of flybar, rad^-1

	double Ixx;        // rolling moment of inertia, kg*m^2  (real measured value)
	double Iyy;        // pitching moment of inertia, kg*m^2 (scaled based on CIFER)
	double Izz;        // yawing moment of inertia, kg*m^2   (real measured value)
	double Ixz;            // product of inertia, kg*m^2

	double A_mr;
	double A_tr;
	double OMG_tr;

	double Ka, Kp, Ki;         // parameters for yaw stability augmentation


protected:
	double m_tObserve;
	double _m_xe[3];		clsVector m_xe;
	double _m_xe_GREMLION[8]; clsVector m_xe_GREMLION;

	//discrete
	double _m_Ad[3][3];		clsMatrix m_Ad;
	double _m_Bd[3][12];	clsMatrix m_Bd;
	double _m_Cd[3][3];		clsMatrix m_Cd;
	double _m_Dd[3][12];	clsMatrix m_Dd;

	//continuous
public:
	double _m_Ae[3][3];		clsMatrix m_Ae;
	double _m_Be[3][9 /*12*/];	clsMatrix m_Be;
	double _m_Ce[3][3];		clsMatrix m_Ce;
	double _m_De[3][9 /*12*/];	clsMatrix m_De;

	double _m_Ae_GREMLION[8][8];	clsMatrix m_Ae_GREMLION;
	double _m_Be_GREMLION[8][9];	clsMatrix m_Be_GREMLION;
	double _m_Ce_GREMLION[2][11];	clsMatrix m_Ce_GREMLION;

protected:
	double _m_Ae2[3][3];	clsMatrix m_Ae2;					//Ae2 = Ae^2
	double _m_Ae3[3][3];	clsMatrix m_Ae3;					//Ae3 = Ae^3

	double _m_Ae2_GREMLION[8][8];	clsMatrix m_Ae2_GREMLION;		//Ae2 = Ae^2
	double _m_Ae3_GREMLION[8][8];	clsMatrix m_Ae3_GREMLION;		//Ae3 = Ae^3

protected:
	void ObserveC();
public:
	void Observe();
	void Observe_GremLion();
	void Observe_HeLion();
protected:
	double m_tObserve2;
	double _m_xr[4];		clsVector m_xr;

	double _m_Ar[4][4];		clsMatrix m_Ar;					//A matrix for observer
	double _m_Br[4][2];		clsMatrix m_Br;					//B matrix for observer

	double _m_Ar_GREMLION[2][2];	clsMatrix m_Ar_GREMLION;
	double _m_Br_GREMLION[2][13];	clsMatrix m_Br_GREMLION;

	double _m_Ar2[4][4];	clsMatrix m_Ar2;
	double _m_Ar3[4][4];	clsMatrix m_Ar3;				//Ar^3

protected:
	void Observe2();				//yaw channel observation

public:
	clsVector &GetRudderState() { return m_xr; }

protected:
	int m_nSimulationType;
	double m_t0Simulation;

	UAVSTATE m_stateSimulation;
	NONLINEAR_STATE m_nonLinStateSim;

	EQUILIBRIUM m_equ;

	double _m_A[11][11];
	clsMatrix m_A;

	double _m_A_GREMLION[11][11];
	clsMatrix m_A_GREMLION;

	double _m_A2[11][11];
	clsMatrix m_A2;

	double _m_A3[11][11];
	clsMatrix m_A3;

	double _m_B[11][4];
	clsMatrix m_B;

	double _m_B_GREMLION[11][4];
	clsMatrix m_B_GREMLION;

	/*
	 * State space model A, B for QuadLion
	 */
	double _m_A_QUADLION[4][4];
	clsMatrix m_A_QUADLION;
	double _m_B_QUADLION[4][4];
	clsMatrix m_B_QUADLION;

	double _m_A2_QUADLION[4][4];
	clsMatrix m_A2_QUADLION;
	double _m_A3_QUADLION[4][4];
	clsMatrix m_A3_QUADLION;

public:
	int GetSimulationType() { return m_nSimulationType; }
	void SetSimulationType(int nType) { m_nSimulationType = nType; }
	void SetSimulationStartTime(double t0) { m_t0Simulation = t0; }
	void ModelSelect(const double velb[3]);

	void SetEqu(EQUILIBRIUM equ) { m_equ = equ; }
	EQUILIBRIUM GetEqu() const { return m_equ; }

public:
	void Simulate();

protected:
	void ModelSimulate();
	void StateSimulate();
	void DataSimulate();

	void LinearModelSimulate();
	void LinearModelSimulate_GremLion();
	void LinearModelSimulate_HeLion();
	void LinearModelSimulate_QuadLion();
	void NonlinearModelSimulate_QuadLion();
	void NonlinearModelSimulate();

protected:
	void NonLinear15Init();
	void NonLinear15(double xin[15], double u[4], double v[3], double xout[15]);

	friend class clsDLG;

protected:
	double Rand();

protected:
	char *m_pData;				//to store loaded data;
	int m_nData;				//data size
	int m_iData;				//current index

public:
	BOOL LoadData(const char *pszFile);
};


#endif				//STATE_H_

