//state.h
//this is the implementation file for clsState, implementing state update and model simulation
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
using namespace std;

#include "state.h"
#include "ctl.h"
#include "parser.h"
#include "uav.h"
#include "cmm.h"

extern clsParser _parser;
extern clsState _state;
extern clsCMM _cmm;
extern EQUILIBRIUM _equ_Hover;
/*
extern void Quad_model(cv::Mat cmd, cv::Mat &X_n_phi, cv::Mat &X_n_theta, double &y_n_for, double &y_n_lat, double &y_n_heav, double &y_n_head, double &x_pos, double &y_pos, double &z_pos, cv::Mat &ya);
extern void matrixOmegaE(cv::Mat eulers, cv::Mat &OmegaE);
extern void matrixOmega(cv::Mat omegab, cv::Mat &Omega);
extern void matrixBgb(cv::Mat u, cv::Mat &Bgb);
extern void qmodel(cv::Mat u, cv::Mat &y);
extern void Runge_Kutta_Mat(cv::Mat A, cv::Mat B, cv::Mat &X_n, double u, double d_t);
*/
extern void Runge_Kutta(double a, double b, double &x_n, double u, double d_t);

void clsState::UpdateSIG(HELICOPTERRUDDER *pSIG)
{
	m_tSIG0 = GetTime();
	if (pSIG != NULL) m_sig0 = *pSIG;

	pthread_mutex_lock(&m_mtxSIG);
	if (m_nSIG != MAX_SIG) {
		m_tSIG[m_nSIG] = m_tSIG0;
		m_sig[m_nSIG++] = m_sig0;
	}
	pthread_mutex_unlock(&m_mtxSIG);
}

void clsState::Simulate()
{
	if (m_nSimulationType == SIMULATIONTYPE_MODEL)
	{
		ModelSimulate();
	}

	if (m_nSimulationType == SIMULATIONTYPE_STATE)
		StateSimulate();

	if (m_nSimulationType == SIMULATIONTYPE_DATA) {
		if (m_pData != NULL) DataSimulate();
		else StateSimulate();
	}

	//push to record
	pthread_mutex_lock(&m_mtxState);
	if (m_nState != MAX_STATE) {
		m_tState[m_nState] = m_tState0;				//record
		m_state[m_nState++] = m_state0;
	}
	pthread_mutex_unlock(&m_mtxState);

	m_nCount ++;
}

void clsState::DataSimulate()
{
	double t = GetTime();

	m_tState1 = t;

	int nSize = sizeof(double)+sizeof(UAVSTATE);
	m_state1 = (UAVSTATE &)m_pData[m_iData*nSize+8];
	if (m_iData < m_nData-1) m_iData ++;

	Filter();				//enter filter and put filtered result in m_state0

	Observe();				//observation
}

void clsState::StateSimulate()
{
	double t = GetTime();

#if (_DEBUG & DEBUGFLAG_SIMULATION)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[CTL] StateSimulate, time %f, tState0 %f\n", t, m_tState0);
}
#endif

	double t2 = t - m_t0Simulation;

	double SECTION = 5;				//5 second per section

	int section = (int)floor((t2-10)/SECTION);				//empty first 10 second
	double tSection = t2 - section*SECTION;

	double signal = sin(2*PI*tSection);				//1 second per period

	m_tState0 = t;
	memset(&m_state0, 0, sizeof(UAVSTATE));
	m_state0.z = 0;

	switch (section) {
	case 0: m_state0.x = signal;	break;
	case 1: m_state0.y = signal;	break;
	case 2: m_state0.z = signal;	break;
	case 3: m_state0.u = signal;	break;
	case 4: m_state0.v = signal;	break;
	case 5: m_state0.w = signal;	break;
	case 6: m_state0.a = 0.1*signal;	break;
	case 7: m_state0.b = 0.1*signal;	break;
	case 8: m_state0.c = 0.1*signal;	break;
	case 9: m_state0.p = signal;	break;
	case 10: m_state0.q = signal;	break;
	case 11: m_state0.r = signal;	break;
	}

	if (m_state0.a > PI) m_state0.a -= 2*PI;
	else if (m_state0.a < -PI) m_state0.a += 2*PI;

	if (m_state0.b > PI) m_state0.b -= 2*PI;
	else if (m_state0.b < -PI) m_state0.b += 2*PI;

	B2G(&m_state0.a, &m_state0.u, &m_state0.ug);
}

void clsState::NonLinear15Init()
{
	rho = 1.292;            // air density, kg/m^3
	g   = 9.781;            // gravitational acceleration, m/s^2

	m          = 9.750;     // total mass of helicopter, kg
	OMG_mr     = 193.73;    // angular speed of main rotor, rad/s
	R_mr       = 0.705;     // effective radius of main rotor, m
	C_la_mr    = 5.5218;      // lift curve slope of main rotor, rad^-1
	n_mr       = 2;         // number of blades
	c_mr       = 0.062;     // chord length of blades, m
	K_col      = -0.165;    // collective pitch gain
	theta0_col = 0.0750;    // collective pitch offset of main rotor, rad

	S_fx = 0.1026;          // effective longitudinal drag area, m^2
	S_fy = 0.9000;          // effective lateral drag area, m^2
	S_fz = 0.0840;          // effective vertical drag area, m^2

	H_mr   = 0.337;         // vertical position of main rotor above CG, m
	K_beta = 113.7942;      // spring constant of main blade, N*m/rad
	C_d0   = 0.01;          // drag coefficient of main blades

	D_hf    = 0.751;        // longitudinal position of horizontal fin behind CG, m
//	%H_hf    = 0.197;       // vertical position of horizontal fin above CG, m
	S_hf    = 0.011;        // effective area of horizontal fin, m^2
	C_la_hf = 2.865;        // lift curve slope of horizontal fin, rad^-1

	R_tr       = 0.128;     // effective radius of tail rotor, m
	D_tr       = 1.035;     // longtiudinal position of tail rotor behind CG, m
	H_tr       = 0.172;     // vertical position of tail rotor above CG, m
	n_tr       = 2;         // number of blades
	c_tr       = 0.029;     // chord length of tail rotor, m
	C_la_tr    = 2.8214;    // lift curve slope, rad^-1
	theta0_ped = 0.143;     // triming offset of pedal pitch, rad
	K_ped      = 1;         // gain from servo defelction to pedal pitch

	D_vf    = 0.9840;       // longitudinal posiotn of vertical fin behind CG, m
	H_vf    = 0.1840;       // vertical position of vertical fin above CG, m
	lmd_vf  = 0;            // not exposed to tail rotor wake
	C_la_vf = 2.8650;       // lift curve slope of the vertical fin, rad^-1
	S_vf    = 0.0072;       // effective area of vertical fin, m^2

	A_lon   = 0.21;         // linkage gain from elevator servo to blade's pitch
	C_lon   = 0.56;         // linkage gain from elevator servo to flybar's pitch
	B_lat   = 0.20;         // linkage gain from aileron servo to blade's pitch
	D_lat   = 0.57;         // linkage gain from aileron servo to flybar's pitch
	K_sb    = 1;            // contribution from flybar teetering to blade pitch

	e_mr      = 0.07;       // flapping hinge offset of main rotor
	I_beta_mr = 0.055;      // moment of inertia for flapping of main rotor, kg*m^2
	r_sb      = 0.231;      // inner radius of flybar, m
	R_sb      = 0.312;      // outer radius of flybar, m
	c_sb      = 0.059;      // chord lenght of flybar, m
	I_beta_sb = 0.004;      // moment of inertia for flapping of flybar, kg*m^2
	C_la_sb   = 2.7180;     // lift curve slope of flybar, rad^-1

	Ixx = 0.2509;       // rolling moment of inertia, kg*m^2  (real measured value)
	Iyy = 0.5483;       // pitching moment of inertia, kg*m^2 (scaled based on CIFER)
	Izz = 0.7870;       // yawing moment of inertia, kg*m^2   (real measured value)
	Ixz = 0;            // product of inertia, kg*m^2

	A_mr  = PI * R_mr * R_mr;
	A_tr = PI * R_tr * R_tr;
	OMG_tr = OMG_mr * 4.65;

	Ka = -3.85;         // parameters for yaw stability augmentation
	Kp = 0.4177;
	Ki = 2.2076;

}

void clsState::NonLinear15(double xin[15], double u[4], double v[3], double xout[15])
{
	/* parse inputs */
	double delta_col = u[0];
	double delta_lat = u[1];
	double delta_lon = u[2];
	double u_ped     = u[3];

	double vel_bx  = xin[3];
	double vel_by  = xin[4];
	double vel_bz  = xin[5];
	double phi     = xin[6];
	double theta   = xin[7];
	double psi     = xin[8];

	double omg_bx  = xin[9];
	double omg_by  = xin[10];
	double omg_bz  = xin[11];
	double beta_1s = xin[12];
	double beta_1c = xin[13];

	double r_f     = xin[14];

	double u_a, v_a, w_a;
	if (v != NULL)
	{
		u_a = vel_bx - v[0];
		v_a = vel_by - v[1];
		w_a = vel_bz - v[2];
	}
	else
	{
		u_a = vel_bx;
		v_a = vel_by;
		w_a = vel_bz;
	}
	/* main rotor thrust and induced velocity */

	double theta_col = K_col * delta_col + theta0_col;
	double w_mr = u_a * beta_1c - v_a * beta_1s + w_a;
	double w_bd = w_mr + 2 * OMG_mr * R_mr * theta_col / 3;
	double T_mr = m * g;    // initialized for iteration below
	double nu = ( u_a*u_a + v_a*v_a + w_mr*w_mr ) / 2;

	double vi_mr;
	for (int k = 0; k<10; k++)
	{
		vi_mr = sqrt( sqrt( nu*nu + (T_mr/(2*rho*A_mr))*(T_mr/(2*rho*A_mr)) ) - nu );
	    T_mr = (w_bd - vi_mr) * rho * OMG_mr * R_mr*R_mr * C_la_mr * n_mr * c_mr / 4;
	    nu = ( u_a*u_a + v_a*v_a + w_mr*(w_mr - 2*vi_mr) ) / 2 ;
	}

	/* fuslage forces */
	double Xfs = -(rho/2) * S_fx * u_a * max( ::fabs(u_a), vi_mr );
	double Yfs = -(rho/2) * S_fy * v_a * max( ::fabs(v_a), vi_mr );
	double Zfs = -(rho/2) * S_fz * (w_a - vi_mr) * (::fabs(w_a - vi_mr));

	/* main rotor forces and moments */
	double Xmr = -T_mr * sin(beta_1c);
	double Ymr =  T_mr * sin(beta_1s);
	double Zmr = -T_mr * cos(beta_1c) * cos(beta_1s);
	double Lmr =  Ymr * H_mr + K_beta * beta_1s;
	double Mmr = -Xmr * H_mr + K_beta * beta_1c;
	double Nmr = -( (rho/2) * C_d0 * n_mr * c_mr * (R_mr / 4) * OMG_mr * R_mr \
	      * ( OMG_mr*OMG_mr * R_mr*R_mr + 4.6 * ( u_a*u_a + v_a*v_a ) ) \
	      + T_mr * vi_mr + m * g * ::fabs(min(w_a,0)) \
	      - Xfs * u_a - Yfs * v_a - Zfs * (w_a - vi_mr) ) / OMG_mr;

	/* horizontal fin forces and moments */
	double w_hf = w_a + omg_by * D_hf - vi_mr;   // local vertical airspeed
	double Zhf;
	if ( fabs(w_hf) <= 0.3*::fabs(u_a) )
	    Zhf = -(rho/2) * S_hf * w_hf * fabs(u_a) * C_la_hf;
	else
	    Zhf = -(rho/2) * S_hf * w_hf * fabs(w_hf);
	double Mhf = Zhf * D_hf;


	/* tail rotor forces and moments */
	double delta_ped = Ki*r_f + Kp * ( Ka * u_ped - omg_bz );
	double theta_ped = K_ped * delta_ped + theta0_ped;
	double v_tr = v_a - omg_bz * D_tr + omg_bx * H_tr;
	double v_bd = v_tr + 2 * OMG_tr * R_tr * theta_ped / 3;
	double T_tr = -Nmr / D_tr;    // initialized for iteration below
	double nu1 = u_a*u_a + v_tr*v_tr + (w_a + omg_by * D_tr)*(w_a + omg_by * D_tr);
	double vi_tr;
	for (int k=0; k<10; k++)
	{
	    vi_tr = sqrt( sqrt( nu1*nu1 + (T_tr / (2*rho*A_tr))*(T_tr / (2*rho*A_tr)) ) - nu1 );
	    T_tr = (v_bd - vi_tr) * rho * OMG_tr * R_tr*R_tr * C_la_tr * n_tr * c_tr / 4;
	    nu1 = (u_a*u_a + v_tr * (v_tr - 2*vi_tr) + (w_a + omg_by * D_tr)*(w_a + omg_by * D_tr)) / 2;
	}

	double Ytr = -T_tr;
	double Ltr =  Ytr * H_tr;
	double Ntr = -Ytr * D_tr;

	/* vertical fin forces and moments (this part need be improved) */
	double v_vf = v_a - omg_bz * D_vf + omg_bx * H_vf - lmd_vf * vi_tr;
	double Yvf;
	if ( fabs(v_vf) <= 0.3*fabs(u_a) )
	    Yvf = -(rho/2) * S_vf * v_vf * fabs(u_a) * C_la_vf;
	else
	    Yvf = -(rho/2) * S_vf * v_vf * fabs(v_vf);
	double Lvf =  Yvf * H_vf;
	double Nvf = -Yvf * D_vf;

	/* resulted forces and moments along body axes */
	double F_bx = Xmr + Xfs;
	double F_by = Ymr + Yfs + Ytr + Yvf;
	double F_bz = Zmr + Zfs + Zhf;
	double M_bx = Lmr + Ltr + Lvf;
	double M_by = Mmr + Mhf;
	double M_bz = Nmr + Ntr + Nvf;

	/* main rotor flapping dynamics (blades lumped with stabilizer bars) */
	double gm_m  = rho * C_la_mr * c_mr * R_mr*R_mr*R_mr*R_mr / I_beta_mr;
	double tau_m = (16 / ( gm_m * OMG_mr )) / ( 1 - (8 * e_mr / 3) / R_mr );
	double gm_s  = rho * C_la_sb * c_sb * (R_sb*R_sb*R_sb*R_sb -r_sb*r_sb*r_sb*r_sb) / I_beta_sb;
	double tau_s = 16 / (gm_s*OMG_mr);
	double Ab    = 8 * K_beta / ( gm_m * OMG_mr*OMG_mr * I_beta_mr );
	double Ba    = - Ab;
	double tau   = tau_m + tau_s;
	double dbeta_1c = -beta_1c/tau - (( tau_m + K_sb * tau_s ) / tau) * omg_by \
	     + ((A_lon + K_sb * C_lon) / tau) * delta_lon + (tau_m * Ab / tau) * beta_1s;
	double dbeta_1s = -beta_1s/tau - (( tau_m + K_sb * tau_s ) / tau) * omg_bx \
	     + (( B_lat + K_sb * D_lat ) / tau) * delta_lat + (tau_m * Ba / tau) * beta_1c;


	/* Rigid body dynamics
	 coordinate transform from body frame to ground grame */
//	dpos    = Rgb * [vel_bx; vel_by; vel_bz];

	double abc[3] = {phi, theta, psi};
	double velb[3] = {vel_bx, vel_by, vel_bz};
	double dpos[3] = {0};
	B2G(abc, velb, dpos);	// dpos = vel_g


	double dphi   = omg_bx + sin(phi)*tan(theta)*omg_by + 	cos(phi)*tan(theta)*omg_bz;
	double dtheta = 		 cos(phi)*omg_by 			+ 	(-sin(phi))*omg_bz;
	double dpsi   = 		 sin(phi)*(1/cos(theta))*omg_by + 	cos(phi)*(1/cos(theta))*omg_bz;

	double dvel_bx = F_bx / m - g * sin(theta) 			- omg_by*vel_bz + omg_bz*vel_by;
	double dvel_by = F_by / m + g * sin(phi)*cos(theta) - omg_bz*vel_bx + omg_bx*vel_bz;
	double dvel_bz = F_bz / m + g * cos(phi)*cos(theta) - omg_bx*vel_by + omg_by*vel_bx;

	double domg_bx = ( M_bx*Izz + M_bz*Ixz + Ixz * ( Ixx-Iyy+Izz ) * omg_bx * omg_by + \
			( Iyy*Izz - Ixz*Ixz - Izz*Izz ) * omg_by * omg_bz ) / ( Ixx*Izz - Ixz*Ixz );

	double domg_by = ( M_by - ( Ixx-Izz ) * omg_bx * omg_bz - Ixz * ( omg_bx*omg_bx - omg_bz*omg_bz ) ) / Iyy;

	double domg_bz = ( M_bx*Ixz + M_bz*Ixx - Ixz * ( Ixx-Iyy+Izz ) * omg_by * omg_bz \
	          + ( Ixx*Ixx - Ixx*Iyy + Ixz*Ixz ) * omg_bx * omg_by )  / ( Ixx*Izz - Ixz*Ixz );

	/* Yaw Stability Augmentation */
	double dr_f = Ka * u_ped - omg_bz;

	xout[0] = dpos[0]; xout[1] = dpos[1]; xout[2] = dpos[2];
	xout[3] = dvel_bx; xout[4] = dvel_by; xout[5] = dvel_bz;
	xout[6] = dphi;  xout[7] = dtheta; xout[8] = dpsi;

	xout[9] = domg_bx; xout[10] = domg_by; xout[11] = domg_bz;
	xout[12] = dbeta_1s; xout[13] = dbeta_1c;  xout[14] = dr_f;

}

void clsState::NonlinearModelSimulate()
{
	double t = GetTime();

	if (m_tState0 < 0) {
		::memset(&m_nonLinStateSim, 0, sizeof(m_nonLinStateSim));
		/* trim values given the hover condition */
		NONLINEAR_STATE nonX0 = {0, 0, 0, 0.0389, 0.000887, 0, 0, 0, 0, 0, 0, 0, -0.000874, 0.0048, 0,
				0,0,0, 0,0,0};
		m_nonLinStateSim = nonX0;

		m_sig0.aileron = m_equ.ea;
		m_sig0.elevator = m_equ.ee;
		m_sig0.auxiliary = m_equ.eu;
		m_sig0.rudder = m_equ.er;
		m_sig0.throttle = m_equ.et;

		m_tState0 = t;
		return;
	}

	double dt = t - m_tState0;
	m_tState0 = t;

#if (_DEBUG & DEBUGFLAG_CTL)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[CTL] state(1)%.3g %.3g %.3g\n",
		m_stateSimulation.u, m_stateSimulation.v, m_stateSimulation.w);
}
#endif

	double xout0[15], xout1[15], xout2[15], xout3[15], xout4[15];

	xout0[0] = m_nonLinStateSim.x;
	xout0[1] = m_nonLinStateSim.y;
	xout0[2] = m_nonLinStateSim.z;
	xout0[3] = m_nonLinStateSim.u /*- m_equ.a*/;
	xout0[4] = m_nonLinStateSim.v /*- m_equ.b*/;
	xout0[5] = m_nonLinStateSim.w /*- m_equ.c*/;
	xout0[6] = m_nonLinStateSim.phi /*- m_equ.u*/;
	xout0[7] = m_nonLinStateSim.theta /*- m_equ.v*/;
	xout0[8] = m_nonLinStateSim.psi /*- m_equ.w*/;
	xout0[9] = m_nonLinStateSim.p /*- m_equ.p*/;
	xout0[10]= m_nonLinStateSim.q /*- m_equ.q*/;
	xout0[11] = m_nonLinStateSim.r /*- m_equ.r*/;
	xout0[12]= m_nonLinStateSim.beta_1s /*- m_equ.as*/;
	xout0[13] = m_nonLinStateSim.beta_1c /*- m_equ.bs*/;
	xout0[14] = m_nonLinStateSim.r_f /*- m_equ.rfb*/;


	double nonU[4] = {
			m_sig0.auxiliary,
			m_sig0.aileron,
			m_sig0.elevator,
			m_sig0.rudder,
	};

	/* iteration 1 */
	NonLinear15(xout0, nonU, NULL, xout1);

	/* iteration 2 */
	double xin2[15];
	for (int i=0; i<15; i++)
	{
		xin2[i] = xout0[i] + xout1[i] * dt/2;
	}
	NonLinear15(xin2, nonU, NULL, xout2);

	/* iteration 3 */
	double xin3[15];
	for (int i=0; i<15; i++)
	{
		xin3[i] = xout0[i] + xout2[i] * dt/2;
	}
	NonLinear15(xin3, nonU, NULL, xout3);

	/* iteration 4 */
	double xin4[15];
	for (int i=0; i<15; i++)
	{
		xin4[i] = xout0[i] + xout3[i] * dt;
	}
	NonLinear15(xin4, nonU, NULL, xout4);

	/* iteration 5 */
	double xout[15];
	for (int i=0; i<15; i++)
	{
		xout[i] = xout0[i] + dt/6 * (xout1[i] + 2*xout2[i] + 2*xout3[i] + xout4[i]);
	}

	m_nonLinStateSim.x = xout[0]; 		m_nonLinStateSim.y = xout[1]; 	m_nonLinStateSim.z = xout[2];
	m_nonLinStateSim.u = xout[3]; 		m_nonLinStateSim.v = xout[4];	m_nonLinStateSim.w = xout[5];
	m_nonLinStateSim.phi = xout[6]; 	m_nonLinStateSim.theta = xout[7]; m_nonLinStateSim.psi = xout[8];

	m_nonLinStateSim.p = xout[9];		m_nonLinStateSim.q = xout[10]; 	m_nonLinStateSim.r = xout[11];
	m_nonLinStateSim.beta_1s = xout[12]; m_nonLinStateSim.beta_1c = xout[13]; 	m_nonLinStateSim.r_f = xout[14];

	B2G(&m_nonLinStateSim.phi, &m_nonLinStateSim.u, &m_nonLinStateSim.ug);

	/*	if (m_nCount % 50 == 0) {
		printf("[SIM] state %.3g %.3g %.3g\n", xout[3], xout[4], xout[5]);
		printf("[SIM] state %.3g %.3g %.3g\n", m_nonLinStateSim.x, m_nonLinStateSim.y, m_nonLinStateSim.z);
	}*/

	m_nonLinStateSim.longitude = m_nonLinStateSim.y / _radius + _state.m_longitude0;
	m_nonLinStateSim.latitude = m_nonLinStateSim.x / _radius + _state.m_latitude0;

	//	m_nonLinStateSim.psi += m_nonLinStateSim.r*dt;
	if (m_nonLinStateSim.psi > PI) m_nonLinStateSim.psi -= 2*PI;
	else if (m_nonLinStateSim.psi < -PI) m_nonLinStateSim.psi += 2*PI;

	m_state0.x = m_nonLinStateSim.x; m_state0.y = m_nonLinStateSim.y; m_state0.z = m_nonLinStateSim.z;
	m_state0.a = m_nonLinStateSim.phi; m_state0.b = m_nonLinStateSim.theta; m_state0.c = m_nonLinStateSim.psi;
	m_state0.u = m_nonLinStateSim.u; m_state0.v = m_nonLinStateSim.v; m_state0.w = m_nonLinStateSim.w;
	m_state0.p = m_nonLinStateSim.p; m_state0.q = m_nonLinStateSim.q; m_state0.r = m_nonLinStateSim.r;
	m_state0.longitude = m_nonLinStateSim.longitude;  m_state0.latitude = m_nonLinStateSim.latitude;
	m_state0.as = m_nonLinStateSim.beta_1c; m_state0.bs = m_nonLinStateSim.beta_1s; m_state0.rfb = m_nonLinStateSim.r_f;
	m_state0.ug = m_nonLinStateSim.ug; m_state0.vg = m_nonLinStateSim.vg; m_state0.wg = m_nonLinStateSim.wg;

	INPI(m_state0.a);
	INPI(m_state0.b);
	INPI(m_state0.c);

	SetRPTState();
}

void clsState::LinearModelSimulate_GremLion()
{
	double t = GetTime();

	if (m_tState0 < 0) {
		::memset(&m_stateSimulation, 0, sizeof(m_stateSimulation));
		m_stateSimulation.u = m_equ.u;
		m_stateSimulation.v = m_equ.v;
		m_stateSimulation.w = m_equ.w;
		m_stateSimulation.p = m_equ.p;
		m_stateSimulation.q = m_equ.q;
		m_stateSimulation.r = m_equ.r;
		m_stateSimulation.a = m_equ.a;
		m_stateSimulation.b = m_equ.b;
		m_stateSimulation.c = PI/4; //m_equ.c;
		m_stateSimulation.as = m_equ.as;
		m_stateSimulation.bs = m_equ.bs;
//		m_stateSimulation.rfb = m_equ.rfb;

//		m_stateSimulation.z = -15;
		m_latitude0 = 31.98895094869658*PI/180;
		m_longitude0 = -81.84823803870367*PI/180;
//		m_latitude0 = 32.01108333*PI/180;
//		m_longitude0 = -81.83261111*PI/180;
		m_altitude0 = 0;
		m_stateSimulation.latitude = m_latitude0;
		m_stateSimulation.longitude = m_longitude0;
		m_stateSimulation.altitude = m_latitude0;

		m_sig0.aileron = 0;//m_equ.ea;
		m_sig0.elevator = 0;//m_equ.ee;
		m_sig0.auxiliary = 0;//m_equ.eu;
		m_sig0.rudder = 0;//m_equ.er;
		m_sig0.throttle = 0;//m_equ.et;

		m_tState0 = t;
//		m_tState0 = m_tState1 = t;
		return;
	}
	double dt = t - m_tState0;
	m_tState0 = t;
//	m_tState0 = m_tState1 = t;

	m_stateSimulation.x += m_stateSimulation.ug*dt;
	m_stateSimulation.y += m_stateSimulation.vg*dt;
	m_stateSimulation.z += m_stateSimulation.wg*dt;

	m_stateSimulation.longitude = m_stateSimulation.y / (_radius*cos(m_latitude0)) + m_longitude0;
	m_stateSimulation.latitude = m_stateSimulation.x / _radius + m_latitude0;

//	m_stateSimulation.c += m_stateSimulation.r*dt;
//	INPI(m_stateSimulation.c);

	EQUILIBRIUM equ = GetEqu();
	double _u[4] = {
		m_sig0.aileron - equ.ea,
		m_sig0.elevator - equ.ee,
		m_sig0.throttle - equ.et,
		m_sig0.rudder - equ.er
	};
#if (_DEBUG & DEBUGFLAG_CTL)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[SIM] equ %.3g %.3g %.3g %.3g\n",
		equ.ea, equ.ee, equ.et, equ.er);
}
#endif


	clsVector u(4, _u, TRUE);

	double _x[11] = {
		m_stateSimulation.u,// - m_equ.u,
		m_stateSimulation.v,// - m_equ.v,
		m_stateSimulation.p,// - m_equ.p,
		m_stateSimulation.q,// - m_equ.q,
		m_stateSimulation.a,// - m_equ.a,
		m_stateSimulation.b,// - m_equ.b,
		m_stateSimulation.as,// - m_equ.as,
		m_stateSimulation.bs,// - m_equ.bs,
		m_stateSimulation.w,// - m_equ.w,
		m_stateSimulation.r,// - m_equ.r,
		m_stateSimulation.c,// - m_equ.c
	};
	clsVector x(11, _x, TRUE);

	double _Ax[11]; clsVector Ax(11, _Ax, TRUE);
	double _Bu[11]; clsVector Bu(11, _Bu, TRUE);

	clsMatrix::X(m_A_GREMLION, x, Ax);
	clsMatrix::X(m_B_GREMLION, u, Bu);

	double _dx0[11]; clsVector dx0(11, _dx0, TRUE);
	dx0 = Ax; dx0 += Bu;

	double _dx[11]; clsVector dx(11, _dx, TRUE);
	dx = dx0;

#ifdef _SIMULATIONODE4
	if (dt < 1) {				//only apply when dt is less than 1 second, otherwise the result will fly away
		//Runge-Kutta approximation
		double _Ah[11][11]; clsMatrix Ah(11,11,(double *)_Ah, TRUE);
		double _Ah2[11][11]; clsMatrix Ah2(11,11,(double *)_Ah2, TRUE);
		double _Ah3[11][11]; clsMatrix Ah3(11,11,(double *)_Ah3, TRUE);

		Ah = m_A_GREMLION; Ah *= dt/2;
		Ah2 = m_A2; Ah2 *= dt*dt/6;
		Ah3 = m_A3; Ah3 *= dt*dt*dt/24;

		double _Ah123[11][11]; clsMatrix Ah123(11,11,(double*)_Ah123,TRUE);
		Ah123 = Ah; Ah123 += Ah2; Ah123 += Ah3;

		double _dx123[11]; clsVector dx123(11,_dx123,TRUE);
		clsMatrix::X(Ah123,dx0,dx123);
		dx += dx123;
	}
#endif

	dx *= dt;

	m_stateSimulation.u += _dx[0];
	m_stateSimulation.v += _dx[1];
	m_stateSimulation.p += _dx[2];
	m_stateSimulation.q += _dx[3];
	m_stateSimulation.a += _dx[4];
	m_stateSimulation.b += _dx[5];
	m_stateSimulation.as += _dx[6];
	m_stateSimulation.bs += _dx[7];
	m_stateSimulation.w += _dx[8];
	m_stateSimulation.r += _dx[9];
	m_stateSimulation.c += _dx[10];

	B2G(&m_stateSimulation.a, &m_stateSimulation.u, &m_stateSimulation.ug);

	m_state0 = m_stateSimulation;

	INPI(m_state0.a);
	INPI(m_state0.b);
	INPI(m_state0.c);
}

void clsState::LinearModelSimulate_HeLion()
{
	double t = GetTime();

	if (m_tState0 < 0) {
		::memset(&m_stateSimulation, 0, sizeof(m_stateSimulation));
		m_stateSimulation.u = m_equ.u;
		m_stateSimulation.v = m_equ.v;
		m_stateSimulation.w = m_equ.w;
		m_stateSimulation.p = m_equ.p;
		m_stateSimulation.q = m_equ.q;
		m_stateSimulation.r = m_equ.r;
		m_stateSimulation.a = m_equ.a;
		m_stateSimulation.b = m_equ.b;
		m_stateSimulation.c = m_equ.c;
		m_stateSimulation.as = m_equ.as;
		m_stateSimulation.bs = m_equ.bs;
		m_stateSimulation.rfb = m_equ.rfb;

		m_sig0.aileron = m_equ.ea;
		m_sig0.elevator = m_equ.ee;
		m_sig0.auxiliary = m_equ.eu;
		m_sig0.rudder = m_equ.er;
		m_sig0.throttle = m_equ.et;

		m_tState0 = t;
		return;
	}

	double dt = t - m_tState0;
	m_tState0 = t;

#if (_DEBUG & DEBUGFLAG_CTL)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[CTL] state(1)%.3g %.3g %.3g\n",
		m_stateSimulation.u, m_stateSimulation.v, m_stateSimulation.w);
}
#endif
	m_stateSimulation.x += m_stateSimulation.ug*dt;
	m_stateSimulation.y += m_stateSimulation.vg*dt;
	m_stateSimulation.z += m_stateSimulation.wg*dt;

	m_stateSimulation.c += m_stateSimulation.r*dt;
	INPI(m_stateSimulation.c);

	double _u[4] = {
		m_sig0.aileron - m_equ.ea,
		m_sig0.elevator - m_equ.ee,
		m_sig0.auxiliary - m_equ.eu,
		m_sig0.rudder - m_equ.er
	};
	clsVector u(4, _u, TRUE);

	double _x[11] = {
		m_stateSimulation.u - m_equ.u,
		m_stateSimulation.v - m_equ.v,
		m_stateSimulation.p - m_equ.p,
		m_stateSimulation.q - m_equ.q,
		m_stateSimulation.a - m_equ.a,
		m_stateSimulation.b - m_equ.b,
		m_stateSimulation.as - m_equ.as,
		m_stateSimulation.bs - m_equ.bs,
		m_stateSimulation.w - m_equ.w,
		m_stateSimulation.r - m_equ.r,
		m_stateSimulation.rfb - m_equ.rfb
	};
	clsVector x(11, _x, TRUE);

	double _Ax[11]; clsVector Ax(11, _Ax, TRUE);
	double _Bu[11]; clsVector Bu(11, _Bu, TRUE);

	clsMatrix::X(m_A, x, Ax);
	clsMatrix::X(m_B, u, Bu);

	double _dx0[11]; clsVector dx0(11, _dx0, TRUE);
	dx0 = Ax; dx0 += Bu;

	double _dx[11]; clsVector dx(11, _dx, TRUE);
	dx = dx0;

#ifdef _SIMULATIONODE4
	if (dt < 1) {				//only apply when dt is less than 1 second, otherwise the result will fly away
		//Runge-Kutta approximation
		double _Ah[11][11]; clsMatrix Ah(11,11,(double *)_Ah, TRUE);
		double _Ah2[11][11]; clsMatrix Ah2(11,11,(double *)_Ah2, TRUE);
		double _Ah3[11][11]; clsMatrix Ah3(11,11,(double *)_Ah3, TRUE);
	
		Ah = m_A; Ah *= dt/2;
		Ah2 = m_A2; Ah2 *= dt*dt/6;
		Ah3 = m_A3; Ah3 *= dt*dt*dt/24;
	
		double _Ah123[11][11]; clsMatrix Ah123(11,11,(double*)_Ah123,TRUE);
		Ah123 = Ah; Ah123 += Ah2; Ah123 += Ah3;
	
		double _dx123[11]; clsVector dx123(11,_dx123,TRUE);
		clsMatrix::X(Ah123,dx0,dx123);
		dx += dx123;
	}
#endif

	dx *= dt;

	m_stateSimulation.u += _dx[0];
	m_stateSimulation.v += _dx[1];
	m_stateSimulation.p += _dx[2];
	m_stateSimulation.q += _dx[3];
	m_stateSimulation.a += _dx[4];
	m_stateSimulation.b += _dx[5];
	m_stateSimulation.as += _dx[6];
	m_stateSimulation.bs += _dx[7];
	m_stateSimulation.w += _dx[8];
	m_stateSimulation.r += _dx[9];
	m_stateSimulation.rfb += _dx[10];

	B2G(&m_stateSimulation.a, &m_stateSimulation.u, &m_stateSimulation.ug);

	m_state0 = m_stateSimulation;

#ifdef _SIMULATIONPERTURBANCE
	//add perturbance
	//generate pseudo-random scalor between (-1,1)
	double dRand = Rand();
	m_state0.p += 0.04/0.8*dRand;

	dRand = Rand();
	m_state0.q += 0.025/0.8*dRand;

	dRand = Rand();
	m_state0.r += 0.07/0.8*dRand;

	dRand = Rand();
	m_state0.u += 0.05 + 0.1/0.8*dRand;

	dRand = Rand();
	m_state0.v += -0.03 + 0.1/0.8*dRand;

	dRand = Rand();
	m_state0.w += -0.11 + 0.08/0.8*dRand;
#endif

#if (_DEBUG & DEBUGFLAG_CTL)
if (m_nCount % _DEBUG_COUNT == 0) {
printf("[SIM] state(2)%.3g %.3g %.3g\n",
	m_stateSimulation.u, m_stateSimulation.v, m_stateSimulation.w);
}
#endif

	INPI(m_state0.a);
	INPI(m_state0.b);
	INPI(m_state0.c);

	SetRPTState();
}

void clsState::NonlinearModelSimulate_QuadLion()
{
	double t = GetTime();

	if (m_tState0 < 0) {
//		SetFormationInitialStateAll();
		::memset(&m_stateSimulation, 0, sizeof(m_stateSimulation));
		m_stateSimulation.acz = -_gravity;

		m_latitude0 = 1.29844*PI/180;
		m_longitude0 = 103.80729*PI/180;
		m_altitude0 = 0;
		m_stateSimulation.latitude = m_latitude0;
		m_stateSimulation.longitude = m_longitude0;
		m_stateSimulation.altitude = m_latitude0;

//		m_stateSimulation.y = -2;
		m_stateSimulation.z = -5;
		m_stateSimulation.c = 0; //double(160)/double(180)*PI;//PI/3;
		memset(X_n_phi_UAV, 0, 4*sizeof(double));
		memset(X_n_theta_UAV, 0, 4*sizeof(double));
	}
	double dt = t - m_tState0;
	m_tState0 = t;

	double cmd_UAV[4] = {m_sig0.aileron - m_equ.ea, m_sig0.elevator - m_equ.ee, m_sig0.rudder - m_equ.er, m_sig0.throttle - m_equ.et};
//	double X_n_phi_UAV[4] = {0};
//	double X_n_theta_UAV[4] = {0};
	double y_n_for_UAV = m_stateSimulation.u; double y_n_lat_UAV = m_stateSimulation.v; double y_n_heav_UAV = m_stateSimulation.w;
	double y_n_head_UAV = m_stateSimulation.c;
	double x_pos_UAV = m_stateSimulation.x; double y_pos_UAV = m_stateSimulation.y; double z_pos_UAV = m_stateSimulation.z;
	double ya_UAV[18] = {
			m_stateSimulation.u, m_stateSimulation.v, m_stateSimulation.w,
			m_stateSimulation.p, m_stateSimulation.q, m_stateSimulation.r,
			m_stateSimulation.a, m_stateSimulation.b, m_stateSimulation.c,
			m_stateSimulation.x, m_stateSimulation.y, m_stateSimulation.z,
			m_stateSimulation.ug, m_stateSimulation.vg, m_stateSimulation.wg,
			m_stateSimulation.acx, m_stateSimulation.acy, m_stateSimulation.acz
	};
	/*
	Mat matcmd_UAV = Mat(4, 1, CV_64F, cmd_UAV);
	Mat matX_n_phi_UAV = Mat(4, 1, CV_64F, X_n_phi_UAV);
	Mat matX_n_theta_UAV = Mat(4, 1, CV_64F, X_n_theta_UAV);
	Mat matya_UAV = Mat(18, 1, CV_64F, ya_UAV);
	Quad_model(matcmd_UAV, matX_n_phi_UAV, matX_n_theta_UAV, y_n_for_UAV, y_n_lat_UAV, y_n_heav_UAV, y_n_head_UAV,
			x_pos_UAV, y_pos_UAV, z_pos_UAV, matya_UAV);
//	printf("matya %.3f, %.3f, %.3f \n", matya_UAV.at<double>(9,0), matya_UAV.at<double>(10,0), matya_UAV.at<double>(11,0));

	m_stateSimulation.u = matya_UAV.at<double>(0,0); m_stateSimulation.v = matya_UAV.at<double>(1,0); m_stateSimulation.w = matya_UAV.at<double>(2,0);
	m_stateSimulation.p = matya_UAV.at<double>(3,0); m_stateSimulation.q = matya_UAV.at<double>(4,0); m_stateSimulation.r = matya_UAV.at<double>(5,0);
	m_stateSimulation.a = matya_UAV.at<double>(6,0); m_stateSimulation.b = matya_UAV.at<double>(7,0); m_stateSimulation.c = matya_UAV.at<double>(8,0);
	m_stateSimulation.x = matya_UAV.at<double>(9,0); m_stateSimulation.y = matya_UAV.at<double>(10,0); m_stateSimulation.z = matya_UAV.at<double>(11,0);
	m_stateSimulation.ug = matya_UAV.at<double>(12,0); m_stateSimulation.vg = matya_UAV.at<double>(13,0); m_stateSimulation.wg = matya_UAV.at<double>(14,0);
	m_stateSimulation.acx = matya_UAV.at<double>(15,0); m_stateSimulation.acy = matya_UAV.at<double>(16,0); m_stateSimulation.acz = matya_UAV.at<double>(17,0);
	*/
	m_stateSimulation.longitude = m_stateSimulation.y / (_radius*cos(m_latitude0)) + m_longitude0;
	m_stateSimulation.latitude = m_stateSimulation.x / _radius + m_latitude0;

	double agg[3] = {0};
	B2G(&m_stateSimulation.a, &m_stateSimulation.acx, agg);
	agg[2] = agg[2] + _gravity;
	m_stateSimulation.acx = agg[0]; m_stateSimulation.acy = agg[1]; m_stateSimulation.acz = agg[2];
	INPI(m_stateSimulation.c);
	m_state0 = m_stateSimulation;
}

void clsState::LinearModelSimulate_QuadLion()
{
	double t = GetTime();

	if (m_tState0 < 0) {
		::memset(&m_stateSimulation, 0, sizeof(m_stateSimulation));
		m_stateSimulation.u = m_equ.u;
		m_stateSimulation.v = m_equ.v;
		m_stateSimulation.w = m_equ.w;
		m_stateSimulation.p = m_equ.p;
		m_stateSimulation.q = m_equ.q;
		m_stateSimulation.r = m_equ.r;
		m_stateSimulation.a = m_equ.a;
		m_stateSimulation.b = m_equ.b;
		m_stateSimulation.c = 0; //m_equ.c;
		m_stateSimulation.as = m_equ.as;
		m_stateSimulation.bs = m_equ.bs;
//		m_stateSimulation.rfb = m_equ.rfb;

		m_stateSimulation.z = 0;
		m_latitude0 = 52.14258*PI/180;
		m_longitude0 = 5.83996*PI/180;
		m_altitude0 = 0;
		m_stateSimulation.latitude = m_latitude0;
		m_stateSimulation.longitude = m_longitude0;
		m_stateSimulation.altitude = m_latitude0;

		m_sig0.aileron = _equ_Hover.ea;
		m_sig0.elevator = _equ_Hover.ee;
		m_sig0.auxiliary = _equ_Hover.eu;
		m_sig0.rudder = _equ_Hover.er;
		m_sig0.throttle = _equ_Hover.et;

		m_tState0 = t;
		return;
	}

	double dt = t - m_tState0;
	m_tState0 = t;

	m_stateSimulation.x += m_stateSimulation.ug*dt;
	m_stateSimulation.y += m_stateSimulation.vg*dt;
	m_stateSimulation.z += m_stateSimulation.wg*dt;

	m_stateSimulation.longitude = m_stateSimulation.y / (_radius*cos(m_latitude0)) + m_longitude0;
	m_stateSimulation.latitude = m_stateSimulation.x / _radius + m_latitude0;

	m_stateSimulation.c += m_stateSimulation.r*dt;
	INPI(m_stateSimulation.c);

	EQUILIBRIUM equ = GetEqu();
	double _u[4] = {
		m_sig0.aileron - _equ_Hover.ea,
		m_sig0.elevator - _equ_Hover.ee,
		m_sig0.throttle - _equ_Hover.et,
		m_sig0.rudder - _equ_Hover.er
	};
#if (_DEBUG & DEBUGFLAG_CTL)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[SIM] equ %.3g %.3g %.3g %.3g\n",
		equ.ea, equ.ee, equ.et, equ.er);
}
#endif

	clsVector u(4, _u, TRUE);

	double _x[4] = {
		m_stateSimulation.u,
		m_stateSimulation.v,
		m_stateSimulation.w,
//		m_stateSimulation.r,
		m_stateSimulation.c,
	};
	clsVector x(4, _x, TRUE);

	double _Ax[4]; clsVector Ax(4, _Ax, TRUE);
	double _Bu[4]; clsVector Bu(4, _Bu, TRUE);

	clsMatrix::X(m_A_QUADLION, x, Ax);
	clsMatrix::X(m_B_QUADLION, u, Bu);

	double _dx0[4]; clsVector dx0(4, _dx0, TRUE);
	dx0 = Ax; dx0 += Bu;

	double _dx[4]; clsVector dx(4, _dx, TRUE);
	dx = dx0;

#ifdef _SIMULATIONODE4
	if (dt < 1) {				//only apply when dt is less than 1 second, otherwise the result will fly away
		//Runge-Kutta approximation
		double _Ah[4][4]; clsMatrix Ah(4,4,(double *)_Ah, TRUE);
		double _Ah2[4][4]; clsMatrix Ah2(4,4,(double *)_Ah2, TRUE);
		double _Ah3[4][4]; clsMatrix Ah3(4,4,(double *)_Ah3, TRUE);

		Ah = m_A_QUADLION; Ah *= dt/2;
		Ah2 = m_A2_QUADLION; Ah2 *= dt*dt/6;
		Ah3 = m_A3_QUADLION; Ah3 *= dt*dt*dt/24;

		double _Ah123[4][4]; clsMatrix Ah123(4,4,(double*)_Ah123,TRUE);
		Ah123 = Ah; Ah123 += Ah2; Ah123 += Ah3;

		double _dx123[4]; clsVector dx123(4,_dx123,TRUE);
		clsMatrix::X(Ah123,dx0,dx123);
		dx += dx123;
	}
#endif

	dx *= dt;

	m_stateSimulation.u += _dx[0];
	m_stateSimulation.v += _dx[1];
	m_stateSimulation.w += _dx[2];
	m_stateSimulation.c += _dx[3];

	B2G(&m_stateSimulation.a, &m_stateSimulation.u, &m_stateSimulation.ug);
	m_state0 = m_stateSimulation;

	INPI(m_state0.a);
	INPI(m_state0.b);
	INPI(m_state0.c);

}
void clsState::LinearModelSimulate()
{
	if ( _HELICOPTER == ID_GREMLION ) {
		LinearModelSimulate_GremLion();
	}
	else if ( _HELICOPTER == ID_HELION || _HELICOPTER == ID_SHELION ) {
		LinearModelSimulate_HeLion();
	}
	else if (_HELICOPTER == ID_QUADLION) {
		LinearModelSimulate_QuadLion();
	//	NonlinearModelSimulate_QuadLion();
	}
}

void clsState::ModelSimulate()
{
//	BOOL bNonlinearModel = TRUE;
	BOOL bNonlinearModel = FALSE;
	if (bNonlinearModel) {
		NonlinearModelSimulate();
	}
	else {
		LinearModelSimulate();
	}
}

void clsState::SetRPTState0()
{
	double abc0[3] = {0, 0, m_c0};
	double posn[3] = {m_state0.x, m_state0.y, m_state0.z};
	N2G(abc0, posn, &m_RPTState0.xg);

	double veln[3] = {m_state0.ug, m_state0.vg, m_state0.wg};
	N2G(abc0, veln, &m_RPTState0.ug);

	m_RPTState0.xn = posn[0]; m_RPTState0.yn = posn[1]; m_RPTState0.zn = posn[2];
}

void clsState::SetRPTState()
{
	double abc0[3] = {0, 0, m_c0};
	double posn[3] = {m_state0.x, m_state0.y, m_state0.z};
	N2G(abc0, posn, &m_RPTState.xg);

	double veln[3] = {m_state0.ug, m_state0.vg, m_state0.wg};
	N2G(abc0, veln, &m_RPTState.ug);
}

/*NONLINEAR_STATE operator = (NONLINEAR_STATE nonX1)
{
	NONLINEAR_STATE nonX;
	nonX.x = nonX1.x;
	nonX.y = nonX1.y;
	nonX.z = nonX1.z;

	nonX.phi = nonX1.phi;
	nonX.theta = nonX1.theta;
	nonX.psi = nonX1.psi;

	nonX.u = nonX1.u;
	nonX.v = nonX1.v;
	nonX.w = nonX1.w;

	nonX.p = nonX1.p;
	nonX.q = nonX1.q;
	nonX.r = nonX1.r;

	nonX.beta_1c = nonX1.beta_1c;
	nonX.beta_1s = nonX1.beta_1s;
	nonX.r_f = nonX1.r_f;

	return nonX;
}*/

NONLINEAR_STATE operator + (NONLINEAR_STATE nonX1, NONLINEAR_STATE nonX2)
{
	NONLINEAR_STATE nonX;
	nonX.x = nonX1.x + nonX2.x;
	nonX.y = nonX1.y + nonX2.y;
	nonX.z = nonX1.z + nonX2.z;

	nonX.phi = nonX1.phi + nonX2.phi;
	nonX.theta = nonX1.theta + nonX2.theta;
	nonX.psi = nonX1.psi + nonX2.psi;

	nonX.u = nonX1.u + nonX2.u;
	nonX.v = nonX1.v + nonX2.v;
	nonX.w = nonX1.w + nonX2.w;

	nonX.p = nonX1.p + nonX2.p;
	nonX.q = nonX1.q + nonX2.q;
	nonX.r = nonX1.r + nonX2.r;

	nonX.beta_1c = nonX1.beta_1c + nonX2.beta_1c;
	nonX.beta_1s = nonX1.beta_1s + nonX2.beta_1s;
	nonX.r_f = nonX1.r_f + nonX2.r_f;

	return nonX;
}

NONLINEAR_STATE operator * ( NONLINEAR_STATE nonX1, double coef)
{
	NONLINEAR_STATE nonX;
	nonX.x = nonX1.x * coef;
	nonX.y = nonX1.y * coef;
	nonX.z = nonX1.z * coef;

	nonX.phi = nonX1.phi * coef;
	nonX.theta = nonX1.theta * coef;
	nonX.psi = nonX1.psi * coef;

	nonX.u = nonX1.u * coef;
	nonX.v = nonX1.v * coef;
	nonX.w = nonX1.w * coef;

	nonX.p = nonX1.p * coef;
	nonX.q = nonX1.q * coef;
	nonX.r = nonX1.r * coef;

	nonX.beta_1c = nonX1.beta_1c * coef;
	nonX.beta_1s = nonX1.beta_1s * coef;
	nonX.r_f = nonX1.r_f * coef;

	return nonX;
}

void clsState::ModelSelect(const double velb[3])
{
	double ub = ::fabs(velb[0]);
	char szA[16], szB[16];
	int nCondition = 0;
	if (ub <= 2)
		nCondition = 0;
	else if (ub > 2 && ub <= 4)
		nCondition = 1;
	else if (ub > 4 && ub <= 6)
		nCondition = 2;
	else if (ub > 6 && ub <= 8)
		nCondition = 3;
	else if (ub > 8 && ub <=10)
		nCondition = 4;
	else if (ub > 10)
		nCondition = 5;

	::sprintf(szA,"_A_Model%d", nCondition);
	::sprintf(szB,"_B_Model%d", nCondition);
//	::sprintf(szEqu,"_equ%d", nCondition);
	_parser.GetVariable(szA, m_A);
	_parser.GetVariable(szB, m_B);

//	_parser.GetVariable(szEqu, &m_equ, sizeof(EQUILIBRIUM));
//	cout<<"Condition: "<<nCondition<<endl;
}

double clsState::Rand()
{
	double dRand = (double)rand()/RAND_MAX*2-1;
	double dRand2 = dRand*dRand;
	if (dRand < 0) dRand2 = -dRand2;

	return dRand2;
}


static double _xc[3] = { 0, 0, 0 };
static clsVector xc(3, _xc, TRUE);

void clsState::ObserveC()
{
	if (m_tObserve < 0) {
		m_tObserve = m_tState0;
		m_state0.as = m_state0.bs = m_state0.rfb = 0;
		m_xe = (double)0;
		return;
	}

	double _u[12] = {
		m_state0.u, m_state0.v, m_state0.p, m_state0.q,
		m_state0.a, m_state0.b, m_state0.w, m_state0.r,
		m_sig0.aileron-m_equ.ea, m_sig0.elevator-m_equ.ee,
		m_sig0.auxiliary-m_equ.eu, m_sig0.rudder-m_equ.er
	};
	clsVector u(12, _u, TRUE);

	double _Ax[3]; clsVector Ax(3, _Ax, TRUE);
	double _Bu[3]; clsVector Bu(3, _Bu, TRUE);

	clsMatrix::X(m_Ad, m_xe, Ax);
	clsMatrix::X(m_Bd, u, Bu);

	m_xe = Ax; m_xe += Bu;

	double _Cx[3]; clsVector Cx(3, _Cx, TRUE);
	clsMatrix::X(m_Cd, m_xe, Cx);

	double _Du[3]; clsVector Du(3, _Du, TRUE);
	clsMatrix::X(m_Dd, u, Du);

	m_state0.as = _Cx[0] + _Du[0];
	m_state0.bs = _Cx[1] + _Du[1];
	m_state0.rfb = _Cx[2] + _Du[2];
}

void clsState::Observe_GremLion()
{
	if (m_tObserve < 0) {
		m_tObserve = m_tState0;
		m_state0.as = m_state0.bs = 0;
		m_xe_GREMLION = (double)0;
		return;
	}

	double dt = m_tState0 - m_tObserve;
	m_tObserve = m_tState0;

	double _u[9] = {
		m_state0.a, m_state0.b, m_state0.c,
		m_state0.p, m_state0.q, m_state0.r,
//		m_sig0.aileron, m_sig0.elevator, m_sig0.rudder
		_ctl.m_abcRef[0], _ctl.m_abcRef[1], _ctl.m_abcRef[2]
	};
	clsVector u(9, _u, TRUE);

	double _Ax[8]; clsVector Ax(8, _Ax, TRUE);
	double _Bu[8]; clsVector Bu(8, _Bu, TRUE);

	clsMatrix::X(m_Ae_GREMLION, m_xe_GREMLION, Ax);
	clsMatrix::X(m_Be_GREMLION, u, Bu);

	double _dx0[8]; clsVector dx0(8, _dx0, TRUE);
	dx0 = Ax; dx0 += Bu;

	double _dx[8]; clsVector dx(8, _dx, TRUE);
	dx = dx0;

//#ifdef _SIMULATIONODE4
//Runge-Kutta approximation
	double _Ah[8][8]; clsMatrix Ah(8,8,(double *)_Ah, TRUE);
	double _Ah2[8][8]; clsMatrix Ah2(8,8,(double *)_Ah2, TRUE);
	double _Ah3[8][8]; clsMatrix Ah3(8,8,(double *)_Ah3, TRUE);

	Ah = m_Ae_GREMLION; Ah *= dt/2;
	Ah2 = m_Ae2_GREMLION; Ah2 *= dt*dt/6;
	Ah3 = m_Ae3_GREMLION; Ah3 *= dt*dt*dt/24;

	double _Ah123[8][8]; clsMatrix Ah123(8,8,(double*)_Ah123,TRUE);
	Ah123 = Ah; Ah123 += Ah2; Ah123 += Ah3;

	double _dx123[8]; clsVector dx123(8,_dx123,TRUE);
	clsMatrix::X(Ah123, dx0, dx123);

	dx += dx123;
//#endif
	dx *= dt;

	m_xe_GREMLION += dx;

	m_state0.as = _m_xe_GREMLION[7];
	m_state0.bs = _m_xe_GREMLION[6];
	m_state0.as = range(m_state0.as, MIN_STATE_AS, MAX_STATE_AS);
	m_state0.bs = range(m_state0.bs, MIN_STATE_BS, MAX_STATE_BS);

}

/*
void clsState::Observe_GremLion()
{
	if (m_tObserve < 0) {
		m_tObserve = m_tState0;
		m_state0.as = m_state0.bs = m_state0.rfb = 0;
		m_xe_GREMLION = (double)0;
		return;
	}

	double dt = m_tState0 - m_tObserve;
	m_tObserve = m_tState0;

	double _u[13] = {
		m_state0.u, m_state0.v,
		m_state0.p, m_state0.q,
		m_state0.a, m_state0.b,
		m_state0.w, m_state0.r,  m_state0.c, // - m_c0,
		m_sig0.aileron, m_sig0.elevator, m_sig0.auxiliary, m_sig0.rudder
	};
	clsVector u(13, _u, TRUE);

	double _Ax[2]; clsVector Ax(2, _Ax, TRUE);
	double _Bu[2]; clsVector Bu(2, _Bu, TRUE);

	clsMatrix::X(m_Ae_GREMLION, m_xe_GREMLION, Ax);
	clsMatrix::X(m_Be_GREMLION, u, Bu);

	double _dx0[2]; clsVector dx0(2, _dx0, TRUE);
	dx0 = Ax; dx0 += Bu;

	double _dx[2]; clsVector dx(2, _dx, TRUE);
	dx = dx0;

//#ifdef _SIMULATIONODE4
//Runge-Kutta approximation
	double _Ah[2][2]; clsMatrix Ah(2,2,(double *)_Ah, TRUE);
	double _Ah2[2][2]; clsMatrix Ah2(2,2,(double *)_Ah2, TRUE);
	double _Ah3[2][2]; clsMatrix Ah3(2,2,(double *)_Ah3, TRUE);

	Ah = m_Ae_GREMLION; Ah *= dt/2;
	Ah2 = m_Ae2_GREMLION; Ah2 *= dt*dt/6;
	Ah3 = m_Ae3_GREMLION; Ah3 *= dt*dt*dt/24;

	double _Ah123[2][2]; clsMatrix Ah123(2,2,(double*)_Ah123,TRUE);
	Ah123 = Ah; Ah123 += Ah2; Ah123 += Ah3;

	double _dx123[2]; clsVector dx123(2,_dx123,TRUE);
	clsMatrix::X(Ah123, dx0, dx123);

	dx += dx123;
//#endif

	dx *= dt;

	m_xe_GREMLION += dx;

	double _Cx[2]; clsVector Cx(2, _Cx, TRUE);
	double _mxeGremLion[11] = {
			_m_xe_GREMLION[0], _m_xe_GREMLION[1],
			m_state0.u, m_state0.v,
			m_state0.p, m_state0.q,
			m_state0.a, m_state0.b,
			m_state0.w, m_state0.r,  m_state0.c // - m_c0
	};
	clsVector mxeGremLion(11, _mxeGremLion, TRUE);
	clsMatrix::X(m_Ce_GREMLION, mxeGremLion, Cx);

	m_state0.as = _Cx[0];
	m_state0.bs = _Cx[1];
	m_state0.as = range(m_state0.as, MIN_STATE_AS, MAX_STATE_AS);
	m_state0.bs = range(m_state0.bs, MIN_STATE_BS, MAX_STATE_BS);
}
*/

void clsState::Observe_HeLion()
{
	if (m_tObserve < 0) {
		m_tObserve = m_tState0;
		m_state0.as = m_state0.bs = m_state0.rfb = 0;
		m_xe = (double)0;
		return;
	}

	double dt = m_tState0 - m_tObserve;
	m_tObserve = m_tState0;

	if (dt > 0.05) dt = 0;				//abnormal, package lose, keep original value, otherwise, RK4 yields extreme large values

/*	double _u[12] = {
		m_state0.u, m_state0.v, m_state0.p, m_state0.q,
		m_state0.a, m_state0.b, m_state0.w, m_state0.r,
		m_sig0.aileron-m_equ.ea, m_sig0.elevator-m_equ.ee,
		m_sig0.auxiliary-m_equ.eu, m_sig0.rudder-m_equ.er
	};
	clsVector u(12, _u, TRUE);*/

	double _u[9] = {
		m_sig0.aileron, m_sig0.elevator, m_sig0.rudder,
		m_state0.a, m_state0.b, m_state0.c - m_c0,
		m_state0.p, m_state0.q, m_state0.r
	};
	clsVector u(9, _u, TRUE);

	double _Ax[3]; clsVector Ax(3, _Ax, TRUE);
	double _Bu[3]; clsVector Bu(3, _Bu, TRUE);

	clsMatrix::X(m_Ae, m_xe, Ax);
	clsMatrix::X(m_Be, u, Bu);

	double _dx0[3]; clsVector dx0(3, _dx0, TRUE);
	dx0 = Ax; dx0 += Bu;

	double _dx[3]; clsVector dx(3, _dx, TRUE);
	dx = dx0;

//#ifdef _SIMULATIONODE4
//Runge-Kutta approximation
	double _Ah[3][3]; clsMatrix Ah(3,3,(double *)_Ah, TRUE);
	double _Ah2[3][3]; clsMatrix Ah2(3,3,(double *)_Ah2, TRUE);
	double _Ah3[3][3]; clsMatrix Ah3(3,3,(double *)_Ah3, TRUE);

	Ah = m_Ae; Ah *= dt/2;
	Ah2 = m_Ae2; Ah2 *= dt*dt/6;
	Ah3 = m_Ae3; Ah3 *= dt*dt*dt/24;

	double _Ah123[3][3]; clsMatrix Ah123(3,3,(double*)_Ah123,TRUE);
	Ah123 = Ah; Ah123 += Ah2; Ah123 += Ah3;

	double _dx123[3]; clsVector dx123(3,_dx123,TRUE);
	clsMatrix::X(Ah123, dx0, dx123);

	dx += dx123;
//#endif

	dx *= dt;

	m_xe += dx;

	double _Cx[3]; clsVector Cx(3, _Cx, TRUE);
	clsMatrix::X(m_Ce, m_xe, Cx);

	double _Du[3]; clsVector Du(3, _Du, TRUE);
	clsMatrix::X(m_De, u, Du);

	//y = C*x+D*u
	m_state0.bs = _Cx[0] + _Du[0];
	m_state0.as = _Cx[1] + _Du[1];
	m_state0.rfb = _Cx[2] + _Du[2];
	m_state0.as = range(m_state0.as, MIN_STATE_AS, MAX_STATE_AS);
	m_state0.bs = range(m_state0.bs, MIN_STATE_BS, MAX_STATE_BS);
	m_state0.rfb = range(m_state0.rfb, MIN_STATE_RFB, MAX_STATE_RFB);
}

void clsState::Observe()
{
	if ( _HELICOPTER == ID_GREMLION ) {
		Observe_GremLion();
	}
	else if ( _HELICOPTER == ID_HELION || _HELICOPTER == ID_SHELION ) {
		Observe_HeLion();
	}
}

void clsState::Observe2()
{
	if (m_tObserve2 < 0) {
		m_tObserve2 = m_tState0;
		m_xr = (double)0;
	}

	double dt = m_tState0 - m_tObserve2;
	m_tObserve2 = m_tState0;

	double _u[2] = { m_sig0.rudder-m_equ.er, m_state0.r };
	clsVector u(2, _u, TRUE);

	double _Ax[4]; clsVector Ax(4, _Ax, TRUE);
	double _Bu[4]; clsVector Bu(4, _Bu, TRUE);

	clsMatrix::X(m_Ar, m_xr, Ax);
	clsMatrix::X(m_Br, u, Bu);

	double _dx0[4]; clsVector dx0(4, _dx0, TRUE);
	dx0 = Ax; dx0 += Bu;

	double _dx[4]; clsVector dx(4, _dx, TRUE);
	dx = dx0;

//#ifdef _SIMULATIONODE4
//Runge-Kutta approximation
	double _Ah[4][4]; clsMatrix Ah(4,4,(double *)_Ah, TRUE);
	double _Ah2[4][4]; clsMatrix Ah2(4,4,(double *)_Ah2, TRUE);
	double _Ah3[4][4]; clsMatrix Ah3(4,4,(double *)_Ah3, TRUE);

	Ah = m_Ar; Ah *= dt/2;
	Ah2 = m_Ar2; Ah2 *= dt*dt/6;
	Ah3 = m_Ar3; Ah3 *= dt*dt*dt/24;

	double _Ah123[4][4]; clsMatrix Ah123(4,4,(double*)_Ah123,TRUE);
	Ah123 = Ah; Ah123 += Ah2; Ah123 += Ah3;

	double _dx123[4]; clsVector dx123(4,_dx123,TRUE);
	clsMatrix::X(Ah123, dx0, dx123);

	dx += dx123;
	//dx = (Ax+Bu)(dt+(A*dt)^2/2+(A*dt)^3/6+(A*dt)^4/24)
	//dx = (Ax+Bu)*dt*(1+(A*dt)/2+(A*dt)^2/6+(A*dt)^3/24)

//#endif

	dx *= dt;
	m_xr += dx;
}


clsState::clsState()
{
	m_nCount = 0;

	m_tState0 = -1;
	m_tState1 = -1;

	m_c0 = 0;
	::memset(&m_RPTState, 0, sizeof(m_RPTState));
	::memset(&m_state1, 0, sizeof(m_state1));
	::memset(&m_state0, 0, sizeof(m_state0));				//all set to zero
	m_bFilter = FALSE; //TRUE;
	m_bVision = FALSE;
	m_bStateFromVision = FALSE;
	m_nState = 0;

	m_bCoordinated = FALSE;
	m_bEulerInited = FALSE;

	m_tSIG0 = -1;
	m_nSIG = 0;

	::memset(&m_event, 0, sizeof(m_event));

	m_camera = 0;

	m_nSimulationType = 0;				//no simulation

	NonLinear15Init();

	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);

	pthread_mutex_init(&m_mtxState, &attr);
	pthread_mutex_init(&m_mtxSIG, &attr);
}

clsState::~clsState()
{
	if (m_pData != NULL) ::free(m_pData);
}

BOOL clsState::LoadData(const char *pszFile)
{
	FILE *pfData = ::fopen(pszFile, "rb");
	if (pfData == NULL) return FALSE;

	::fseek(pfData, 0, SEEK_END);
	long length = ::ftell(pfData);

	int nSize = sizeof(double) + sizeof(UAVSTATE);
	int nData = length/(nSize-3*sizeof(double));
	//in "two.dat", only 25 variables are recorded, no as, bs and rfb in the end

	::rewind(pfData);
	m_pData = (char *)::malloc(nSize*nData);
	int iData = 0;

	double t;
	UAVSTATE state = {0};
	while (1) {
		if (::fread(&t, sizeof(double), 1, pfData) != 1) break;
		if (::fread(&state, sizeof(UAVSTATE)-3*sizeof(double), 1, pfData) != 1) break;				//as, bs and rfb is missed in two.dat
		(double &)m_pData[iData*nSize] = t;
		(UAVSTATE &)m_pData[iData*nSize+8] = state;
		iData ++;
	}
	m_nData = iData;

	::fclose(pfData);
	return TRUE;
}

void clsState::Update1(double tPack, IM6PACK *pPack)
{
	double dt = m_tState1 < 0 ? 0 : tPack - m_tState1;

	double posIM6[3], posAntenna[3];
	_im6.GetIM6Position(posIM6);
	_im6.GetAntennaPosition(posAntenna);

	double xa = posAntenna[0]; double ya = posAntenna[1]; double za = posAntenna[2];
	double xi = posIM6[0]; double yi = posIM6[1]; double zi = posIM6[2];

	switch (pPack->gp6.type) {
	case DATA_GP6: {
	if ( !_ctl.isGPSDenied() )
	{
		GP6PACK &gp6 = pPack->gp6;

		m_state1.p = gp6.p; m_state1.q = gp6.q; m_state1.r = gp6.r;

		m_state1.a = gp6.a; m_state1.b = gp6.b;
		double dc = gp6.c - m_state1.c;
		INPI(dc);
		m_state1.c += dc /*/(1+rFilter)*/;
		INPI(m_state1.c);

		double cg = gp6.c - m_c0;
		INPI(cg);

		double abc0[3] = {/*gp6.a*/ 0, /*gp6.b*/ 0, m_c0};
		double abcg[3] = {gp6.a, gp6.b, cg};
		//verify
		//relationship between signal and actual value -
		//signal = real + complementation, i.e., real = signal - complementation(error)
		//calculate errors in position and velocity caused by atenna position
		double duvw[3] = {
			gp6.q*za-gp6.r*ya,
			gp6.r*xa-gp6.p*za,
			gp6.p*ya-gp6.q*xa
		};
		double nduvw[3];
		B2G(&gp6.a, duvw, nduvw);	// to NED

		double dxyz[3] = { xa, ya, za };
		double ndxyz[3];
		B2G(&gp6.a, dxyz, ndxyz);	// to NED

		//update
		//if original long, lat and alt not set, set first
		if (!m_bCoordinated) {
				Coordinate(gp6.longitude, gp6.latitude, gp6.altitude);
				m_bCoordinated = TRUE;
		}				//record the initial orientation

//		//attitude and angular rate only use AHR packets
//		EulerNext(&m_state1.a, &m_state1.p, dt);

		m_state1.longitude = gp6.longitude;
		m_state1.latitude = gp6.latitude;
		m_state1.altitude = gp6.altitude;

		//calculate position by longitude, latitude and altitude
		m_state1.x = (m_state1.latitude-m_latitude0)*_radius;
		m_state1.y = (m_state1.longitude-m_longitude0)*_radius;
		m_state1.z = m_altitude0-m_state1.altitude;

		//subtract position errors
//		m_state1.x -= ndxyz[0];
//		m_state1.y -= ndxyz[1];
//		m_state1.z -= ndxyz[2];

		double npos[3] = {m_state1.x - ndxyz[0], m_state1.y - ndxyz[1], m_state1.z - ndxyz[2]};
		N2G(abc0, npos, &m_RPTState.xg);
		m_state1.x = npos[0];
		m_state1.y = npos[1];
		m_state1.z = npos[2];

		double nvel[3] = {gp6.u - nduvw[0], gp6.v - nduvw[1], gp6.w - nduvw[2]};
		N2G(abc0, nvel, &m_RPTState.ug);
		m_state1.ug = nvel[0];
		m_state1.vg = nvel[1];
		m_state1.wg = nvel[2];

		G2B(abcg, &m_state1.ug, &m_state1.u);
		//velocity = signal - error
//		m_state1.ug = gp6.u-duvwg[0];
//		m_state1.vg = gp6.v-duvwg[1];
//		m_state1.wg = gp6.w-duvwg[2];

//		G2B(&m_state1.a, &m_state1.ug, &m_state1.u);

		m_tState1 = tPack;
	}
		break;
	}

	case DATA_AH6: {
		AH6PACK &ah6 = pPack->ah6;

		//verify
		ah6.acx += ah6.p*ah6.q*yi-ah6.q*ah6.q*xi-ah6.r*ah6.r*xi+ah6.p*ah6.r*zi;
		ah6.acy += ah6.r*ah6.q*zi-ah6.r*ah6.r*yi-ah6.p*ah6.p*yi+ah6.p*ah6.q*xi;
		ah6.acz += ah6.p*ah6.r*xi-ah6.p*ah6.p*zi-ah6.q*ah6.q*zi+ah6.q*ah6.r*yi;
//		ah6.acx += m_state0.acq*zi-m_state0.acr*yi;
//		ah6.acy += m_state0.acr*xi-m_state0.acp*zi;
//		ah6.acz += m_state0.acp*yi-m_state0.acq*xi;

		//update
		double rFilter = dt > 0 ? RATIO_FILTER : 0;

		m_state1.x += m_state1.ug*dt;
		m_state1.y += m_state1.vg*dt;
		m_state1.z += m_state1.wg*dt;

		m_state1.a = (ah6.a+rFilter*m_state1.a)/(1+rFilter);
		m_state1.b = (ah6.b+rFilter*m_state1.b)/(1+rFilter);

		double dc = ah6.c - m_state1.c;
		INPI(dc);
		m_state1.c += dc/(1+rFilter);
		INPI(m_state1.c);

		double pNew = (ah6.p+rFilter*m_state1.p)/(1+rFilter);
		double qNew = (ah6.q+rFilter*m_state1.q)/(1+rFilter);
		double rNew = (ah6.r+rFilter*m_state1.r)/(1+rFilter);

		if (dt > 0) {
			m_state1.acp = (pNew - m_state1.p)/dt;
			m_state1.acq = (qNew - m_state1.q)/dt;
			m_state1.acr = (rNew - m_state1.r)/dt;
		}

		double u = m_state1.u;
		double v = m_state1.v;
		double w = m_state1.w;

		m_state1.u += dt*(m_state1.acx-m_state1.q*w+m_state1.r*v);
		m_state1.v += dt*(m_state1.acy-m_state1.r*u+m_state1.p*w);
		m_state1.w += dt*(m_state1.acz-m_state1.p*v+m_state1.q*u);

		m_state1.p = pNew;
		m_state1.q = qNew;
		m_state1.r = rNew;

		double Mgf[3][3];
		clsMetric::AttitudeToTransformMatrix(&m_state1.a, NULL, Mgf);
		clsMetric::X(Mgf, &m_state1.u, &m_state1.ug);

		double acxa = ah6.acx + Mgf[2][0]*_gravity;
		double acya = ah6.acy + Mgf[2][1]*_gravity;
		double acza = ah6.acz + Mgf[2][2]*_gravity;

		m_state1.acx = (acxa+rFilter*m_state1.acx)/(1+rFilter);
		m_state1.acy = (acya+rFilter*m_state1.acy)/(1+rFilter);
		m_state1.acz = (acza+rFilter*m_state1.acz)/(1+rFilter);

		m_tState1 = tPack;
		break;
	}

	case DATA_SC6: {
		SC6PACK &sc6 = pPack->sc6;

		//verify
		sc6.acx += sc6.p*sc6.q*yi-sc6.q*sc6.q*xi-sc6.r*sc6.r*xi+sc6.p*sc6.r*zi;
		sc6.acy += sc6.r*sc6.q*zi-sc6.r*sc6.r*yi-sc6.p*sc6.p*yi+sc6.p*sc6.q*xi;
		sc6.acz += sc6.p*sc6.r*xi-sc6.p*sc6.p*zi-sc6.q*sc6.q*zi+sc6.q*sc6.r*yi;
//		sc6.acx += m_state0.acq*zi-m_state0.acr*yi;
//		sc6.acy += m_state0.acr*xi-m_state0.acp*zi;
//		sc6.acz += m_state0.acp*yi-m_state0.acq*xi;

		double rFilter = dt > 0 ? RATIO_FILTER : 0;

		m_state1.x += m_state1.ug*dt;
		m_state1.y += m_state1.vg*dt;
		m_state1.z += m_state1.wg*dt;

		EulerNext(&m_state1.a, &m_state1.p, dt);

		double pNew = (sc6.p+rFilter*m_state1.p)/(1+rFilter);
		double qNew = (sc6.q+rFilter*m_state1.q)/(1+rFilter);
		double rNew = (sc6.r+rFilter*m_state1.r)/(1+rFilter);

		if (dt > 0) {
			m_state1.acp = (pNew-m_state1.p)/dt;
			m_state1.acq = (qNew-m_state1.q)/dt;
			m_state1.acr = (rNew-m_state1.r)/dt;
		}

		double u = m_state1.u;
		double v = m_state1.v;
		double w = m_state1.w;

		m_state1.u += dt*(m_state1.acx-m_state1.q*w+m_state1.r*v);
		m_state1.v += dt*(m_state1.acy-m_state1.r*u+m_state1.p*w);
		m_state1.w += dt*(m_state1.acz-m_state1.p*v+m_state1.q*u);

		m_state1.p = pNew;
		m_state1.q = qNew;
		m_state1.r = rNew;

		double Mgf[3][3];
		clsMetric::AttitudeToTransformMatrix(&m_state1.a, NULL, Mgf);

		clsMetric::X(Mgf, &m_state1.u, &m_state1.ug);

		double acxa = sc6.acx + Mgf[2][0]*_gravity;
		double acya = sc6.acy + Mgf[2][1]*_gravity;
		double acza = sc6.acz + Mgf[2][2]*_gravity;

		m_state1.acx = (acxa+rFilter*m_state1.acx)/(1+rFilter);
		m_state1.acy = (acya+rFilter*m_state1.acy)/(1+rFilter);
		m_state1.acz = (acza+rFilter*m_state1.acz)/(1+rFilter);

		m_tState1 = tPack;
		break;
	}
	}
}

void clsState::Update(double tPack, IM6PACK *pPack)		/* NAV420 */
{
	Update1(tPack, pPack);				//update m_state1

	Filter();				//enter filter and put filtered result in m_state0
	Observe();				//observation: as, bs, rfb

	//push to record
	pthread_mutex_lock(&m_mtxState);

	if (m_nState != MAX_STATE) {
		m_tState[m_nState] = m_tState0;				//record
		m_state[m_nState++] = m_state0;
	}

	pthread_mutex_unlock(&m_mtxState);

	m_nCount ++;
}

void clsState::Update(double tPack, IM7PACK *pPack)		/* NAV440 */
{
	Update1(tPack, pPack);				//update m_state1

	Filter();				//enter filter and put filtered result in m_state0
	Observe();				//observation

//	VisionFusion();						// data fusion: m_state0 & vision output

	// put in the user-ground frame once vision filter is done
/*	double npos[3] = {m_state0.x, m_state0.y, m_state0.z};
	double abc0[3] = {0, 0, m_state0.c};
	N2G(abc0, npos, &m_RPTState.xg);

	double nvel[3] = {m_state0.ug, m_state0.vg, m_state0.wg};
	N2G(abc0, nvel, &m_RPTState.ug);
*/
	//push to record
	pthread_mutex_lock(&m_mtxState);

	if (m_nState != MAX_STATE) {
		m_tState[m_nState] = m_tState0;				//record
		m_state[m_nState++] = m_state0;
	}

	pthread_mutex_unlock(&m_mtxState);

	m_nCount ++;
};

void clsState::Update(double tPack, IM8PACK *pPack)		/* IG500N */
{
	Update1(tPack, pPack);				//update m_state1

	Filter();				//enter filter and put filtered result in m_state0
//	Observe();				//observation

	// start fusion when user issues "camrun" command
	if (false /*m_visionFilter.GetFusionFlag()*/)
	{
		if ( false /*m_visionFilter.GetbFirst() */)
		{
			//m_visionFilter.ResetbFirst();
			UAVSTATE &uavstate = GetState();
/*			uavstate.x = uavstate.y = 0; uavstate.z = -5;
			uavstate.ug = 2; uavstate.vg = uavstate.wg = 0;
			uavstate.a = uavstate.b = uavstate.c = 0;*/
			//m_visionFilter.Init(uavstate);
			m_camInfo.ta = 1; m_camInfo.tb = 0; m_camInfo.tc = 0;
			m_camInfo.tx = 0; m_camInfo.ty = 1; m_camInfo.tz = 0;
			m_camInfo.tmp1 = 0; m_camInfo.tmp2 = 0; m_camInfo.tmp3 = 1;
		}
		else	// consecutive update
		{

			if (m_nCount % 5 == 0) {
				SetVisionIntervalFlag();
			}

			BOOL bVisionUpdate = TRUE;//_state.GetVisionMeasurementUpdateFlag();
			BOOL bVisionInterval = _state.GetVisionIntervalFlag();
			UAVSTATE &uavstate = _state.GetState();

			if ( bVisionUpdate )
			{
/*				m_visionFilter.m_Mat_curUAVState.at<double>(0,0)=m_state0.x;
				m_visionFilter.m_Mat_curUAVState.at<double>(0,1)=m_state0.y;
				m_visionFilter.m_Mat_curUAVState.at<double>(0,2)=m_state0.z;
				m_visionFilter.m_Mat_curUAVState.at<double>(0,3)=m_state0.ug;
				m_visionFilter.m_Mat_curUAVState.at<double>(0,4)=m_state0.vg;
				m_visionFilter.m_Mat_curUAVState.at<double>(0,5)=m_state0.wg;
				m_visionFilter.m_Mat_curUAVState.at<double>(0,6)=m_state0.a;
				m_visionFilter.m_Mat_curUAVState.at<double>(0,7)=m_state0.b;
				m_visionFilter.m_Mat_curUAVState.at<double>(0,8)=m_state0.c;

				Mat Homo = Mat(3,3,CV_64FC1);
				m_visionFilter.fcn_GetHomoMatrixFrom1to2(m_visionFilter.m_Mat_prevUAVState, m_visionFilter.m_Mat_curUAVState, Homo);
				m_visionFilter.m_Mat_curUAVState.copyTo(m_visionFilter.m_Mat_prevUAVState);

//
				m_camInfo.ta = Homo.at<double>(0,0); m_camInfo.tb = Homo.at<double>(0,1); m_camInfo.tc = Homo.at<double>(0,2);
				m_camInfo.tx = Homo.at<double>(1,0); m_camInfo.ty = Homo.at<double>(1,1); m_camInfo.tz = Homo.at<double>(1,2);
				m_camInfo.tmp1 = Homo.at<double>(2,0); m_camInfo.tmp2 = Homo.at<double>(2,1); m_camInfo.tmp3 = Homo.at<double>(2,2);
*/
//				m_camInfo.ta = 1; m_camInfo.tb = 0; m_camInfo.tc = 0;
//				m_camInfo.tx = 0; m_camInfo.ty = 1; m_camInfo.tz = 0;
//				m_camInfo.tmp1 = 0; m_camInfo.tmp2 = 0; m_camInfo.tmp3 = 1;

				CAMTELEINFO &camInfo = _cam.GetCAMTeleInfo();
				m_camInfo.ta = camInfo.ta; m_camInfo.tb = camInfo.tb; m_camInfo.tc = camInfo.tc;
				m_camInfo.tx = camInfo.tx; m_camInfo.ty = camInfo.ty; m_camInfo.tz = camInfo.tz;
				m_camInfo.tmp1 = camInfo.tmp1; m_camInfo.tmp2 = camInfo.tmp2; m_camInfo.tmp3 = camInfo.tmp3;
			}//if ( bVisionUpdate )

//			uavstate.c = 0;
//			uavstate.acx = uavstate.acy = 0; uavstate.acz = -_gravity;
//			uavstate.z = -5;
//			uavstate.p = uavstate.q = uavstate.r = 0;

			//m_visionFilter.KalmanFilterWithVision(bVisionUpdate, bVisionInterval, uavstate, m_camInfo);
			if (bVisionInterval)
			{
				_state.ResetVisionMeasurementUpdateFlag();
			}
			_state.ResetVisionIntervalFlag();
		}

		// push vision estimation to record
		pthread_mutex_lock(&m_mtxVisionState);
		if (m_nVisionState != MAX_STATE) {
			m_tVisionState[m_nVisionState] = ::GetTime(); //m_tState0;
			/*
			 * m_visionState[m_nVisionState++] = m_visionFilter.GetVisionState();
			 */
		}
		pthread_mutex_unlock(&m_mtxVisionState);
	}

	//push UAV state to record
	pthread_mutex_lock(&m_mtxState);

	if (m_nState != MAX_STATE) {
		m_tState[m_nState] = m_tState0;				//record
		m_state[m_nState++] = m_state0;
	}

	pthread_mutex_unlock(&m_mtxState);

	m_nCount ++;
};

void clsState::Update1(double tPack, IM7PACK *pPack)		/* NAV440 */
{
	double dt = m_tState1 < 0 ? 0 : tPack - m_tState1;

	double posIM7[3], posAntenna[3];
	_im7.GetIM7Position(posIM7);
	_im7.GetAntennaPosition(posAntenna);

	double xa = posAntenna[0]; double ya = posAntenna[1]; double za = posAntenna[2];
	double xi = posIM7[0]; double yi = posIM7[1]; double zi = posIM7[2];

	switch (pPack->gp7.type) {
	case DATA_GP7: {
		GP7PACK &gp7 = pPack->gp7;

		m_state1.p = gp7.p; m_state1.q = gp7.q; m_state1.r = gp7.r;

		m_state1.a = gp7.a; m_state1.b = gp7.b;
		double dc = gp7.c - m_state1.c;
		INPI(dc);
		m_state1.c += dc /*/(1+rFilter)*/;
		INPI(m_state1.c);

		double cg = gp7.c - m_c0;
		INPI(cg);

		double abc0[3] = {/*gp6.a*/ 0, /*gp6.b*/ 0, m_c0};
		double abcg[3] = {gp7.a, gp7.b, cg};
		//verify
		//relationship between signal and actual value -
		//signal = real + complementation, i.e., real = signal - complementation(error)
		//calculate errors in position and velocity caused by atenna position
		double duvw[3] = {
			gp7.q*za-gp7.r*ya,
			gp7.r*xa-gp7.p*za,
			gp7.p*ya-gp7.q*xa
		};
		double nduvw[3];
		B2G(&gp7.a, duvw, nduvw);	// to NED

		double dxyz[3] = { xa, ya, za };
		double ndxyz[3];
		B2G(&gp7.a, dxyz, ndxyz);	// to NED

		//update
		//if original long, lat and alt not set, set first
		if (!m_bCoordinated) {
				Coordinate(gp7.longitude, gp7.latitude, gp7.altitude);
				m_bCoordinated = TRUE;
		}				//record the initial orientation

//		//attitude and angular rate only use AHR packets
//		EulerNext(&m_state1.a, &m_state1.p, dt);

		m_state1.longitude = gp7.longitude;
		m_state1.latitude = gp7.latitude;
		m_state1.altitude = gp7.altitude;

		//calculate position by longitude, latitude and altitude
		m_state1.x = (m_state1.latitude-m_latitude0)*_radius;
		m_state1.y = (m_state1.longitude-m_longitude0)*_radius;
		m_state1.z = m_altitude0-m_state1.altitude;

		//subtract position errors
//		m_state1.x -= ndxyz[0];
//		m_state1.y -= ndxyz[1];
//		m_state1.z -= ndxyz[2];

		double npos[3] = {m_state1.x - ndxyz[0], m_state1.y - ndxyz[1], m_state1.z - ndxyz[2]};
		N2G(abc0, npos, &m_RPTState.xg);
		m_state1.x = npos[0];
		m_state1.y = npos[1];
		m_state1.z = npos[2];

		double nvel[3] = {gp7.u - nduvw[0], gp7.v - nduvw[1], gp7.w - nduvw[2]};
		N2G(abc0, nvel, &m_RPTState.ug);
		m_state1.ug = nvel[0];
		m_state1.vg = nvel[1];
		m_state1.wg = nvel[2];

		G2B(abcg, &m_state1.ug, &m_state1.u);

		// Debug usage below
/*		m_state1.acx = m_RPTState.xg;
		m_state1.acy = m_RPTState.yg;
		m_state1.acz = m_RPTState.zg;
		
		m_state1.acp = m_RPTState.ug;
		m_state1.acq = m_RPTState.vg;
		m_state1.acr = m_RPTState.wg;*/
		
/*		double Mgf[3][3];
		clsMetric::AttitudeToTransformMatrix(&m_state1.a, NULL, Mgf);

		double acxa = gp7.acx + Mgf[2][0]*_gravity;
		double acya = gp7.acy + Mgf[2][1]*_gravity;
		double acza = gp7.acz + Mgf[2][2]*_gravity;
		m_state1.acx = acxa; m_state1.acy = acya; m_state1.acz = acza; */
		
		m_state1.acx = gp7.acx; m_state1.acy = gp7.acy; m_state1.acz = gp7.acz;
		
		//velocity = signal - error
//		m_state1.ug = gp6.u-duvwg[0];
//		m_state1.vg = gp6.v-duvwg[1];
//		m_state1.wg = gp6.w-duvwg[2];

//		G2B(&m_state1.a, &m_state1.ug, &m_state1.u);

		m_tState1 = tPack;
		break;
	}

	case DATA_AH7: {
		AH7PACK &ah7 = pPack->ah7;

		//verify
		ah7.acx += ah7.p*ah7.q*yi-ah7.q*ah7.q*xi-ah7.r*ah7.r*xi+ah7.p*ah7.r*zi;
		ah7.acy += ah7.r*ah7.q*zi-ah7.r*ah7.r*yi-ah7.p*ah7.p*yi+ah7.p*ah7.q*xi;
		ah7.acz += ah7.p*ah7.r*xi-ah7.p*ah7.p*zi-ah7.q*ah7.q*zi+ah7.q*ah7.r*yi;
//		ah6.acx += m_state0.acq*zi-m_state0.acr*yi;
//		ah6.acy += m_state0.acr*xi-m_state0.acp*zi;
//		ah6.acz += m_state0.acp*yi-m_state0.acq*xi;

		//update
		double rFilter = dt > 0 ? RATIO_FILTER : 0;

		m_state1.x += m_state1.ug*dt;
		m_state1.y += m_state1.vg*dt;
		m_state1.z += m_state1.wg*dt;

		m_state1.a = (ah7.a+rFilter*m_state1.a)/(1+rFilter);
		m_state1.b = (ah7.b+rFilter*m_state1.b)/(1+rFilter);

		double dc = ah7.c - m_state1.c;
		INPI(dc);
		m_state1.c += dc/(1+rFilter);
		INPI(m_state1.c);

		double pNew = (ah7.p+rFilter*m_state1.p)/(1+rFilter);
		double qNew = (ah7.q+rFilter*m_state1.q)/(1+rFilter);
		double rNew = (ah7.r+rFilter*m_state1.r)/(1+rFilter);

		if (dt > 0) {
			m_state1.acp = (pNew - m_state1.p)/dt;
			m_state1.acq = (qNew - m_state1.q)/dt;
			m_state1.acr = (rNew - m_state1.r)/dt;
		}

		double u = m_state1.u;
		double v = m_state1.v;
		double w = m_state1.w;

		m_state1.u += dt*(m_state1.acx-m_state1.q*w+m_state1.r*v);
		m_state1.v += dt*(m_state1.acy-m_state1.r*u+m_state1.p*w);
		m_state1.w += dt*(m_state1.acz-m_state1.p*v+m_state1.q*u);

		m_state1.p = pNew;
		m_state1.q = qNew;
		m_state1.r = rNew;

		double Mgf[3][3];
		clsMetric::AttitudeToTransformMatrix(&m_state1.a, NULL, Mgf);
		clsMetric::X(Mgf, &m_state1.u, &m_state1.ug);

		double acxa = ah7.acx + Mgf[2][0]*_gravity;
		double acya = ah7.acy + Mgf[2][1]*_gravity;
		double acza = ah7.acz + Mgf[2][2]*_gravity;

		m_state1.acx = (acxa+rFilter*m_state1.acx)/(1+rFilter);
		m_state1.acy = (acya+rFilter*m_state1.acy)/(1+rFilter);
		m_state1.acz = (acza+rFilter*m_state1.acz)/(1+rFilter);

		m_tState1 = tPack;

		double cg = m_state1.c - m_c0;
		INPI(cg);
		double abc0[3] = {/*gp7.a*/ 0, /*gp7.b*/ 0, m_c0};
		double abcg[3] = {m_state1.a, m_state1.b, cg};

		double npos[3] = {m_state1.x, m_state1.y, m_state1.z};
		N2G(abc0, npos, &m_RPTState.xg);
		m_state1.x = npos[0];
		m_state1.y = npos[1];
		m_state1.z = npos[2];

		double nvel[3] = {m_state1.ug, m_state1.vg, m_state1.wg};
		N2G(abc0, nvel, &m_RPTState.ug);
		m_state1.ug = nvel[0];
		m_state1.vg = nvel[1];
		m_state1.wg = nvel[2];

		G2B(abcg, &m_state1.ug, &m_state1.u);
		break;
	}

	case DATA_SC7: {
		SC7PACK &sc7 = pPack->sc7;

		//verify
		sc7.acx += sc7.p*sc7.q*yi-sc7.q*sc7.q*xi-sc7.r*sc7.r*xi+sc7.p*sc7.r*zi;
		sc7.acy += sc7.r*sc7.q*zi-sc7.r*sc7.r*yi-sc7.p*sc7.p*yi+sc7.p*sc7.q*xi;
		sc7.acz += sc7.p*sc7.r*xi-sc7.p*sc7.p*zi-sc7.q*sc7.q*zi+sc7.q*sc7.r*yi;
//		sc7.acx += m_state0.acq*zi-m_state0.acr*yi;
//		sc7.acy += m_state0.acr*xi-m_state0.acp*zi;
//		sc7.acz += m_state0.acp*yi-m_state0.acq*xi;

		double rFilter = dt > 0 ? RATIO_FILTER : 0;

		m_state1.x += m_state1.ug*dt;
		m_state1.y += m_state1.vg*dt;
		m_state1.z += m_state1.wg*dt;

		EulerNext(&m_state1.a, &m_state1.p, dt);

		double pNew = (sc7.p+rFilter*m_state1.p)/(1+rFilter);
		double qNew = (sc7.q+rFilter*m_state1.q)/(1+rFilter);
		double rNew = (sc7.r+rFilter*m_state1.r)/(1+rFilter);

		if (dt > 0) {
			m_state1.acp = (pNew-m_state1.p)/dt;
			m_state1.acq = (qNew-m_state1.q)/dt;
			m_state1.acr = (rNew-m_state1.r)/dt;
		}

		double u = m_state1.u;
		double v = m_state1.v;
		double w = m_state1.w;

		m_state1.u += dt*(m_state1.acx-m_state1.q*w+m_state1.r*v);
		m_state1.v += dt*(m_state1.acy-m_state1.r*u+m_state1.p*w);
		m_state1.w += dt*(m_state1.acz-m_state1.p*v+m_state1.q*u);

		m_state1.p = pNew;
		m_state1.q = qNew;
		m_state1.r = rNew;

		double Mgf[3][3];
		clsMetric::AttitudeToTransformMatrix(&m_state1.a, NULL, Mgf);

		clsMetric::X(Mgf, &m_state1.u, &m_state1.ug);

		double acxa = sc7.acx + Mgf[2][0]*_gravity;
		double acya = sc7.acy + Mgf[2][1]*_gravity;
		double acza = sc7.acz + Mgf[2][2]*_gravity;

		m_state1.acx = (acxa+rFilter*m_state1.acx)/(1+rFilter);
		m_state1.acy = (acya+rFilter*m_state1.acy)/(1+rFilter);
		m_state1.acz = (acza+rFilter*m_state1.acz)/(1+rFilter);

		m_tState1 = tPack;
		break;
	}
	}
}


void clsState::Update1(double tPack, IM8PACK *pPack)		/* IG500N */
{
//	double dt = m_tState1 < 0 ? 0 : tPack - m_tState1;

	double posIM8[3], posAntenna[3];
	_im8.GetIM8Position(posIM8);
	_im8.GetAntennaPosition(posAntenna);

	double xa = posAntenna[0]; double ya = posAntenna[1]; double za = posAntenna[2];

	switch (pPack->gp8.type) {
	case DATA_GP8: {
		GP8PACK &gp8 = pPack->gp8;
		m_state1.p = gp8.p; m_state1.q = gp8.q; m_state1.r = gp8.r;

		m_state1.a = gp8.a; m_state1.b = gp8.b;
		m_state1.c = gp8.c;

		if (_cmm.GetViconFlag()) {
			VICON_DATA viconData = _cmm.GetViconData();
			m_state1.c = viconData.c;
		}
		//verify
		//relationship between signal and actual value -
		//signal = real + complementation, i.e., real = signal - complementation(error)
		//calculate errors in position and velocity caused by antenna position
		double duvw[3] = {
			gp8.q*za-gp8.r*ya,
			gp8.r*xa-gp8.p*za,
			gp8.p*ya-gp8.q*xa
		};
		double nduvw[3];
		double abc[3] = {m_state1.a, m_state1.b, m_state1.c};
		B2G(abc, duvw, nduvw);	// to NED

		double Mgf[3][3];
		clsMetric::AttitudeToTransformMatrix(&m_state1.a, NULL, Mgf);

		double acxa = gp8.acx + Mgf[2][0]*_gravity;
		double acya = gp8.acy + Mgf[2][1]*_gravity;
		double acza = gp8.acz + Mgf[2][2]*_gravity;
		m_state1.acx = acxa; m_state1.acy = acya; m_state1.acz = acza;

		if (!m_bCoordinated) {
				Coordinate(gp8.longitude, gp8.latitude, gp8.altitude);
				SetInitPressure(gp8.pressure);
				m_bCoordinated = TRUE;
		}				//record the initial orientation

		/*
		 * No GPS signal, the vision data such as body velocity is used,
		 * Ground velocity and position are also derived based on the vision information
		 */
		if ( FALSE /*gp8.nGPS < 4 || ((gp8.gpsinfo & 0x03) != 3)*/ ) {
			CAMTELEINFO camInfo = _cam.GetCAMTeleInfo();

			m_KalmanState.a = m_state1.a; m_KalmanState.b = m_state1.b; m_KalmanState.c = m_state1.c;
			m_KalmanState.p = m_state1.p; m_KalmanState.q = m_state1.q; m_KalmanState.r = m_state1.r;
			m_KalmanState.acx = m_state1.acx; m_KalmanState.acy = m_state1.acy; m_KalmanState.acz = m_state1.acz;
			m_KalmanState.u = camInfo.ta; m_KalmanState.v = camInfo.tb; m_KalmanState.w = camInfo.tc;
			m_KalmanState.x = m_state1.x; m_KalmanState.y = m_state1.y; m_KalmanState.z = CalculateHeightViaBaro(gp8.pressure);

			KalmanTimeUpdate(m_state1, m_KalmanState);

			if (m_bMeasureUpdate) {
				KalmanMeasurementUpdate(m_state1, m_KalmanState);
				m_bMeasureUpdate = FALSE;
			}
		} else if (_cmm.GetViconFlag()) {
			VICON_DATA viconData = _cmm.GetViconData();
			m_KalmanState.a = m_state1.a; m_KalmanState.b = m_state1.b; m_KalmanState.c = m_state1.c;
			m_KalmanState.p = m_state1.p; m_KalmanState.q = m_state1.q; m_KalmanState.r = m_state1.r;
			m_KalmanState.acx = m_state1.acx; m_KalmanState.acy = m_state1.acy; m_KalmanState.acz = m_state1.acz;
	//		m_KalmanState.acx = 0; m_KalmanState.acy = 0; m_KalmanState.acz = 0;
			m_KalmanState.u = m_state1.u; m_KalmanState.v = m_state1.v; m_KalmanState.w = 0;
			m_KalmanState.x = viconData.x; m_KalmanState.y = viconData.y; m_KalmanState.z = viconData.z;
//			printf("[state] vicon x %.3f, y %.3f, z %.3f, c %.3f\n", viconData.x, viconData.y, viconData.z, viconData.c);

			KalmanTimeUpdate(m_state1, m_KalmanState);
			KalmanMeasurementUpdate(m_state1, m_KalmanState);
		}
		else {
			//update GPS related data
			//if original long, lat and alt not set, set first
			m_state1.longitude = gp8.longitude;
			m_state1.latitude = gp8.latitude;
			m_state1.altitude = gp8.altitude;

			//calculate position by longitude, latitude and altitude
			m_state1.x = (m_state1.latitude-m_latitude0)*_radius;
			m_state1.y = (m_state1.longitude-m_longitude0)*_radius * cos(m_state1.latitude);
			m_state1.z = /* CalculateHeightViaBaro(gp8.pressure);  */ m_altitude0-m_state1.altitude;
			m_state1.as = CalculateHeightViaBaro(gp8.pressure);

			double npos[3] = {m_state1.x, m_state1.y, m_state1.z};
			m_state1.x = npos[0];
			m_state1.y = npos[1];
			m_state1.z = npos[2];
			m_state1.u = gp8.u; m_state1.v = gp8.v; m_state1.w = gp8.w;
			B2G(abc, &m_state1.u, &m_state1.ug);

			// --------- Kalman filter for the z-axix, provide height and wg -----------
			memcpy(&m_KalmanState, &m_state1, sizeof(UAVSTATE));
			KalmanTimeUpdateZAxis(m_state1, m_KalmanState);
			KalmanMeasurementUpdateZAxis(m_state1, m_KalmanState);

			if (m_nCount % 50 == 0) {
				//printf("[IM8 baro filter]: x: %.2f y: %.2f z %.3f, wg %.3f\n", m_state1.x, m_state1.y, m_state1.z, m_state1.wg);
				//printf("[IM8 GPS] lat %f lon %f\n", m_state1.latitude*180/PI, m_state1.longitude*180/PI);
			}
		}
		m_tState1 = tPack;
		break;
	}

	}
}

void clsState::Update(double tPack, IM9PACK *pPack)
{
	Update1(tPack, pPack);	//update m_measure0: current measurement

//	Filter();				//enter filter and put filtered result in m_state0: current state
//	Observe();				//observation: a_up, b_up, r_fb, omg_up, omg_dw

	//push to record
	pthread_mutex_lock(&m_mtxState);
	if (m_nState != MAX_STATE) {
		m_tState[m_nState] = m_tState0;				//record
		m_state[m_nState++] = m_state0;
	}
	pthread_mutex_unlock(&m_mtxState);

	m_nCount ++;
}

void clsState::Update1(double tPack, IM9PACK *pPack)
{
/*	m_measure0.a = pPack->a;
	m_measure0.b = pPack->b;
	m_measure0.c = pPack->c;
	INPI(m_measure0.c);

	m_measure0.p = pPack->p;
	m_measure0.q = pPack->q;
	m_measure0.r = pPack->r;

	m_measure0.acx = pPack->acx;
	m_measure0.acy = pPack->acy;
	m_measure0.acz = pPack->acz;

	m_tMeasure0 = tPack;*/

	m_tState0 = tPack;
	m_state0.a = pPack->a; m_state0.b = pPack->b;
	m_state0.c = pPack->c; INPI(m_state0.c);

	m_state0.p = pPack->p; m_state0.q = pPack->q; m_state0.r = pPack->r;
	m_state0.acx = pPack->acx; m_state0.acy = pPack->acy; m_state0.acz = pPack->acz;

	CAMTELEINFO camTeleInfo = _cam.GetCAMTeleInfo();
	m_state0.u = camTeleInfo.ta; m_state0.v = camTeleInfo.tb; m_state0.w = camTeleInfo.tc;

}

double clsState::CalculateHeightViaBaro(double pressure)
{
	double frac = pressure/m_pressure0;

	return -44307 * (1 - pow(frac, 0.1902));
}

BOOL clsState::KalmanTimeUpdateZAxis(UAVSTATE &uavstate, const UAVSTATE KalmanState)
{
	clsVector m_x_ZAxis(2, _m_x_ZAxis, TRUE);
    double _u = uavstate.acz; clsVector u(1, &_u, TRUE);

    double _Ax[2]; clsVector Ax(2, _Ax, TRUE);
    clsMatrix::X(m_Kalman_A_ZAxis, m_x_ZAxis, Ax);

    double _Bu[2]; clsVector Bu(2, _Bu, TRUE);
    clsMatrix::X(m_Kalman_B_ZAxis, u, Bu);

    m_x_ZAxis = Ax;
    m_x_ZAxis += Bu;

    double _AP[2][2]; clsMatrix AP;
    AP.Reset(2, 2, (double *)_AP, TRUE);
    clsMatrix::X(m_Kalman_A_ZAxis, m_Kalman_P_ZAxis, AP);

    double _At[2][2]; clsMatrix At;
    At.Reset(2, 2, (double *)_At, TRUE);
    clsMatrix::T(m_Kalman_A_ZAxis, At);

    double _APAt[2][2]; clsMatrix APAt;
    APAt.Reset(2, 2, (double *)_APAt, TRUE);
    clsMatrix::X(AP, At, APAt);

    APAt += m_BQBt_ZAxis;
    m_Kalman_P_ZAxis = APAt;

    return TRUE;
}


BOOL clsState::KalmanTimeUpdate(UAVSTATE &uavstate, const UAVSTATE KalmanState)
{
	clsVector m_xVision_WF(6, _m_xVision_WF, TRUE);

    double ab[3] = {KalmanState.acx, KalmanState.acy, KalmanState.acz};
    double an[3] = {0};
    double angle[3] = {uavstate.a, uavstate.b, uavstate.c};
    B2G(angle, ab, an);

    double _u[3] = {an[0], an[1], an[2]};
    clsVector u(3, _u, TRUE);

    double _Ax[6]; clsVector Ax(6, _Ax, TRUE);
    clsMatrix::X(m_Kalman_A_WF, m_xVision_WF, Ax);
//    printf("Ax: %f, %f, %f\n", Ax[0], Ax[1], Ax[2]);

    double _Bu[6]; clsVector Bu(6, _Bu, TRUE);
    clsMatrix::X(m_Kalman_B_WF, u, Bu);
//    printf("Bu: %f, %f, %f\n", Bu[0], Bu[1], Bu[2]);
    m_xVision_WF = Ax;
    m_xVision_WF += Bu;
//    printf("xVision: %f, %f, %f, %f, %f, %f\n",
//                xVision[0], xVision[1], xVision[2], xVision[3], xVision[4], xVision[5]);

    double _AP[6][6]; clsMatrix AP;
    AP.Reset(6, 6, (double *)_AP, TRUE);
    clsMatrix::X(m_Kalman_A_WF, m_Kalman_P_WF, AP);

    double _At[6][6]; clsMatrix At;
    At.Reset(6, 6, (double *)_At, TRUE);
    clsMatrix::T(m_Kalman_A_WF, At);

    double _APAt[6][6]; clsMatrix APAt;
    APAt.Reset(6, 6, (double *)_APAt, TRUE);
    clsMatrix::X(AP, At, APAt);

    APAt += m_BQBt_WF;
    m_Kalman_P_WF = APAt;

    // state update after fusion
    uavstate.x = m_xVision_WF[0]; uavstate.y = m_xVision_WF[1]; uavstate.z = m_xVision_WF[2];
    uavstate.ug = m_xVision_WF[3]; uavstate.vg = m_xVision_WF[4]; uavstate.wg = m_xVision_WF[5];
    double ug[3] = {m_xVision_WF[3], m_xVision_WF[4], m_xVision_WF[5]};
    G2B(angle, ug, &uavstate.u);

    return TRUE;
}

BOOL clsState::KalmanMeasurementUpdateZAxis(UAVSTATE &uavstate, const UAVSTATE KalmanState)
{
	clsVector m_x_ZAxis(2, _m_x_ZAxis, TRUE);

    double _yv = KalmanState.as;
//    double angle[3] = {uavstate.a, uavstate.b, uavstate.c};

//    _yv[0] = KalmanState.as;
//    _yv[1] = KalmanState.wg;

    clsVector yv(1, &_yv, TRUE);

    double _H[2][1]; clsMatrix H;
    H.Reset(2, 1, (double *)_H, TRUE);

    double _PCt[2][1]; clsMatrix PCt;
    PCt.Reset(2, 1, (double *)_PCt, TRUE);

    double _Ct[2][1]; clsMatrix Ct;
    Ct.Reset(2, 1, (double *)_Ct, TRUE);
    clsMatrix::T(m_Kalman_C_ZAxis, Ct);
    clsMatrix::X(m_Kalman_P_ZAxis, Ct, PCt);

    double _CP[1][2]; clsMatrix CP;
    CP.Reset(1, 2, (double *)_CP, TRUE);
    clsMatrix::X(m_Kalman_C_ZAxis, m_Kalman_P_ZAxis, CP);

    double _CPCt[1][1]; clsMatrix CPCt;
    CPCt.Reset(1, 1, (double *)_CPCt, TRUE);
    clsMatrix::X(CP, Ct, CPCt);
    CPCt += m_Kalman_R_ZAxis;

    double _CPCtInverse[1][1]; clsMatrix CPCtInverse;
    CPCtInverse.Reset(1, 1, (double *)_CPCtInverse, TRUE);
    clsMatrix::R(CPCt, CPCtInverse);
    clsMatrix::X(PCt, CPCtInverse, H);

    double _Cx; clsVector Cx(1, &_Cx, TRUE);
    clsMatrix::X(m_Kalman_C_ZAxis, m_x_ZAxis, Cx);
    yv -= Cx;

    double _Hyv[2]; clsVector Hyv(2, _Hyv, TRUE);
    clsMatrix::X(H, yv, Hyv);

    m_x_ZAxis += Hyv;

    double _EYE[2][2] = {
                { 1,0 },
                { 0,1}
    };
    clsMatrix EYE;
    EYE.Reset(2, 2, (double *)_EYE, TRUE);

    double _HC[2][2]; clsMatrix HC;
    HC.Reset(2, 2, (double *)_HC, TRUE);
    clsMatrix::X(H, m_Kalman_C_ZAxis, HC);
    EYE -= HC;

    clsMatrix::X(EYE, m_Kalman_P_ZAxis, m_Kalman_P_ZAxis);          // P update

    // state update after fusion
    uavstate.z = m_x_ZAxis[0];
//    uavstate.wg = m_x_ZAxis[1];

	return TRUE;
}

BOOL clsState::KalmanMeasurementUpdate(UAVSTATE &uavstate, const UAVSTATE KalmanState)
{
	clsVector m_xVision_WF(6, _m_xVision_WF, TRUE);

    double tpos[3] = {KalmanState.u, KalmanState.v, 0};

    double _yv[3] = {0};
    double angle[3] = {uavstate.a, uavstate.b, uavstate.c};

//    B2G(angle, tpos, _yv);
//    _yv[2] = _yv[1]; _yv[1] = _yv[0]; _yv[0] = KalmanState.z;

    _yv[0] = KalmanState.x;
    _yv[1] = KalmanState.y;
    _yv[2] = KalmanState.z;

   // printf("_yv: %f, %f, %f\n", _yv[0], _yv[1], _yv[2]);
    clsVector yv(3, _yv, TRUE);

    double _H[6][3]; clsMatrix H;
    H.Reset(6, 3, (double *)_H, TRUE);

    double _PCt[6][3]; clsMatrix PCt;
    PCt.Reset(6, 3, (double *)_PCt, TRUE);

    double _Ct[6][3]; clsMatrix Ct;
    Ct.Reset(6, 3, (double *)_Ct, TRUE);
    clsMatrix::T(m_Kalman_C_WF, Ct);
    clsMatrix::X(m_Kalman_P_WF, Ct, PCt);

    double _CP[3][6]; clsMatrix CP;
    CP.Reset(3, 6, (double *)_CP, TRUE);
    clsMatrix::X(m_Kalman_C_WF, m_Kalman_P_WF, CP);

    double _CPCt[3][3]; clsMatrix CPCt;
    CPCt.Reset(3, 3, (double *)_CPCt, TRUE);
    clsMatrix::X(CP, Ct, CPCt);
    CPCt += m_Kalman_R_WF;

    double _CPCtInverse[3][3]; clsMatrix CPCtInverse;
    CPCtInverse.Reset(3, 3, (double *)_CPCtInverse, TRUE);
    clsMatrix::R(CPCt, CPCtInverse);
    clsMatrix::X(PCt, CPCtInverse, H);

    double _Cx[3]; clsVector Cx(3, _Cx, TRUE);
    clsMatrix::X(m_Kalman_C_WF, m_xVision_WF, Cx);
    yv -= Cx;

    double _Hyv[6]; clsVector Hyv(6, _Hyv, TRUE);
    clsMatrix::X(H, yv, Hyv);

    m_xVision_WF += Hyv;

    double _EYE[6][6] = {
                { 1,0,0,0,0,0 },
                { 0,1,0,0,0,0 },
                { 0,0,1,0,0,0 },
                { 0,0,0,1,0,0 },
                { 0,0,0,0,1,0 },
                { 0,0,0,0,0,1 }
    };
    clsMatrix EYE;
    EYE.Reset(6, 6, (double *)_EYE, TRUE);

    double _HC[6][6]; clsMatrix HC;
    HC.Reset(6, 6, (double *)_HC, TRUE);
    clsMatrix::X(H, m_Kalman_C_WF, HC);
    EYE -= HC;

    clsMatrix::X(EYE, m_Kalman_P_WF, m_Kalman_P_WF);          // P update

    // state update after fusion
    uavstate.x = m_xVision_WF[0]; uavstate.y = m_xVision_WF[1]; uavstate.z = m_xVision_WF[2];
    uavstate.ug = m_xVision_WF[3]; uavstate.vg = m_xVision_WF[4]; uavstate.wg = m_xVision_WF[5];
    double ug[3] = {m_xVision_WF[3], m_xVision_WF[4], m_xVision_WF[5]};
    G2B(angle, ug, &uavstate.u);

	return TRUE;
}

clsFilter::clsFilter()
{
	Reta = 10;
	Reps[0][0] = 0.1; Reps[0][1] = 0;
	Reps[1][0] = 0; Reps[1][1] = 0.1;

	//initialize
	t = -1;
}

void clsFilter::Update(double tx, double xm, double um)
{
	if (t <= 0) {
		t = tx;
		xi = xm;

		ee = 0;
		bee = 0;

		ue = um;

		xei = xm;
		xeif = xei;

		xif = xi;

		xmf = xm;
		xmfv = xmf;

		P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0;
		Q[0][0] = Q[0][1] = Q[1][0] = Q[1][1] = 0;

		return;
	}

	double dt = tx -t;
	t = tx;

	//integration of um
	xi += um*dt;
	double em = xi - xm;

	//P = A*Q*A'+Reps
	P[0][0] = Q[0][0]+dt*(Q[0][1]+Q[1][0])+dt*dt*Q[1][1]+Reps[0][0];
	P[0][1] = Q[0][1]+dt*Q[1][1]+Reps[0][1];
	P[1][0] = Q[1][0]+dt*Q[1][1]+Reps[1][0];
	P[1][1] = Q[1][1]+Reps[1][1];

	//B = P*C'/(C*P*C'+Reta);
	double B[2] = { P[0][0]/(P[0][0]+Reta), P[1][0]/(P[0][0]+Reta) };

	//e = A*e+B*(em-C*A*e)
	double error = em-ee-dt*bee;
	double ee2 = ee+dt*bee+B[0]*error;
	double bee2 = bee+B[1]*error;

	ee = ee2; bee = bee2;

	//Q = P*(I-B*C);
	if (Q[0][0] < 1e6 && Q[0][1] < 1e6 && Q[1][0] < 1e6 && Q[1][1] < 1e6) {
	//in this work the Q will increase infinitely, and B will converge to a
	//value, this is to prevent Q increase inifinitely and B is kept at a
	//convergent value
		Q[0][0] = (1-B[0])*P[0][0]-B[1]*P[0][1];
		Q[0][1] = P[0][1];
		Q[1][0] = (1-B[0])*P[1][0]-B[1]*P[1][1];
		Q[1][1] = P[1][1];
	}

	ue = um - bee;
	xei += ue*dt;

	//low-pass filter
	double r = 0.01;

	xeif = (1-r)*xeif + r*xei;
	xmf = (1-r)*xmf + r*xm;
	xmfv = xmf - (xeif - xei);

//	xif = (1-r)*xif+r*xi;
//	xmfv = xmf - (yif-yi);
}

void clsState::Filter()
{
	m_tState0 = m_tState1;
	m_state0 = m_state1;

	return;
	if (!m_bFilter) return;				//skip filter

	//filter x, y, z and made verifications
	m_fx.Update(m_tState1, m_state1.x, m_state1.ug);
	m_fy.Update(m_tState1, m_state1.y, m_state1.vg);
	m_fz.Update(m_tState1, m_state1.z, m_state1.wg);

	m_state0.x = m_fx.GetPositionEstimation();
	m_state0.ug = m_fx.GetVelocityEstimation();

	m_state0.y = m_fy.GetPositionEstimation();
	m_state0.vg = m_fy.GetVelocityEstimation();

	m_state0.z = m_fz.GetPositionEstimation();
	m_state0.wg = m_fz.GetVelocityEstimation();

	G2B(&m_state0.a, &m_state0.ug, &m_state0.u);
}

BOOL clsState::Emergency()
{
	//judge based on state1 (not state0, state0 is filtered result)
//	double t = ::GetTime();
//	BOOL bValid0 = /*m_tState0 < 0 ||*/ t - m_tState0 < 0.1;
	//regular each period 0.02 second, if larger than 0.1, that means 5 periods blocked

//	BOOL bValid1 = Valid(&m_state1);
	BOOL bValid2 = Valid(&m_state0);				//use for model simulation

	return /*!bValid0 || !bValid1 || */ !bValid2;
}

BOOL clsState::Valid(UAVSTATE *pState)
{
	BOOL bValid =
		::fabs(pState->x) <= 5000 &&
		::fabs(pState->y) <= 5000 &&
		pState->z <= 400 && pState->z >= -500 &&
		::fabs(pState->u) <= 20 &&
		::fabs(pState->v) <= 10 &&
		::fabs(pState->w) <= 20 &&
		::fabs(pState->a) <= /*PI/4*/ PI/3 &&
		::fabs(pState->b) <= /*PI/4*/ PI/3 &&
		::fabs(pState->p) <= 2 &&
		::fabs(pState->q) <= 2 &&
		::fabs(pState->r) <= 2 &&
		::fabs(pState->as) <= 0.5 &&
		::fabs(pState->bs) <= 0.5 &&
		::fabs(pState->rfb) <= 0.5;

	return bValid;
}

void clsState::Init()
{
	m_Ad.Reset(3, 3, (double *)_m_Ad, TRUE);
	m_Bd.Reset(3, 12, (double *)_m_Bd, TRUE);
	m_Cd.Reset(3, 3, (double *)_m_Cd, TRUE);
	m_Dd.Reset(3, 12, (double *)_m_Dd, TRUE);

/*
	m_Ad = (double *)_Ad;
	m_Bd = (double *)_Bd;
	m_Cd = (double *)_Cd;
	m_Dd = (double *)_Dd;
*/
	_parser.GetVariable("_Ad", m_Ad);
	_parser.GetVariable("_Bd", m_Bd);
	_parser.GetVariable("_Cd", m_Cd);
	_parser.GetVariable("_Dd", m_Dd);

	m_tObserve = -1;

	if (_HELICOPTER == ID_HELION || _HELICOPTER == ID_SHELION) {
		m_xe.Reset(3, _m_xe, TRUE);

		m_Ae.Reset(3, 3, (double *)_m_Ae, TRUE);
	//	m_Be.Reset(3, 12, (double *)_m_Be, TRUE);
		m_Be.Reset(3, 9, (double *)_m_Be, TRUE);
		m_Ce.Reset(3, 3, (double *)_m_Ce, TRUE);
		m_De.Reset(3, 9, (double *)_m_De, TRUE);

		_parser.GetVariable("_Ae", m_Ae);
		_parser.GetVariable("_Be", m_Be);
		_parser.GetVariable("_Ce", m_Ce);
		_parser.GetVariable("_De", m_De);

		m_Ae2.Reset(3, 3, (double *)_m_Ae2, TRUE);
		m_Ae3.Reset(3, 3, (double *)_m_Ae3, TRUE);

		clsMatrix::X(m_Ae, m_Ae, m_Ae2);
		clsMatrix::X(m_Ae2, m_Ae, m_Ae3);
	}
	else if (_HELICOPTER == ID_GREMLION) {
		m_xe_GREMLION.Reset(8, _m_xe_GREMLION, TRUE);

		// load observer related matric
		m_Ae_GREMLION.Reset(8, 8, (double *)_m_Ae_GREMLION, TRUE);
		m_Be_GREMLION.Reset(8, 9, (double *)_m_Be_GREMLION, TRUE);
//		m_Ce_GREMLION.Reset(2, 11, (double *)_m_Ce_GREMLION, TRUE);
//		m_De.Reset(3, 9, (double *)_m_De, TRUE);

		_parser.GetVariable("_Ae_GREMLION1", m_Ae_GREMLION);
		_parser.GetVariable("_Be_GREMLION1", m_Be_GREMLION);
//		_parser.GetVariable("_Ce_GREMLION0", m_Ce_GREMLION);

		m_Ae2_GREMLION.Reset(8, 8, (double *)_m_Ae2_GREMLION, TRUE);
		m_Ae3_GREMLION.Reset(8, 8, (double *)_m_Ae3_GREMLION, TRUE);

		clsMatrix::X(m_Ae_GREMLION, m_Ae_GREMLION, m_Ae2_GREMLION);
		clsMatrix::X(m_Ae2_GREMLION, m_Ae_GREMLION, m_Ae3_GREMLION);

		// load model related paras
		m_A_GREMLION.Reset(11, 11, (double *)_m_A_GREMLION, TRUE);
		m_B_GREMLION.Reset(11, 4, (double *)_m_B_GREMLION, TRUE);
		_parser.GetVariable("_m_A_GREMLION", m_A_GREMLION);
		_parser.GetVariable("_m_B_GREMLION", m_B_GREMLION);
	}
	else if (_HELICOPTER == ID_QUADLION) {
		// load model related paras
		m_A_QUADLION.Reset(4, 4, (double *)_m_A_QUADLION, TRUE);
		m_B_QUADLION.Reset(4, 4, (double *)_m_B_QUADLION, TRUE);
		_parser.GetVariable("_m_A_QUADLION", m_A_QUADLION);
		_parser.GetVariable("_m_B_QUADLION", m_B_QUADLION);

		m_A2_QUADLION.Reset(4, 4, (double *)_m_A_QUADLION, TRUE);
		m_A3_QUADLION.Reset(4, 4, (double *)_m_A_QUADLION, TRUE);
		clsMatrix::X(m_A_QUADLION, m_A_QUADLION, m_A2_QUADLION);
		clsMatrix::X(m_A2_QUADLION, m_A_QUADLION, m_A3_QUADLION);

		m_Kalman_A_WF.Reset(6, 6, (double *)_m_Kalman_A_WF, TRUE);
		m_Kalman_B_WF.Reset(6, 3, (double *)_m_Kalman_B_WF, TRUE);
		m_Kalman_C_WF.Reset(3, 6, (double *)_m_Kalman_C_WF, TRUE);
		m_Kalman_Q_WF.Reset(3, 3, (double *)_m_Kalman_Q_WF, TRUE);
		m_Kalman_R_WF.Reset(3, 3, (double *)_m_Kalman_R_WF, TRUE);
		m_Kalman_P_WF.Reset(6, 6, (double *)_m_Kalman_P_WF, TRUE);
		m_BQBt_WF.Reset(6, 6, (double *)_m_BQBt_WF, TRUE);

		_parser.GetVariable("_Kalman_A_WF", m_Kalman_A_WF);
		_parser.GetVariable("_Kalman_B_WF", m_Kalman_B_WF);
		_parser.GetVariable("_Kalman_C_WF", m_Kalman_C_WF);
		_parser.GetVariable("_Kalman_Q_WF", m_Kalman_Q_WF);
		_parser.GetVariable("_Kalman_R_WF", m_Kalman_R_WF);
		_parser.GetVariable("_Kalman_P_WF", m_Kalman_P_WF);
		_parser.GetVariable("_BQBt_WF", m_BQBt_WF);

		::memset(_m_xVision_WF, 0, sizeof(double)*6);
		m_bMeasureUpdate = FALSE; 	// initial no vision fusion, start when vision data come

		// load Z axis needed Kalman matrices
		m_Kalman_A_ZAxis.Reset(2, 2, (double *)_m_Kalman_A_ZAxis, TRUE);
		m_Kalman_B_ZAxis.Reset(2, 1, (double *)_m_Kalman_B_ZAxis, TRUE);
		m_Kalman_C_ZAxis.Reset(1, 2, (double *)_m_Kalman_C_ZAxis, TRUE);
		m_Kalman_Q_ZAxis.Reset(1, 1, (double *)_m_Kalman_Q_ZAxis, TRUE);
		m_Kalman_R_ZAxis.Reset(1, 1, (double *)_m_Kalman_R_ZAxis, TRUE);
		m_Kalman_P_ZAxis.Reset(2, 2, (double *)_m_Kalman_P_ZAxis, TRUE);
		m_BQBt_ZAxis.Reset(2, 2, (double *)_m_BQBt_ZAxis, TRUE);

		_parser.GetVariable("_Kalman_A_ZAxis", m_Kalman_A_ZAxis);
		_parser.GetVariable("_Kalman_B_ZAxis", m_Kalman_B_ZAxis);
		_parser.GetVariable("_Kalman_C_ZAxis", m_Kalman_C_ZAxis);
		_parser.GetVariable("_Kalman_Q_ZAxis", m_Kalman_Q_ZAxis);
		_parser.GetVariable("_Kalman_R_ZAxis", m_Kalman_R_ZAxis);
		_parser.GetVariable("_Kalman_P_ZAxis", m_Kalman_P_ZAxis);
		_parser.GetVariable("_BQBt_ZAxis", m_BQBt_ZAxis);

		::memset(_m_x_ZAxis, 0, sizeof(double)*2);
	}

	m_tObserve2 = -1;
	m_xr.Reset(4, _m_xr, TRUE);

	m_Ar.Reset(4, 4, (double *)_m_Ar, TRUE);
	m_Br.Reset(4, 2, (double *)_m_Br, TRUE);

	m_Ar2.Reset(4, 4, (double *)_m_Ar2, TRUE);
	m_Ar3.Reset(4, 4, (double *)_m_Ar3, TRUE);

	clsMatrix::X(m_Ar, m_Ar, m_Ar2);
	clsMatrix::X(m_Ar2, m_Ar, m_Ar3);

	m_equ = _equ_Hover;				//equ for model simulation

	m_A.Reset(11, 11, (double *)_m_A, TRUE);
	m_B.Reset(11, 4, (double *)_m_B, TRUE);

//	m_A = (double *)_A_Hover;
//	m_B = (double *)_B_Hover;

	_parser.GetVariable("_A_Hover", m_A);
	_parser.GetVariable("_B_Hover", m_B);

	m_A2.Reset(11, 11, (double *)_m_A2, TRUE);
	m_A3.Reset(11, 11, (double *)_m_A3, TRUE);

	clsMatrix::X(m_A, m_A, m_A2);
	clsMatrix::X(m_A2, m_A, m_A3);

	m_pData = NULL;
	m_nData = 0;
	m_iData = 0;

}

