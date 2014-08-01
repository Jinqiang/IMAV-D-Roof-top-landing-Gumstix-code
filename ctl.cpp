//ctl.cpp
//implementation file for automatic control
#include <stdio.h>
#include <pthread.h>
#include <iostream.h>

#include "matrix.h"
#include "uav.h"
#include "ctl.h"
#include "cmm.h"
#include "coop.h"
#include "daq.h"
#include "parser.h"
#include "svo.h"
#include "laser.h"
#include "state.h"
#include "im9.h"

extern clsParser _parser;
extern clsURG _urg;
extern clsCMM _cmm;
extern clsState _state;
extern clsURG _urg;
extern clsIM9 _im9;

extern EQUILIBRIUM _equ_Hover;

#define SAFMC_CUTENGINE_HEIGHT (-0.06) // in local-ned frame, downward -> positive
#define LANDING_CUTENGINE_HEIGHT_OUTDOOR (-0.06)
#define LANDING_CUTENGINE_HEIGHT_VICON (-0.34)
#define SAFMC_TARGET_X (9.2)
#define SAFMC_TARGET_Y (-3.5)

clsPath::clsPath()
{
  SAMFC = FALSE;
}

double clsPath::GetMappedRefTimeOnNorm(double tPath, double ldPos[3])
{
	int m = m_path.GetM() - 1;
	int iMin = 0;
	int iMax = m;

	double tMin = m_path[iMin][0];
	double tMax = m_path[iMax][0];

	if (tPath < tMin)
		tPath = m_path[iMin][0];
	if (tPath > tMax)
		tPath = m_path[iMax][0];

	while (iMin < iMax)
	{
		int iMid = (iMin+iMax)/2;
		if (tPath > m_path[iMid][0])
			iMin = iMid;
		else
			iMax = iMid;
	}
	// search time range [tPath-2s, tPath+2s]
	double tMinSearch = m_path[iMin][0] - 2;
	double tMaxSearch = m_path[iMin][0]	+ 2;

	int iMinSearch1 = 0;
	int iMaxSearch1 = m;

	if (tMinSearch < m_path[iMinSearch1][0])
		tMinSearch = m_path[iMinSearch1][0];

	if (tMaxSearch > m_path[iMaxSearch1][0])
		tMaxSearch = m_path[iMaxSearch1][0];

	while (iMinSearch1 < iMaxSearch1)
	{
		int iMidSearch1 = (iMinSearch1+iMaxSearch1)/2;
		if ( m_path[iMidSearch1][0] < tMinSearch )
			iMinSearch1 = iMidSearch1;
		else
			iMaxSearch1 = iMidSearch1;
	}
//	tMinSearch = m_path[iMinSearch1][0];	// find the index iMinSearch1


	// find the index of tPath+2
	int iMinSearch2 = 0;
	int iMaxSearch2 = m;

	if (tMinSearch < m_path[iMinSearch2][0])
		tMinSearch = m_path[iMinSearch2][0];

	if (tMaxSearch > m_path[iMaxSearch2][0])
		tMaxSearch = m_path[iMaxSearch2][0];

	while (iMinSearch2 < iMaxSearch2)
	{
		int iMidSearch2 = (iMinSearch2+iMaxSearch2)/2;
		if ( m_path[iMidSearch2][0] < tMaxSearch )
			iMinSearch2 = iMidSearch2;
		else
			iMaxSearch2 = iMidSearch2;
	}
	tMaxSearch = m_path[iMinSearch2][0];		// find the index iMinSearch2


	double minNorm = 0;
	int iMinNorm = 0;
	double norm = 0;
	double de[3];
	for (int i = iMinSearch1; i<= iMinSearch2; i++)
	{
		minNorm = norm;
		double ldRef[3] = {m_path[i][1], m_path[i][2], m_path[i][3]};
		norm = ::clsMetric::Distance(ldPos, ldRef, de);

		if (norm < minNorm)
			iMinNorm = i;
	}

	return m_path[iMinNorm][0];


}

void clsPath::GetPosVelAcc(double time, double pos[4], double vel[3], double acc[3])
{
	clsMatrix &path = m_path;

	double t = time;
	int iMin = 0; int iMax = path.GetM()-1;

	double tMin = path[iMin][0];
	double tMax = path[iMax][0];
//	double durance = tMax-tMin;

/*	if (bRepeat && durance >= 0.001)
	{				//prevent zero divident
		double tin = t-tMin;
		tin -= ::floor(tin/durance)*durance;
		t = tMin + tin;
	}*/

	if (t >= tMax)
	{
		if (pos != NULL)
		{
			pos[0] = path[iMax][1];
			pos[1] = path[iMax][2];
			pos[2] = path[iMax][3];
			pos[3] = path[iMax][4];
		}

		if (vel != NULL)
		{
//			vel[0] = vel[1] = vel[2] = 0;
			vel[0] = path[iMax][5];
			vel[1] = path[iMax][6];
			vel[2] = path[iMax][7];
		}

		if (acc != NULL)
		{
//			acc[0] = acc[1] = acc[2] = 0;
			acc[0] = path[iMax][8];
			acc[1] = path[iMax][9];
			acc[2] = path[iMax][10];
		}
		return;
	}

	if (t <= tMin) {
		if (pos != NULL) {
			pos[0] = path[iMin][1];
			pos[1] = path[iMin][2];
			pos[2] = path[iMin][3];
			pos[3] = path[iMin][4];
		}

		if (vel != NULL) {
			vel[0] = path[iMin][5];
			vel[1] = path[iMin][6];
			vel[2] = path[iMin][7];
		}

		if (acc != NULL) {
			acc[0] = path[iMin][8];
			acc[1] = path[iMin][9];
			acc[2] = path[iMin][10];
		}
		return;
	}

	while (iMax - iMin > 1)
	{
		int iMid = (iMin+iMax)/2;

		if (path[iMid][0] < t) iMin = iMid;
		else iMax = iMid;
	}

	double t1 = path[iMin][0];
	double t2 = path[iMax][0];
	double r2 = (t-t1)/(t2-t1);
	double r1 = (t2-t)/(t2-t1);

	if (pos != NULL)
	{
		pos[0] = r1*path[iMin][1] + r2*path[iMax][1];
		pos[1] = r1*path[iMin][2] + r2*path[iMax][2];
		pos[2] = r1*path[iMin][3] + r2*path[iMax][3];

//		cout<<"pos0: "<<pos[0]<<" pos1: "<<pos[1]<<endl;
		double c1 = path[iMin][4];
		double dc = path[iMax][4]-path[iMin][4];
		INPI(dc);

		double c;
		if (t < tMin) c = path[iMin][4];
		else if (t > tMax) c = path[iMax][4];
		else c = c1 + r2*dc;
		INPI(c);

		pos[3] = c;
	}

	if (vel != NULL)
	{
		vel[0] = r1*path[iMin][5] + r2*path[iMax][5];	// ugr
		vel[1] = r1*path[iMin][6] + r2*path[iMax][6];	// vgr
		vel[2] = r1*path[iMin][7] + r2*path[iMax][7];	// wgr
	}

	if (acc != NULL)
	{
		acc[0] = r1*path[iMin][8] + r2*path[iMax][8];
		acc[1] = r1*path[iMin][9] + r2*path[iMax][9];
		acc[2] = r1*path[iMin][10] + r2*path[iMax][10];
	}
}
void clsPath::GetPositionVelocity(double time, double pos[4], double vel[4], BOOL bRepeat)
{
	//pos to store x, y, z and c
	//vel to store u, v, w and r

	clsMatrix &path = m_path;

	double t = time;
	double tTakeOff = 6;
	double tLand = 200;
	int iMin = 0; int iMax = path.GetM()-1;

	double tMin = path[iMin][0];
	double tMax = path[iMax][0];
	double durance = tMax-tMin;

	if (bRepeat && durance >= 0.001)
	{				//prevent zero divident
		double tin = t-tMin;
		tin -= ::floor(tin/durance)*durance;
		t = tMin + tin;
	}

	if (t >= tMax)
	{
		if (pos != NULL)
		{
			pos[0] = path[iMax][1];
			pos[1] = path[iMax][2];
			pos[2] = path[iMax][3];
			pos[3] = path[iMax][4];
		}

		if (vel != NULL)
		{
			vel[0] = vel[1] = vel[2] = vel[3] = 0;
		}

		return;
	}

	if (t <= tMin) {
		if (pos != NULL) {
			pos[0] = path[iMin][1];
			pos[1] = path[iMin][2];
			pos[2] = path[iMin][3];
			pos[3] = path[iMin][4];
		}
		if (vel != NULL) {
			vel[0] = vel[1] = vel[2] = vel[3] = 0;
		}
		return;
	}

	while (iMax - iMin > 1)
	{
		int iMid = (iMin+iMax)/2;

		if (path[iMid][0] < t) iMin = iMid;
		else iMax = iMid;
	}

	if ( time<=tTakeOff+8)
	{
		if(time>=tTakeOff)
		{
		path[iMin+1][1] = init_state.x + 2*init_state.u;
		path[iMin+1][2] = init_state.y + 2*init_state.v;
		//path[iMin+1][3] = init_state.z;
		path[iMin+1][4] = init_state.c;
	//    cout<<"x is"<<init_state.x<<endl;
		}
		double t1 = path[iMin][0];
		double t2 = path[iMax][0];
		double r2 = (t-t1)/(t2-t1);
		double r1 = (t2-t)/(t2-t1);

		double t_0_1,t_0_2;
		double t_1_1,t_1_2;
		double t_2_1,t_2_2;
		int sign1,sign2,sign3;
		double a;
	    if(path[iMax][1]-path[iMin][1]<0)
	    	sign1 = -1;
	    else
	    	sign1 = 1;


	    if(path[iMax][2]-path[iMin][2]<0)
	    	sign2 = -1;
	    else
	    	sign2 = 1;

	    if(path[iMax][3]-path[iMin][3]<0)
	    	sign3 = -1;
	    else
	    	sign3 = 1;


		if(SAMFC&&time<tTakeOff)
			  a = 1;
		else if(SAMFC&&time>tLand&&time<tLand+3)
			 a = 0.6;
		else
		{
			a = 0.2;//accleration

		}



	    t_0_1 = (a*(t2-t1)-sqrt(a*a*(t2-t1)*(t2-t1)-4*a*fabs(path[iMax][1]-path[iMin][1])))/(2*a);
	    t_0_2 = (a*(t2-t1)+sqrt(a*a*(t2-t1)*(t2-t1)-4*a*fabs(path[iMax][1]-path[iMin][1])))/(2*a);



	    t_1_1 = (a*(t2-t1)-sqrt(a*a*(t2-t1)*(t2-t1)-4*a*fabs(path[iMax][2]-path[iMin][2])))/(2*a);
	    t_1_2 = (a*(t2-t1)+sqrt(a*a*(t2-t1)*(t2-t1)-4*a*fabs(path[iMax][2]-path[iMin][2])))/(2*a);


	    t_2_1 = (a*(t2-t1)-sqrt(a*a*(t2-t1)*(t2-t1)-4*a*fabs(path[iMax][3]-path[iMin][3])))/(2*a);
	    t_2_2 = (a*(t2-t1)+sqrt(a*a*(t2-t1)*(t2-t1)-4*a*fabs(path[iMax][3]-path[iMin][3])))/(2*a);


        if (pos != NULL)
	{
            if(t-t1<=t_0_1)
            	pos[0] = sign1*(t-t1)*(t-t1)*0.5*a + path[iMin][1];
            else if(t-t1<=t_0_2)
            	pos[0] = path[iMin][1] + sign1*(0.5*a*t_0_1*t_0_1+(t-t1-t_0_1)*(a*t_0_1));
            else
            	pos[0] = path[iMax][1] - sign1*0.5*a*(t2-t)*(t2-t);
      /*  if(t-t1<=t_0_1)
        	pos[0] = sign1*(t-t1)*(t-t1)*0.5*a + path[iMin][1];
        else if(t-t1<t_0_2)
        	pos[0] = path[iMin][1] + sign1*(0.5*a*t_0_1*t_0_1+(t-t1-t_0_1)*(a*t_0_1));
        else
        	pos[0] = path[iMax][1] - sign1*0.5*a*(t2-t)*(t2-t);*/

        if(t-t1<=t_1_1)
        	pos[1] = sign2*(t-t1)*(t-t1)*0.5*a + path[iMin][2];
        else if(t-t1<=t_1_2)
        	pos[1] = path[iMin][2] + sign2*(0.5*a*t_1_1*t_1_1+(t-t1-t_1_1)*(a*t_1_1));
        else
        	pos[1] = path[iMax][2] - sign2*0.5*a*(t2-t)*(t2-t);

        if(t-t1<=t_2_1)
        	pos[2] = sign3*(t-t1)*(t-t1)*0.5*a + path[iMin][3];
        else if(t-t1<t_2_2)
        	pos[2] = path[iMin][3] + sign3*(0.5*a*t_2_1*t_2_1+(t-t1-t_2_1)*(a*t_2_1));
        else
        	pos[2] = path[iMax][3] - sign3*0.5*a*(t2-t)*(t2-t);


//		cout<<"pos0: "<<pos[0]<<" pos1: "<<pos[1]<<endl;
		double c1 = path[iMin][4];
		double dc = path[iMax][4]-path[iMin][4];
		INPI(dc);

		double c;
		if (t < tMin) c = path[iMin][4];
		else if (t > tMax) c = path[iMax][4];
		else c = c1 + r2*dc;
		INPI(c);

		pos[3] = c;
	}

	if (vel != NULL)
	{
        if(t-t1<=t_0_1)
        	vel[0] = (t-t1)*a;
        else if(t-t1<t_0_2)
        	vel[0] = (t_0_1)*a;
        else
        	vel[0] = (t2-t)*a;
            vel[0] = vel[0]*sign1;

        if(t-t1<=t_1_1)
          	vel[1] = (t-t1)*a;
          else if(t-t1<t_1_2)
          	vel[1] = (t_1_1)*a;
          else
          	vel[1] = (t2-t)*a;
        vel[1] = vel[1]*sign2;

        if(t-t1<=t_2_1)
          	vel[2] = (t-t1)*a;
          else if(t-t1<t_2_2)
          	vel[2] = (t_2_1)*a;
          else
          	vel[2] = (t2-t)*a;
        vel[2] = vel[2]*sign3;

        double dt = t2-t1;
		double dc = path[iMax][4]-path[iMin][4];
		INPI(dc);
		vel[3] = dc/dt;				//rg

	}
	}

}
void clsPath::GetPositionVelocity1(double time, double pos[4], double vel[4], double acc[3], BOOL bRepeat)
{
	//pos to store x, y, z and c
	//vel to store u, v, w and r

	clsMatrix &path = m_path;

	double t = time;
	double tTakeOff = 6;
	int iMin = 0;
	int iMax = path.GetM()-1;

	double tMin = path[iMin][0];
	double tMax = path[iMax][0];
	double durance = tMax-tMin;

	if (bRepeat && durance >= 0.001)
	{				//prevent zero divident
		double tin = t-tMin;
		tin -= ::floor(tin/durance)*durance;
		t = tMin + tin;
	}

	if (t >= tMax)
	{
		if (pos != NULL)
		{
			pos[0] = path[iMax][1];
			pos[1] = path[iMax][2];
			pos[2] = path[iMax][3];
			pos[3] = path[iMax][4];
		}

		if (vel != NULL)
		{
			vel[0] = vel[1] = vel[2] = vel[3] = 0;
		}

		return;
	}

	if (t <= tMin) {
		if (pos != NULL) {
			pos[0] = path[iMin][1];
			pos[1] = path[iMin][2];
			pos[2] = path[iMin][3];
			pos[3] = path[iMin][4];
		}
		if (vel != NULL) {
			vel[0] = vel[1] = vel[2] = vel[3] = 0;
		}
		return;
	}


	while (iMax - iMin > 1)
	{
		int iMid = (iMin+iMax)/2;

		if (path[iMid][0] < t)
			iMin = iMid;
		else
			iMax = iMid;
	}

	if ( time<=tTakeOff || time>tLand) // While taking off or landing
	{
		double t1 = path[iMin][0];
		double t2 = path[iMax][0];
		double r2 = (t-t1)/(t2-t1);
		double r1 = (t2-t)/(t2-t1);

		double t_0_1, t_0_2;
		double t_1_1, t_1_2;
		double t_2_1, t_2_2;
		int sign1, sign2, sign3;
		double a;
	    if (path[iMax][1]-path[iMin][1]<0)
	    	sign1 = -1;
	    else
	    	sign1 = 1;

	    if(path[iMax][2]-path[iMin][2]<0)
	    	sign2 = -1;
	    else
	    	sign2 = 1;

	    if(path[iMax][3]-path[iMin][3]<0)
	    	sign3 = -1;
	    else
	    	sign3 = 1;

	    acc[0] = 0;
	    acc[1] = 0;

		a = 0.2; // Slope of velocity OR acceleration

	    t_0_1 = (a*(t2-t1)-sqrt(a*a*(t2-t1)*(t2-t1)-4*a*fabs(path[iMax][1]-path[iMin][1])))/(2*a);
	    t_0_2 = (a*(t2-t1)+sqrt(a*a*(t2-t1)*(t2-t1)-4*a*fabs(path[iMax][1]-path[iMin][1])))/(2*a);

	    t_1_1 = (a*(t2-t1)-sqrt(a*a*(t2-t1)*(t2-t1)-4*a*fabs(path[iMax][2]-path[iMin][2])))/(2*a);
	    t_1_2 = (a*(t2-t1)+sqrt(a*a*(t2-t1)*(t2-t1)-4*a*fabs(path[iMax][2]-path[iMin][2])))/(2*a);

	    t_2_1 = (a*(t2-t1)-sqrt(a*a*(t2-t1)*(t2-t1)-4*a*fabs(path[iMax][3]-path[iMin][3])))/(2*a);
	    t_2_2 = (a*(t2-t1)+sqrt(a*a*(t2-t1)*(t2-t1)-4*a*fabs(path[iMax][3]-path[iMin][3])))/(2*a);

        if (pos != NULL)
        {
            if(t-t1<=t_0_1)
            	pos[0] = sign1*(t-t1)*(t-t1)*0.5*a + path[iMin][1];
            else if(t-t1<=t_0_2)
            	pos[0] = path[iMin][1] + sign1*(0.5*a*t_0_1*t_0_1+(t-t1-t_0_1)*(a*t_0_1));
            else
            	pos[0] = path[iMax][1] - sign1*0.5*a*(t2-t)*(t2-t);

			if(t-t1<=t_1_1)
				pos[1] = sign2*(t-t1)*(t-t1)*0.5*a + path[iMin][2];
			else if(t-t1<=t_1_2)
				pos[1] = path[iMin][2] + sign2*(0.5*a*t_1_1*t_1_1+(t-t1-t_1_1)*(a*t_1_1));
			else
				pos[1] = path[iMax][2] - sign2*0.5*a*(t2-t)*(t2-t);

			if(t-t1<=t_2_1)
				pos[2] = sign3*(t-t1)*(t-t1)*0.5*a + path[iMin][3];
			else if(t-t1<t_2_2)
				pos[2] = path[iMin][3] + sign3*(0.5*a*t_2_1*t_2_1+(t-t1-t_2_1)*(a*t_2_1));
			else
				pos[2] = path[iMax][3] - sign3*0.5*a*(t2-t)*(t2-t);

	//		cout<<"pos0: "<<pos[0]<<" pos1: "<<pos[1]<<endl;
			double c1 = path[iMin][4];
			double dc = path[iMax][4]-path[iMin][4];
			INPI(dc);

			double c;
			if (t < tMin) c = path[iMin][4];
			else if (t > tMax) c = path[iMax][4];
			else c = c1 + r2*dc;
			INPI(c);

			pos[3] = c;
		}

		if (vel != NULL)
		{
			if(t-t1<=t_0_1)
				vel[0] = (t-t1)*a;
			else if(t-t1<t_0_2)
				vel[0] = (t_0_1)*a;
			else
				vel[0] = (t2-t)*a;
				vel[0] = vel[0]*sign1;

			if(t-t1<=t_1_1)
				vel[1] = (t-t1)*a;
			  else if(t-t1<t_1_2)
				vel[1] = (t_1_1)*a;
			  else
				vel[1] = (t2-t)*a;
			vel[1] = vel[1]*sign2;

			if(t-t1<=t_2_1)
				vel[2] = (t-t1)*a;
			else if(t-t1<t_2_2)
				vel[2] = (t_2_1)*a;
			else
				vel[2] = (t2-t)*a;
			vel[2] = vel[2]*sign3;

			double dt = t2-t1;
			double dc = path[iMax][4]-path[iMin][4];
			INPI(dc);
			vel[3] = dc/dt;				//rg
		}
	}

	else
	{
		pos[0] = path[iMax][1];
	    pos[1] = path[iMax][2];
		pos[2] = path[iMax][3];
		pos[3] = path[iMax][4];
		vel[0] = path[iMax][5];
		vel[1] = path[iMax][6];
		vel[2] = path[iMax][7];
		acc[0] = path[iMax][8];
		acc[1] = path[iMax][9];
		acc[2] = path[iMax][10];
	}
}


BOOL clsPath::LoadPath(char *pszFile)
{
	BOOL bLoad = m_path.LoadT(pszFile);

	if (!bLoad) {
		printf("[main] Load path from %s failed\n", pszFile);
		return FALSE;
	}

//	int m = m_path.GetM();
	int n = m_path.GetN();

//	bLoad = n == 5;
	bLoad = (n == 5 || n == 11) ;
	if (!bLoad) {
		printf("[main] Incorrect path size, n = %d\n", n);
		m_path.Reset();
		return FALSE;
	}

	/*
	for (int i=0; i<=m-1; i++) {
		INPI(m_path[i][4]);
	}
*/

	printf("[main] Path loaded from %s\n", pszFile);
	return bLoad;
}

double clsPath::GetEndTime()
{
	int m = m_path.GetM();
	if (m == 0) return -1;

	return m_path[m-1][0];
}

clsCTL::clsCTL()
{
	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutex_init(&m_mtxCmd, &attr);

	pthread_mutex_init(&m_mtxCTL, &attr);
}

clsCTL::~clsCTL()
{
	pthread_mutex_destroy(&m_mtxCmd);
	pthread_mutex_destroy(&m_mtxCTL);
}

void clsCTL::PutCommand(COMMAND *pCmd)
{
	pthread_mutex_lock(&m_mtxCmd);

	m_cmd = *pCmd;

	pthread_mutex_unlock(&m_mtxCmd);
}

void clsCTL::GetCommand(COMMAND *pCmd)
{
	pthread_mutex_lock(&m_mtxCmd);

	if (m_cmd.code == 0) pCmd->code = 0;
	else {
		*pCmd = m_cmd;
		m_cmd.code = 0;				//clear
	}
	pthread_mutex_unlock(&m_mtxCmd);
}

BOOL clsCTL::InitThread()
{
	//for receiving command
	m_cmd.code = 0;

	//plan
	m_pPlan = NULL;				//no plan used currently

	m_tNotify = -1;				//no datalink notify in default, activated once notify command is received
	m_bNotify = FALSE;

	m_bGPSDeny = FALSE;		// use GPS by default
	m_bUseRefInOuter = FALSE;	// use practical data by default

	m_behavior.behavior = 0;				//no behavior
	m_fControl = 0;				//no control

	m_camera = 0;

	m_nCTL = 0;

	printf("[CTL] Start\n");
	return TRUE;
}

int clsCTL::EveryRun()
{
	char msg[256];

	//model simulation
	if (_state.GetSimulationType() == SIMULATIONTYPE_MODEL)
	{
		_state.Simulate();
//		_state.Filter();
//		_state.Observe();	// observer put in the innerloop
	}

	//check user command
	COMMAND cmd;
	GetCommand(&cmd);

	if (cmd.code != 0) {
		if (ProcessCommand(&cmd)) cmd.code = 0;
	}
	//if any command processed, the determined behavior to be execute is stored in m_behavior

	//check if datalink is lost
	if (m_tNotify > 0 && ::GetTime()-m_tNotify > 5 && m_bNotify) {
		m_bNotify = FALSE;				//indicate no notify signal, only response once the datalink is lost

		UAVSTATE &state = _state.GetState();
		double pos[4] = { state.x, state.y, state.z, state.c };
		_pathTmp.CreateReturnPath(pos);
		m_behavior.behavior = BEHAVIOR_PATH;
		PUTLONG(m_behavior.parameter, (DWORD)-1);
		PUTLONG(m_behavior.parameter+4, PATHTRACKING_FIXED);				//reserve 8bytes for 64bit system
	}

	// deal with the behavior rcved from ProcessCommand() instead of plan
	//if behavior specified by user commands or datalink lose, cancel & reset plan
	if (m_behavior.behavior != 0)
	{
		if (m_pPlan != NULL) m_pPlan->Reset();
		m_pPlan = NULL;
	}

	//if plan not null, get the next behavior according to the plan
	if (m_pPlan != NULL) {
		m_pPlan->SetState(&_state.GetState());				//store the initial state when the plan begins
		BOOL bEnd = m_pPlan->Run() == 0;
		m_pPlan->GetBehavior(&m_behavior);		// assign new behavior scheduled by the plan to clsCTL::m_behavior
		if (bEnd) m_pPlan = NULL;
	}

	int nBehaviorThis = m_behavior.behavior;				//for purpose of save

	//execute behavior, dispatch behavior to corresponding control functions
	if (m_behavior.behavior != 0) 		// new behavior assigned
	{
		if (m_behavior.behavior & BEHAVIOR_ADD)
		{
			m_behavior.behavior &= ~BEHAVIOR_ADD;
			AddBehavior(&m_behavior);
		}
		else
			SetBehavior(&m_behavior);

		m_behaviorCurrent = m_behavior;
		m_behavior.behavior = 0;		//clear to continue the execution of the previous behavior

		UAVSTATE &state = _state.GetState();
		_state.Setc0(state.c);
//		_state.SetRPTState0();
		SetSemiPsi(state.c);
		::sprintf(msg, "Behavior %d(%s)", m_behaviorCurrent.behavior, GetBehaviorString(m_behaviorCurrent.behavior));
		_cmm.PutMessage(msg);
	}

	//if emergency encountered, enter emergency behavior
/*	if (_state.Emergency()) {
		DAQDATA &daq = _daq.GetDAQData();
//		if (daq.height <= 0.5)		//near ground
//			nBehaviorThis = BEHAVIOR_EMERGENCYGROUND;
//		else
			nBehaviorThis = BEHAVIOR_EMERGENCY;
		SetBehavior(nBehaviorThis);

		if (m_behaviorCurrent.behavior != nBehaviorThis) {
			::sprintf(msg, "Emergency!! Enter emergency control!!");
			_cmm.PutMessage(msg);
		}
		
		m_behaviorCurrent.behavior = nBehaviorThis;
	}
*/
	//execute control functions
//	if (m_fControl & CONTROLFLAG_B1) B1();
//	if (m_fControl & CONTROLFLAG_B2) B2();
	if (m_fControl & CONTROLFLAG_B3) B3();
	if (m_fControl & CONTROLFLAG_B4) B4();
	if (m_fControl & CONTROLFLAG_B5) B5();
	if (m_fControl & CONTROLFLAG_B6) B6();
	if (m_fControl & CONTROLFLAG_OUTERLOOP_QUADLION) Outerloop_QuadLion();
	if (m_fControl & CONTROLFLAG_B8) B8();
	if (m_fControl & CONTROLFLAG_B9) B9();
	if (m_fControl & CONTROLFLAG_B10) B10();
	if (m_fControl & CONTROLFLAG_B11) B11();
	if (m_fControl & CONTROLFLAG_B12) B12();
	if (m_fControl & CONTROLFLAG_B13) B13();
	if (m_fControl & CONTROLFLAG_B14) B14();
//	if (m_fControl & CONTROLFLAG_B15) B15();
//	if (m_fControl & CONTROLFLAG_A1) A1();
	if (m_fControl & CONTROLFLAG_A2) A2();
	if (m_fControl & CONTROLFLAG_A3) A3();
	if (m_fControl & CONTROLFLAG_A4) A4();
	if (m_fControl & CONTROLFLAG_A5) A5();
	if (m_fControl & CONTROLFLAG_A6) A6();
	if (m_fControl & CONTROLFLAG_A6) A7();
	if (m_fControl & CONTROLFLAG_A8) A8();				//engine up or down
	if (m_fControl & CONTROLFLAG_A9) A9();				//for steady ea, ee, eu, er and et
	if (m_fControl & CONTROLFLAG_INNERLOOP_QUADLION) Innerloop_QuadLion();

	//update&store control signals
	HELICOPTERRUDDER *pSig = m_fControl == 0 ? NULL : &m_sig;
	_state.UpdateSIG(pSig);
	//_state.SetCamera(m_camera);

//	double t = GetTime();

	pthread_mutex_lock(&m_mtxCTL);

	if (m_nCTL != MAX_CTL)
	{
		m_tCTL[m_nCTL] = ::GetTime();
		m_ctl[m_nCTL].nBehavior = nBehaviorThis;
		m_ctl[m_nCTL].fControl = m_fControl;

		if ( (m_innerloop == INNERLOOP_LQR) || (m_innerloop == INNERLOOP_CNF) || (m_innerloop == INNERLOOP_GAINSCHEDULE) )
		{
			m_ctl[m_nCTL].u = A1A2A3_u;
			m_ctl[m_nCTL].v = A1A2A3_v;
			m_ctl[m_nCTL].w = A1A2A3_w;
			m_ctl[m_nCTL].r = A1A2A3_r;
		}
		else if (m_innerloop == INNERLOOP_RPT) {
			m_ctl[m_nCTL].u = B_acxr_ub;
			m_ctl[m_nCTL].v = B_acyr_vb;
			m_ctl[m_nCTL].w = B_aczr_wb;
			m_ctl[m_nCTL].r = B_cr;
		}
		else if (m_innerloop == INNERLOOP_DECOUPLE)
		{
			m_ctl[m_nCTL].u = A5_u;
			m_ctl[m_nCTL].v = A5_v;
			m_ctl[m_nCTL].w = A5_w;
			m_ctl[m_nCTL].r = A5_c;
		}

		m_ctl[m_nCTL].u = B5_vnr[0];
		m_ctl[m_nCTL].v = B5_vnr[1];
		m_ctl[m_nCTL].w = B5_vnr[2];
		m_ctl[m_nCTL].r = 0;

		m_ctl[m_nCTL].vChirp[0] = B5_pnr[0]; //B5_xref;
		m_ctl[m_nCTL].vChirp[1] = B5_pnr[1]; //B5_yref;
		m_ctl[m_nCTL].vChirp[2] = B5_pnr[2]; //B5_zref;
		m_ctl[m_nCTL].vChirp[3] = B5_pnr[3]; //B5_cref;

		m_nCTL ++;

	}

	pthread_mutex_unlock(&m_mtxCTL);

	return TRUE;
}

void clsCTL::ExitThread()
{
	printf("[CTL] quit\n");
}

BOOL clsCTL::ProcessCommand(COMMAND *pCommand)
{
//	cout<<"clsCTL::ProcessCommand"<<endl;
	COMMAND &cmd = *pCommand;
	char *paraCmd = cmd.parameter;

	int &behavior = m_behavior.behavior;
	char *paraBeh = m_behavior.parameter;

	BOOL bProcess = TRUE;

	switch (cmd.code) {
	case COMMAND_TAKEOFF:
		behavior = BEHAVIOR_TAKEOFF;
		break;
	case COMMAND_LAND:
		behavior = BEHAVIOR_LAND;
		break;
	case COMMAND_ENGINEUP:
		behavior = BEHAVIOR_ENGINEUP;
		break;
	case COMMAND_ENGINEDOWN:
		behavior = BEHAVIOR_ENGINEDOWN;
		break;
	case COMMAND_EMERGENCY:
		behavior = BEHAVIOR_EMERGENCY;
		break;
	case COMMAND_EMERGENCYGROUND:
		behavior = BEHAVIOR_EMERGENCYGROUND;
		break;

	case COMMAND_DESCEND:
		behavior = BEHAVIOR_FLY;
		memset(paraBeh, 0, 4*sizeof(double));				//u,v,w,r
		memcpy(paraBeh+16, paraCmd, 8);				//w
		break;

	case COMMAND_LIFT: {
		behavior = BEHAVIOR_FLY;
		memset(paraBeh, 0, 4*sizeof(double));				//u,v,w,r

		double w = -GETDOUBLE(paraCmd);
		PUTDOUBLE(paraBeh+16, w);				//w
		break; }

	case COMMAND_ENGINE:
		behavior = BEHAVIOR_ENGINE;
		memcpy(paraBeh, paraCmd, sizeof(double));				//value of engine
		break;

	case COMMAND_HEADTO:
		behavior = BEHAVIOR_HEADTO;
		memcpy(paraBeh, paraCmd, 3*sizeof(double));				//x, y, z
		break;

	case COMMAND_HFLY: {
		UAVSTATE &state = _state.GetState();
		behavior = BEHAVIOR_HFLY;
		memcpy(paraBeh, paraCmd, 4*sizeof(double));				//x, y, z, v_2d - heading frame

		int nPath = -1;
		int nMode = PATHTRACKING_ADDON;

		if (_HELICOPTER == ID_GREMLION) {
			B5_outerloopMode = MODE_NAVIGATION;
			B5_bSemi1stFlag = TRUE;
		}

		double x = GETDOUBLE(paraBeh);
		double y = GETDOUBLE(paraBeh + 8);
		double z = GETDOUBLE(paraBeh + 16);
		double vel_2d = -GETDOUBLE(paraBeh + 24);

		double hpos[3] = {x, y, z};
		_pathTmp.CreateHPath(hpos, 1, &state, vel_2d);
		clsPath *pPath = &_pathTmp;
		if (pPath != NULL && !pPath->IsEmpty()) {
			behavior = BEHAVIOR_PATH;
			PUTLONG(paraBeh, nPath);
			PUTLONG(paraBeh+4, nMode);
		}
	}
		break;

	case COMMAND_PATH: {
		int nPath = GETLONG(paraCmd);
		int nMode = GETLONG(paraCmd+4);

		clsPath *pPath = NULL;

		if (nPath == -1) pPath = &_pathTmp;
		if (nPath >= 1 && nPath <= MAX_PATH) pPath = &_path[nPath-1];


		if (pPath != NULL && !pPath->IsEmpty()) {
			behavior = BEHAVIOR_PATH;
			PUTLONG(paraBeh, nPath);
			PUTLONG(paraBeh+4, nMode);
		}
		break;
	}

	case COMMAND_GPATH: {
		int nPath = (int &)paraCmd[0];

		if (nPath < 1 || nPath > MAX_PATH) break;
		UAVSTATE &state = _state.GetState();
		double pos[4] = { state.x, state.y, state.z, state.c };
		double coor[2];
		_state.GetCoordination(&coor[0], &coor[1]);
		_pathTmp.CreateFromGPSPath(&_path[nPath-1], coor, pos);

		behavior = BEHAVIOR_PATH;
		(int &)paraBeh[0] = -1;
		(int &)paraBeh[4] = PATHTRACKING_FIXED;

		break;
	}

	case COMMAND_PATHA: {
		behavior = BEHAVIOR_PATHA;
		double nPoint = GETDOUBLE(paraCmd);
		int npoint = (int)nPoint;
		::memcpy(paraBeh, paraCmd, sizeof(double) + npoint * (sizeof(LOCATION)+2*sizeof(double)) );
//		m_smoothPath.acc = 0.2;
//		m_smoothPath.vCruise = 1.5;
		m_smoothPath.nPoints = npoint;
		m_smoothPath.curPoint = 0;
		::memcpy(m_smoothPath.waypoints, paraCmd+8, npoint * (sizeof(LOCATION)+2*sizeof(double)));
		break;
	}

	case COMMAND_DYNAMICPATHRESET:
		_pathTmp.CreateHoldingPath((double *)cmd.parameter+8);				//skip t
		break;

	case COMMAND_DYNAMICPATH:
		_pathTmp.AddPathPoint((double &)cmd.parameter[0], (double *)(cmd.parameter+8));
		break;

	case COMMAND_TRACK: {
		int nPath = (int &)paraCmd[0];
		int nMode = (int &)paraCmd[4];

		clsPath *pPath = NULL;
		if (nPath >= 1 && nPath <= MAX_PATH)
			if (!_path[nPath-1].IsEmpty()) pPath = &_path[nPath-1];

		if (pPath != NULL) {
			behavior = BEHAVIOR_AIM;
			(int &)paraBeh[0] = nPath;
			(int &)paraBeh[4] = nMode;				//reserve 8bytes for 64bit system

			(double &)paraBeh[8] = (double &)paraCmd[8];				//the objective tracking point
			(double &)paraBeh[16] = (double &)paraCmd[16];
			(double &)paraBeh[24] = (double &)paraCmd[24];
		}

		break;
	}

	case COMMAND_TEST: {
		int nTest = GETLONG(paraCmd);
		if (nTest == GET_TRIMVALUE) {
//			_svo.GetManualTrimRawData();
			_svo.SetManualTrimFlag();
			SVORAWDATA &svoRawData = _svo.GetSVODataRaw();
			_svo.SetManualTrimRawData(svoRawData);
		}
		else if (nTest == RESET_TRIMVALUE)
		{
			_svo.ResetManualTrimFlag();
		}
		else if (nTest == WP1_TRACK) {
			LOCATION pos0;
			_state.GetCoordination(&pos0.longitude, &pos0.latitude);
//			_pathTmp.CreatePath(pPoint, nPoint, &_state.GetState(), &pos0);

			//set command to path tracking
			cmd.code = COMMAND_PATH;
			PUTLONG(cmd.parameter, -1);
			PUTLONG(cmd.parameter, PATHTRACKING_FIXED);
		}
		else if (nTest == UAVFORGE_DST) {
			LOCATION pos1;
			// for simulation dst
//			pos1.latitude = 32.01291667*PI/180;
//			pos1.longitude = -81.83197222*PI/180;

			// to soccer field
//			pos1.latitude = 32.01071890932509*PI/180;
//			pos1.longitude = -81.83262855119301*PI/180;

			pos1.latitude = 32.010722222222221*PI/180;
			pos1.longitude = -81.831304166666655*PI/180;

			/// try point at launch location
//			pos1.latitude = 31.98895094869658*PI/180;
//			pos1.longitude = -81.84823803870367*PI/180;

			LOCATION loc0;
			if (_state.GetSimulationType() == SIMULATIONTYPE_MODEL) {
//				loc0.latitude = 32.01108333*PI/180;
//				loc0.longitude = -81.83261111*PI/180;
//				_state.Set(&loc0.longitude, &loc0.latitude);
				_state.GetCoordination(&loc0.longitude, &loc0.latitude);
//				_state.Coordinate(loc0.longitude, loc0.latitude, 0);

			} else {
				_state.GetCoordination(&loc0.longitude, &loc0.latitude);
			}

			if (_HELICOPTER == ID_GREMLION) {
				B5_outerloopMode = MODE_NAVIGATION;
				B5_bSemi1stFlag = TRUE;
			}
			_pathTmp.CreatePath(&pos1, &_state.GetState(), &loc0);
			clsPath *pPath = &_pathTmp;
			if (pPath != NULL && !pPath->IsEmpty()) {
				behavior = BEHAVIOR_PATH;
				PUTLONG(paraBeh, -1);
				PUTLONG(paraBeh+4, PATHTRACKING_FIXED);
			}
		}
		else if (nTest == 7) {
			LOCATION pos1;
//			pos1.latitude = 32.01291667*PI/180;
//			pos1.longitude = -81.83197222*PI/180;

			pos1.latitude = 31.953712441958*PI/180;
			pos1.longitude = -81.314196921178*PI/180;

			LOCATION loc0;
			if (_state.GetSimulationType() == SIMULATIONTYPE_MODEL) {
//				loc0.latitude = 32.01108333*PI/180;
//				loc0.longitude = -81.83261111*PI/180;
//				_state.Set(&loc0.longitude, &loc0.latitude);
				_state.GetCoordination(&loc0.longitude, &loc0.latitude);
//				_state.Coordinate(loc0.longitude, loc0.latitude, 0);

			} else {
				_state.GetCoordination(&loc0.longitude, &loc0.latitude);
			}

			if (_HELICOPTER == ID_GREMLION) {
				B5_outerloopMode = MODE_NAVIGATION;
				B5_bSemi1stFlag = TRUE;
			}
			_pathTmp.CreatePath1(&pos1, &_state.GetState(), &loc0);
			clsPath *pPath = &_pathTmp;
			if (pPath != NULL && !pPath->IsEmpty()) {
				behavior = BEHAVIOR_PATH;
				PUTLONG(paraBeh, -1);
				PUTLONG(paraBeh+4, PATHTRACKING_FIXED);
			}
		}
		else if (nTest == 8) {
			LOCATION pos1;
			pos1.latitude = 32.01291667*PI/180;
			pos1.longitude = -81.83197222*PI/180;


			LOCATION loc0;
			if (_state.GetSimulationType() == SIMULATIONTYPE_MODEL) {
//				loc0.latitude = 32.01108333*PI/180;
//				loc0.longitude = -81.83261111*PI/180;
//				_state.Set(&loc0.longitude, &loc0.latitude);
				_state.GetCoordination(&loc0.longitude, &loc0.latitude);
//				_state.Coordinate(loc0.longitude, loc0.latitude, 0);

			} else {
				_state.GetCoordination(&loc0.longitude, &loc0.latitude);
			}

			if (_HELICOPTER == ID_GREMLION) {
				B5_outerloopMode = MODE_NAVIGATION;
				B5_bSemi1stFlag = TRUE;
			}
			_pathTmp.CreatePath2(&pos1, &_state.GetState(), &loc0);
			clsPath *pPath = &_pathTmp;
			if (pPath != NULL && !pPath->IsEmpty()) {
				behavior = BEHAVIOR_PATH;
				PUTLONG(paraBeh, -1);
				PUTLONG(paraBeh+4, PATHTRACKING_FIXED);
			}
		}
		else if (nTest == 10) {
			behavior = BEHAVIOR_PATH;
			SetIndoorPath();
		}
		else if (nTest == 11) {
			SetIntegratorFlag();
		}
		else if (nTest == 12) {
			ResetIntegratorFlag();
		}
		//tests translated to control functions
//		behavior = BEHAVIOR_TEST;
//		PUTLONG(paraBeh, nTest);

		break;
	}

	case COMMAND_CHIRP: {
		behavior = BEHAVIOR_CHIRP;
		memcpy(paraBeh, paraCmd, sizeof(int)+4*sizeof(double));				//channel, amplitude, om1, om2, durance
		break;
	}

	case COMMAND_PLAN: {
		int nPlan = GETLONG(paraCmd);
		printf("[CTL] Command plan received, nPlan %d\n", nPlan);

		_ctl.SetPlan(nPlan);
		m_pPlan->SetPlanID(nPlan);
		break;
	}

	case COMMAND_NOTIFY:
		m_tNotify = ::GetTime();
		m_bNotify = TRUE;
		break;

	case COMMAND_NONOTIFY:
		m_tNotify = -1;
		m_bNotify = FALSE;
		break;

	case COMMAND_HOVER:
		behavior = BEHAVIOR_FLY;
		::memcpy(paraBeh, paraCmd, 4*sizeof(double));				//u,v,w,r
		break;

	case COMMAND_FLY: {
		behavior = BEHAVIOR_FLY;
		memcpy(paraBeh, paraCmd, 4*sizeof(double));				//x, y, z, v_2d
		break; }

	case COMMAND_HOLD:
		behavior = BEHAVIOR_HOLD;
		memcpy(paraBeh, paraCmd, 4*sizeof(double));				//x,y,z,c
		if (_HELICOPTER == ID_GREMLION) {
			B5_outerloopMode = MODE_NAVIGATION;
			B5_bSemi1stFlag = TRUE;
		}
		break;

	case COMMAND_HOLDPI: {
		behavior = BEHAVIOR_HOLDPI;
		memcpy(paraBeh, paraCmd, 4*sizeof(double));				//x,y,z,c
		break; }

	case COMMAND_COORDINATE: {
		double longitude = GETDOUBLE(paraCmd);
		double latitude = GETDOUBLE(paraCmd+8);
		double altitude = GETDOUBLE(paraCmd+16);
		_state.Coordinate(longitude, latitude, altitude);
		break; }

	case COMMAND_MODE: {
		int mode = GETLONG(paraCmd);
//		SetMode(mode);		// set to semi-auto as long as "mode" cmd received
		B5_outerloopMode = MODE_SEMIAUTO;
//		if (mode == MODE_SEMIAUTO)
			behavior = BEHAVIOR_SEMIAUTO;
			B5_bSemi1stFlag = TRUE;
		break; }

	case COMMAND_PARA: {
		int nChoice = GETLONG(paraCmd);
		printf("para value: %d\n", nChoice);

#if (_DEBUG & DEBUGFLAG_CTL)
		printf("[CTL] para %d\n", nChoice);
#endif

		switch (nChoice) {
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
			char szF[MAXSTR_VARIABLENAME], szG[MAXSTR_VARIABLENAME];
//			::sprintf(szF, "_A2_F_RPT%d", nChoice);
//			::sprintf(szG, "_A2_G_RPT%d", nChoice);
//
////			_parser.GetVariable(szF, A2_F);
////			_parser.GetVariable(szG, A2_G);
//			_parser.GetVariable(szF, A2_F_RPT);
//			_parser.GetVariable(szG, A2_G_RPT);
			::sprintf(szF, "_A2_F_GREMLION%d", nChoice);
			::sprintf(szG, "_A2_G_GREMLION%d", nChoice);
			_parser.GetVariable(szF, A2_F_GREMLION);
			_parser.GetVariable(szG, A2_G_GREMLION);
			
			char szFVn[MAXSTR_VARIABLENAME];
			::sprintf(szFVn, "B5_FVn%d", nChoice);
			_parser.GetVariable(szFVn, &B5_FVn);
/*			::sprintf(szF, "_B5_F%d", nChoice);
			::sprintf(szG, "_B5_G%d", nChoice);
			_parser.GetVariable(szF, B5_F);
			_parser.GetVariable(szG, B5_G);*/
			
			break; 

//		case 5:
//			SetGPSDeny();
/*			char szF[MAXSTR_VARIABLENAME], szG[MAXSTR_VARIABLENAME];
			::sprintf(szF, "_B5_F%d", nChoice);
			::sprintf(szG, "_B5_G%d", nChoice);

			_parser.GetVariable(szF, B5_F);
			_parser.GetVariable(szG, B5_G);*/
			break;
//		case 6:
//			ResetGPSDeny();
			break;

/*		case 7:
			SetRefUseInOuter();		// use reference in outerloop
			break;*/
		case 8:
			ResetRefUseInOuter();	// use practical data in outerloop
			break;

		case 9:
			_parser.GetVariable("_Ae0", _state.m_Ae);
			_parser.GetVariable("_Be0", _state.m_Be);
			_parser.GetVariable("_Ce0", _state.m_Ce);
			_parser.GetVariable("_De0", _state.m_De);
			break;

		case 10:
			_parser.GetVariable("_Ae1", _state.m_Ae);
			_parser.GetVariable("_Be1", _state.m_Be);
			_parser.GetVariable("_Ce1", _state.m_Ce);
			_parser.GetVariable("_De1", _state.m_De);
			break;

		case 11:
			_parser.GetVariable("_Ae2", _state.m_Ae);
			_parser.GetVariable("_Be2", _state.m_Be);
			_parser.GetVariable("_Ce2", _state.m_Ce);
			_parser.GetVariable("_De2", _state.m_De);
			break;

		case 12:
			B5_kx = B5_ky = B5_kz = -0.25;
			break;
		case 13:
			B5_kx = B5_ky = B5_kz = -0.15;
			break;
/*		case 14:
			B5_kx = B5_ky = B5_kz = -0.1;
			break;*/
		case 14:
			B5_kx = B5_ky = B5_kz = -0.05;
			break;
		/* Ali's outerloop Kp: case 15, 16 */
		case 15:
			B5_kx = -0.3; B5_ky = -0.3; B5_kz = -0.3; B5_kc = -0.7;
			break;
		case 16:
			B5_kx = -0.3; B5_ky = -0.3; B5_kz = -0.5; B5_kc = -1;
			break;

		case 17:
			m_innerloop = INNERLOOP_LQR;
			break;

		case 18:
			m_innerloop = INNERLOOP_GAINSCHEDULE;
			break;

		case 19:
			m_innerloop = INNERLOOP_DECOUPLE;
			break;

		case 20:
			m_innerloop = INNERLOOP_RPT;
			break;

		case 21:
			B5_kc = -0.7;
			break;

		case 22:
			B5_kc = -0.4;
			break;

		case 23:
			B5_kc = -0.2;
			break;

		case 31:
			B6_add = CHIRPADD_OUTERLOOP;
			break;

		case 32:
			B6_add = CHIRPADD_CONTROLSIGNAL;
			break;

		case 51:
		case 52:
		case 53:
		case 54:
		case 55:
			m_plan1.SetPath(nChoice-50);
			break;

		default:
			bProcess = FALSE;
			break;
		}
		break;
	}

	default:
		bProcess = FALSE;
		break;
	}

	return bProcess;
}

void clsCTL::A1_Lookup(double velb[3], EQUILIBRIUM& equ, clsMatrix &F, clsMatrix &G)
{
//	equ = A1_equ0;

	char szF1[16], szF2[16], szG1[16], szG2[16], szEqu1[16], szEqu2[16];
	double _F1[4][11]; clsMatrix F1;
	double _F2[4][11]; clsMatrix F2;
	double _G1[4][4]; clsMatrix G1;
	double _G2[4][4]; clsMatrix G2;

	F1.Reset(4, 11, (double *)_F1, TRUE);
	F2.Reset(4, 11, (double *)_F2, TRUE);
	G1.Reset(4, 4, (double *)_G1, TRUE);
	G2.Reset(4, 4, (double *)_G2, TRUE);

	EQUILIBRIUM equ1, equ2;
	int nEqu1 = 0; int nEqu2 = 0;
	int nCtl1 = 0; int nCtl2 = 0;
//	int nCondUV = 0;
	double ub = ::fabs(velb[0]);
//	double vb = ::fabs(velb[1]);
	double wCtl, wEqu;		// weighting for interpolation


	if (velb[0]<-3)
		{ nCtl1 = 1; nCtl2 = 1; wCtl = 1; }
	else if (velb[0]>=-3 && velb[0]<0)
		{ nCtl1 = 1; nCtl2 = 0; wCtl = (velb[0]+3)/3; }
	else if (velb[0]>=0 && velb[0]<3)
		{ nCtl1 = 0; nCtl2 = 1; wCtl = velb[0]/3; }
	else if (velb[0]>=3 && velb[0]<6)
		{ nCtl1 = 1; nCtl2 = 2; wCtl = (velb[0]-3)/3; }
	else if (velb[0]>=6 && velb[0]<9)
		{ nCtl1 = 2; nCtl2 = 3; wCtl = (velb[0]-6)/3; }
	else if (velb[0]>=9 && velb[0]<12)
		{ nCtl1 = 3; nCtl2 = 4; wCtl = (velb[0]-9)/3; }
	else				//velb[0]>=12
		{ nCtl1 = 4; nCtl2 = 4; wCtl = 1; }


	if (velb[0]<-4.5)
		{ nEqu1 = 0; nEqu2 = 0; wEqu = 1; }
	else if (velb[0]>=-4.5 && velb[0]<-3)
		{ nEqu1 = 0; nEqu2 = 1; wEqu = (velb[0]+4.5)/1.5; }
	else if (velb[0]>=-3 && velb[0]<-1.5)
		{ nEqu1 = 1; nEqu2 = 2; wEqu = (velb[0]+3)/1.5; }
	else if ( ub>=0 && ub<1.5 )
	{
		if (velb[1]<-6)
			{ nEqu1 = 11; nEqu2 = 11; wEqu = 1; }
		else if (velb[1]>=-6 && velb[1]<-4.5)
			{ nEqu1 = 11; nEqu2 = 12; wEqu = (velb[1]+6)/1.5; }
		else if (velb[1]>=-4.5 && velb[1]<-3)
			{ nEqu1 = 12; nEqu2 = 13; wEqu = (velb[1]+4.5)/1.5; }
		else if (velb[1]>=-3 && velb[1]<-1.5)
			{ nEqu1 = 13; nEqu2 = 14; wEqu = (velb[1]+3)/1.5; }
		else if (velb[1]>=-1.5 && velb[1]<0)
			{ nEqu1 = 14; nEqu2 = 15; wEqu = (velb[1]+1.5)/1.5; }
		else if (velb[1]>=0 && velb[1]<1.5)
			{ nEqu1 = 15; nEqu2 = 16; wEqu = velb[1]/1.5; }
		else if (velb[1]>=1.5 && velb[1]<3)
			{ nEqu1 = 16; nEqu2 = 17; wEqu = (velb[1]-1.5)/1.5; }
		else if (velb[1]>=3 && velb[1]<4.5)
			{ nEqu1 = 17; nEqu2 = 18; wEqu = (velb[1]-3)/1.5; }
		else if (velb[1]>=4.5 && velb[1]<6)
			{ nEqu1 = 18; nEqu2 = 19; wEqu = (velb[1]-4.5)/1.5; }
		else 				//velb[1]>=6
			{ nEqu1 = 19; nEqu2 = 19; wEqu = 1; }
	}

	else if (velb[0]>=1.5 && velb[0]<3)
		{ nEqu1 = 3; nEqu2 = 4; wEqu = (velb[0]-1.5)/1.5; }
	else if (velb[0]>=3 && velb[0]<4.5)
		{ nEqu1 = 4; nEqu2 = 5; wEqu = (velb[0]-3)/1.5; }
	else if (velb[0]>=4.5 && velb[0]<6)
		{ nEqu1 = 5; nEqu2 = 6; wEqu = (velb[0]-4.5)/1.5; }
	else if (velb[0]>=6 && velb[0]<7.5)
		{ nEqu1 = 6; nEqu2 = 7; wEqu = (velb[0]-6)/1.5; }
	else if (velb[0]>=7.5 && velb[0]<9)
		{ nEqu1 = 7; nEqu2 = 8; wEqu = (velb[0]-7.5)/1.5; }
	else if (velb[0]>=9 && velb[0]<10.5)
		{ nEqu1 = 8; nEqu2 = 9; wEqu = (velb[0]-9)/1.5; }
	else if (velb[0]>=10.5 && velb[0]<12)
		{ nEqu1 = 9; nEqu2 = 10; wEqu = (velb[0]-10.5)/1.5; }
	else				//velb[0]>12
		{ nEqu1 = 10; nEqu2 = 10; wEqu = 1; }


	::sprintf(szEqu1, "_equ%d", nEqu1); ::sprintf(szEqu2, "_equ%d", nEqu2);
	::sprintf(szF1, "_A1_F%d", nCtl1);	::sprintf(szF2, "_A1_F%d", nCtl2);
	::sprintf(szG1, "_A1_G%d", nCtl1);	::sprintf(szG2, "_A1_G%d", nCtl2);

	_parser.GetVariable(szEqu1, &equ1, sizeof(EQUILIBRIUM)); _parser.GetVariable(szEqu2, &equ2, sizeof(EQUILIBRIUM));
	_parser.GetVariable(szF1, F1);	_parser.GetVariable(szF2, F2);
	_parser.GetVariable(szG1, G1);	_parser.GetVariable(szG2, G2);

		F2 *= wCtl; F1 *= (1-wCtl);
		F1 += F2; F = F1;

		G2 *= wCtl; G1 *= (1-wCtl);
		G1 += G2; G = G1;

	equ.u = (1-wEqu)*equ1.u + wEqu*equ2.u;
	equ.v = (1-wEqu)*equ1.v + wEqu*equ2.v;
	equ.w = (1-wEqu)*equ1.w + wEqu*equ2.w;
	equ.p = (1-wEqu)*equ1.p + wEqu*equ2.p;
	equ.q = (1-wEqu)*equ1.q + wEqu*equ2.q;
	equ.r = (1-wEqu)*equ1.r + wEqu*equ2.r;
	equ.a = (1-wEqu)*equ1.a + wEqu*equ2.a;
	equ.b = (1-wEqu)*equ1.b + wEqu*equ2.b;
	equ.c = (1-wEqu)*equ1.c + wEqu*equ2.c;
	equ.as = (1-wEqu)*equ1.as + wEqu*equ2.as;
	equ.bs = (1-wEqu)*equ1.bs + wEqu*equ2.bs;
	equ.rfb = (1-wEqu)*equ1.rfb + wEqu*equ2.rfb;

	equ.ea = wEqu*equ2.ea + (1-wEqu)*equ1.ea;
	equ.ee = wEqu*equ2.ee + (1-wEqu)*equ1.ee;
	equ.eu = wEqu*equ2.eu + (1-wEqu)*equ1.eu;
	equ.er = wEqu*equ2.er + (1-wEqu)*equ1.er;
	equ.et = wEqu*equ2.et + (1-wEqu)*equ1.et;
}

void clsCTL::A1_Lookup(double velb[3], EQUILIBRIUM& equ)
{
//	equ = A1_equ0;

	char szEqu1[16], szEqu2[16];
	EQUILIBRIUM equ1, equ2;
	int nEqu1 = 0; int nEqu2 = 0;

	double ub = ::fabs(velb[0]);
//	double vb = ::fabs(velb[1]);
	double wEqu = 0;		// weighting for interpolation


	if (velb[0]<-4.5)
		{ nEqu1 = 0; nEqu2 = 0; wEqu = 1; }
	else if (velb[0]>=-4.5 && velb[0]<-3)
		{ nEqu1 = 0; nEqu2 = 1; wEqu = (velb[0]+4.5)/1.5; }
	else if (velb[0]>=-3 && velb[0]<-1.5)
		{ nEqu1 = 1; nEqu2 = 2; wEqu = (velb[0]+3)/1.5; }
	else if ( ub>=0 && ub<1.5 )
	{
		if (velb[1]<-6)
			{ nEqu1 = 10; nEqu2 = 10; wEqu = 1; }
		else if (velb[1]>=-6 && velb[1]<-4.5)
			{ nEqu1 = 10; nEqu2 = 11; wEqu = (velb[1]+6)/1.5; }
		else if (velb[1]>=-4.5 && velb[1]<-3)
			{ nEqu1 = 11; nEqu2 = 12; wEqu = (velb[1]+4.5)/1.5; }
		else if (velb[1]>=-3 && velb[1]<-1.5)
			{ nEqu1 = 12; nEqu2 = 13; wEqu = (velb[1]+3)/1.5; }
		else if (velb[1]>=-1.5 && velb[1]<0)
			{ nEqu1 = 13; nEqu2 = 14; wEqu = (velb[1]+1.5)/1.5; }
		else if (velb[1]>=0 && velb[1]<1.5)
			{ nEqu1 = 14; nEqu2 = 15; wEqu = velb[1]/1.5; }
		else if (velb[1]>=1.5 && velb[1]<3)
			{ nEqu1 = 15; nEqu2 = 16; wEqu = (velb[1]-1.5)/1.5; }
		else if (velb[1]>=3 && velb[1]<4.5)
			{ nEqu1 = 16; nEqu2 = 17; wEqu = (velb[1]-3)/1.5; }
		else if (velb[1]>=4.5 && velb[1]<6)
			{ nEqu1 = 17; nEqu2 = 18; wEqu = (velb[1]-4.5)/1.5; }
		else if (velb[1]>=6)
			{ nEqu1 = 18; nEqu2 = 19; wEqu = 1; }
	}

	else if (velb[0]>=1.5 && velb[0]<3)
		{ nEqu1 = 2; nEqu2 = 3; wEqu = (velb[0]-1.5)/1.5; }
	else if (velb[0]>=3 && velb[0]<4.5)
		{ nEqu1 = 3; nEqu2 = 4; wEqu = (velb[0]-3)/1.5; }
	else if (velb[0]>=4.5 && velb[0]<6)
		{ nEqu1 = 4; nEqu2 = 5; wEqu = (velb[0]-4.5)/1.5; }
	else if (velb[0]>=6 && velb[0]<7.5)
		{ nEqu1 = 5; nEqu2 = 6; wEqu = (velb[0]-6)/1.5; }
	else if (velb[0]>=7.5 && velb[0]<9)
		{ nEqu1 = 6; nEqu2 = 7; wEqu = (velb[0]-7.5)/1.5; }
	else if (velb[0]>=9 && velb[0]<10.5)
		{ nEqu1 = 7; nEqu2 = 8; wEqu = (velb[0]-9)/1.5; }
	else if (velb[0]>=10.5 && velb[0]<12)
		{ nEqu1 = 8; nEqu2 = 9; wEqu = (velb[0]-10.5)/1.5; }
	else if (velb[0]>12)
		{ nEqu1 = 9; nEqu2 = 8; wEqu = 1; }

	::sprintf(szEqu1, "_equ%d", nEqu1); ::sprintf(szEqu2, "_equ%d", nEqu2);
	_parser.GetVariable(szEqu1, &equ1, sizeof(EQUILIBRIUM)); _parser.GetVariable(szEqu2, &equ2, sizeof(EQUILIBRIUM));

	equ.u = wEqu*equ2.u + (1-wEqu)*equ1.u;
	equ.v = wEqu*equ2.v + (1-wEqu)*equ1.v;
	equ.w = wEqu*equ2.w + (1-wEqu)*equ1.w;
	equ.p = wEqu*equ2.p + (1-wEqu)*equ1.p;
	equ.q = wEqu*equ2.q + (1-wEqu)*equ1.q;
	equ.r = wEqu*equ2.r + (1-wEqu)*equ1.r;
	equ.a = wEqu*equ2.a + (1-wEqu)*equ1.a;
	equ.b = wEqu*equ2.b + (1-wEqu)*equ1.b;
	equ.c = wEqu*equ2.c + (1-wEqu)*equ1.c;
	equ.as = wEqu*equ2.as + (1-wEqu)*equ1.as;
	equ.bs = wEqu*equ2.bs + (1-wEqu)*equ1.bs;
	equ.rfb = wEqu*equ2.rfb + (1-wEqu)*equ1.rfb;

	equ.ea = wEqu*equ2.ea + (1-wEqu)*equ1.ea;
	equ.ee = wEqu*equ2.ee + (1-wEqu)*equ1.ee;
	equ.eu = wEqu*equ2.eu + (1-wEqu)*equ1.eu;
	equ.er = wEqu*equ2.er + (1-wEqu)*equ1.er;
	equ.et = wEqu*equ2.et + (1-wEqu)*equ1.et;
}

void clsCTL::A2()
{
	if (_HELICOPTER == ID_GREMLION)
		A2_GremLion_New();
	else if (_HELICOPTER == ID_HELION || _HELICOPTER == ID_SHELION)
		A2_HeLion();
}

void clsCTL::A2_GremLion()
{
	SVORAWDATA &svoRawTrim = _svo.GetManualTrimRawData();
	A2_equ.ea = double(svoRawTrim.aileron - 15000)/5000;
	A2_equ.ee = double(svoRawTrim.elevator - 15000)/5000;
//	A2_equ.eu = double(svoRawTrim.auxiliary - 15000)/5000;
	A2_equ.er = double(svoRawTrim.rudder - 15000)/5000;
//	A2_equ.et = double(svoRawTrim.throttle - 15000)/5000;

	UAVSTATE &state = _state.GetState();
	double c0 = _state.Getc0();
	double velb[3] = {state.u, state.v, state.w};

//	A1_Lookup(velb, A2_equ /*, A2_F, A2_G*/);

//	double c1 = state.c - c0 /*- A2_equ.c*/; INPI(c1);
//	double c1 = state.c - c0 - B_cr; INPI(c1);
//	double c1 = state.c - c0; INPI(c1);
	double c1 = state.c - B_cr; INPI(c1);
	double _x[11] = {
		state.u - A2_equ.u, state.v - A2_equ.v,
//		0, 0,
		state.p - A2_equ.p, state.q - A2_equ.q,
		state.a - A2_equ.a, state.b - A2_equ.b,
		state.as, state.bs,
		state.w - A2_equ.w, state.r - A2_equ.r, c1
//		0, 0, c1
	};
	clsVector x(11, _x, TRUE);
	double _Fx[4]; clsVector Fx(4, _Fx, TRUE);
	clsMatrix::X(A2_F_GREMLION, x, Fx);

	double _v[4] = {
		B_acxr_ub,
		B_acyr_vb,
		B_aczr_wb,
		0	/* c2 */			/* B_cr - c0 */ 	  /*- A2_equ.c*/
	};
	clsVector v(4, _v, TRUE);
	double _Gv[4]; clsVector Gv(4, _Gv, TRUE);
	clsMatrix::X(A2_G_GREMLION, v, Gv);

	double _u[4]; clsVector u(4, _u, TRUE);
	u = Fx; u += Gv;

	HELICOPTERRUDDER sig;
	sig.aileron = A2_equ.ea+_u[0];
	sig.elevator = A2_equ.ee+_u[1]*1.2;
	sig.auxiliary = A2_equ.eu+_u[2];
	sig.rudder = A2_equ.er+_u[3];
	sig.throttle = A2_equ.et;

#if (_DEBUG & DEBUGFLAG_CTL)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[CTL] A2, ail %f, ele %f, aux %f, rud %f\n",
		sig.aileron, sig.elevator, sig.auxiliary, sig.rudder);
}
#endif
	if (m_nCount % _DEBUG_COUNT == 0) {
//		printf("[CTL] A2_equ, u %f, v %f, a %f, b %f, p %f, q %f\n",
//					A2_equ.u, A2_equ.v, A2_equ.a, A2_equ.b, A2_equ.p, A2_equ.q);
		printf("[CTL] A2_equ, ail %f, ele %f, aux %f, rud %f\n",
				A2_equ.ea, A2_equ.ee, A2_equ.eu, A2_equ.er);
	}
	m_sig = sig;
}

void clsCTL::A2_GremLion_New()
{
	SVORAWDATA &svoRawTrim = _svo.GetManualTrimRawData();
	A2_equ.ea = double(svoRawTrim.aileron - 15000)/5000;
	A2_equ.ee = double(svoRawTrim.elevator - 15000)/5000;
//	A2_equ.eu = double(svoRawTrim.auxiliary - 15000)/5000;
	A2_equ.er = double(svoRawTrim.rudder - 15000)/5000;
//	A2_equ.et = double(svoRawTrim.throttle - 15000)/5000;

	_state.SetEqu(A2_equ);
	UAVSTATE &state = _state.GetState();
	double c0 = _state.Getc0();
	double velb[3] = {state.u, state.v, state.w};

//	A1_Lookup(velb, A2_equ /*, A2_F, A2_G*/);

//	double c1 = state.c - c0 /*- A2_equ.c*/; INPI(c1);
//	double c1 = state.c - c0 - B_cr; INPI(c1);
//	double c1 = state.c - c0; INPI(c1);
	double c1 = state.c - B_cr; INPI(c1);
	double _x[8] = {
		state.a - A2_equ.a, state.b - A2_equ.b, c1,
		state.p - A2_equ.p, state.q - A2_equ.q, state.r - A2_equ.r,
		state.bs, state.as
	};
	clsVector x(8, _x, TRUE);
	double _Fx[3]; clsVector Fx(3, _Fx, TRUE);
	clsMatrix::X(A2_F_GREMLION_NEW, x, Fx);

	double _vi[3] = {
		B_acxr_ub,
		B_acyr_vb,
		B_aczr_wb,
	};
	clsVector vi(3, _vi, TRUE);
	double _Gvi[3]; clsVector Gvi(3, _Gvi, TRUE);
	clsMatrix::X(A2_TRANS_GREMLION, vi, Gvi);

	double _abcRef[3] = {
		_Gvi[1],
		_Gvi[2],
		0
	};
	m_abcRef[0] = _Gvi[1]; m_abcRef[1] = _Gvi[2]; m_abcRef[2] = 0;
	clsVector abcRef(3, _abcRef, TRUE);
	double _Gvo[3]; clsVector Gvo(3, _Gvo, TRUE);
	clsMatrix::X(A2_G_GREMLION_NEW, abcRef, Gvo);

	double _u[3]; clsVector u(3, _u, TRUE);
	u = Fx; u += Gvo;

	HELICOPTERRUDDER sig;
	sig.aileron = A2_equ.ea + _u[0];
	sig.elevator = A2_equ.ee + _u[1];
	sig.auxiliary = A2_equ.eu + _Gvi[0];
	sig.rudder = A2_equ.er + _u[2];
	sig.throttle = A2_equ.et + _Gvi[0];

	/// run observer function and update as, bs for the next control loop
	_state.Observe_GremLion();
#if (_DEBUG & DEBUGFLAG_CTL)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[CTL] A2, ail %f, ele %f, aux %f, rud %f\n",
		sig.aileron, sig.elevator, sig.auxiliary, sig.rudder);
}
#endif
/*	if (m_nCount % _DEBUG_COUNT == 0) {
		printf("[CTL] A2_equ, u %f, v %f, a %f, b %f, p %f, q %f\n",
					A2_equ.u, A2_equ.v, A2_equ.a, A2_equ.b, A2_equ.p, A2_equ.q);
		printf("[CTL] A2_equ, ail %f, ele %f, aux %f, rud %f\n",
				A2_equ.ea, A2_equ.ee, A2_equ.eu, A2_equ.er);
	}*/

	m_sig = sig;
}

void clsCTL::A2_HeLion()
{

	// set the rudders in svoControl
	// output: m_sig0(ea,ee,eu,er,et)
/*	HELICOPTERRUDDER &equ = _svo.GetTrimvalue();
	A2_equ.ea = equ.aileron;
	A2_equ.ee = equ.elevator;
	A2_equ.eu = equ.auxiliary;
	A2_equ.er = equ.rudder;
	A2_equ.et = equ.throttle;*/

	UAVSTATE &state = _state.GetState();
//	RPTSTATE &RPTState = _state.GetRPTState();
	double c0 = _state.Getc0();
//	if (m_nCount % 25 == 0)
//		cout<<"c0: "<<c0<<endl;

	double velb[3] = {state.u, state.v, state.w};

//	A1_Lookup(velb, A2_equ /*, A2_F, A2_G*/);

//	double c1 = state.c - c0 /*- A2_equ.c*/; INPI(c1);
	double c1 = state.c - c0 - B_cr; INPI(c1);
	double _x[9] = {
		state.a - A2_equ.a, state.b - A2_equ.b, c1,
		state.p - A2_equ.p, state.q - A2_equ.p, state.r - A2_equ.r,
		state.bs /*- A2_equ.bs*/, state.as /*- A2_equ.as*/, state.rfb /*- A2_equ.rfb*/
	};
	clsVector x(9, _x, TRUE);

	double _Fx[4]; clsVector Fx(4, _Fx, TRUE);
	clsMatrix::X(A2_F_RPT, x, Fx);

	double c2 = -c1 /* B_cr - (state.c - c0) */; INPI(c2);

	double _v[4] = {
//		A1A2A3_u - A2_equ.u,
//		A1A2A3_v - A2_equ.v,
//		A1A2A3_w - A2_equ.w,
//		A1A2A3_r - A2_equ.r
		B_acxr_ub /*- A2_equ.u*/,
		B_acyr_vb /*- A2_equ.v*/,
		B_aczr_wb /*- A2_equ.w*/,
		0	/* c2 */			/* B_cr - c0 */ 	  /*- A2_equ.c*/
	};
	clsVector v(4, _v, TRUE);

	double _Gv[4]; clsVector Gv(4, _Gv, TRUE);
	clsMatrix::X(A2_G_RPT, v, Gv);

	double _u[4]; clsVector u(4, _u, TRUE);
	u = Fx; u += Gv;

	HELICOPTERRUDDER sig;
	sig.aileron = A2_equ.ea+_u[1];
	sig.elevator = A2_equ.ee+_u[2];
	sig.auxiliary = A2_equ.eu+_u[0];
	sig.rudder = A2_equ.er+_u[3];
//	sig.throttle = A2_equ.et-0.47356*sig.auxiliary-0.14727*sig.auxiliary*sig.auxiliary;
	sig.throttle = A2_equ.et;

#if (_DEBUG & DEBUGFLAG_CTL)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[CTL] A2, ail %f, ele %f, aux %f, rud %f\n",
		sig.aileron, sig.elevator, sig.auxiliary, sig.rudder);
}
#endif

	m_sig = sig;
}

void clsCTL::A3()
{
#if (_DEBUG & DEBUGFLAG_CTL)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[CTL] A3 (cnf)\n");
}
#endif

	HELICOPTERRUDDER &equ = _svo.GetTrimvalue();
	A3_equ.ea = equ.aileron;
	A3_equ.ee = equ.elevator;
	A3_equ.eu = equ.auxiliary;
	A3_equ.er = equ.rudder;
	A3_equ.et = equ.throttle;

	UAVSTATE &state = _state.GetState();

	double _x[11] = {
		state.u - A3_equ.u, state.v - A3_equ.v,
		state.p - A3_equ.p, state.q - A3_equ.q,
		state.a - A3_equ.a, state.b - A3_equ.b,
		state.as, state.bs,
		state.w - A3_equ.w, state.r - A3_equ.r,
		state.rfb };
	clsVector x(11, _x, TRUE);

	double _r[4] = { A1A2A3_u - A3_equ.u, A1A2A3_v - A3_equ.v, A1A2A3_w - A3_equ.w, A1A2A3_r - A3_equ.r	};
	clsVector r(4, _r, TRUE);

	double _Fx[4]; clsVector Fx(4, _Fx, TRUE);
	double _Gr[4]; clsVector Gr(4, _Gr, TRUE);

	clsMatrix::X(A3_F, x, Fx);
	clsMatrix::X(A3_G, r, Gr);

	double _y[4] = { _x[0], _x[1], _x[8], _x[9] };				//u,v,w,r

	double al[4] = { A3_al[0], A3_al[1], A3_al[2], A3_al[3] };
	if (A3_bCut) {
		if (::fabs(_y[0]-_r[0]) <= 0.5) al[0] = 0;
		if (::fabs(_y[1]-_r[1]) <= 0.5) al[1] = 0;
		if (::fabs(_y[2]-_r[2]) <= 0.5) al[2] = 0;
		if (::fabs(_y[3]-_r[3]) <= 0.1) al[3] = 0;
	}

	double _rh[4];
	for (int i=0; i<=3; i++) {
		_rh[i] = -1.582*A3_be[i]*::fabs(::exp(al[i]*::fabs(_y[i]-_r[i]))-0.3679);
	}

	double _Ger[11]; clsVector Ger(11, _Ger, TRUE);
	clsMatrix::X(A3_Ge, r, Ger);

	double _xGer[11]; clsVector xGer(11, _xGer, TRUE);
	xGer = x; xGer -= Ger;

	double _uCNF[4]; clsVector uCNF(4, _uCNF, TRUE);
	clsMatrix::X(A3_N, xGer, uCNF);

	for (int i=0; i<=3; i++) _uCNF[i] *= _rh[i];

	double _u[4]; clsVector u(4, _u, TRUE);

	u = Fx; u += Gr; u += uCNF;

	HELICOPTERRUDDER sig;
	sig.aileron = A3_equ.ea+_u[0];
	sig.elevator = A3_equ.ee+_u[1];
	sig.auxiliary = A3_equ.eu+_u[2];
	sig.rudder = A3_equ.er+_u[3];
	sig.throttle = A3_equ.et;

#if (_DEBUG & DEBUGFLAG_CTL)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[CTL] A3, ail %f, ele %f, aux %f, rud %f\n",
		sig.aileron, sig.elevator, sig.auxiliary, sig.rudder);
}
#endif

	m_sig = sig;
}

void clsCTL::A4()				//A4 for test
{
	HELICOPTERRUDDER &equ = _svo.GetTrimvalue();
	A4_equ.ea = equ.aileron;
	A4_equ.ee = equ.elevator;
	A4_equ.eu = equ.auxiliary;
	A4_equ.er = equ.rudder;
	A4_equ.et = equ.throttle;

	HELICOPTERRUDDER sig = { A4_equ.ea, A4_equ.ee, A4_equ.eu, A4_equ.er, A4_equ.et };
	double t = GetTime()-A4_t0;

	if (A4_nTest == 71) {
		sig.aileron = 0.5*::sin(t/5*2*PI);
		sig.elevator = 0.5*::sin(t/6*2*PI);
		sig.auxiliary = 0.5*::sin(t/7*2*PI);
		sig.rudder = 0.5*::sin(t/8*2*PI);
		sig.throttle = 0.5+0.3*::sin(t/9*2*PI);
	}
	else if (A4_nTest == 72) {
		double T = 10;				//section durance
		double f = 1;				//frequency

		int n = (int)::floor((t-10)/T);
		double value = ::sin(2*PI*f*t);

		switch (n) {
		case 0: sig.aileron = A4_equ.ea + 0.05*value; break;
		case 1: sig.elevator = A4_equ.ee + 0.05*value; break;
		case 2: sig.auxiliary = A4_equ.eu + 0.05*value; break;
		case 3: sig.rudder = A4_equ.er + 0.05*value; break;
		case 4: sig.throttle = A4_equ.et + 0.05*value; break;
		}
	}
	else if (A4_nTest == 73) {
		sig.aileron = -0.5;
		sig.elevator = 0;
		sig.auxiliary = 0;
		sig.rudder = 0;
		sig.throttle = 0;
	}
	else if (A4_nTest == 74) {
		sig.aileron = 0;
		sig.elevator = -0.5;
		sig.auxiliary = 0;
		sig.rudder = 0;
		sig.throttle = 0;
	}
	else if (A4_nTest == 75) {
		sig.aileron = 0;
		sig.elevator = 0;
		sig.auxiliary = -0.5;
		sig.rudder = 0;
		sig.throttle = 0;
	}
	else if (A4_nTest == 76) {
		sig.aileron = 0;
		sig.elevator = 0;
		sig.auxiliary = 0;
		sig.rudder = 0;
		sig.throttle = 0;
	}

#if (_DEBUG & DEBUGFLAG_CTL)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[CTL] A4, ail %.3g, ele %.3g, aux %.3g, rud %.3g\n",
		sig.aileron, sig.elevator, sig.auxiliary, sig.rudder);
}
#endif

	m_sig = sig;
}

void clsCTL::A5()
{
//	if (m_nCount % 50 ==0)
//		cout<<"[CTL] A5"<<endl;

	HELICOPTERRUDDER &equ = _svo.GetTrimvalue();
	A5_equ.ea = equ.aileron;
	A5_equ.ee = equ.elevator;
	A5_equ.eu = equ.auxiliary;
	A5_equ.er = equ.rudder;
	A5_equ.et = equ.throttle;

	UAVSTATE &state = _state.GetState();

	//horizontal control
	double _x1[8] = {
		state.u - A5_equ.u, state.v - A5_equ.v,
		state.a - A5_equ.a, state.b - A5_equ.b,
		state.p - A5_equ.p, state.q - A5_equ.q,
		state.as, state.bs
	};
	clsVector x1(8, _x1, TRUE);
	_x1[2] = range(_x1[2], -0.25, 0.25);				//limite error in psi and theta
	_x1[3] = range(_x1[3], -0.25, 0.25);

	double _v1[2] = { state.u, state.v }; clsVector v1(2, _v1, TRUE);
	double _v1c[2] = { A5_u, A5_v }; clsVector v1c(2, _v1c, TRUE);

	double _F1x1[2]; clsVector F1x1(2, _F1x1, TRUE);
	double _G1v1c[2]; clsVector G1v1c(2, _G1v1c, TRUE);
	double _L1x1[2]; clsVector L1x1(2, _L1x1, TRUE);
	double _M1v1c[2]; clsVector M1v1c(2, _M1v1c, TRUE);
	double _Krho[2]; clsVector K1rho(2, _Krho, TRUE);

	clsMatrix::X(A5_F1, x1, F1x1);
	clsMatrix::X(A5_G1, v1c, G1v1c);
	clsMatrix::X(A5_L1, x1, L1x1);
	clsMatrix::X(A5_M1, v1c, M1v1c);

	double _L1x1M1v1c[2] = { L1x1[0]-M1v1c[0], L1x1[1]-M1v1c[1] }; clsVector L1x1M1v1c(2, _L1x1M1v1c, TRUE);

	double _P1v1[2]; clsVector P1v1(2, _P1v1, TRUE);
	double _Q1v1c[2]; clsVector Q1v1c(2, _Q1v1c, TRUE);

	clsMatrix::X(A5_P1, v1, P1v1);
	clsMatrix::X(A5_Q1, v1c, Q1v1c);

	double _abc[2]; clsVector abc(2, _abc, TRUE);
	abc = P1v1; abc += Q1v1c;				//abc = P1v1+Q1v1c;

	double ac = abc[0];
	double bc = abc[1];

	double cb = 1/EPS;
	double den = 1-cb;

	double rho1 = -A5_be1*::fabs(::exp(-A5_al1*::fabs(state.a-ac))-cb)/den;
	double rho2 = -A5_be2*::fabs(::exp(-A5_al2*::fabs(state.b-bc))-cb)/den;
/*	double rho1 = 0;
	double rho2 = 0;*/

	//Krho = K*rho*(L1x1-M1v1c);
	L1x1M1v1c[0] *= rho1;
	L1x1M1v1c[1] *= rho2;

	clsMatrix::X(A5_K1, L1x1M1v1c, K1rho);

	//de1 = F1*x1+G1*v1c+K1*rho*(L1*x1-M1*v1c);
	double _de1[2] = { F1x1[0]+G1v1c[0]+K1rho[0], F1x1[1]+G1v1c[1]+K1rho[1] };

	//hovering and heading control
	double dc = state.c - A5_c;
	if (dc > PI) dc -= 2*PI;
	else if (dc < -PI) dc += 2*PI;
	dc = range(dc, -1, 1);				//limit error psi

	double _x4[4] = {
		state.w - A5_equ.w, A5_c+dc,				//to choose a value close to A5_c
		state.r - A5_equ.r, state.rfb
	};
	clsVector x4(4, _x4, TRUE);
	double _wc[2] = { A5_w, A5_c }; clsVector wc(2, _wc, TRUE);

	double _F4x4[2]; clsVector F4x4(2, _F4x4, TRUE);
	double _G4wc[2]; clsVector G4wc(2, _G4wc, TRUE);

	clsMatrix::X(A5_F4, x4, F4x4);
	clsMatrix::X(A5_G4, wc, G4wc);

	double _M4wc[4]; clsVector M4wc(4, _M4wc, TRUE);
	clsMatrix::X(A5_M4, wc, M4wc);

	double _x4M4wc[4] = { x4[0]-M4wc[0], x4[1]-M4wc[1], x4[2]-M4wc[2], x4[3]-M4wc[3] };
	clsVector x4M4wc(4, _x4M4wc, TRUE);

	//K4x4M4wc = K4*(x4-M4*wc);
	double _K4x4M4wc[2]; clsVector K4x4M4wc(2, _K4x4M4wc, TRUE);
	clsMatrix::X(A5_K4, x4M4wc, K4x4M4wc);

	double rho4 = -A5_be4*::fabs(::exp(-A5_al4*::fabs(dc))-cb)/den;
//	double rho4 = 0;
	K4x4M4wc *= rho4;

	double _de3[2] = { F4x4[0]+G4wc[0]+K4x4M4wc[0], F4x4[1]+G4wc[1]+K4x4M4wc[1] };

	HELICOPTERRUDDER sig;
	sig.aileron = A5_equ.ea + _de1[0];
	sig.elevator = A5_equ.ee + _de1[1];
	sig.auxiliary = A5_equ.eu + _de3[0];
	sig.rudder = A5_equ.er + _de3[1];
	sig.throttle = A5_equ.et;

#if (_DEBUG & DEBUGFLAG_CTL)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[CTL] A5, A5_u %f, A5_v %f, A5_w %f, A5_c %f\n", A5_u, A5_v, A5_w, A5_c);
	printf("[CTL] A5, ail %f, ele %f, aux %f, rud %f\n", sig.aileron, sig.elevator, sig.auxiliary, sig.rudder);
}
#endif

	m_sig = sig;
}

void clsCTL::A6()
{
	// set the rudders in svoControl
	// output: m_sig0(ea,ee,eu,er,et)

	if (_HELICOPTER == ID_GREMLION)
		A6_GremLion();
	else if (_HELICOPTER == ID_HELION || _HELICOPTER == ID_SHELION)
		A6_HeLion();

}

void clsCTL::A6_GremLion()
{
	UAVSTATE &state = _state.GetState();

	double _x[11] = {
		state.u - m_equ.u, state.v - m_equ.v,
		state.p - m_equ.p, state.q - m_equ.q,
		state.a - m_equ.a, state.b - m_equ.b,
		state.as - m_equ.as, state.bs - m_equ.bs,
		state.w - m_equ.w, state.r - m_equ.r,
		state.c - m_equ.c
	};
	clsVector x(11, _x, TRUE);

	double _Fx[4]; clsVector Fx(4, _Fx, TRUE);
	clsMatrix::X(A6_F, x, Fx);

	double _v[4] = {
		A1A2A3_u - m_equ.u,
		A1A2A3_v - m_equ.v,
		A1A2A3_w - m_equ.w,
		A1A2A3_r - m_equ.r
	};
	clsVector v(4, _v, TRUE);

	double _Gv[4]; clsVector Gv(4, _Gv, TRUE);
	clsMatrix::X(A6_G, v, Gv);

	double _u[4]; clsVector u(4, _u, TRUE);
	u = Fx; u += Gv;

	HELICOPTERRUDDER sig;
	sig.aileron = m_equ.ea+_u[0];
	sig.elevator = m_equ.ee+_u[1];
	sig.auxiliary = m_equ.eu+_u[2];
	sig.rudder = m_equ.er+_u[3];
	sig.throttle = m_equ.et;		// not used in GremLion

	m_sig = sig;
}

void clsCTL::A6_HeLion()
{
	UAVSTATE &state = _state.GetState();

	double _x[11] = {
		state.u - m_equ.u, state.v - m_equ.v,
		state.p - m_equ.p, state.q - m_equ.q,
		state.a - m_equ.a, state.b - m_equ.b,
		state.as - m_equ.as, state.bs - m_equ.bs,
		state.w - m_equ.w, state.r - m_equ.r,
		state.rfb - m_equ.rfb
	};
	clsVector x(11, _x, TRUE);

	double _Fx[4]; clsVector Fx(4, _Fx, TRUE);
	clsMatrix::X(A6_F, x, Fx);

//	_Fx[0] *= 4;
//	_Fx[1] *= 4;
//	_Fx[2] *= 4;

	double _v[4] = {
		A1A2A3_u - m_equ.u,
		A1A2A3_v - m_equ.v,
		A1A2A3_w - m_equ.w,
		A1A2A3_r - m_equ.r
	};
	clsVector v(4, _v, TRUE);

	if (B6_bChirp && B6_add == CHIRPADD_OUTERLOOP) {
		_v[0] += B6_vChirp[0];
		_v[1] += B6_vChirp[1];
		_v[2] += B6_vChirp[2];
		_v[3] += B6_vChirp[3];
	}

	double _Gv[4]; clsVector Gv(4, _Gv, TRUE);
	clsMatrix::X(A6_G, v, Gv);

	double _u[4]; clsVector u(4, _u, TRUE);
	u = Fx; u += Gv;

	HELICOPTERRUDDER sig;
	sig.aileron = m_equ.ea+_u[0];
	sig.elevator = m_equ.ee+_u[1];
	sig.auxiliary = m_equ.eu+_u[2];
	sig.rudder = m_equ.er+_u[3];
//	sig.throttle = m_equ.et-0.47356*sig.auxiliary-0.14727*sig.auxiliary*sig.auxiliary;
	sig.throttle = m_equ.et;
	if (B6_bChirp && B6_add == CHIRPADD_CONTROLSIGNAL) {
		sig.aileron += B6_vChirp[0];
		sig.elevator += B6_vChirp[1];
		sig.auxiliary += B6_vChirp[2];
		sig.rudder += B6_vChirp[3];
	}

#if (_DEBUG & DEBUGFLAG_CTL)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[CTL] A6, ail %f, ele %f, aux %f, rud %f\n",
		sig.aileron, sig.elevator, sig.auxiliary, sig.rudder);
}
#endif

	m_sig = sig;
}

void clsCTL::A8()				//for engine up and down control (auto takeoff/landing)
{
	double t = ::GetTime()-A8_t0;

	if (A8_mode == A8MODE_ENGINEUP) {
		if (_HELICOPTER == ID_HELION || _HELICOPTER == ID_SHELION) {
			double TUP = 10;
	//		double TUP = 2;
			BOOL bEnd = t >= TUP + 1.5;
			if (m_behaviorCurrent.behavior == BEHAVIOR_ENGINEUP &&	bEnd && !A8_bEnd)
			{
				_state.SetEvent(EVENT_BEHAVIOREND, BEHAVIOR_ENGINEUP);
				cout<<"ENGINEUP event end at "<<::GetTime()<<endl;
			}
			A8_bEnd = bEnd;

			m_sig.auxiliary = AUXILIARY_LOW;
			t = range(t, 0, TUP);				//0-12 seconds
			m_sig.throttle = (THROTTLE_LOW*(TUP-t)+THROTTLE_HIGH*t)/TUP;
		}
		else if (_HELICOPTER == ID_QUADLION) {
			m_sig.aileron = 0; m_sig.elevator = 0; m_sig.rudder = 0;
			double TUP = 3;
			// 0-4s increase throttle to 80% trim value
			if (t> 0 && t <= TUP) {
					m_sig.throttle = (SAFMC2014_THROTTLE_LOW*(TUP-t) + _equ_Hover.et*0.8*t)/TUP;
			}
			else if ( t> TUP ) {
					_state.SetEvent(EVENT_BEHAVIOREND, BEHAVIOR_ENGINEUP);
					cout<<"Rotor warm-up event end at "<<::GetTime()<<endl;
			}
			if (m_nCount%100 == 0){
				printf("[ctl:A8] time %.2f: throttle %.3f; A2_equ.et %.2f\n", t, m_sig.throttle, A2_equ.et);
			}
		}
	}

	if (A8_mode == A8MODE_ENGINEDOWN) {
		m_sig.aileron = 0; m_sig.elevator = 0; m_sig.rudder = 0; m_sig.throttle = 0;
		static double startTime = ::GetTime();
		if (GetTime() - startTime > 3*60){
			_state.SetEvent(EVENT_BEHAVIOREND, BEHAVIOR_ENGINEDOWN);
			printf("[ctl::A8] Task Finished\n");
		}
	}

	if (A8_mode == A8MODE_AUXILIARYDOWN) {
		m_sig.auxiliary = A8_sig0.auxiliary + 0.01*t;
		if (m_sig.auxiliary >= AUXILIARY_LOW) m_sig.auxiliary = AUXILIARY_LOW;
	}

#if (_DEBUG & DEBUGFLAG_CTL)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[CTL] A8, eu %.3f, et %.3f\n", m_sig.auxiliary, m_sig.throttle);
}
#endif
}

void clsCTL::A9()
{
	if (A9_flag & A9FLAG_AILERON) m_sig.aileron = A9_ea;
	if (A9_flag & A9FLAG_ELEVATOR) m_sig.elevator = A9_ee;
	if (A9_flag & A9FLAG_AUXILIARY) m_sig.auxiliary = A9_eu;
	if (A9_flag & A9FLAG_RUDDER) m_sig.rudder = A9_er;
	if (A9_flag & A9FLAG_THROTTLE) m_sig.throttle = A9_et;

	if(m_nCount%50 == 0){
		printf("[CTL:EngineLow] A9, ea %.3f, ee %.3f, eu %.3f, er %.3f, et %.3f\n",
			m_sig.aileron, m_sig.elevator, m_sig.auxiliary, m_sig.rudder, m_sig.throttle);
	}
#if (_DEBUG & DEBUGFLAG_CTL)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[CTL] A9, ea %.3f, ee %.3f, eu %.3f, er %.3f, et %.3f\n",
		m_sig.aileron, m_sig.elevator, m_sig.auxiliary, m_sig.rudder, m_sig.throttle);
}
#endif
}

void clsCTL::A7()				// Emergency control for FeiLion
{
	HELICOPTERRUDDER sig;
	sig.aileron = A1_equ.ea;
	sig.elevator = A1_equ.ee;
	sig.rudder	= A1_equ.er;
	sig.throttle = A1_equ.et;
	sig.auxiliary = A1_equ.eu;

	m_sig = sig;
}

void clsCTL::B3()
{

}

void clsCTL::B4()
{
	UAVSTATE &state = _state.GetState();

	A1A2A3_w = B4_kz*(state.z-B4_z);

#if (_DEBUG & DEBUGFLAG_CTL)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[CTL] B4, B4_z %.4g, z %.4g\n", B4_z, state.z);
}
#endif

	A1A2A3_w = range(A1A2A3_w, MIN_STATE_W, MAX_STATE_W);
}

void clsCTL::B5()				//path tracking
{
	if (_HELICOPTER == ID_GREMLION) {
		int nMode = GetMode();
		if (nMode == MODE_SEMIAUTO)
			ManualReferenceGeneration();
		else if (nMode == MODE_ALLAUTO)
			B5_GremLion_AllAuto();
		else if (nMode == MODE_NAVIGATION)
			AutoPathGeneration();

		/// then call the common outer-loop B5 control block of GremLion
			B5_GremLion();
	}
	else if (_HELICOPTER == ID_HELION || _HELICOPTER == ID_SHELION)
		B5_HeLion();
}

void clsCTL::B5_GremLion()
{
	/*
	 * RPT outer-loop control law
	 */
	UAVSTATE state = _state.GetState();
	double _x[6] = {
		state.x, state.y, state.z,
		state.ug, state.vg, state.wg
	};

	clsVector x(6, _x, TRUE);

	double _Fx[3]; clsVector Fx(3, _Fx, TRUE);
	clsMatrix::X(B5_F_GREMLION_RPT, x, Fx);

	double _v[9] = {
		B5_pnr[0], B5_pnr[1], B5_pnr[2],
		B5_vnr[0], B5_vnr[1], B5_vnr[2],
		B5_anr[0], B5_anr[1], B5_anr[2]
	};
	clsVector v(9, _v, TRUE);
	double _Gv[3]; clsVector Gv(3, _Gv, TRUE);
	clsMatrix::X(B5_G_GREMLION_RPT, v, Gv);

	double _u[3]; clsVector u(3, _u, TRUE);
	u = Fx; u += Gv;		// u, acc reference in NED frame

	double abc[3] = {state.a, state.b, state.c};
	double abcr[3] = {0};	// acc reference in body frame
	G2B(abc, _u, abcr);

	B_acxr_ub = abcr[0];
	B_acyr_vb = abcr[1];
	B_aczr_wb = abcr[2];
	B_cr = B5_pnr[3];

//	B_acxr_ub = range(B_acxr_ub, MIN_STATE_ACX, MAX_STATE_ACX);
//	B_acyr_vb = range(B_acyr_vb, MIN_STATE_ACY, MAX_STATE_ACY);
//	B_aczr_wb = range(B_aczr_wb, MIN_STATE_ACW, MAX_STATE_ACW);

//	if (m_nCount % _DEBUG_COUNT == 0) {
//		printf("[CTL] B5, B_acxr_ub %.4g, B_acyr_vb %.4g, B_aczr_wb %.4g\n", B_acxr_ub, B_acyr_vb, B_aczr_wb);
//	}

}

void clsCTL::B5_GremLion_SemiAuto_RPT()
{
	UAVSTATE &state = _state.GetState();
	double c0 = _state.Getc0();

	double velRef[4] = {0}; //uref, vref, wref, rref;
	double posRef[4] = {0}; //xref, yref, zref, cref

	// convert to physical velocity and yaw rate
	SVODATA &svodata  = _svo.GetSVOData();
	double manualSig[4] = {svodata.elevator, svodata.aileron, svodata.throttle, svodata.rudder};

	if ( (manualSig[3] < MANUAL_DEADZON) && (manualSig[3] > -MANUAL_DEADZON) ) {
		manualSig[3] = 0; velRef[3] = 0;
	}

	if ( (manualSig[0] < MANUAL_DEADZON) && (manualSig[0] > -MANUAL_DEADZON) ) {
		manualSig[0] = 0; velRef[0] = 0;
	}

	if ( (manualSig[1] < MANUAL_DEADZON) && (manualSig[1] > -MANUAL_DEADZON) ) {
		manualSig[1] = 0; velRef[1] = 0;
	}

	if ( (manualSig[2] < MANUAL_DEADZON) && (manualSig[2] > -MANUAL_DEADZON) ) {
		manualSig[2] = 0; velRef[2] = 0;
	}

	if (manualSig[3]>=MANUAL_DEADZON)
		velRef[3] = (manualSig[3] - MANUAL_DEADZON) * (-MAX_YAWRATE);
	else if (manualSig[3] <= -MANUAL_DEADZON )
		velRef[3] = (manualSig[3] + MANUAL_DEADZON) * (-MAX_YAWRATE);

	if (manualSig[0]>=MANUAL_DEADZON)
		velRef[0] = (manualSig[0] - MANUAL_DEADZON) * (-MAX_LONSPEED);
	else if (manualSig[0] <= -MANUAL_DEADZON )
		velRef[0] = (manualSig[0] + MANUAL_DEADZON) * (-MAX_LONSPEED);

	if (manualSig[1]>=MANUAL_DEADZON)
		velRef[1] = (manualSig[1] - MANUAL_DEADZON) * (-MAX_LATSPEED);
	else if (manualSig[1] <= -MANUAL_DEADZON )
		velRef[1] = (manualSig[1] + MANUAL_DEADZON) * (-MAX_LATSPEED);

	if (manualSig[2]>=MANUAL_DEADZON)
		velRef[2] = (manualSig[2] - MANUAL_DEADZON) * (-MAX_VERSPEED);
	else if (manualSig[2] <= -MANUAL_DEADZON )
		velRef[2] = (manualSig[2] + MANUAL_DEADZON) * (-MAX_VERSPEED);

	/// apply 2nd order LPF to generate vel_ref and acc_ref
	// calculate dt
	double t = ::GetTime();
	double dt = 0;
	if (B5_t1 < 0) {
//		::memset(B5_vax, 0, 2*sizeof(double));
//		::memset(B5_vay, 0, 2*sizeof(double));
//		::memset(B5_vaz, 0, 2*sizeof(double));
		B5_vax[0] = state.ug; B5_vax[1] = 0;
		B5_vay[0] = state.vg; B5_vay[1] = 0;
		B5_vaz[0] = state.wg; B5_vaz[1] = 0;
		B_cr = state.c;
		dt = 0;
	} else {
		dt = ::GetTime() - B5_t1;
	}
	B5_t1 = t;

	velRef[2] = 0;
	/// acc reference
	double vhr[3] = {B5_vax[0], B5_vay[0], B5_vaz[0]};
	double ahr[3] = {B5_vax[1], B5_vay[1], B5_vaz[1]};
	double vnr[3] = {0};
	double anr[3] = {0};

	double abc0[3] = {0, 0, B5_semiPsi};
	G2N(abc0, vhr, vnr);
	G2N(abc0, ahr, anr);

	double _x[6] = {
		state.x, state.y, state.z,
		state.ug, state.vg,	state.wg
	};
	clsVector x(6, _x, TRUE);

	double _Fx[3]; clsVector Fx(3, _Fx, TRUE);
	clsMatrix::X(B5_F_GREMLION_RPT, x, Fx);

	double _v[9] = {
		state.x, state.y, state.z,
		vnr[0], vnr[1], vnr[2],
		anr[0], anr[1], anr[2]
	};
	clsVector v(9, _v, TRUE);
	double _Gv[3]; clsVector Gv(3, _Gv, TRUE);
	clsMatrix::X(B5_G_GREMLION_RPT, v, Gv);

	double _u[3]; clsVector u(3, _u, TRUE);
	u = Fx; u += Gv;		// u, acc reference in NED frame

	double abc[3] = {state.a, state.b, state.c};
	double abcr[3] = {0};	// acc reference in body frame
	G2B(abc, _u, abcr);

	B_acxr_ub = abcr[0];
	B_acyr_vb = abcr[1];
	B_aczr_wb = abcr[2];

	/// Runge Kutta update
	for (int i=0; i<3; i++) {
		double _tmp[2] = {0};
		if (i == 0) {
//			::memcpy(_tmp, B5_vax, 2*sizeof(double));
			_tmp[0] = B5_vax[0]; _tmp[1] = B5_vax[1];
		} else if (i == 1) {
//			::memcpy(_tmp, B5_vay, 2*sizeof(double));
			_tmp[0] = B5_vay[0]; _tmp[1] = B5_vay[1];
		} else if (i == 2) {
//			::memcpy(_tmp, B5_vaz, 2*sizeof(double));
			_tmp[0] = B5_vaz[0]; _tmp[1] = B5_vaz[1];
		}

		clsVector tmp(2, _tmp, TRUE);
		double _Ax1[2]; clsVector Ax1(2, _Ax1, TRUE);
		double _Bu1[2]; clsVector Bu1(2, _Bu1, TRUE);

		clsMatrix::X(B5_Aorg_GREMLION, tmp, Ax1);

//		Bu1 = B5_Borg_GREMLION;
//		Bu1.operator *= (velRef[i]);
		_Bu1[0] = _B5_Borg_GREMLION[0] * velRef[i];
		_Bu1[1] = _B5_Borg_GREMLION[1] * velRef[i];
		double _dx0[2]; clsVector dx0(2, _dx0, TRUE);
		dx0 = Ax1; dx0 += Bu1;

		double _dx[2]; clsVector dx(2, _dx, TRUE);
		dx = dx0;

		double _Ah[2][2]; clsMatrix Ah(2,2,(double *)_Ah, TRUE);
		double _Ah2[2][2]; clsMatrix Ah2(2,2,(double *)_Ah2, TRUE);
		double _Ah3[2][2]; clsMatrix Ah3(2,2,(double *)_Ah3, TRUE);

		Ah = B5_Aorg_GREMLION; Ah *= dt/2;
		Ah2 = B5_Aorg2_GREMLION; Ah2 *= dt*dt/6;
		Ah3 = B5_Aorg3_GREMLION; Ah3 *= dt*dt*dt/24;

		double _Ah123[2][2]; clsMatrix Ah123(2,2,(double*)_Ah123,TRUE);
		Ah123 = Ah; Ah123 += Ah2; Ah123 += Ah3;

		double _dx123[2]; clsVector dx123(2,_dx123,TRUE);
		clsMatrix::X(Ah123, dx0, dx123);

		dx += dx123;
		dx *= dt;

		tmp += dx;

		if (i == 0) {
//			::memcpy(B5_vax, _tmp, 2*sizeof(double));
			B5_vax[0] = _tmp[0]; B5_vax[1] = _tmp[1];
		}
		else if (i == 1) {
//			::memcpy(B5_vay, _tmp, 2*sizeof(double));
			B5_vay[0] = _tmp[0]; B5_vay[1] = _tmp[1];
		}
		else if (i == 2) {
//			::memcpy(B5_vaz, _tmp, 2*sizeof(double));
			B5_vaz[0] = _tmp[0]; B5_vaz[1] = _tmp[1];
		}
	}

	/// heading reference
	B5_semiPsi += velRef[3]*dt;
	INPI(B5_semiPsi);

	double dc = state.c - B5_semiPsi; INPI(dc);
	if (::fabs(dc) < HEADINGERR_DEADZONE ) dc = 0;
	B5_PsiErr += dc*dt;
	B5_PsiErr = range(B5_PsiErr, MIN_TRIM_ERROR, MAX_TRIM_ERROR);
	B_cr =  B5_semiPsi - 0.2*B5_PsiErr; INPI(B_cr);

//	B_acxr_ub = range(B_acxr_ub, MIN_STATE_ACX, MAX_STATE_ACX);
//	B_acyr_vb = range(B_acyr_vb, MIN_STATE_ACY, MAX_STATE_ACY);
//	B_aczr_wb = range(B_aczr_wb, MIN_STATE_ACW, MAX_STATE_ACW);
	if (m_nCount % _DEBUG_COUNT == 0) {
//		printf("[CTL] B5_semiauto, agghc[0] %.4g, agghc[1] %.4g \n", agghc[0], agghc[1]);
		printf("[CTL] B5, B_acxr_ub %.4g, B_acyr_vb %.4g, B_aczr_wb %.4g\n", B_acxr_ub, B_acyr_vb, B_aczr_wb);
	}
}

void clsCTL::B5_GremLion_SemiAuto()
{
	UAVSTATE &state = _state.GetState();

	// convert to physical velocity and yaw rate
	SVODATA &svodata  = _svo.GetSVOData();
	SVODATA &svodata1 = _svo.GetSVOData1();
	double manualSig[3] = {svodata.aileron, svodata.elevator, svodata.rudder};

	double uref, vref, rref;
	if ( (manualSig[2] < MANUAL_DEADZON) && (manualSig[2] > -MANUAL_DEADZON) ) {
		manualSig[2] = 0; rref = 0;
	}

	if ( (manualSig[0] < MANUAL_DEADZON) && (manualSig[0] > -MANUAL_DEADZON) ) {
		manualSig[0] = 0; vref = 0;
	}

	if ( (manualSig[1] < MANUAL_DEADZON) && (manualSig[1] > -MANUAL_DEADZON) ) {
		manualSig[1] = 0; uref = 0;
	}

	if (manualSig[2]>=MANUAL_DEADZON)
		rref = (manualSig[2] - MANUAL_DEADZON) * (-MAX_YAWRATE);
	else if (manualSig[2] <= -MANUAL_DEADZON )
		rref = (manualSig[2] + MANUAL_DEADZON) * (-MAX_YAWRATE);

	if (manualSig[0]>=MANUAL_DEADZON)
		vref = (manualSig[0] - MANUAL_DEADZON) * (-MAX_LATSPEED);
	else if (manualSig[0] <= -MANUAL_DEADZON )
		vref = (manualSig[0] + MANUAL_DEADZON) * (-MAX_LATSPEED);

	if (manualSig[1]>=MANUAL_DEADZON)
		uref = (manualSig[1] - MANUAL_DEADZON) * (-MAX_LONSPEED);
	else if (manualSig[1] <= -MANUAL_DEADZON )
		uref = (manualSig[1] + MANUAL_DEADZON) * (-MAX_LONSPEED);

	// update inner loop trim values given different velocity references
	double manualSig1[3] = {svodata1.aileron, svodata1.elevator, svodata1.rudder};
	double uref1, vref1;
	if ( (manualSig1[0] < MANUAL_DEADZON) && (manualSig1[0] > -MANUAL_DEADZON) ) {
		manualSig1[0] = 0; vref1 = 0;
	}

	if ( (manualSig1[1] < MANUAL_DEADZON) && (manualSig1[1] > -MANUAL_DEADZON) ) {
		manualSig1[1] = 0; uref1 = 0;
	}

	if (manualSig1[0]>=MANUAL_DEADZON)
		vref1 = (manualSig1[0] - MANUAL_DEADZON) * (-MAX_LATSPEED);
	else if (manualSig1[0] <= -MANUAL_DEADZON )
		vref1 = (manualSig1[0] + MANUAL_DEADZON) * (-MAX_LATSPEED);

	if (manualSig1[1]>=MANUAL_DEADZON)
		uref1 = (manualSig1[1] - MANUAL_DEADZON) * (-MAX_LONSPEED);
	else if (manualSig1[1] <= -MANUAL_DEADZON )
		uref1 = (manualSig1[1] + MANUAL_DEADZON) * (-MAX_LONSPEED);

	double velRef[3] = {uref1, vref1, 0};
	double abc[3] = {state.a, state.b, 0};
	double velRefb[3] = {0};
	G2B(abc, velRef, velRefb);

//	A2_equ.u = velRefb[0]; A2_equ.v = velRefb[1];
//	A2_equ.a = velRefb[1]/200;
//	A2_equ.b = -velRefb[0]/200;

	// calculate dt
	double t = ::GetTime();
	double dt = 0;
	if (B5_t1 < 0) {
		dt = 0;
	} else {
		dt = ::GetTime() - B5_t1;
	}
	B5_t1 = t;

	// semi-auto control law
	B5_semiPsi += rref*dt;
	INPI(B5_semiPsi);

	double dc = state.c - B5_semiPsi; INPI(dc);
	if (::fabs(dc) < HEADINGERR_DEADZONE ) dc = 0;
	B5_PsiErr += dc*dt;
	B5_PsiErr = range(B5_PsiErr, MIN_TRIM_ERROR, MAX_TRIM_ERROR);
	B5_semiPsic = B5_semiPsi - 0.2*B5_PsiErr;

	double Bref[2][2] = {{cos(B5_semiPsi), sin(B5_semiPsi)}, {-sin(B5_semiPsi), cos(B5_semiPsi)}};
	double vn[2] = {0};
	vn[0] = Bref[0][0]*state.ug + Bref[0][1]*state.vg;
	vn[1] = Bref[1][0]*state.ug + Bref[1][1]*state.vg;

	double aggnhc[2] = {0};
	aggnhc[0] = B5_FVn*(vn[0] - uref) - rref*vn[1];
	aggnhc[1] = B5_FVn*(vn[1] - vref) + rref*vn[0];

	double agghc[2] = {0};
	agghc[0] = Bref[0][0]*aggnhc[0] + Bref[1][0]*aggnhc[1];
	agghc[1] = Bref[0][1]*aggnhc[0] + Bref[1][1]*aggnhc[1];

	double agb[3] = {0};
	double aggh[3] = {agghc[0],agghc[1], state.acz};
	G2B(&state.a, aggh, agb);

	B_acxr_ub = agb[0];
	B_acyr_vb = agb[1];
	B_aczr_wb = 0;
	B_cr = B5_semiPsic; //B5_semiPsi;

	B_acxr_ub = range(B_acxr_ub, MIN_STATE_ACX, MAX_STATE_ACX);
	B_acyr_vb = range(B_acyr_vb, MIN_STATE_ACY, MAX_STATE_ACY);
	B_aczr_wb = range(B_aczr_wb, MIN_STATE_ACW, MAX_STATE_ACW);
/*	if (m_nCount % _DEBUG_COUNT == 0) {
		printf("[CTL] B5_semiauto, agghc[0] %.4g, agghc[1] %.4g \n", agghc[0], agghc[1]);
		printf("[CTL] B5_semiauto, B_acxr_ub %.4g, B_acyr_vb %.4g, B_cr %.4g\n", B_acxr_ub, B_acyr_vb, B_cr);
	}*/
//	A1A2A3_r = range(A1A2A3_r, MIN_STATE_R, MAX_STATE_R);
}

void clsCTL::AutoPathGeneration()
{
	UAVSTATE state = _state.GetState();

	/// below is 2D control
	double tGet = ::GetTime();
	if (B5_pPath != NULL) {
		// pre-generated path
		double t = tGet - B5_t0;

		BOOL bEnd = t >= B5_pPath->GetEndTime();
		if (bEnd && !B5_bEnd) {
			cout<<"clsCTL::B5 EVENT_BEHAVIOREND at "<<::GetTime()<<endl;
			_state.SetEvent(EVENT_BEHAVIOREND, m_behaviorCurrent.behavior);
		}
		B5_bEnd = bEnd;
		//the B5_end flag is used to prevent setting event endlessly, just send only once;

		if (B5_pPath->m_path.GetN() == 11) {
			if(start == 2)
			{
			    B5_pPath->init_state = init_state;
			    B5_pPath->start_control = 1;
			}
			else
			{
			    B5_pPath->start_control = 0;
			}
			if(SAMFC)
				B5_pPath->SetSAMFC();
			    B5_pPath->tLand = tLand;
			B5_pPath->GetPositionVelocity1(t, B5_pnr, B5_vnr,B5_anr, FALSE);
//			B5_pPath->GetPosVelAcc(t, B5_pnr, B5_vnr,B5_anr);
//			if (m_nCount % 50 == 0) {
//				printf("Indoor pnr ref: %f, %f, %f, %f \n", B5_pnr[0], B5_pnr[1], B5_pnr[2], B5_pnr[3]);
//				printf("Indoor vnr ref: %f, %f, %f %f %f %f \n", B5_vnr[0], B5_vnr[1], B5_vnr[2],B5_anr[0],B5_anr[1],B5_anr[2]);
//			}
			B5_xref = B5_pnr[0]; B5_yref = B5_pnr[1]; B5_zref = B5_pnr[2]; B5_cref = B5_pnr[3];
			//B5_anr[0] = B5_anr[1] = B5_anr[2] = 0;
		}
	}
	// online smooth path
	else if ( IsPathSmooth() ) {
		if ( m_pPlan!=NULL ) {
			if ( m_pPlan->GetPlanID() == 2 ) {// take-off plan
				ConstructTakeOffPath(state, -5, B5_pnr, B5_vnr, B5_anr);
			}
			else if ( m_pPlan->GetPlanID() == 3 ) {// landing plan
//				_pathTmp.CreateLandingPathOnline(&state, B5_pnr, B5_vnr, B5_anr);
				_ctl.ConstructLandingPath2(state, B5_pnr, B5_vnr, B5_anr);
			}
			else if (m_pPlan->GetPlanID() == 4){ //2014 SAFMC plan
				if (_ctl.GetTakeOffFlag()){
				    ConstructTakeOffPath(state, -7.5, B5_pnr, B5_vnr, B5_anr);
				}
				else if (_ctl.GetTransition1Flag()) {
					_ctl.ConstructTransition1PathRef(B5_pnr, B5_vnr, B5_anr);
				}
				else if (_ctl.GetPath1Flag()){
					_ctl.ConstructPath1Ref(B5_pnr, B5_vnr, B5_anr);
				}
				else if (_ctl.GetVisionInitializationFlag()){
					_ctl.ConstructVisionInitializationPathref(B5_pnr, B5_vnr, B5_anr);
				}
				else if (_ctl.GetVisionGuidanceFlag()){
					_ctl.ConstructVisionGuidancePathRef(B5_pnr, B5_vnr, B5_anr);
				}
				else if (_ctl.GetTransition2Flag()){
					_ctl.ConstructTransition2PathRef(B5_pnr, B5_vnr, B5_anr);
				}
				else if (_ctl.GetPath2Flag()){
					_ctl.ConstructPath2Ref(B5_pnr, B5_vnr, B5_anr);
				}
				else if (_ctl.GetLandingFlag()){
					_ctl.ConstructLandingPath(state, B5_pnr, B5_vnr, B5_anr);
				}
				else{ //hover
					B5_vnr[0] = 0; B5_vnr[1] = 0; B5_vnr[2] = 0;
					B5_anr[0] = 0; B5_anr[1] = 0; B5_anr[2] = 0;
					printf("[ctl:autoPathgeneration-Hover] pnr: %.2f %.2f %.2f %.2f; vnr: %.2f %.2f %.2f\n", B5_pnr[0], B5_pnr[1], B5_pnr[2], B5_pnr[3], B5_vnr[0], B5_vnr[1], B5_vnr[2]);
				}
			}
		}
		else{
			LOCATION pos0;
//			_state.GetCoordination(&pos0.longitude, &pos0.latitude);
//			_pathTmp.CreateSmoothPath(&m_smoothPath, &state, &pos0, B5_pnr, B5_vnr, B5_anr);
			B5_vnr[0] = 0; B5_vnr[1] = 0; B5_vnr[2] = 0;
			B5_anr[0] = 0; B5_anr[1] = 0; B5_anr[2] = 0;

			if (m_nCount % 50 == 0) {
				printf("pnr ref: %f, %f, %f, %f \n", B5_pnr[0], B5_pnr[1], B5_pnr[2], B5_pnr[3]);
				printf("vnr ref: %f, %f, %f \n", B5_vnr[0], B5_vnr[1], B5_vnr[2]);
			}
		}
	}

	else {	// hover
		B5_xref = B5_pnr[0] = B5_x; B5_yref = B5_pnr[1] = B5_y; B5_zref = B5_pnr[2] = B5_z; B5_cref = B5_pnr[3] = B5_c;
		::memset(B5_vnr, 0, 3*sizeof(double));
		::memset(B5_anr, 0, 3*sizeof(double));

		if (m_nCount % 50 == 0) {
			printf("Indoor pnr ref: %f, %f, %f, %f \n", B5_pnr[0], B5_pnr[1], B5_pnr[2], B5_pnr[3]);
			printf("Indoor vnr ref: %f, %f, %f \n", B5_vnr[0], B5_vnr[1], B5_vnr[2]);
		}
	}
}

void clsCTL::GCSRefGeneration()
{
	UAVSTATE state = _state.GetState();

/*	B5_vax[0] += B5_vax[0]; B5_vay[0] += B5_vay[0]; B5_vaz[0] += B5_vaz[0];
	B5_vax[1] = B5_vay[1] = B5_vaz[1] = 0;
	B5_vac += B5_vac;*/

	double vhr[3] = {B5_vax[0], B5_vay[0], B5_vaz[0]};
	double ahr[3] = {B5_vax[1], B5_vay[1], B5_vaz[1]};

	double abc0[3] = {0, 0, /*B5_semiPsi*/ state.c};
	G2N(abc0, vhr, B5_vnr);
	G2N(abc0, ahr, B5_anr);

	/// B5_pnr always follow the current position
	B5_pnr[0] = state.x; B5_pnr[1] = state.y; B5_pnr[2] = state.z;
	B5_pnr[3] = B5_vac + B5_c; INPI(B5_pnr[3]);
}

void clsCTL::ManualReferenceGeneration()
{
	UAVSTATE state = _state.GetState();

	double velRef[4] = {0}; //uref, vref, wref, rref;
	double posRef[4] = {0}; //xref, yref, zref, cref

	// convert to physical velocity and yaw rate
	SVODATA &svodata  = _svo.GetSVOData();
//	double simSin = ::sin(0.1*PI*::GetTime())*0.2;
//	svodata.throttle = simSin;
	double manualSig[4] = {svodata.elevator, svodata.aileron, svodata.throttle, svodata.rudder};

//	manualSig[0] = ::sin(0.2*PI*::GetTime())*0.2;
//	manualSig[2] = ::sin(0.1*PI*::GetTime())*0.2;

	if ( (manualSig[3] < MANUAL_DEADZON) && (manualSig[3] > -MANUAL_DEADZON) ) {
		manualSig[3] = 0; velRef[3] = 0;
	}

	if ( (manualSig[0] < MANUAL_DEADZON) && (manualSig[0] > -MANUAL_DEADZON) ) {
		manualSig[0] = 0; velRef[0] = 0;
	}

	if ( (manualSig[1] < MANUAL_DEADZON) && (manualSig[1] > -MANUAL_DEADZON) ) {
		manualSig[1] = 0; velRef[1] = 0;
	}

	if ( (manualSig[2] < MANUAL_DEADZON) && (manualSig[2] > -MANUAL_DEADZON) ) {
		manualSig[2] = 0; velRef[2] = 0;
	}

	if (manualSig[3]>=MANUAL_DEADZON)
		velRef[3] = (manualSig[3] - MANUAL_DEADZON) * (-MAX_YAWRATE);
	else if (manualSig[3] <= -MANUAL_DEADZON )
		velRef[3] = (manualSig[3] + MANUAL_DEADZON) * (-MAX_YAWRATE);

	if (manualSig[0]>=MANUAL_DEADZON)
		velRef[0] = (manualSig[0] - MANUAL_DEADZON) * (-MAX_LONSPEED);
	else if (manualSig[0] <= -MANUAL_DEADZON )
		velRef[0] = (manualSig[0] + MANUAL_DEADZON) * (-MAX_LONSPEED);

	if (manualSig[1]>=MANUAL_DEADZON)
		velRef[1] = (manualSig[1] - MANUAL_DEADZON) * (-MAX_LATSPEED);
	else if (manualSig[1] <= -MANUAL_DEADZON )
		velRef[1] = (manualSig[1] + MANUAL_DEADZON) * (-MAX_LATSPEED);

	if (manualSig[2]>=MANUAL_DEADZON)
		velRef[2] = (manualSig[2] - MANUAL_DEADZON) * (-MAX_VERSPEED);
	else if (manualSig[2] <= -MANUAL_DEADZON )
		velRef[2] = (manualSig[2] + MANUAL_DEADZON) * (-MAX_VERSPEED);

	/// apply 2nd order LPF to generate vel_ref and acc_ref
	// calculate dt
	double t = ::GetTime();
	double dt = 0;
	if (B5_bSemi1stFlag) {
		B5_vax[0] = state.ug; B5_vax[1] = 0;
		B5_vay[0] = state.vg; B5_vay[1] = 0;
		B5_vaz[0] = state.wg; B5_vaz[1] = 0;
		B_cr = state.c;
		dt = 0;
		B5_bSemi1stFlag = FALSE;
	} else {
		dt = ::GetTime() - B5_t1;
	}
	B5_t1 = t;

//	velRef[2] = 0;
	velRef[2] = state.wg;
//	GremLionThrottleControl(velRef[2]);

	double vhr[3] = {B5_vax[0], B5_vay[0], B5_vaz[0]};
	double ahr[3] = {B5_vax[1], B5_vay[1], B5_vaz[1]};

	double abc0[3] = {0, 0, B5_semiPsi/*state.c*/};
	G2N(abc0, vhr, B5_vnr);
	G2N(abc0, ahr, B5_anr);

/*	if (m_nCount % 50 == 0) {
		printf("vnr[0] %f, vnr[1] %f \n", B5_vnr[0], B5_vnr[1]);
		printf("anr[0] %f, anr[1] %f \n", B5_anr[0], B5_anr[1]);
	}*/

	/// B5_pnr always follow the current position
	B5_pnr[0] = state.x; B5_pnr[1] = state.y; B5_pnr[2] = state.z;

	/// Runge Kutta update
	for (int i=0; i<3; i++) {
		double _tmp[2] = {0};
		if (i == 0) {
//			::memcpy(_tmp, B5_vax, 2*sizeof(double));
			_tmp[0] = B5_vax[0]; _tmp[1] = B5_vax[1];
		} else if (i == 1) {
//			::memcpy(_tmp, B5_vay, 2*sizeof(double));
			_tmp[0] = B5_vay[0]; _tmp[1] = B5_vay[1];
		} else if (i == 2) {
//			::memcpy(_tmp, B5_vaz, 2*sizeof(double));
			_tmp[0] = B5_vaz[0]; _tmp[1] = B5_vaz[1];
		}

		clsVector tmp(2, _tmp, TRUE);
		double _Ax1[2]; clsVector Ax1(2, _Ax1, TRUE);
		double _Bu1[2]; clsVector Bu1(2, _Bu1, TRUE);

		clsMatrix::X(B5_Aorg_GREMLION, tmp, Ax1);

//		Bu1 = B5_Borg_GREMLION;
//		Bu1.operator *= (velRef[i]);
		_Bu1[0] = _B5_Borg_GREMLION[0] * velRef[i];
		_Bu1[1] = _B5_Borg_GREMLION[1] * velRef[i];
		double _dx0[2]; clsVector dx0(2, _dx0, TRUE);
		dx0 = Ax1; dx0 += Bu1;

		double _dx[2]; clsVector dx(2, _dx, TRUE);
		dx = dx0;

		double _Ah[2][2]; clsMatrix Ah(2,2,(double *)_Ah, TRUE);
		double _Ah2[2][2]; clsMatrix Ah2(2,2,(double *)_Ah2, TRUE);
		double _Ah3[2][2]; clsMatrix Ah3(2,2,(double *)_Ah3, TRUE);

		Ah = B5_Aorg_GREMLION; Ah *= dt/2;
		Ah2 = B5_Aorg2_GREMLION; Ah2 *= dt*dt/6;
		Ah3 = B5_Aorg3_GREMLION; Ah3 *= dt*dt*dt/24;

		double _Ah123[2][2]; clsMatrix Ah123(2,2,(double*)_Ah123,TRUE);
		Ah123 = Ah; Ah123 += Ah2; Ah123 += Ah3;

		double _dx123[2]; clsVector dx123(2,_dx123,TRUE);
		clsMatrix::X(Ah123, dx0, dx123);

		dx += dx123;
		dx *= dt;

		tmp += dx;

		if (i == 0) {
//			::memcpy(B5_vax, _tmp, 2*sizeof(double));
			B5_vax[0] = _tmp[0]; B5_vax[1] = _tmp[1];
		}
		else if (i == 1) {
//			::memcpy(B5_vay, _tmp, 2*sizeof(double));
			B5_vay[0] = _tmp[0]; B5_vay[1] = _tmp[1];
		}
		else if (i == 2) {
//			::memcpy(B5_vaz, _tmp, 2*sizeof(double));
			B5_vaz[0] = _tmp[0]; B5_vaz[1] = _tmp[1];
		}
	}

	/// heading reference
	B5_semiPsi += velRef[3]*dt;
	INPI(B5_semiPsi);

	double dc = state.c - B5_semiPsi; INPI(dc);
	if (::fabs(dc) < HEADINGERR_DEADZONE ) dc = 0;
	B5_PsiErr += dc*dt;
	B5_PsiErr = range(B5_PsiErr, MIN_TRIM_ERROR, MAX_TRIM_ERROR);
//	B_cr =  B5_semiPsi - 0.2*B5_PsiErr; INPI(B_cr);
	B5_pnr[3] = B5_semiPsi - 0.2*B5_PsiErr; INPI(B5_pnr[3]);
}

void clsCTL::GremLionThrottleControl(double& velz)
{
	SVODATA svodata = _svo.GetSVOData();
	if ( (m_nThrottleMode == 1) && (svodata.throttle <= 0) ) {
		m_nThrottleCase = 1;
		m_nThrottleMode = 1;
		if (m_bThrMode1) {
			B5_vaz[0] = B5_vaz[1] = 0;
			m_bThrMode1 = FALSE;
		}
		m_bThrottleBypass = TRUE;
		velz = 0;	// wr = 0 m/s
//		printf("Throttle control: mode 1 \n");
	}
	else if ( (m_nThrottleMode == 1) && (svodata.throttle > 0) ) {
		m_nThrottleCase = 2;
		m_nThrottleMode = 0;
		B5_vaz[0] = B5_vaz[1] = 0;
		m_bThrottleBypass = FALSE;
		m_bThrMode1 = TRUE;
//		printf("Throttle control: mode 2 \n");
	}
	else if ( (m_nThrottleMode == 0) && (svodata.throttle >= THROTTLE_SHUTDOWN) ) {
		m_nThrottleCase = 3;
		m_nThrottleMode = 0;
		m_bThrottleBypass = FALSE;
//		printf("Throttle control: mode 3 \n");
		if (!m_bLandFlag) {
			if (m_bThrMode3){
				B5_vaz[0] = B5_vaz[1] = 0;
				m_bThrMode3 = FALSE;
			}
			if (m_bLandCmd) {
				if (velz >= 1) velz = 1;

				DAQDATA daqdata = _daq.GetDAQData();
				double sonarHeight = daqdata.height;
				if (sonarHeight < LAND_HEIGHT) m_nLandCnt++;
				else m_nLandCnt--;

				if (m_nLandCnt == 5)
					m_bLandFlag = TRUE;
			}
		}
		else if (m_bLandFlag) {
			m_sig.throttle -= 0.05;
			velz = 0;
			if (m_sig.throttle <= -MAX_THROTTLE) m_sig.throttle = -MAX_THROTTLE;
		}
	}
	else if ( (m_nThrottleMode == 0) && (svodata.throttle < THROTTLE_SHUTDOWN) ) {
//		printf("Throttle control: mode 4 \n");
		m_nThrottleCase = 4;
		m_nThrottleMode = 1;
		B5_vaz[0] = B5_vaz[1] = 0;
		m_bThrMode3 = TRUE;
		m_bThrottleBypass = TRUE;
		m_bLandCmd = m_bLandFlag = FALSE;
	}

}

void clsCTL::B5_GremLion_AllAuto()
{
	UAVSTATE &state = _state.GetState();

	// convert to physical velocity and yaw rate
	SVODATA &svodata  = _svo.GetSVOData();
	SVODATA &svodata1 = _svo.GetSVOData1();
	double manualSig[4] = {svodata.aileron, svodata.elevator, svodata.rudder, svodata.throttle};

	double uref, vref, rref, wref;
	if ( (manualSig[2] < MANUAL_DEADZON) && (manualSig[2] > -MANUAL_DEADZON) ) {
		manualSig[2] = 0; rref = 0;
	}

	if ( (manualSig[0] < MANUAL_DEADZON) && (manualSig[0] > -MANUAL_DEADZON) ) {
		manualSig[0] = 0; vref = 0;
	}

	if ( (manualSig[1] < MANUAL_DEADZON) && (manualSig[1] > -MANUAL_DEADZON) ) {
		manualSig[1] = 0; uref = 0;
	}

	if ( (manualSig[3] < MANUAL_DEADZON) && (manualSig[3] > -MANUAL_DEADZON) ) {
		manualSig[3] = 0; wref = 0;
	}

	if (manualSig[2]>=MANUAL_DEADZON)
		rref = (manualSig[2] - MANUAL_DEADZON) * (-MAX_YAWRATE);
	else if (manualSig[2] <= -MANUAL_DEADZON )
		rref = (manualSig[2] + MANUAL_DEADZON) * (-MAX_YAWRATE);

	if (manualSig[0]>=MANUAL_DEADZON)
		vref = (manualSig[0] - MANUAL_DEADZON) * (-MAX_LATSPEED);
	else if (manualSig[0] <= -MANUAL_DEADZON )
		vref = (manualSig[0] + MANUAL_DEADZON) * (-MAX_LATSPEED);

	if (manualSig[1]>=MANUAL_DEADZON)
		uref = (manualSig[1] - MANUAL_DEADZON) * (-MAX_LONSPEED);
	else if (manualSig[1] <= -MANUAL_DEADZON )
		uref = (manualSig[1] + MANUAL_DEADZON) * (-MAX_LONSPEED);

	if (manualSig[3]>=MANUAL_DEADZON)
		wref = (manualSig[3] - MANUAL_DEADZON) * (-MAX_VERSPEED);
	else if (manualSig[3] <= -MANUAL_DEADZON )
		wref = (manualSig[3] + MANUAL_DEADZON) * (-MAX_VERSPEED);

	// update inner loop trim values given different velocity references
	double manualSig1[3] = {svodata1.aileron, svodata1.elevator, svodata1.rudder};
	double uref1, vref1;
	if ( (manualSig1[0] < MANUAL_DEADZON) && (manualSig1[0] > -MANUAL_DEADZON) ) {
		manualSig1[0] = 0; vref1 = 0;
	}

	if ( (manualSig1[1] < MANUAL_DEADZON) && (manualSig1[1] > -MANUAL_DEADZON) ) {
		manualSig1[1] = 0; uref1 = 0;
	}

	if (manualSig1[0]>=MANUAL_DEADZON)
		vref1 = (manualSig1[0] - MANUAL_DEADZON) * (-MAX_LATSPEED);
	else if (manualSig1[0] <= -MANUAL_DEADZON )
		vref1 = (manualSig1[0] + MANUAL_DEADZON) * (-MAX_LATSPEED);

	if (manualSig1[1]>=MANUAL_DEADZON)
		uref1 = (manualSig1[1] - MANUAL_DEADZON) * (-MAX_LONSPEED);
	else if (manualSig1[1] <= -MANUAL_DEADZON )
		uref1 = (manualSig1[1] + MANUAL_DEADZON) * (-MAX_LONSPEED);

	double velRef[3] = {uref1, vref1, 0};
	double abc[3] = {state.a, state.b, 0};
	double velRefb[3] = {0};
	G2B(abc, velRef, velRefb);

//	A2_equ.u = velRefb[0]; A2_equ.v = velRefb[1];
//	A2_equ.a = velRefb[1]/200;
//	A2_equ.b = -velRefb[0]/200;

	// calculate dt
	double t = ::GetTime();
	double dt = 0;
	if (B5_t1 < 0) {
		dt = 0;
	} else {
		dt = ::GetTime() - B5_t1;
	}
	B5_t1 = t;

	// semi-auto control law
	B5_semiPsi += rref*dt;
	INPI(B5_semiPsi);

	double dc = state.c - B5_semiPsi; INPI(dc);
	if (::fabs(dc) < HEADINGERR_DEADZONE ) dc = 0;
	B5_PsiErr += dc*dt;
	B5_PsiErr = range(B5_PsiErr, MIN_TRIM_ERROR, MAX_TRIM_ERROR);
	B5_semiPsic = B5_semiPsi - 0.2*B5_PsiErr;

	double Bref[2][2] = {{cos(B5_semiPsi), sin(B5_semiPsi)}, {-sin(B5_semiPsi), cos(B5_semiPsi)}};
	double vn[2] = {0};
	vn[0] = Bref[0][0]*state.ug + Bref[0][1]*state.vg;
	vn[1] = Bref[1][0]*state.ug + Bref[1][1]*state.vg;
	vn[2] = state.wg;

	double aggnhc[3] = {0};
	aggnhc[0] = B5_FVn*(vn[0] - uref) - rref*vn[1];
	aggnhc[1] = B5_FVn*(vn[1] - vref) + rref*vn[0];
	aggnhc[2] = B5_FVn*(vn[2] - wref);

	double agghc[3] = {0};
	agghc[0] = Bref[0][0]*aggnhc[0] + Bref[1][0]*aggnhc[1];
	agghc[1] = Bref[0][1]*aggnhc[0] + Bref[1][1]*aggnhc[1];
	agghc[2] = aggnhc[2];
	double agb[3] = {0};
	double aggh[3] = {agghc[0],agghc[1], agghc[2]};
	G2B(&state.a, aggh, agb);

	B_acxr_ub = agb[0];
	B_acyr_vb = agb[1];
	B_aczr_wb = agb[2];
	B_cr = B5_semiPsic; //B5_semiPsi;

	B_acxr_ub = range(B_acxr_ub, MIN_STATE_ACX, MAX_STATE_ACX);
	B_acyr_vb = range(B_acyr_vb, MIN_STATE_ACY, MAX_STATE_ACY);
	B_aczr_wb = range(B_aczr_wb, MIN_STATE_ACW, MAX_STATE_ACW);
/*	if (m_nCount % _DEBUG_COUNT == 0) {
		printf("[CTL] B5_semiauto, agghc[0] %.4g, agghc[1] %.4g \n", agghc[0], agghc[1]);
		printf("[CTL] B5_semiauto, B_acxr_ub %.4g, B_acyr_vb %.4g, B_cr %.4g\n", B_acxr_ub, B_acyr_vb, B_cr);
	}*/
//	A1A2A3_r = range(A1A2A3_r, MIN_STATE_R, MAX_STATE_R);
}

/*
void clsCTL::B5_GremLion_SemiAuto()
{
//	UAVSTATE &state = _state.GetState();
	SVODATA &svodata = _svo.GetSVOData();
	double _transfer[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};
	double manualSig[3] = {svodata.aileron, svodata.elevator, svodata.auxiliary};
	double semiSig[3] = {0};

	clsMetric::X(_transfer, manualSig, semiSig);

	B_acxr_ub = semiSig[0];
	B_acyr_vb = semiSig[1];
	B_aczr_wb = semiSig[2];
//	B_cr = 0;
	B_acxr_ub = range(B_acxr_ub, MIN_STATE_ACX, MAX_STATE_ACX);
	B_acyr_vb = range(B_acyr_vb, MIN_STATE_ACY, MAX_STATE_ACY);
	B_aczr_wb = range(B_aczr_wb, MIN_STATE_ACW, MAX_STATE_ACW);
//	A1A2A3_r = range(A1A2A3_r, MIN_STATE_R, MAX_STATE_R);
}
*/

void clsCTL::B5_HeLion()
{
	UAVSTATE &state = _state.GetState();
	RPTSTATE &RPTState = _state.GetRPTState();
	RPTSTATE &RPTState0 = _state.GetRPTState0();

	double c0 = _state.Getc0();
	double psig = state.c - c0;
	double abcg[3] = {state.a, state.b, psig /*state.c - c0*/};
	double abc0[3] = {0, 0, c0};

//	double cosc = ::cos(psig); double sinc = ::sin(psig);
//	BOOL bRepeat = B5_mode & PATHTRACKING_REPEAT;

	double xcg, ycg, zcg;			// pos ref in ground frame
	double ucg, vcg, wcg;			// vel ref in ground frame
	double axcg, aycg, azcg;
//	double vrg[4];		// velocity reference in ground frame

	double tGet = ::GetTime();
	if (B5_pPath != NULL) {
		double t = tGet - B5_t0;

		BOOL bEnd = t >= B5_pPath->GetEndTime();
		if (bEnd && !B5_bEnd) {
			cout<<"clsCTL::B5 EVENT_BEHAVIOREND at "<<::GetTime()<<endl;
			_state.SetEvent(EVENT_BEHAVIOREND, m_behaviorCurrent.behavior);
		}
		B5_bEnd = bEnd;
		//the B5_end flag is used to prevent setting event endlessly, just send only once;

		double pos[4], vel[3], acc[3];
		if (B5_pPath->m_path.GetN() == 5) {
			B5_pPath->GetPositionVelocity(t, pos, vel, FALSE);
			if (m_nCount % 50 == 0) {
//				cout<<"Normal path ref: xcg"<<xcg<<" ycg "<<ycg<<" zcg "<<zcg<<endl;
				cout<<"Normal 5-column path ref: x"<<pos[0]<<" y "<<pos[1]<<" z "<<pos[2]<<endl;
				cout<<"Normal 5-column path ref: u"<<vel[0]<<" v "<<vel[1]<<" w "<<vel[2]<<endl;
			}
			acc[0] = acc[1] = acc[2] = 0;
		}
		else {
			B5_pPath->GetPosVelAcc(t, pos, vel, acc);
		}
		
		if ( _coop.GetRole() == FOLLOWER  && _coop.GetCoopTime() > 0 && _coop.GetConnectFlag() ) {
			double psider[2] = {0};
			if (m_nCount%50 == 0) {
				cout<<"leader velocity: "<<"ug "<<vel[0]<<" vg "<<vel[1]<<endl;
			}
			FormationKalman(pos,vel,acc,psider);
			
			double posnFollower[3], posgFollower[3];
			double longitude0, latitude0;
			_state.GetCoordination(&longitude0, &latitude0);

			posnFollower[0] = pos[0] + ::cos(_B_psi[0])*DIST_FD - ::sin(_B_psi[0])*DIST_LD;
			posnFollower[1] = pos[1] + ::sin(_B_psi[0])*DIST_FD + ::cos(_B_psi[0])*DIST_LD;
			posnFollower[2] = pos[2];
			posnFollower[3] = pos[3];

/*			if (m_nCount % 50 == 0)
			{
				cout<<"Leader pos: "<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<" "<<pos[3]*180/PI<<endl;
				cout<<"Follower pos n: "<<posnFollower[0]<<" "<<posnFollower[1]<<" "<<posnFollower[2]<<" "<<posnFollower[3]*180/PI<<endl;
			}*/

			double dx = (latitude0 - _coop.GetLeaderLati0())*_radius;
			double dy = (longitude0 - _coop.GetLeaderLong0())*_radius;

			posnFollower[0] -= dx;
			posnFollower[1] -= dy;

			N2G(abc0, posnFollower, posgFollower);

			xcg = posgFollower[0] - RPTState0.xg; ycg = posgFollower[1] - RPTState0.yg;
			zcg = 0 /*posgFollower[2] - RPTState0.zg*/;

/*			if (m_nCount % 25 == 0)
			{
				cout<<"Follower pos g: "<<posgFollower[0]<<" "<<posgFollower[1]<<" "<<posgFollower[2]<<" "<<posgFollower[3]*180/PI<<endl;
				cout<<"Follower pos g2: "<<xcg<<" "<<ycg<<" "<<zcg<<endl;
//				cout<<"xcg "<<xcg<<" ycg "<<ycg<<endl;
			}
*/
			// formation velocity generation
			double fveln[3] = {0}; double fvelg[3] = {0};
			fveln[0] = vel[0] - ::cos(_B_psi[0])*_B_psi[1]*DIST_LD - ::sin(_B_psi[0])*_B_psi[1]*DIST_FD;
			fveln[1] = vel[1] - ::sin(_B_psi[0])*_B_psi[1]*DIST_LD + ::cos(_B_psi[0])*_B_psi[1]*DIST_FD;
			fveln[2] = 0 /*vel[2]*/;
			N2G(abc0, fveln, fvelg);
			ucg = fvelg[0]; vcg = fvelg[1]; wcg = fvelg[2];
			
			if (m_nCount % 50 == 0) {
				cout<<"Follower ground velocity: ucg "<<ucg<<" vcg "<<vcg<<endl;
			}
//			ucg = vel[0]; vcg = vel[1]; wcg = 0 /*vel[2]*/;
//			ucg = vcg = wcg = 0;
			
			// formation acc generation
//			axcg = aycg = azcg = 0;
			double faccn[3] = {0}; double faccg[3] = {0};
			faccn[0] = acc[0] - ::cos(_B_psi[0])*_B_psi[2]*DIST_LD - ::sin(_B_psi[0])*_B_psi[2]*DIST_FD - ::cos(_B_psi[0])*_B_psi[1]*_B_psi[1]*DIST_FD + ::sin(_B_psi[0])*_B_psi[1]*_B_psi[1]*DIST_LD;
			faccn[1] = acc[1] - ::sin(_B_psi[0])*_B_psi[2]*DIST_LD + ::cos(_B_psi[0])*_B_psi[2]*DIST_FD - ::sin(_B_psi[0])*_B_psi[1]*_B_psi[1]*DIST_FD - ::cos(_B_psi[0])*_B_psi[1]*_B_psi[1]*DIST_LD;
			faccn[2] = 0;
			N2G(abc0, faccn, faccg);
			axcg = faccg[0]; aycg = faccg[1]; azcg = faccg[2];
			
			
			B_cr = _B_psi[0] /*pos[3]*/ - c0; INPI(B_cr);

			B5_xref = posnFollower[0] /*+ RPTState0.xn*/; B5_yref = posnFollower[1] /*+ RPTState0.yn*/;
//			B5_zref = posnFollower[2] /*+ RPTState0.zn*/;
			B5_zref = zcg;
			B5_cref = _B_psi[0]; INPI(B5_cref);

			A1A2A3_u = ucg; A1A2A3_v = vcg; A1A2A3_w = axcg; A1A2A3_r = aycg;
		}
		else {		// non-cooperative normal path tracking
				xcg = pos[0]; ycg = pos[1]; zcg = pos[2];
				ucg = vel[0]; vcg = vel[1]; wcg = vel[2];
				axcg = acc[0]; aycg = acc[1]; azcg = acc[2];
				B_cr = pos[3]; INPI(B_cr);

				double posrg[3] = {xcg, ycg, zcg};
				double posrn[3];
				G2N(abc0, posrg, posrn);

				A1A2A3_u = ucg; A1A2A3_v = vcg; A1A2A3_w = axcg; A1A2A3_r = aycg;
				B5_xref = posrn[0] + RPTState0.xn; B5_yref = posrn[1] + RPTState0.yn; B5_zref = posrn[2] + RPTState0.zn;
				B5_cref = pos[3] + c0; INPI(B5_cref);

/*				if (m_nCount % 50 == 0) {
//					cout<<"RPT 9-column path ref: xcg"<<xcg<<" ycg "<<ycg<<" zcg "<<zcg<<endl;
					cout<<"RPT 9-column path ref: ucg"<<ucg<<" vcg "<<vcg<<" wcg "<<wcg<<endl;
				}*/
		}

/*		xcg = pos[0]; ycg = pos[1];	zcg = pos[2];
		B_cr = pos[3]; INPI(B_cr);
		ucg = vel[0]; vcg = vel[1]; wcg = vel[2];
		axcg = acc[0]; aycg = acc[1]; azcg = acc[2];

		double posrg[3] = {pos[0], pos[1], pos[2]};
		double posrn[3];
		G2N(abc0, posrg, posrn);

		B5_xref = posrn[0] + RPTState0.xn; B5_yref = posrn[1] + RPTState0.yn; B5_zref = posrn[2] + RPTState0.zn;
		B5_cref = pos[3] + c0; INPI(B5_cref);*/

	}
	else {	// hover
		B5_xref = RPTState0.xn; B5_yref = RPTState0.yn; B5_zref = RPTState0.zn;
		B5_cref = c0;

		B_cr = 0;
		xcg = ycg = zcg = 0;
		ucg = vcg = wcg = 0;
		axcg = aycg = azcg = 0;
	}

	// outerloop control law
	double _xo[6] = {
		RPTState.xg - RPTState0.xg, RPTState.yg - RPTState0.yg, RPTState.zg - RPTState0.zg,
		RPTState.ug, RPTState.vg, RPTState.wg
	};
	clsVector xo(6, _xo, TRUE);

	double _Fxo[6]; clsVector Fxo(3, _Fxo, TRUE);
	clsMatrix::X(B5_F, xo, Fxo);

	double _vo[9] = {
		xcg, ycg, zcg,
		ucg, vcg, wcg,
		axcg, aycg, azcg,
	};
	clsVector vo(9, _vo, TRUE);
	double _Gvo[3]; clsVector Gvo(3, _Gvo, TRUE);
	clsMatrix::X(B5_G, vo, Gvo);

	double _agc[3]; clsVector agc(3, _agc, TRUE);
	agc = Fxo; agc += Gvo;

	double abc[3];
	G2B(abcg, _agc, abc);

	B_acxr_ub = abc[0];
	B_acyr_vb = abc[1];
	B_aczr_wb = abc[2];
//	B_cr = 0;
	B_acxr_ub = range(B_acxr_ub, MIN_STATE_ACX, MAX_STATE_ACX);
	B_acyr_vb = range(B_acyr_vb, MIN_STATE_ACY, MAX_STATE_ACY);
	B_aczr_wb = range(B_aczr_wb, MIN_STATE_ACW, MAX_STATE_ACW);
//	A1A2A3_r = range(A1A2A3_r, MIN_STATE_R, MAX_STATE_R);
}

void clsCTL::B15()				//B5 added with PI control
{
	UAVSTATE &state = _state.GetState();

	BOOL bRepeat = B15_mode & PATHTRACKING_REPEAT;

	double xc, yc, zc, cc;
	double uc, vc, wc, rc;				//ground frame

	double tGet = ::GetTime();

	if (B15_pPath != NULL) {
		double t = tGet - B15_t0;

		BOOL bEnd = t >= B15_pPath->GetEndTime();
		if (bEnd && !B15_bEnd)
			_state.SetEvent(EVENT_BEHAVIOREND, m_behaviorCurrent.behavior);
		B15_bEnd = bEnd;
		//the B5_end flag is used to prevent setting event endlessly
		//just send only once;

		double pc[4], pc2[4], pc02[4];				//x, y, z, c and u, v, w, r
//		B5_pPath->GetPositionVelocity(t, pc, vr, bRepeat);
		B5_pPath->GetPositionVelocity(t, pc, NULL, bRepeat);
		B5_pPath->GetPositionVelocity(t+6, pc2, NULL, bRepeat);
		B5_pPath->GetPositionVelocity(t+0.2, pc02, NULL, bRepeat);

		if (B15_pPath2 != NULL) {
			double ac[4], ac2[4], ac02[4];
			B5_pPath2->GetPositionVelocity(t, ac, NULL, bRepeat);
			B5_pPath2->GetPositionVelocity(t+6, ac2, NULL, bRepeat);
			B5_pPath2->GetPositionVelocity(t+0.2, ac02, NULL, bRepeat);

			for (int i=0; i<=3; i++) {
				pc[i] += ac[i];
				pc2[i] += ac2[i];
				pc02[i] += ac02[i];
			}
		}

		double dc = pc02[3]-pc[3];
		INPI(dc);

		double vr[4] = {
			(pc2[0]-pc[0])/6, (pc2[1]-pc[1])/6, (pc2[2]-pc[2])/6, dc/0.2
		};

//		double vr[4], vrr[4];
//		B5_pPath->GetPositionVelocity(t+1, NULL, vr, bRepeat);
//		B5_pPath->GetPositionVelocity(t+0.1, NULL, vrr, bRepeat);				//get predictive yaw rate
//		vr[3] = vrr[3];

		if (B15_mode & PATHTRACKING_ADDON) {
			double sinc = ::sin(B15_c0);
			double cosc = ::cos(B15_c0);

			double x = cosc*pc[0]-sinc*pc[1];
			double y = sinc*pc[0]+cosc*pc[1];
			pc[0] = x; pc[1] = y;

			double u = cosc*vr[0]-sinc*vr[1];
			double v = sinc*vr[0]+cosc*vr[1];
			vr[0] = u; vr[1] = v;

			pc[0] += B15_x0; pc[1] += B15_y0; pc[2] += B15_z0;
			pc[3] += B15_c0;
		}

		xc = pc[0]; yc = pc[1]; zc = pc[2]; cc = pc[3];
		uc = vr[0]; vc = vr[1]; wc = vr[2]; rc = vr[3];
	}
	else {
		xc = B15_x; yc = B15_y; zc = B15_z; cc = B15_c;
		uc = vc = wc = rc = 0;
	}

	double dx = state.x - xc;
	double dy = state.y - yc;
	double dz = state.z - zc;
	double dc = state.c - cc;
	INPI(dc);

	double dt = tGet-B15_t;
	B15_dxi += dx*dt;
	B15_dyi += dy*dt;
	B15_dzi += dz*dt;
	B15_dci += dc*dt;

	B15_t = tGet;

	double velg[3] = {
		uc+B15_kx*dx+B15_kxi*B15_dxi,				//predictive velocity plus position error feedback
		vc+B15_ky*dy+B15_kyi*B15_dyi,
		wc+B15_kz*dz+B15_kzi*B15_dzi
	};

	double vel[3];
	G2B(&state.a, velg, vel);

	A1A2A3_u = vel[0];
	A1A2A3_v = vel[1];
	A1A2A3_w = vel[2];

	A1A2A3_r = rc + B15_kc*dc + B15_kci*B15_dci;

#if (_DEBUG & DEBUGFLAG_CTL)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[CTL] B5, x %.4g, y %.4g, z %.4g, c %.4g\n", xc, yc, zc, cc);
	printf("[CTL] B5, ug %.4g, vg %.4g, wg %.4g, r %.4g\n", velg[0], velg[1], velg[2], A1A2A3_r);
}
#endif

	A1A2A3_u = range(A1A2A3_u, MIN_STATE_U, MAX_STATE_U);
	A1A2A3_v = range(A1A2A3_v, MIN_STATE_V, MAX_STATE_V);
	A1A2A3_w = range(A1A2A3_w, MIN_STATE_W, MAX_STATE_W);
	A1A2A3_r = range(A1A2A3_r, MIN_STATE_R, MAX_STATE_R);
}

void clsCTL::B6()
{
	B6_vChirp[0] = B6_vChirp[1] = B6_vChirp[2] = B6_vChirp[3] = 0;
//	cout<<"B6"<<endl;
	if (!B6_bChirp) return;

	double t = GetTime() - B6_t0;
	if (t > B6_T) {
		B6_bChirp = FALSE;
		return;
	}

	double chirp = B6_a*::sin(B6_om1*2*PI*t+(B6_om2-B6_om1)*2*PI*t*t/(2*B6_T));

#if (_DEBUG & DEBUGFLAG_CTL)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[CTL] B6, time %f, T %f, channel %d, chirp %f\n", t, B6_T, B6_channel, chirp);
	printf("[CTL] B6, a %f, om1 %f, om2 %f\n", B6_a, B6_om1, B6_om2);
}
#endif

	switch (B6_channel) {
	case 1:
		B6_vChirp[0] = chirp;
		break;
	case 2:
		B6_vChirp[1] = chirp;
		break;
	case 3:
		B6_vChirp[2] = chirp;
		break;
	case 4:
		B6_vChirp[3] = chirp;
		break;
	}
}

void clsCTL::B7()
{
	UAVSTATE &state = _state.GetState();

	double dx[3] = {
		state.x - B7_x,
		state.y - B7_y,
		state.z - B7_z
	};

#if (_DEBUG & DEBUGFLAG_CTL)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[CTL] B7, B7_pos %f %f %f\n", B7_x, B7_y, B7_z);
	printf("[CTL] B7, position %f %f %f\n", state.x, state.y, state.z);
}
#endif

	double velg[3] = { B7_kx*dx[0], B7_ky*dx[1], B7_kz*dx[2] };

	double Mfg[3][3];
	clsMetric::AttitudeToTransformMatrix(&state.a, Mfg, NULL);

	double vel[3];
	clsMetric::X(Mfg, velg, vel);

	double dpsi = state.c - B7_c;
	if (dpsi < -PI) dpsi += 2*PI;
	else if (dpsi > PI) dpsi -= 2*PI;

	A1A2A3_u = vel[0];
	A1A2A3_v = vel[1];
	A1A2A3_w = vel[2];

	A1A2A3_r = B7_kc*dpsi;

	A1A2A3_u = range(A1A2A3_u, MIN_STATE_U, MAX_STATE_U);
	A1A2A3_v = range(A1A2A3_v, MIN_STATE_V, MAX_STATE_V);
	A1A2A3_w = range(A1A2A3_w, MIN_STATE_W, MAX_STATE_W);
	A1A2A3_r = range(A1A2A3_r, MIN_STATE_R, MAX_STATE_R);
}

void clsCTL::B8()
{
	UAVSTATE &state = _state.GetState();

	double dc = state.c - B8_c;
	if (dc > PI) dc -= 2*PI;
	else if (dc < -PI) dc += 2*PI;

	A1A2A3_r = B8_kc*dc;
}

void clsCTL::B9()
{
	UAVSTATE &state = _state.GetState();

	double dpos[3] = { state.x - B9_x, state.y - B9_y, state.z - B9_z };

	double velg[3] = { B9_kx*dpos[0], B9_ky*dpos[1], B9_kz*dpos[2] };

	double Mfg[3][3];
	clsMetric::AttitudeToTransformMatrix(&state.a, Mfg, NULL);

	double vel[3];
	clsMetric::X(Mfg, velg, vel);

	A1A2A3_u = vel[0];
	A1A2A3_v = vel[1];
	A1A2A3_w = vel[2];

	double distance = clsMetric::Norm(dpos);

	if (distance < 5) A1A2A3_r = 0;
	else {
		double cref = ::atan2(dpos[1], dpos[0]);
		double dc = state.c - cref;

		if (dc > PI) dc -= 2*PI;
		else if (dc < -PI) dc += 2*PI;

		A1A2A3_r = B9_kc*dc;
	}

	A1A2A3_u = range(A1A2A3_u, MIN_STATE_U, MAX_STATE_U);
	A1A2A3_v = range(A1A2A3_v, MIN_STATE_V, MAX_STATE_V);
	A1A2A3_w = range(A1A2A3_w, MIN_STATE_W, MAX_STATE_W);
	A1A2A3_r = range(A1A2A3_r, MIN_STATE_R, MAX_STATE_R);
}

void clsCTL::B10()
{
}

void clsCTL::B11()
{
	UAVSTATE &state = _state.GetState();

	double tGet = ::GetTime();
	double t = tGet -B11_t0;

	BOOL bRepeat = B11_mode & PATHTRACKING_REPEAT;

	double dt = 0.1;
	double pc[4], pc2[4];				//x, y, z, c and u, v, w, r
//	B11_pPath->GetPositionVelocity(t, pc, vr, bRepeat);
	B11_pPath->GetPositionVelocity(t, pc, NULL, bRepeat);

	B11_pPath->GetPositionVelocity(t+2, pc2, NULL, bRepeat);				//to get predictive velocity
	double vr[4] = {
		(pc2[0]-pc[0])/2, (pc2[1]-pc[1])/2, (pc2[2]-pc[2])/2, 0
	};

	B11_pPath->GetPositionVelocity(t+dt, pc2, NULL, bRepeat);				//to estimate predictive yaw rate

	if (B11_mode & PATHTRACKING_ADDON) {
		double sinc = ::sin(B11_c0);
		double cosc = ::cos(B11_c0);

		double x = cosc*pc[0]-sinc*pc[1];
		double y = sinc*pc[0]+cosc*pc[1];
		pc[0] = x; pc[1] = y;

		x = cosc*pc2[0]-sinc*pc2[1];
		y = sinc*pc2[0]+cosc*pc2[1];
		pc2[0] = x; pc2[1] = y;

		double u = cosc*vr[0]-sinc*vr[1];
		double v = sinc*vr[0]+cosc*vr[1];
		vr[0] = u; vr[1] = v;

		pc[0] += B11_x0; pc[1] += B11_y0; pc[2] += B11_z0;
		pc2[0] += B11_x0; pc2[1] += B11_y0; pc2[2] += B11_z0;
	}

	//set reference velocity
	double velg[3] = {
		vr[0]+B11_kx*(state.x-pc[0]),
		vr[1]+B11_ky*(state.y-pc[1]),
		vr[2]+B11_kz*(state.z-pc[2])
	};

	double vel[3];
	G2B(&state.a, velg, vel);

	//set reference velocity
	A1A2A3_u = vel[0];
	A1A2A3_v = vel[1];
	A1A2A3_w = vel[2];

	//calculate predictive yaw rate
	double c2 = ::atan2(B11_yt-pc2[1], B11_xt-pc2[0]);
	double c1 = ::atan2(B11_yt-pc[1], B11_xt-pc[0]);
	double dc = c2 - c1;
	INPI(dc);
	double rc = dc/dt;

	//calculate yaw angle error and desired camera angle
	double dxt = B11_xt - state.x;
	double dyt = B11_yt - state.y;
	double dzt = B11_zt - state.z;
	double dxyt = ::sqrt(dxt*dxt+dyt*dyt);

	//camera angle
	m_camera = ::atan2(dzt, dxyt);

	//yaw angle error
	double c = ::atan2(dyt, dxt);				//the objective heading angle for camera tracking
	dc = state.c - c;
	INPI(dc);

	//set yaw rate, precdictive yaw rate plus feedback of yaw angle error
	A1A2A3_r = rc + B11_kc*dc;

#if (_DEBUG & DEBUGFLAG_CTL)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[CTL] B11, t %.4g, x %.4g, y %.4g, z %.4g, c %.4g\n", t, pc[0], pc[1], pc[2], c);
	printf("[CTL] B11, ug %.4g, vg %.4g, wg %.4g, r %.4g\n", velg[0], velg[1], velg[2], A1A2A3_r);
}
#endif

	//saturation
	A1A2A3_u = range(A1A2A3_u, MIN_STATE_U, MAX_STATE_U);
	A1A2A3_v = range(A1A2A3_v, MIN_STATE_V, MAX_STATE_V);
	A1A2A3_w = range(A1A2A3_w, MIN_STATE_W, MAX_STATE_W);
	A1A2A3_r = range(A1A2A3_r, MIN_STATE_R, MAX_STATE_R);
}

void clsCTL::B12()
{
	UAVSTATE &state = _state.GetState();

	double uc = B12_u + B12_ku*(state.u - B12_u);
	double vc = B12_v + B12_kv*(state.v - B12_v);
	double wc = B12_w + B12_kw*(state.w - B12_w);

	double rc = B12_r + B12_kr*(state.r - B12_r);

	A1A2A3_u = range(uc, MIN_STATE_U, MAX_STATE_U);
	A1A2A3_v = range(vc, MIN_STATE_V, MAX_STATE_V);
	A1A2A3_w = range(wc, MIN_STATE_W, MAX_STATE_W);
	A1A2A3_r = range(rc, MIN_STATE_R, MAX_STATE_R);
}

void clsCTL::B13()
{
	UAVSTATE &state = _state.GetState();

	double xc, yc, zc, cc;
	double uc, vc, wc;

	double tGet = ::GetTime();

	if (B13_pPath != NULL) {

		double t = tGet - B13_t0;
		BOOL bEnd = t >= B13_pPath->GetEndTime();
		if (bEnd && !B13_bEnd)
			_state.SetEvent(EVENT_BEHAVIOREND, m_behaviorCurrent.behavior);
		B13_bEnd = bEnd;

		BOOL bRepeat = B13_mode & PATHTRACKING_REPEAT;

		double pc[4], pc1[4], pc2[4];				//x, y, z, c
		B13_pPath->GetPositionVelocity(t, pc, NULL, bRepeat);
		B13_pPath->GetPositionVelocity(t+0.5, pc1, NULL, bRepeat);
		B13_pPath->GetPositionVelocity(t+2, pc2, NULL, bRepeat);

		double vr[3] = { (pc2[0]-pc[0])/2, (pc2[1]-pc[1])/2, (pc2[2]-pc[2])/2 };

		//transformation
		if (B13_mode & PATHTRACKING_ADDON) {
			double sinc = ::sin(B13_c0);
			double cosc = ::cos(B13_c0);

			double x = cosc*pc[0]-sinc*pc[1];
			double y = sinc*pc[0]+cosc*pc[1];
			pc[0] = x; pc[1] = y;

			double u = cosc*vr[0]-sinc*vr[1];
			double v = sinc*vr[0]+cosc*vr[1];
			vr[0] = u; vr[1] = v;

			pc[0] += B13_x0; pc[1] += B13_y0; pc[2] += B13_z0;
			pc[3] += B13_c0;
		}

		xc = pc[0]; yc = pc[1]; zc = pc[2]; cc = pc1[3];
		uc = vr[0]; vc = vr[1]; wc = vr[2];
	}
	else {
		xc = B13_x; yc = B13_y; zc = B13_z; cc = B13_c;
		uc = vc = wc = 0;
	}

	double dx = state.x - xc;
	double dy = state.y - yc;
	double dz = state.z - zc;

	double dt = tGet-B13_t;
	B13_dxi += dx*dt;
	B13_dyi += dy*dt;
	B13_dzi += dz*dt;

	B13_t = tGet;

	double velg[3] = {
		uc+B13_kx*dx+B13_kxi*B13_dxi,				//predictive velocity plus position error feedback
		vc+B13_ky*dy+B13_kyi*B13_dyi,
		wc+B13_kz*dz+B13_kzi*B13_dzi
	};


	double vel[3];
	G2B(&state.a, velg, vel);

	A5_u = vel[0];
	A5_v = vel[1];
	A5_w = vel[2];

	A5_c = cc;
	INPI(A5_c);

#if (_DEBUG & DEBUGFLAG_CTL)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[CTL] B13, x %.4g, y %.4g, z %.4g, c %.4g\n", xc, yc, zc, cc);
	printf("[CTL] B13, ug %.4g, vg %.4g, wg %.4g, c %.4g\n", velg[0], velg[1], velg[2], A5_c);
}
#endif

	A5_u = range(A5_u, MIN_STATE_U, MAX_STATE_U);
	A5_v = range(A5_v, MIN_STATE_V, MAX_STATE_V);
	A5_w = range(A5_w, MIN_STATE_W, MAX_STATE_W);
}

void clsCTL::B14()
{

}

void clsCTL::B14CalculateVirtualFollower(double pcl[4], double pcf[4])
{

}
double mod(double x,double y)
{
	while(x>y)
		x = x-y;
	return x;
}
void clsCTL::Outerloop_QuadLion()
{
if (m_bAutoPath) {
		AutoPathGeneration();
//		CreateSmoothPath();
	}
	else {
		// semi-auto control from GCS
		GCSRefGeneration();
	}

	/* RPT outer-loop control law */

	UAVSTATE &state = _state.GetState();

	// calculate dt
	double t = ::GetTime();
	double dt = 0;
	if (B5_t4 < 0) {
		dt = 0;
	} else {
		dt = t - B5_t4;
	}
	B5_t4 = t;

	if (GetIntegratorFlag()) {
		double xerr = state.x - B5_pnr[0];
		m_xerrint += xerr*dt;
		m_xerrint = range(m_xerrint, XERRINT_MIN, XERRINT_MAX);

		double yerr = state.y - B5_pnr[1];
		m_yerrint += yerr*dt;
		m_yerrint = range(m_yerrint, YERRINT_MIN, YERRINT_MAX);

		double zerr = state.z - B5_pnr[2];
		m_zerrint += zerr*dt;
		m_zerrint = range(m_zerrint, ZERRINT_MIN, ZERRINT_MAX);

		double cerr = state.c - B5_pnr[3]; INPI(cerr);
		m_cerrint += cerr*dt;
		m_cerrint = range(m_cerrint, CERRINT_MIN, CERRINT_MAX);
	}
	else {
		m_xerrint = m_yerrint = m_zerrint = m_cerrint = 0;
	}

	double agxy_r[3] = {0};
	agxy_r[0] = state.x * _Fxy[3] + B5_pnr[0]*_Fxy[0] + state.ug *_Fxy[4] + B5_vnr[0]*_Fxy[1] +
			B5_anr[0] + m_xerrint*_Fxy[2];

	agxy_r[1] = state.y * _Fxy[3] + B5_pnr[1]*_Fxy[0] + state.vg *_Fxy[4] + B5_vnr[1]*_Fxy[1] +
			B5_anr[1] + m_yerrint*_Fxy[2];

	agxy_r[2] = state.z * _Fz[3] + B5_pnr[2]*_Fz[0] + state.wg *_Fz[4] + B5_vnr[2]*_Fz[1] +
			/*B5_anr[2]*/ + m_zerrint*_Fz[2];

	double rud_temp = B5_pnr[3] - state.c;
	B_cr = INPI(rud_temp)*_Fc[0] + m_cerrint*_Fc[1];

	B_acxr_ub = agxy_r[0];
	B_acyr_vb = agxy_r[1];
	B_aczr_wb = agxy_r[2];
}

void clsCTL::Innerloop_QuadLion()
{
	UAVSTATE &state = _state.GetState();
	double abc[3] = {0, 0, state.c};

	double abcgr2[3] = {B_acxr_ub, B_acyr_vb, 0};
	double abcbr2[3] = {0};
	G2B(abc, abcgr2, abcbr2);

	m_sig.aileron  = A2_equ.ea + abcbr2[1]/_gravity * 1.25; //
	m_sig.elevator = A2_equ.ee - abcbr2[0]/_gravity;
	m_sig.throttle = A2_equ.et - B_aczr_wb*0.055;
	m_sig.rudder   = A2_equ.er + B_cr;

	m_sig.aileron  = range(m_sig.aileron,  -0.6, 0.6);
	m_sig.elevator = range(m_sig.elevator, -0.6, 0.6);
	m_sig.throttle = range(m_sig.throttle,  0.0, 0.8);
	m_sig.rudder   = range(m_sig.rudder,   -0.6, 0.6);

	double t = ::GetTime();
//	m_sig.laser     = 0.1 + 0.5*sin(0.1*PI*t); // trim + 25 deg to the left
//	m_sig.auxiliary = 25.0*sin(0.1*PI*(t-0.1)); // delayed by 0.1s

//	double payload_x = state.x + cos(state.c)*(-0.12);
//	double payload_y = state.y + sin(state.c)*(-0.12);
//	double error_x   = payload_x-SAFMC_TARGET_X;
//	double error_y   = payload_y-SAFMC_TARGET_Y;

/*	if (_ctl.GetPath2Flag() || (error_x*error_x+error_y*error_y)<0.01 )
	{
		m_sig.triger = -1.0; // payload drop -1.0; load 0.25
		cout << "Drop payload!" << endl;
	}
	else
		m_sig.triger = 0.25;
*/
	static double pre_ail = m_sig.aileron;
	static double pre_ele = m_sig.elevator;
	static double pre_thr = m_sig.throttle;
	static double pre_rud = m_sig.rudder;

	if (GetIntegratorFlag())
	{
//		double temp1 = 0.9995;
//		double temp2 = 0.00025;
//		A2_equ.ea = temp1*A2_equ.ea + temp2*(m_sig.aileron+pre_ail);
//		A2_equ.ee = temp1*A2_equ.ee + temp2*(m_sig.elevator+pre_ele);
//		A2_equ.er = temp1*A2_equ.er + temp2*(m_sig.rudder+pre_rud);
//		A2_equ.et = 0.9683*A2_equ.et + 0.0159*(m_sig.throttle+pre_thr);
		A2_equ.et = 0.99*A2_equ.et + 0.005*(m_sig.throttle+pre_thr);
	}
	else{
		A2_equ.et = _equ_Hover.et;
	}

	pre_ail = m_sig.aileron;
	pre_ele = m_sig.elevator;
	pre_thr = m_sig.throttle;
	pre_rud = m_sig.rudder;

//	cout << " ail trim: " <<  A2_equ.ea << endl;
//	cout << " ele trim: " <<  A2_equ.ee << endl;
//	cout << " thr trim: " <<  A2_equ.et << endl;
//	cout << " rud trim: " <<  A2_equ.er << endl;
}

/*
void clsCTL::Outerloop_QuadLion()
{
	UAVSTATE state = _state.GetState();
	double t2 = GetTime();
	double T_laser = 14;

	if (start == 1) {
		xy_control = 1;
		init_state = state; // Set initial state

		if (state.z < -0.5)
		{
		  _urg.set_start_localisation(1);
		  start = 2;
		}
	}

   else if (start == 2) {
		if ( t2-B5_t0 > tLand+1 )
			xy_control = 1;
		else
			xy_control = 1;

		if (fabs(state.x-targetx)<0.3&&fabs(state.y-targety)<0.3) // drop payload
			B_tr = 1;
   }

				// Make the laser scanner servo rotate
				if (SAMFC)
				{
				  if(t2-B5_t0>90)
					  B_tr = 1;
					//	cout<<"SAMFC"<<endl;
				  if(t2-B5_t0>5&&t2-B5_t0<19)
				  {
					double t_turn = mod(t2-B5_t0-5+1.0/4*T_laser,T_laser);
			  //  	cout<<"t_laser is "<<t_turn;
					if(t_turn<T_laser/2)
						B_lr = ((1)*t_turn +-1*(T_laser/2-t_turn))/(T_laser/2)-0.08;
				   else
						B_lr = ((-1)*(t_turn-T_laser/2) +(1)*(T_laser-t_turn))/(T_laser/2)-0.08;
				  }
				  if(t2-B5_t0>105&&t2-B5_t0<110.25)
				  {
					double t_turn = mod(t2-B5_t0-105+1.0/4*T_laser,T_laser);
				//	cout<<"t_laser is "<<t_turn;
					if(t_turn<T_laser/2)
						B_lr = ((1)*t_turn +-1*(T_laser/2-t_turn))/(T_laser/2)-0.08;
				   else
						B_lr = (-1*(t_turn-T_laser/2) +(1)*(T_laser-t_turn))/(T_laser/2)-0.08;
				  }
			 //     cout<<"B_lr is"<<B_lr<<endl;
				  laser_angle = (B_lr+0.08)*60/180*3.141592654;
			//	  cout<<"laser_angle"<<laser_angle<<endl;
				  _urg.set_laser_angle(laser_angle);
			//      cout<<"laser_angle"<<laser_angle;
				}


	if (m_bAutoPath) {
		AutoPathGeneration();
//		CreateSmoothPath();
	}
	else
		GCSRefGeneration(); // semi-auto control from GCS

//	double _x[7] = {
//		state.x, state.y, state.z,
//		state.ug, state.vg, state.wg,
//		state.c
//	};

	double abc[3] = {state.a, state.b, state.c};

//	cout<< "xycontrol"<<xy_control<<endl;

	double t = ::GetTime();
	double dt = 0;
	if (B5_t4 < 0)
		dt = 0;
	else
		dt = ::GetTime() - B5_t4;
	B5_t4 = t;

	if (GetIntegratorFlag()) {
		double xerr = state.x - B5_pnr[0];
		m_xerrint += xerr*dt;
		m_xerrint = range(m_xerrint, XERRINT_MIN, XERRINT_MAX);

		double yerr = state.y - B5_pnr[1];
		m_yerrint += yerr*dt;
		m_yerrint = range(m_yerrint, YERRINT_MIN, YERRINT_MAX);

		double zerr = state.z - B5_pnr[2];
		m_zerrint += zerr*dt;
		m_zerrint = range(m_zerrint, ZERRINT_MIN, ZERRINT_MAX);

		double cerr = state.c - B5_pnr[3]; INPI(cerr);
		m_cerrint += cerr*dt;
		m_cerrint = range(m_cerrint, CERRINT_MIN, CERRINT_MAX);

//		cout<<"x_error is "<<m_xerrint<<endl;
//		cout<<"y_error is "<<m_xerrint<<endl;
//		cout<<"z_error is "<<m_zerrint<<endl;
//		cout<<"c_error is "<<m_cerrint<<endl;
	}
	else {
		m_xerrint = m_yerrint = m_zerrint = m_cerrint = 0;
	}

	//cout<<"ax is "<<B5_anr[0]<<" "<<" ay is " << B5_anr[1]<<endl;

	double agxy_r[2] = {0};
	agxy_r[0] = state.x * _Fxy[3] + B5_pnr[0]*_Fxy[0] + state.ug *_Fxy[4] + B5_vnr[0]*_Fxy[1] +
			B5_anr[0] + m_xerrint*_Fxy[2];

	agxy_r[1] = state.y * _Fxy[3] + B5_pnr[1]*_Fxy[0] + state.vg *_Fxy[4] + B5_vnr[1]*_Fxy[1] +
			B5_anr[1] + m_yerrint*_Fxy[2];

	double wg_r = state.z*_Fz[2] + B5_pnr[2]*_Fz[0] + B5_vnr[2] + m_zerrint*_Fz[1];

//	double rud = ( B5_pnr[3]*_Fc[0]  + state.c*_Fc[2] + m_cerrint*_Fc[1] ) / dc_rud2r;
	double rud_temp = B5_pnr[3] - state.c;
	double rud = ( INPI(rud_temp)*_Fc[0] + m_cerrint*_Fc[1] ) / dc_rud2r;

	B_acxr_ub = agxy_r[0];
	B_acyr_vb = agxy_r[1];
	B_aczr_wb = wg_r;
	B_cr = rud;
}*/

/*
void clsCTL::Innerloop_QuadLion()
{
	double t2 = ::GetTime();
	UAVSTATE &state = _state.GetState();
	double abc[3] = {state.a, state.b, state.c};
	double abcgr1[3] = {state.ug, state.vg, B_aczr_wb};
	double abcbr1[3] = {0};
	G2B(abc, abcgr1, abcbr1);

	double tTakeOff = 6;

	// acz in NED
	double accb[3] = {state.acx, state.acy, state.acz};
	double accg[3] = {0};
	B2G(abc, accb, accg);
	double abcgr2[3] = {B_acxr_ub, B_acyr_vb, accg[2]};
	double abcbr2[3] = {0};
	G2B(abc, abcgr2, abcbr2);

	double throttle_value = abcbr1[2] / dc_thr2w;

	// During taking off
	if ((t2-B5_t0)<=tTakeOff)
	    m_sig.throttle = -0.2 + (A2_equ.et+0.2)/tTakeOff*(t2-B5_t0) + throttle_value;

	// During path tracking
	else if ((t2-B5_t0)>tTakeOff && (t2-B5_t0)<=tLand )
	{
		// Input trim estimation by averaging
		if ((t2-B5_t0)>(tTakeOff+3) && (t2-B5_t0)<=(tTakeOff+6))
		{
			Ave_equ.ea += m_sig.aileron;
			Ave_equ.ee += m_sig.elevator;
			Ave_equ.er += m_sig.rudder;
			Ave_equ.et += m_sig.throttle;
			equ_count++;
		}

		if ((t2-B5_t0)>(tTakeOff+6) && (t2-B5_t0)<=(tTakeOff+9) && !SetTrim)
		{
			Ave_equ.ea = (Ave_equ.ea / equ_count - A2_equ.ea) / 3;
			Ave_equ.ee = (Ave_equ.ee / equ_count - A2_equ.ee) / 3;
			Ave_equ.er = (Ave_equ.er / equ_count - A2_equ.er) / 3;
			Ave_equ.et = (Ave_equ.et / equ_count - A2_equ.et) / 3; // 3 seconds
			SetTrim = 1;
			cout<<"Trim start updating!"<<endl;
		}

		if ((t2-B5_t0)>(tTakeOff+6) && (t2-B5_t0)<=(tTakeOff+9) && SetTrim) {
			A2_equ.ea += Ave_equ.ea * 0.02;
			A2_equ.ee += Ave_equ.ee * 0.02;
			A2_equ.er += Ave_equ.er * 0.02;
			A2_equ.et += Ave_equ.et * 0.02;
		}

		if ((t2-B5_t0)>(tTakeOff+9) && !GetIntegratorFlag())
		{
			SetIntegratorFlag();
			cout<<"Start integral control!"<<endl;
		}

		m_sig.throttle = A2_equ.et + throttle_value;
	}

	// Landing path
	else if (state.z <= -0.1)
		m_sig.throttle = A2_equ.et - 0.01*(t2-B5_t0-tLand) + throttle_value;

	// Motor totally cut off
	else
	{
		m_sig.throttle = -0.85;
//		cout<<"Motor cut down"<<endl;
	}

	if (xy_control == 0)
	{
		m_sig.elevator = A2_equ.ee;
		m_sig.aileron  = A2_equ.ea;
		m_sig.rudder   = A2_equ.er;
	}

	else
	{
		m_sig.elevator = A2_equ.ee + ( state.u * damping_u + abcbr2[0] ) / ratio_u / dc_ele2tht;
		m_sig.aileron  = A2_equ.ea + ( state.v * damping_v + abcbr2[1] ) / ratio_v / dc_ail2phi;
		m_sig.rudder   = A2_equ.er + B_cr;
	}

	m_sig.elevator 	= range(m_sig.elevator,-0.4,0.4);
	m_sig.aileron 	= range(m_sig.aileron,-0.4,0.4);
	m_sig.rudder 	= range(m_sig.rudder,-0.4,0.4);
}
*/

void clsCTL::SetBehavior(BEHAVIOR *pBeh)
{
	int nBehavior = pBeh->behavior;
	char *para = pBeh->parameter;

#if (_DEBUG & DEBUGFLAG_CTL)
	printf("[CTL] New behavior %d\n", nBehavior);
#endif

//	if (nBehavior == BEHAVIOR_HFLY) _svo.SetLimit(0.02);
//	else _svo.SetLimit(0.05);				//damping for hfly

	switch (pBeh->behavior) {
	case BEHAVIOR_EMERGENCY:
	case BEHAVIOR_EMERGENCYGROUND:
	case BEHAVIOR_ENGINEUP:
	case BEHAVIOR_ENGINEDOWN:
	case BEHAVIOR_TAKEOFF:
	case BEHAVIOR_LAND:
		SetBehavior(nBehavior);
		break;

	case BEHAVIOR_ENGINE:			//engine
		SetBehavior(nBehavior, (double &)para[0]);
		break;

	case BEHAVIOR_HEADTO:				//head to (x,y,z)
		SetBehavior(nBehavior, (double &)para[0], (double &)para[8], (double &)para[16]);
		break;

	case BEHAVIOR_HFLY:				//height keeping fly (u,v,r,h)
	case BEHAVIOR_FLY:				//fly (u,v,w,r)
	case BEHAVIOR_HOLD:				//hold (x,y,z,c)
	case BEHAVIOR_VELTRACK:
		SetBehavior(nBehavior, (double &)para[0], (double &)para[8], (double &)para[16], (double &)para[24]);
		break;

	case BEHAVIOR_HOLDPI:
		SetBehavior(nBehavior, (double &)para[0], (double &)para[8], (double &)para[16], (double &)para[24]);
		break;

	case BEHAVIOR_PATH:				//path tracking
		SetBehavior(nBehavior, (int &)para[0], (int &)para[4]);
		break;

	case BEHAVIOR_PATHA:
		SetBehavior(nBehavior, (double &)para[0], (double &)para[8], (double &)para[16], (double &)para[24], (int &)para[32], (int &)para[36]);
		break;

	case BEHAVIOR_TEST:				//test 1, servo driving
		SetBehavior(nBehavior, (int &)para[0]);
		break;

	case BEHAVIOR_CHIRP:				//chirp signal
		SetBehavior(nBehavior, (int &)para[0], (double &)para[4], (double &)para[12], (double &)para[20], (double &)para[28]);
		break;

	case BEHAVIOR_AIM:
		SetBehavior(nBehavior, (int &)para[0], (int &)para[4], (double &)para[8], (double &)para[16], (double &)para[24]);
		break;

	case BEHAVIOR_SEMIAUTO:
		m_fControl = CONTROLFLAG_B5 | CONTROLFLAG_A2;
		break;
	}
}

void clsCTL::AddBehavior(BEHAVIOR *pBehavior)
{
	if (pBehavior->behavior == BEHAVIOR_AUXILIARYDOWN) {
		AddBehavior(pBehavior->behavior);
	}
	else if (pBehavior->behavior == BEHAVIOR_ENGINE) {
		AddBehavior(pBehavior->behavior, (double &)pBehavior->parameter[0]);
	}
}

void clsCTL::AddBehavior(int nBehavior)
{
	if (nBehavior == BEHAVIOR_AUXILIARYDOWN) {
		m_fControl |= CONTROLFLAG_A8;
		A8_t0 = ::GetTime();
		A8_sig0 = m_sig;
		A8_mode = A8MODE_AUXILIARYDOWN;
	}
}

void clsCTL::AddBehavior(int nBehavior, double para1)
{
	if (nBehavior == BEHAVIOR_ENGINE) {
		m_fControl |= CONTROLFLAG_A9;

		A9_flag = A9FLAG_THROTTLE;
		A9_et = range(para1, 0, 1);
	}
}

void clsCTL::SetBehavior(int nBehavior)
{
	//BEHAVIOR_H1, H2, H3, H6, H13
	UAVSTATE &state = _state.GetState();

	switch (nBehavior) {
	case BEHAVIOR_EMERGENCY:
	case BEHAVIOR_EMERGENCYGROUND:
		if (_HELICOPTER == ID_HELION || _HELICOPTER == ID_SHELION) {
			m_fControl = CONTROLFLAG_A9;
			A9_flag = A9FLAG_ALL;
			A9_ea = _equ_Hover.ea;
			A9_ee = _equ_Hover.ee;
			A9_eu = _equ_Hover.eu;
			A9_er = _equ_Hover.er;
			A9_et = _equ_Hover.et;

			if (nBehavior == BEHAVIOR_EMERGENCYGROUND) {
				A9_eu = AUXILIARY_LOW;				//ground, close engine
				A9_et = THROTTLE_LOW;
			}
		}
		else if (_HELICOPTER == ID_FEILION) {
			m_fControl = CONTROLFLAG_A7;
		}
		break;

	case BEHAVIOR_ENGINEUP:
	case BEHAVIOR_ENGINEDOWN:
//		m_fControl = CONTROLFLAG_A6 | CONTROLFLAG_A8;
		m_fControl = CONTROLFLAG_A8;
		A1A2A3_u = A1A2A3_v = A1A2A3_w = A1A2A3_r = 0;
		A8_t0 = ::GetTime();
		A8_sig0 = m_sig;
		if (nBehavior == BEHAVIOR_ENGINEUP)
			A8_mode = A8MODE_ENGINEUP;
		else
			A8_mode = A8MODE_ENGINEDOWN;

		break;

	case BEHAVIOR_TAKEOFF: {
		double pos0[4] = { state.x, state.y, state.z, state.c };
		double pose[4] = { state.x, state.y, state.z-2, state.c };
//		double pos0[4] = { 0, 0, 0, 0};
//		double pose[4] = { 0, 0, -10, 0 };
		cout<<"clsCTL::SetBehavior--CreateTakeoffPath() at "<<::GetTime()<<endl;
		_pathTmp.CreateTakeoffPath(pos0, pose);

		SetBehavior(BEHAVIOR_PATH, -1, PATHTRACKING_FIXED);

		break; }

	case BEHAVIOR_LAND: {
/*		double poss[4] = { state.x, state.y, state.z, state.c };
		double pos0[4] = { state.x, state.y, 0, state.c };
		cout<<"clsCTL::SetBehavior--CreateLandingPath() at "<<::GetTime()<<endl;
		_pathTmp.CreateLandingPath(poss, pos0);

		SetBehavior(BEHAVIOR_PATH, -1, PATHTRACKING_FIXED);*/

/*		double poss[4] = { 0, 0, state.z, state.c };
		double pos0[4] = { 0, 0, 0, state.c };
		cout<<"clsCTL::SetBehavior--CreateLandingPath() at "<<::GetTime()<<endl;
		_pathTmp.CreateLandingPath(poss, pos0);

		SetBehavior(BEHAVIOR_PATH, -1, PATHTRACKING_ADDON);*/

		m_bLandCmd = TRUE;
		break; }
	}
}

void clsCTL::SetBehavior(int nBehavior, int nTest)
{
	//BEHAVIOR_TEST(test)
	UAVSTATE &state = _state.GetState();

	if (nTest == 1) {				//leader-follower virtual
		m_fControl = CONTROLFLAG_B14 | CONTROLFLAG_A6;

		B14_pPath = &_pathTmp;
		B14_t0 = GetTime();

		double cosc = ::cos(state.c);
		double sinc = ::sin(state.c);

		B14_x0 = state.x + 5*cosc - 5*sinc;
		B14_y0 = state.y + 5*sinc + 5*cosc;
		B14_z0 = state.z;
		B14_c0 = state.c;

		B14_t = B14_t0;
		B14_dxi = B14_dyi = B14_dzi = B14_dci = 0;
	}
	else {
		m_fControl = CONTROLFLAG_A4;
		A4_nTest = nTest;
	}
}

void clsCTL::SetBehavior(int nBehavior, double d1)
{
	m_fControl = CONTROLFLAG_A6;

	if (nBehavior == BEHAVIOR_ENGINE) {
		m_fControl = CONTROLFLAG_A6 | CONTROLFLAG_A9;
		A1A2A3_u = A1A2A3_v = A1A2A3_w = A1A2A3_r = 0;
		A9_flag = A9FLAG_AUXILIARY | A9FLAG_THROTTLE;
		A9_eu = AUXILIARY_LOW;

		A9_ea = 0;
		A9_ee = 0;
		A9_er = 0;
		A9_et = 0;
	}
}

void clsCTL::SetBehavior(int nBehavior, double d1, double d2, double d3)
{
	//BEHAVIOR_H4, H7, H8, H9
	double x, y, z;

	switch (nBehavior) {
	case BEHAVIOR_HEADTO:
		x = d1; y = d2; z = d3;
		m_fControl = CONTROLFLAG_B9 | CONTROLFLAG_A6;

		B9_x = x; B9_y = y; B9_z = z;

		break;
	}
}

void clsCTL::SetBehavior(int nBehavior, double d1, double d2, double d3, double d4)
{
	//BEHAVIOR_H5, H10, H11, FLY, HOLD
	UAVSTATE &state = _state.GetState();

	double u, v, w, x, y, z, r, c;
	switch (nBehavior) {

	case BEHAVIOR_HFLY:
/*		x = d1 > MAXVALUE ? state.x : d1;
		y = d2 > MAXVALUE ? state.y : d2;
		z = d3 > MAXVALUE ? state.z : d3;
		vh = d4 > MAXVALUE ? state.ug : d4;

		HLOCATION hloc = {x, y};
		_pathTmp.CreateHPath(hloc, 1, &state);
		m_fControl = CONTROLFLAG_B5 | CONTROLFLAG_A2;*/
		break;

	case BEHAVIOR_FLY:
		u = d1 > MAXVALUE ? 0 : d1;
		v = d2 > MAXVALUE ? 0 : d2;
		w = d3 > MAXVALUE ? 0 : d3;
		c = d4 > MAXVALUE ? 0 : d4;

		B5_vax[0] = u; B5_vay[0] = v; B5_vaz[0] = w;
		B5_c = state.c;
		B5_vac = c;
		m_bAutoPath = FALSE;
		m_fControl = CONTROLFLAG_OUTERLOOP_QUADLION | CONTROLFLAG_INNERLOOP_QUADLION;
		break;

	case BEHAVIOR_VELTRACK:
		m_fControl = CONTROLFLAG_A2;
		A2_pVel = GetPath(18);
		A2_t0 = ::GetTime();
		break;

	case BEHAVIOR_HOLD:
		x = d1 > MAXVALUE ? state.x : d1;
		y = d2 > MAXVALUE ? state.y : d2;
		z = d3 > MAXVALUE ? state.z : d3;
		c = d4 > MAXVALUE ? state.c : d4;

		x = range(x, MIN_STATE_X, MAX_STATE_X);
		y = range(y, MIN_STATE_Y, MAX_STATE_Y);
		z = range(z, MIN_STATE_Z, MAX_STATE_Z);

		INPI(c);

		if (m_innerloop == INNERLOOP_DECOUPLE) {
			m_fControl = CONTROLFLAG_B13 | CONTROLFLAG_A5;

			B13_x = x; B13_y = y; B13_z = z; B13_c = c;
			B13_pPath = NULL;				//no path tracking, only holding

			B13_t0 = ::GetTime();
			B13_t = B13_t0;
			B13_dxi = B13_dyi = B13_dzi = 0;
		}
		else if (m_innerloop == INNERLOOP_RPT) {
			m_fControl = CONTROLFLAG_B5 | CONTROLFLAG_A2;

			B5_x = x; B5_y = y; B5_z = z; B5_c = c; B5_pPath = B5_pPath2 = NULL;
			B5_t0 = ::GetTime();
			B5_t = B5_t0;
			B5_dxi = B5_dyi = B5_dzi = B5_dci = 0;
			B5_bEnd = FALSE;

			if (_HELICOPTER == ID_GREMLION) {
				B5_outerloopMode = MODE_NAVIGATION;
				B5_bSemi1stFlag = TRUE;
			}

			if (_HELICOPTER == ID_FEILION)
				m_fControl = CONTROLFLAG_B2 | CONTROLFLAG_A1;

			if (_HELICOPTER == ID_QUADLION) {
				m_fControl = CONTROLFLAG_OUTERLOOP_QUADLION | CONTROLFLAG_INNERLOOP_QUADLION;
				ResetIndoorPath();
			}
		}
		else {
			if (m_innerloop == INNERLOOP_GAINSCHEDULE)		// used in fast forward flight
				m_fControl = CONTROLFLAG_B2 | CONTROLFLAG_A1;
			else if (m_innerloop == INNERLOOP_CNF)
				m_fControl = CONTROLFLAG_B2 | CONTROLFLAG_A3;
			else				//lqr, default
				m_fControl = CONTROLFLAG_B2 | CONTROLFLAG_A6;

			B2_x = x; B2_y = y; B2_z = z; B2_c = c; B2_pPath = NULL;
			B2_t0 = ::GetTime();
			B2_t = B2_t0;
			B2_dxi = B2_dyi = B2_dzi = B2_dci = 0;
			B2_bEnd = FALSE;
		}
		break;

	case BEHAVIOR_HOLDPI:
		x = d1 > MAXVALUE ? state.x : d1;
		y = d2 > MAXVALUE ? state.y : d2;
		z = d3 > MAXVALUE ? state.z : d3;
		c = d4 > MAXVALUE ? state.c : d4;

		x = range(x, MIN_STATE_X, MAX_STATE_X);
		y = range(y, MIN_STATE_Y, MAX_STATE_Y);
		z = range(z, MIN_STATE_Z, MAX_STATE_Z);

		INPI(c);

		if (m_innerloop == INNERLOOP_DECOUPLE) {
			m_fControl = CONTROLFLAG_B13 | CONTROLFLAG_A5;

			B13_x = x; B13_y = y; B13_z = z; B13_c = c;
			B13_pPath = NULL;				//no path tracking, only holding

			B13_t0 = ::GetTime();
			B13_t = B13_t0;
			B13_dxi = B13_dyi = B13_dzi = 0;
		}
		else {
			if (m_innerloop == INNERLOOP_GAINSCHEDULE) m_fControl = CONTROLFLAG_B2 | CONTROLFLAG_A1;
			else if (m_innerloop == INNERLOOP_CNF) m_fControl = CONTROLFLAG_B2 | CONTROLFLAG_A3;
			else m_fControl = CONTROLFLAG_B2 | CONTROLFLAG_A6;

			B2_x = x; B2_y = y; B2_z = z; B2_c = c; B2_pPath = NULL;
			B2_t0 = ::GetTime();
			B2_t = B2_t0;
			B2_dxi = B2_dyi = B2_dzi = B2_dci = 0;

			B2_bEnd = FALSE;
		}
		break;
	}
}

void clsCTL::SetBehavior(int nBehavior, int channel, double a, double om1, double om2, double T)
{
	if (nBehavior == BEHAVIOR_CHIRP) {
		m_fControl |= CONTROLFLAG_B6;

		B6_channel = channel;
		B6_a = a; B6_om1 = om1; B6_om2 = om2; B6_T = T;

		B6_bChirp = TRUE;
		B6_vChirp[0] = B6_vChirp[1] = B6_vChirp[2] = B6_vChirp[3] = 0;
		B6_t0 = GetTime();
	}
}

void clsCTL::SetBehavior(int nBehavior, int nPath, int nMode)
{
	//BEHAVIOR_PATH
	UAVSTATE &state = _state.GetState();

	printf("[CTL] Path tracking behavior, path %d\n", nPath);

	if ( _HELICOPTER == ID_HELION /* || _HELICOPTER == ID_SHELION || _HELICOPTER == ID_ GREMLION */ ) {
		B5_pPath = GetPath(nPath); B5_pPath2 = NULL;

		B5_mode = nMode;

		B5_t0 = GetTime();

		B5_x0 = state.x;
		B5_y0 = state.y;
		B5_z0 = state.z;
		B5_c0 = state.c;

		B5_adjustc = PI/2 - state.c;		// adjustment heading angle determined when cmd is issued

		B5_t = B5_t0;
		B5_dxi = B5_dyi = B5_dzi = B5_dci = 0;

		B5_bEnd = FALSE;

		m_fControl = CONTROLFLAG_B5 | CONTROLFLAG_A2;
	}
	else if (_HELICOPTER == ID_FEILION) {

	}
	else if (_HELICOPTER == ID_QUADLION) {
		B5_pPath = GetPath(nPath); B5_pPath2 = NULL;

		B5_mode = nMode;

		B5_t0 = GetTime();
		if(nPath == 3)
        {SAMFC = TRUE;
        cout<<"SAMFC start!"<<endl;
        }
		B5_pnr[0] = B5_x0 = state.x;
		B5_pnr[1] = B5_y0 = state.y;
		B5_pnr[2] = B5_z0 = state.z;
		B5_pnr[3] = B5_c0 = state.c;

		B5_t = B5_t0;
		B5_dxi = B5_dyi = B5_dzi = B5_dci = 0;

		B5_bEnd = FALSE;

		m_fControl = CONTROLFLAG_OUTERLOOP_QUADLION | CONTROLFLAG_INNERLOOP_QUADLION;
	}

}

void clsCTL::SetBehavior(int nBehavior, double x, double y, double z, double c, int nPath1, int nPath2)
{
	//BEHAVIOR_PATHA
	UAVSTATE &state = _state.GetState();

		if (_HELICOPTER == ID_QUADLION) {
			m_fControl = CONTROLFLAG_OUTERLOOP_QUADLION | CONTROLFLAG_INNERLOOP_QUADLION;
		}

		B5_pPath = NULL;
		B5_pPath2 = NULL;
		m_pLoadedTextPath = NULL;
		SetPathSmooth();

		B5_t0 = GetTime();
		B5_wg0 = state.wg;

		if (_ctl.GetTakeOffFlag()){
				printf("[ctl] Taking-off Start!\n");
				B5_t2 = -1;

				B5_pnr[0] = B5_x0 = state.x;
				B5_pnr[1] = B5_y0 = state.y;
				B5_pnr[2] = B5_z0 = state.z;
				B5_pnr[3] = B5_c0 = state.c;
		}
		else if (_ctl.GetTransition1Flag()) {
				printf("[ctl] Transition to path-1 Start!\n");
				B5_t2 = -1;

				m_pLoadedTextPath = GetPath(1);
				if (m_pLoadedTextPath == NULL){
					printf("[ctl-Transition 1] Error with loading 1st text path!\n");
					return;
				}
		}
		else if	(_ctl.GetPath1Flag()){
				m_pLoadedTextPath = GetPath(1);
				if (m_pLoadedTextPath == NULL){
					printf("[ctl-Transition 1] Error with loading 1st text path!\n");
					return;
				}
				printf("[ctl] Path-1 Start!\n");
				B5_t2 = -1;
		}
		else if (_ctl.GetVisionInitializationFlag()){
				printf("[ctl] Vision Initialization Start!\n");
				B5_t2 = -1;
		}
		else if (_ctl.GetVisionGuidanceFlag()){
				printf("[ctl] Vision Guidance Start!\n");
				B5_t2 = -1;
		}
		else if (_ctl.GetTransition2Flag()){
			printf("[ctl] Transition to path-2 Start!\n");
			B5_t2 = -1;
			//Take the current measurements as the initial references
			B5_pnr[0] = state.x;
			B5_pnr[1] = state.y;
			B5_pnr[2] = state.z;
			B5_pnr[3] = state.c;

			B5_vnr[0]=0; B5_vnr[1] = 0; B5_vnr[2] = 0;
			B5_anr[0]=0; B5_anr[1] = 0; B5_anr[2] = 0;

			/*
			m_pLoadedTextPath = GetPath(2);
			if (m_pLoadedTextPath == NULL){
				printf("[ctl-Transition 2] Error with loading 2nd text path!\n");
				return;
			}
			*/
		}
		else if (_ctl.GetPath2Flag()){
			m_pLoadedTextPath = GetPath(2);
			if (m_pLoadedTextPath == NULL){
				printf("[ctl-Transition 1] Error with loading 1st text path!\n");
				return;
			}
			printf("[ctl] Path-2 Start!\n");
			B5_t2 = -1;
		}
		else if (_ctl.GetLandingFlag()){
				printf("[ctl] Landing Start!\n");
				B5_t2 = -1;
		}
		else {
			printf("[ctl] Behavior Exception!\n");
			B5_t2 = -1;
		}

	//	B5_t = B5_t0;
		B5_bEnd = FALSE;
}

void clsCTL::SetBehavior(int nBehavior, int nPath, int nMode, double xt, double yt, double zt)
{
	//BEHAVIOR_AIM
	UAVSTATE &state = _state.GetState();

	m_fControl = CONTROLFLAG_B11 | CONTROLFLAG_A6;				//A7 for camera

	if (nPath == -1) B11_pPath = &_pathTmp;
	else B11_pPath = &_path[nPath-1];
	B11_mode = nMode;

	B11_t0 = GetTime();

	B11_x0 = state.x;
	B11_y0 = state.y;
	B11_z0 = state.z;
//	B11_c0 = state.c;
	B11_c0 = ::atan2(yt-state.y, xt-state.x);						//for demonstration

	double x = xt > MAXVALUE ? 0 : xt;
	double y = yt > MAXVALUE ? 0 : yt;
	double z = zt > MAXVALUE ? 0 : zt;

	B11_xt = range(x, MIN_STATE_X, MAX_STATE_X);
	B11_yt = range(y, MIN_STATE_Y, MAX_STATE_Y);
	B11_zt = range(z, MIN_STATE_Z, MAX_STATE_Z);
}

clsPlan1::clsPlan1()
{
	m_mode = READY;
	m_behavior.behavior = 0;

	m_kx = -0.35;
	m_ky = -0.35;
	m_kz = -0.05;
	m_kc = -0.7;

	m_nPath = 1;				//default path, raceway
}

clsPlan1::~clsPlan1()
{
}

void clsPlan1::Reset() {
	 printf("[CTL] plan1, mode reseted\n");
	 m_mode = READY;
}

int clsPlan1::Run()
{
	EVENT &event = _state.GetEvent();
	m_behavior.behavior = 0;

	switch (m_mode)	{
	case READY:
		m_pos0[0] = m_state.x;
		m_pos0[1] = m_state.y;
		m_pos0[2] = m_state.z;
		m_pos0[3] = m_state.c;

		m_mode = ENGINEUP;
		m_behavior.behavior = BEHAVIOR_ENGINEUP;
		break;

	case ENGINEUP: {
		if (event.code == EVENT_BEHAVIOREND && (int &)event.info[0] == BEHAVIOR_ENGINEUP)
		{
			_state.ClearEvent();

//			m_mode = TAKEOFF;
			m_mode = PATH;
			m_behavior.behavior = BEHAVIOR_PATH;
			(int &)m_behavior.parameter[0] = 3;
			(int &)m_behavior.parameter[4] = PATHTRACKING_FIXED;
		}
		break; }

/*	case TAKEOFF: {
		if (m_pos0[2] - m_state.z > 1.5) {
			m_mode = PATH;
			m_behavior.behavior = BEHAVIOR_PATH;

			double pos0[4] = { m_pos0[0], m_pos0[1], m_state.z, m_pos0[3] };
			_pathTmp.CreateFixedPath(&_path[m_nPath-1], pos0);

			// make the path to be tracked this one
//			(int &)m_behavior.parameter[0] = -1;
//			(int &)m_behavior.parameter[4] = PATHTRACKING_FIXED;
			(int &)m_behavior.parameter[0] = 3;
			(int &)m_behavior.parameter[4] = PATHTRACKING_ADDON;
		}
		break; }
*/
	case PATH: {
		if (event.code == EVENT_BEHAVIOREND && (int &)event.info[0] == BEHAVIOR_PATH) {
//			_state.ClearEvent();

			m_mode = END; //LANDING;
			m_behavior.behavior = BEHAVIOR_LAND;
//			m_behavior.behavior = BEHAVIOR_PATH;
//			_ctl.B2Para(m_kx, m_ky, m_kz, m_kc);
			printf("[CTL][plan1] mode = LANDING\n");


//			double poss[4] = { 0, 0, 0, 0 };
//			double pose[4] = { 0, 0, -m_state.z, 0 };
//			_pathTmp.CreateLandingPath(poss, pose);
//
//			(int &)m_behavior.parameter[0] = -1;
//			(int &)m_behavior.parameter[4] = PATHTRACKING_ADDON /* PATHTRACKING_FIXED */;
		}

		break;}

	case LANDING: {
		if (event.code == EVENT_BEHAVIOREND && (int &)event.info[0] == BEHAVIOR_PATH /*BEHAVIOR_LAND*/) {
			DAQDATA &daq = _daq.GetDAQData();
			if (m_state.z > m_pos0[2]-0.05) {	// for simulation test
//			if (daq.height <= 0.05) {	// for practical test use DAQ data
				_ctl.B2Parai(0, 0, 0, 0);				//clear integration control

				m_mode = ENGINEDOWN;
				m_behavior.behavior = BEHAVIOR_ENGINEDOWN;
			}
		}
		break; }

	case ENGINEDOWN:
		if (event.code == EVENT_BEHAVIOREND && (int &)event.info[0] == BEHAVIOR_ENGINEDOWN) {
			m_mode = END;
//			_ctl.B2Para(-0.35, -0.35, -0.25, -0.7);
		}
		break;

	case END:
		break;
	}

	return m_mode != END;
}

/*int clsPlan1::Run()
{
	EVENT &event = _state.GetEvent();
	m_behavior.behavior = 0;

	switch (m_mode)	{
	case READY:
		m_pos0[0] = m_state.x;
		m_pos0[1] = m_state.y;
		m_pos0[2] = m_state.z;
		m_pos0[3] = m_state.c;

		m_mode = ENGINEUP;
		m_behavior.behavior = BEHAVIOR_ENGINEUP;
		break;

	case ENGINEUP: {
		if (event.code == EVENT_BEHAVIOREND && (int &)event.info[0] == BEHAVIOR_ENGINEUP)
		{
			_state.ClearEvent();

			m_mode = TAKEOFF;
			m_behavior.behavior = BEHAVIOR_TAKEOFF;
			_ctl.B2Para(m_kx, m_ky, m_kz, m_kc);
		}
		break; }

	case TAKEOFF: {
		if (m_pos0[2] - m_state.z > 15) {
			_ctl.B2Para(-0.35, -0.35, -0.25, -0.7);				//resume regular parameters

			m_mode = PATH;
			m_behavior.behavior = BEHAVIOR_PATH;

			double pos0[4] = { m_pos0[0], m_pos0[1], m_state.z, m_pos0[3] };
			_pathTmp.CreateFixedPath(&_path[m_nPath-1], pos0);

			// make the path to be tracked this one
			(int &)m_behavior.parameter[0] = -1;
			(int &)m_behavior.parameter[4] = PATHTRACKING_FIXED;
			(int &)m_behavior.parameter[0] = 1;
			(int &)m_behavior.parameter[4] = PATHTRACKING_ADDON;
		}
		break; }

	case PATH: {
		if (event.code == EVENT_BEHAVIOREND && (int &)event.info[0] == BEHAVIOR_PATH) {
			_state.ClearEvent();

			m_mode = LANDING;
			m_behavior.behavior = BEHAVIOR_LAND;
			_ctl.B2Para(m_kx, m_ky, m_kz, m_kc);
			printf("[CTL][plan1] mode = LANDING\n");

			double dx = m_state.x - m_pos0[1];
			double dy = m_state.y - m_pos0[2];
			double distance = ::sqrt(dx*dx+dy*dy);

			double poss[4] = { m_state.x, m_state.y, m_state.z, m_state.c };
			double pose[4];
			double angle = ::atan2(distance, ::fabs(m_state.z-m_pos0[2]));
			if (angle <= PI/4) {
				pose[0] = m_pos0[0]; pose[1] = m_pos0[1];
				pose[2] = m_pos0[2]; pose[3] = m_pos0[3];
			}
			else {
				pose[0] = m_state.x; pose[1] = m_state.y;
				pose[2] = m_pos0[2]; pose[3] = m_state.c;
			}

			_pathTmp.CreateLandingPath(poss, pose);

			(int &)m_behavior.parameter[0] = -1;
			(int &)m_behavior.parameter[4] = PATHTRACKING_FIXED;

			_ctl.B2Para(m_kx, m_ky, m_kz, m_kc);
			_ctl.B2Parai(0, 0, -0.003, 0);				//integration control to overcome ground effect
		}

		break;}

	case LANDING: {
		DAQDATA &daq = _daq.GetDAQData();
//		if (m_state.z > m_pos0[2]-0.05) {
		if (daq.height <= 0.05) {
			_ctl.B2Parai(0, 0, 0, 0);				//clear integration control

			m_mode = ENGINEDOWN;
			m_behavior.behavior = BEHAVIOR_ENGINEDOWN;
		}
		break; }

	case ENGINEDOWN:
		if (event.code == EVENT_BEHAVIOREND && (int &)event.info[0] == BEHAVIOR_ENGINEDOWN) {
			m_mode = END;
			_ctl.B2Para(-0.35, -0.35, -0.25, -0.7);				//resume
		}
		break;

	case END:
		break;
	}

	return m_mode != END;
}*/

clsTmpPath::clsTmpPath()
{
}

clsTmpPath::~clsTmpPath()
{
}

void clsTmpPath::CreatePath(LINEPATH *pPath)
{
	double durance;
	if (pPath->T > 0) durance = pPath->T;
	else {
		//_ASSERT(pPath->v != 0);

		double distance = clsMetric::Distance(pPath->ps, pPath->pe);
		durance = distance / pPath->v;
	}

	//_ASSERT(pPath->dt > 0);
	int m = (int)(durance/pPath->dt+1);
	if (pPath->dt < 0) m = 1;

	if (m < 1) m = 1;
	else if (m > MAX_TMPPATHSIZE) m = MAX_TMPPATHSIZE;

	m_path.Reset(m+1, 5, (double *)m_data, TRUE);

	double dc = pPath->pe[3] - pPath->ps[3];
	INPI(dc);

	for (int i=0; i<=m; i++) {
		double r = (double)i/m;
		m_path[i][0] = durance*r;
		m_path[i][1] = pPath->ps[0]*(1-r)+pPath->pe[0]*r;
		m_path[i][2] = pPath->ps[1]*(1-r)+pPath->pe[1]*r;
		m_path[i][3] = pPath->ps[2]*(1-r)+pPath->pe[2]*r;

		double c = pPath->ps[3] + dc*r;
		INPI(c);
		m_path[i][4] = c;
	}
}

void clsTmpPath::CreateFixedPath(clsPath *pPath, double pos0[4])
{
	clsMatrix &mtrx = pPath->GetMatrix();
	int m = mtrx.GetM();
//	int n = mtrx.GetN();

//	_ASSERT(m<=MAX_TMPPATHSIZE && n == 5);
	m_path.Reset(m, 5, (double *)m_data, TRUE);

	double sinc = ::sin(pos0[3]); double cosc = ::cos(pos0[3]);

	for (int i=0; i<=m-1; i++) {
		m_path[i][0] = mtrx[i][0];
		m_path[i][1] = mtrx[i][1]*cosc-mtrx[i][2]*sinc+pos0[0];
		m_path[i][2] = mtrx[i][1]*sinc+mtrx[i][2]*cosc+pos0[1];
		m_path[i][3] = mtrx[i][3]+pos0[2];
		m_path[i][4] = mtrx[i][4]+pos0[3];
		INPI(m_path[i][4]);
	}
}

void clsTmpPath::CreateHPath(double hpos[3], int nPath, UAVSTATE *cur, double vel_2d) {
	m_path.Reset(8, 11, (double *)m_data, TRUE);

	double t = 0;

	/// construct 1st matrix point, current location and heading
	m_path[0][0] = 0;
	m_path[0][1] = cur->x;
	m_path[0][2] = cur->y;
	m_path[0][3] = cur->z;
	m_path[0][4] = cur->c;

	/// construct 2nd matrix point, turn heading to c(rad)
	double abc[3] = {0 /*cur->a*/, 0 /*cur->b*/, cur->c};
	double npos[3] = {0};
	G2N(abc, hpos, npos);

	double dx = npos[0];
	double dy = npos[1];
	if ( (hpos[0]<0.1) && (hpos[1]<0.1) ) {
		m_path[1][0] = 0;
		m_path[1][1] = cur->x;
		m_path[1][2] = cur->y;
		m_path[1][3] = cur->z;
		m_path[1][4] = cur->c;
	}
	else {
		double dc = atan2(dy, dx) - cur->c; INPI(dc);
		double dcTmp = ::fabs(dc);
		t += dcTmp/0.2 + 1;
		m_path[1][0] = t;
		m_path[1][1] = cur->x;
		m_path[1][2] = cur->y;
		m_path[1][3] = cur->z;
		m_path[1][4] = atan2(dy, dx);
	}


	// construct 3rd point, hover 5s
	t += 5;
	m_path[2][0] = t;
	m_path[2][1] = m_path[1][1];
	m_path[2][2] = m_path[1][2];
	m_path[2][3] = m_path[1][3];
	m_path[2][4] = m_path[1][4];

	// construct 4th point, height control
	t += ::fabs(npos[2])/0.5;	// height velocity 0.5m/s
	m_path[3][0] = t;
	m_path[3][1] = m_path[2][1];
	m_path[3][2] = m_path[2][2];
	m_path[3][3] = m_path[2][3] + npos[2];
	m_path[3][4] = m_path[2][4];

	// construct 5th point, hover 5s
	t += 5;
	m_path[4][0] = t;
	m_path[4][1] = m_path[3][1];
	m_path[4][2] = m_path[3][2];
	m_path[4][3] = m_path[3][3];
	m_path[4][4] = m_path[3][4];

	// construct 6th matrix point, go to position(x,y,z)
	if (vel_2d < 0.1) {
		m_path[5][0] = t;
		m_path[5][1] = m_path[4][1];
		m_path[5][2] = m_path[4][2];
		m_path[5][3] = m_path[4][3];
		m_path[5][4] = m_path[4][4];
	}
	else {
		//set x, y, z, c
		double d = sqrt(dx*dx + dy*dy);
		t += d/vel_2d;				//	vel_2d m/s

		m_path[5][0] = t;
		m_path[5][1] = m_path[4][1] + dx;
		m_path[5][2] = m_path[4][2] + dy;
		m_path[5][3] = m_path[4][3];
		m_path[5][4] = m_path[4][4];
	}

	// 7th point, hover 5s
	t += 5;
	m_path[6][0] = t;
	m_path[6][1] = m_path[5][1];
	m_path[6][2] = m_path[5][2];
	m_path[6][3] = m_path[5][3];
	m_path[6][4] = m_path[5][4];

	// 8th point, descend to the original height
	t += ::fabs(npos[2])/0.5;
	m_path[7][0] = t;
	m_path[7][1] = m_path[6][1];
	m_path[7][2] = m_path[6][2];
	m_path[7][3] = m_path[6][3] - npos[2];
	m_path[7][4] = m_path[6][4];


	for (int i=0; i<8; i++) {
		printf("path: t %f, x %f, y %f, z %f, c %f \n",
				m_path[i][0], m_path[i][1], m_path[i][2], m_path[i][3], m_path[i][4]);
	}
}
/*
void clsTmpPath::CreatePath(LOCATION *loc, UAVSTATE *from, LOCATION *loc0) {
	m_path.Reset(5, 11, (double *)m_data, TRUE);

	double t = 0;
	m_path[0][0] = t;
	m_path[0][1] = from->x;
	m_path[0][2] = from->y;
	m_path[0][3] = from->z;
	m_path[0][4] = from->c;

	t += 45;
	m_path[1][0] = t;
	m_path[1][1] = m_path[0][1];
	m_path[1][2] = m_path[0][2];
	m_path[1][3] = m_path[0][3] - 60;
	m_path[1][4] = m_path[0][4];

	double x1 = (loc->latitude - loc0->latitude) * _radius;
	double y1 = (loc->longitude - loc0->longitude) * _radius * cos(loc0->latitude);
	double dx = x1 - from->x;
	double dy = y1 - from->y;

	double c1 = atan2(dy, dx);
	double dc = c1 - from->c;
	INPI(dc);
	t += fabs(dc)/0.2;

	m_path[2][0] = t;
	m_path[2][1] = m_path[1][1];
	m_path[2][2] = m_path[1][2];
	m_path[2][3] = m_path[1][3];
	m_path[2][4] = c1;

	dx = x1 - m_path[2][1];
	dy = y1 - m_path[2][2];
	double dr = sqrt(dx*dx+dy*dy);
	t += dr/5;

	m_path[3][0] = t;
	m_path[3][1] = x1;
	m_path[3][2] = y1;
	m_path[3][3] = m_path[2][3];
	m_path[3][4] = m_path[2][4];

	t += 200;
	m_path[4][0] = t;
	m_path[4][1] = x1;
	m_path[4][2] = y1;
	m_path[4][3] = m_path[3][3]+200;
	m_path[4][4] = m_path[3][4];

	for (int i=0; i<5; i++) {
		printf("path: t %f, x %f, y %f, z %f, c %f \n",
				m_path[i][0], m_path[i][1], m_path[i][2], m_path[i][3], m_path[i][4]);
	}
}
*/

void clsTmpPath::CreatePath1(LOCATION *loc, UAVSTATE *from, LOCATION *loc0) {
	m_path.Reset(12, 11, (double *)m_data, TRUE);

	int n = 0;

	double t = 0;
	m_path[n][0] = t;
	m_path[n][1] = from->x;
	m_path[n][2] = from->y;
	m_path[n][3] = from->z;
	m_path[n++][4] = from->c;

	// heading track
	double x1 = (loc->latitude - loc0->latitude) * _radius;
	double y1 = (loc->longitude - loc0->longitude) * _radius * cos(loc0->latitude);
	double dx = x1 - from->x;
	double dy = y1 - from->y;

	double c1 = atan2(dy, dx);
	double dc = c1 - from->c;
	INPI(dc);
	t += fabs(dc)/0.2;

	m_path[n][0] = t;
	m_path[n][1] = m_path[n-1][1];
	m_path[n][2] = m_path[n-1][2];
	m_path[n][3] = m_path[n-1][3];
	m_path[n][4] = c1;
	n++;

	/// 2d track
	dx = x1 - m_path[n][1];
	dy = y1 - m_path[n][2];
	double dr = sqrt(dx*dx+dy*dy);

	t += 1;
	m_path[n][0] = t;
	m_path[n][1] = m_path[n-1][1] + 0.5*cos(c1);
	m_path[n][2] = m_path[n-1][2] + 0.5*sin(c1);
	m_path[n][3] = m_path[n-1][3];
	m_path[n][4] = c1;
	n++;

	t += 1;
	m_path[n][0] = t;
	m_path[n][1] = m_path[n-1][1] + 1*cos(c1);
	m_path[n][2] = m_path[n-1][2] + 1*sin(c1);
	m_path[n][3] = m_path[n-1][3];
	m_path[n][4] = c1;
	n++;

	t += 1;
	m_path[n][0] = t;
	m_path[n][1] = m_path[n-1][1] + 1.5*cos(c1);
	m_path[n][2] = m_path[n-1][2] + 1.5*sin(c1);
	m_path[n][3] = m_path[n-1][3];
	m_path[n][4] = c1;
	n++;

	t += (dr-6)/2;
	m_path[n][0] = t;
	m_path[n][1] = x1-3*cos(c1);
	m_path[n][2] = y1-3*sin(c1);
	m_path[n][3] = m_path[n-1][3];
	m_path[n][4] = m_path[n-1][4];
	n++;

	t += 1;
	m_path[n][0] = t;
	m_path[n][1] = x1 - 1.5*cos(c1);
	m_path[n][2] = y1 - 1.5*sin(c1);
	m_path[n][3] = m_path[n-1][3];
	m_path[n][4] = c1;
	n++;

	t += 1;
	m_path[n][0] = t;
	m_path[n][1] = x1 - 0.5*cos(c1);
	m_path[n][2] = y1 - 0.5*sin(c1);
	m_path[n][3] = m_path[n-1][3];
	m_path[n][4] = c1;
	n++;

	t += 1;
	m_path[n][0] = t;
	m_path[n][1] = x1;
	m_path[n][2] = y1;
	m_path[n][3] = m_path[n-1][3];
	m_path[n][4] = c1;
	n++;

/// hover 10s before descending
	t += 10;
	m_path[n][0] = t;
	m_path[n][1] = m_path[n-1][1];
	m_path[n][2] = m_path[n-1][2];
	m_path[n][3] = m_path[n-1][3];
	m_path[n][4] = m_path[n-1][4];
	n++;

	/// descend
	double z1 = from->z - 5;
	if (m_path[n-1][3] < z1) {
		t += (z1 - m_path[n-1][3])/0.5;
		m_path[n][0] = t;
		m_path[n][1] = m_path[n-1][1];
		m_path[n][2] = m_path[n-1][2];
		m_path[n][3] = z1;
		m_path[n][4] = m_path[n-1][4];
		n++;
	}
	else {
		z1 = m_path[n-1][3];
	}

	double d = 100;
	t += (d-z1)/0.2;
	m_path[n][0] = t;
	m_path[n][1] = m_path[n-1][1];
	m_path[n][2] = m_path[n-1][2];
	m_path[n][3] = d;
	m_path[n][4] = m_path[n-1][4];
	n++;

	if (n < 12) {
		t += 100;
		m_path[n][0] = t;
		m_path[n][1] = m_path[n-1][1];
		m_path[n][2] = m_path[n-1][2];
		m_path[n][3] = m_path[n-1][3];
		m_path[n][4] = m_path[n-1][4];
		n++;
	}

	for (int i=0; i<n; i++) {
		printf("path: index %d, t %f, x %f, y %f, z %f, c %f \n",
				i, m_path[i][0], m_path[i][1], m_path[i][2], m_path[i][3], m_path[i][4]);
	}
}

void clsTmpPath::CreatePath2(LOCATION *loc, UAVSTATE *from, LOCATION *loc0) {
	m_path.Reset(3, 11, (double *)m_data, TRUE);

	int n = 0;

	double t = 0;
	m_path[n][0] = t;
	m_path[n][1] = from->x;
	m_path[n][2] = from->y;
	m_path[n][3] = from->z;
	m_path[n++][4] = from->c;

	double h = 5;
	t += h/1;
	m_path[n][0] = t;
	m_path[n][1] = m_path[n-1][1];
	m_path[n][2] = m_path[n-1][2];
	m_path[n][3] = m_path[n-1][3] - h;
	m_path[n][4] = m_path[n-1][4];
	n++;

//	hover 10s
	t += 10;
	m_path[n][0] = t;
	m_path[n][1] = m_path[n-1][1];
	m_path[n][2] = m_path[n-1][2];
	m_path[n][3] = m_path[n-1][3];
	m_path[n][4] = m_path[n-1][4];
	n++;

	for (int i=0; i<n; i++) {
		printf("path: index %d, t %f, x %f, y %f, z %f, c %f \n",
				i, m_path[i][0], m_path[i][1], m_path[i][2], m_path[i][3], m_path[i][4]);
	}
}

void clsTmpPath::CreatePath(LOCATION *loc, UAVSTATE *from, LOCATION *loc0) {
	m_path.Reset(45, 11, (double *)m_data, TRUE);

	int n = 0;

	double t = 0;
	m_path[n][0] = t;
	m_path[n][1] = from->x;
	m_path[n][2] = from->y;
	m_path[n][3] = from->z;
	m_path[n++][4] = from->c;

	double h = 50;
	t += h/1;
	m_path[n][0] = t;
	m_path[n][1] = m_path[n-1][1];
	m_path[n][2] = m_path[n-1][2];
	m_path[n][3] = m_path[n-1][3] - h;
	m_path[n][4] = m_path[n-1][4];
	n++;

//	hover 10s
	t += 10;
	m_path[n][0] = t;
	m_path[n][1] = m_path[n-1][1];
	m_path[n][2] = m_path[n-1][2];
	m_path[n][3] = m_path[n-1][3];
	m_path[n][4] = m_path[n-1][4];
	n++;

	// heading track
	double x1 = (loc->latitude - loc0->latitude) * _radius;
	double y1 = (loc->longitude - loc0->longitude) * _radius * cos(loc0->latitude);
	double dx = x1 - from->x;
	double dy = y1 - from->y;

	double c1 = atan2(dy, dx);
	double dc = c1 - from->c;
	INPI(dc);
	t += fabs(dc)/0.2;

	m_path[n][0] = t;
	m_path[n][1] = m_path[n-1][1];
	m_path[n][2] = m_path[n-1][2];
	m_path[n][3] = m_path[n-1][3];
	m_path[n][4] = c1;
	n++;

	/// 2d track
	dx = x1 - from->x;
	dy = y1 - from->y;
	double dr = sqrt(dx*dx+dy*dy);

	t += 1;
	m_path[n][0] = t;
	m_path[n][1] = m_path[n-1][1] + 0.5*cos(c1);
	m_path[n][2] = m_path[n-1][2] + 0.5*sin(c1);
	m_path[n][3] = m_path[n-1][3];
	m_path[n][4] = c1;
	n++;

	t += 1;
	m_path[n][0] = t;
	m_path[n][1] = m_path[n-1][1] + 1*cos(c1);
	m_path[n][2] = m_path[n-1][2] + 1*sin(c1);
	m_path[n][3] = m_path[n-1][3];
	m_path[n][4] = c1;
	n++;

	t += 1;
	m_path[n][0] = t;
	m_path[n][1] = m_path[n-1][1] + 1.5*cos(c1);
	m_path[n][2] = m_path[n-1][2] + 1.5*sin(c1);
	m_path[n][3] = m_path[n-1][3];
	m_path[n][4] = c1;
	n++;

	t += 1;
	m_path[n][0] = t;
	m_path[n][1] = m_path[n-1][1] + 2*cos(c1);
	m_path[n][2] = m_path[n-1][2] + 2*sin(c1);
	m_path[n][3] = m_path[n-1][3];
	m_path[n][4] = c1;
	n++;

	t += 1;
	m_path[n][0] = t;
	m_path[n][1] = m_path[n-1][1] + 2.5*cos(c1);
	m_path[n][2] = m_path[n-1][2] + 2.5*sin(c1);
	m_path[n][3] = m_path[n-1][3];
	m_path[n][4] = c1;
	n++;

	t += 1;
	m_path[n][0] = t;
	m_path[n][1] = m_path[n-1][1] + 3*cos(c1);
	m_path[n][2] = m_path[n-1][2] + 3*sin(c1);
	m_path[n][3] = m_path[n-1][3];
	m_path[n][4] = c1;
	n++;

	t += 1;
	m_path[n][0] = t;
	m_path[n][1] = m_path[n-1][1] + 3.5*cos(c1);
	m_path[n][2] = m_path[n-1][2] + 3.5*sin(c1);
	m_path[n][3] = m_path[n-1][3];
	m_path[n][4] = c1;
	n++;

	t += 1;
	m_path[n][0] = t;
	m_path[n][1] = m_path[n-1][1] + 4*cos(c1);
	m_path[n][2] = m_path[n-1][2] + 4*sin(c1);
	m_path[n][3] = m_path[n-1][3];
	m_path[n][4] = c1;
	n++;

	double a = 0;
	for (int i = 29; i>0; i--) {
		a += i*0.15;
	}

	t += (dr-a-18)/4.5;
	m_path[n][0] = t;
	m_path[n][1] = x1-a*cos(c1);
	m_path[n][2] = y1-a*sin(c1);
	m_path[n][3] = m_path[n-1][3];
	m_path[n][4] = m_path[n-1][4];
	n++;

	for (int i=29; i>0; i--) {
		double v = 0.15*i;
		t += 1;

		m_path[n][0] = t;
		m_path[n][1] = m_path[n-1][1]+v*cos(c1);
		m_path[n][2] = m_path[n-1][2]+v*sin(c1);
		m_path[n][3] = m_path[n-1][3];
		m_path[n][4] = m_path[n-1][4];
		n++;
	}

	/// hover 10s before descending
	t += 10;
	m_path[n][0] = t;
	m_path[n][1] = m_path[n-1][1];
	m_path[n][2] = m_path[n-1][2];
	m_path[n][3] = m_path[n-1][3];
	m_path[n][4] = m_path[n-1][4];
	n++;

	/// descend
	double z1 = from->z - 5;
	t += (z1 - m_path[n-1][3])/0.5;
	m_path[n][0] = t;
	m_path[n][1] = m_path[n-1][1];
	m_path[n][2] = m_path[n-1][2];
	m_path[n][3] = z1;
	m_path[n][4] = m_path[n-1][4];
	n++;

	double d = 100;
	t += (d-z1)/0.2;
	m_path[n][0] = t;
	m_path[n][1] = m_path[n-1][1];
	m_path[n][2] = m_path[n-1][2];
	m_path[n][3] = d;
	m_path[n][4] = m_path[n-1][4];
	n++;

	for (int i=0; i<n; i++) {
		printf("path: index %d, t %f, x %f, y %f, z %f, c %f \n",
				i, m_path[i][0], m_path[i][1], m_path[i][2], m_path[i][3], m_path[i][4]);
	}
}


void clsTmpPath::CreateSmoothPath(SMOOTHPATH *smoothPath, UAVSTATE *from, LOCATION *pPos0,
		double outerRefPos[4], double outerRefVel[3], double outerRefAcc[3])
{
	int nPoint = smoothPath->nPoints;
	int nCurPoint = smoothPath->curPoint;
	char *pPath = smoothPath->waypoints;

	double tStart = _ctl.GetPathStartTime();
	double tElapse = ::GetTime() - tStart;

	for (int i=nCurPoint; i<nPoint-1; i++) {
		double dt = 0;
		if (_ctl.B5_t1 < 0) {
			outerRefAcc[0] = outerRefAcc[1] = outerRefAcc[2] = 0;
			outerRefVel[0] = outerRefVel[1] = outerRefVel[2] = 0;
			outerRefPos[0] = _ctl.B5_x0; outerRefPos[1] = _ctl.B5_y0; outerRefPos[2] = _ctl.B5_z0; outerRefPos[3] = _ctl.B5_c0;
			dt = 0;
			_ctl.B5_t1 = ::GetTime();
			return;
		}
		else {
			dt = ::GetTime() - _ctl.B5_t1;
			dt = dt>1 ? 0 : dt;
		}
		_ctl.B5_t1 = ::GetTime();

		/* put condition evaluation here
		 * such as cruise speed, acceleration, yaw turning rate
		 */
//		if ( fabs(acc)< 0.01 )
//			return;

		double lat = *(double *)((char *)pPath + (i+1)*40);
		double lon = *(double *)((char *)pPath + (i+1)*40 + 8);
		double alt = *(double *)((char *)pPath + (i+1)*40 + 16);
		double vCruise = *(double *)((char *)pPath + (i+1)*40 + 24);
		double acc = *(double *)((char *)pPath + (i+1)*40 + 32);
		double tRamp = vCruise / acc;

		double x = (lat - pPos0->latitude) * _radius;
		double y = (lon - pPos0->longitude) * _radius * cos(pPos0->latitude);
		double dx = x - _ctl.B5_x0;
		double dy = y - _ctl.B5_y0;
		double d = sqrt(dx*dx+dy*dy);
		double tCruise = d/vCruise - tRamp;

		double c = atan2(dy, dx); INPI(c);
		double dc = c - _ctl.B5_c0; INPI(dc);
		if (tCruise < 0)
			return;

		// construct ref in each loop

		if (tElapse < tRamp) {
			outerRefAcc[0] = acc * cos(c);
			outerRefAcc[1] = acc * sin(c);
			outerRefAcc[2] = 0;
			outerRefPos[3] += dc/tRamp * dt; INPI(outerRefPos[3]);
//			printf("In tRamp c: %f \n", outerRefPos[3]);
		}
		else if (tElapse >= tRamp && (tElapse <= tCruise + tRamp)) {
			outerRefAcc[0] = outerRefAcc[1] = outerRefAcc[2] = 0;
		}
		else if ( (tElapse > (tRamp + tCruise)) && (tElapse < (tRamp*2 + tCruise)) ) {
			outerRefAcc[0] = -acc * cos(c);
			outerRefAcc[1] = -acc * sin(c);
			outerRefAcc[2] = 0;
		}
		else if (tElapse >= (tRamp*2 + tCruise)) {
			outerRefAcc[0] = outerRefAcc[1] = outerRefAcc[2] = 0;
			outerRefVel[0] = outerRefVel[1] = outerRefVel[2] = 0;




			// restart for the next way point tracking
			_ctl.B5_t1 = -1;
			_ctl.B5_x0 = from->x; _ctl.B5_y0 = from->y; /*_ctl.B5_z0 = from->z;*/ _ctl.B5_c0 = from->c;
			_ctl.B5_t0 = ::GetTime();
			tStart += tElapse;
//			continue;
			smoothPath->curPoint++;
		}

		outerRefVel[0] += outerRefAcc[0]*dt;
		outerRefVel[1] += outerRefAcc[1]*dt;
		outerRefVel[2] = 0;

		outerRefPos[0] += outerRefVel[0]*dt;
		outerRefPos[1] += outerRefVel[1]*dt;
		outerRefPos[2] = _ctl.B5_z0;

	}


}

void clsTmpPath::CreatePath(LOCATION *pPath, int nPath, UAVSTATE *from, LOCATION *pPos0)
{
	m_path.Reset(nPath*6 + 1, 11, (double *)m_data, TRUE);
	double t = 0;
	LOCATION cur = {0};
	int iPath = 0;
	for (int i = 0; i<nPath; i++) {
		if (iPath == 0) {
			cur.latitude = from->latitude;
			cur.longitude = from->longitude;
			cur.altitude = from->altitude;

			m_path[iPath][0] = 0;
			m_path[iPath][1] = from->x;
			m_path[iPath][2] = from->y;
			m_path[iPath][3] = from->z;
			m_path[iPath][4] = from->c;
			iPath++;
		}
		else {
			cur.latitude = *(double *)((char *)pPath + (i-1)*24);
			cur.longitude = *(double *)((char *)pPath + (i-1)*24 + 8);
			cur.altitude = *(double *)((char *)pPath + (i-1)*24 + 16);
		}
		printf("Cur location: lat %.8f, lon %.8f, lat %f\n", cur.latitude, cur.longitude, cur.altitude);

		double lat = *(double *)((char *)pPath + i*24);
		double lon = *(double *)((char *)pPath + i*24 + 8);
		double alt = *(double *)((char *)pPath + i*24 + 16);
		double x = (lat - pPos0->latitude) * _radius;
		double y = (lon - pPos0->longitude) * _radius * cos(pPos0->latitude);
		double dx = (lat - cur.latitude)*_radius;
		double dy = (lon - cur.longitude)*_radius*cos(lat);
		double d = sqrt(dx*dx+dy*dy);
		double dh = alt - cur.altitude;

		double c = 0;
		if (fabs(dx)<1 || fabs(dy)<1) {
			c = m_path[iPath-1][4];
		} else {
			c = atan2(dy, dx); INPI(c);
		}

		// point 1 - hover 5s from previous point
		t += 5;
		m_path[iPath][0] = t;
		m_path[iPath][1] = m_path[iPath-1][1];
		m_path[iPath][2] = m_path[iPath-1][2];
		m_path[iPath][3] = m_path[iPath-1][3];
		m_path[iPath][4] = m_path[iPath-1][4];
		iPath++;

		// point 2, head turn
		double dc = c - m_path[iPath][4];
		INPI(dc);
		t += ::fabs(dc)/0.2;
		m_path[iPath][0] = t;
		m_path[iPath][1] = m_path[iPath-1][1];
		m_path[iPath][2] = m_path[iPath-1][2];
		m_path[iPath][3] = m_path[iPath-1][3];
		m_path[iPath][4] = c;
		iPath++;

		// point 3, hover 5s after head turn
		t += 3;
		m_path[iPath][0] = t;
		m_path[iPath][1] = m_path[iPath-1][1];
		m_path[iPath][2] = m_path[iPath-1][2];
		m_path[iPath][3] = m_path[iPath-1][3];
		m_path[iPath][4] = m_path[iPath-1][4];
		iPath++;

		// point 4, heave control
		t += fabs(dh);
		m_path[iPath][0] = t;
		m_path[iPath][1] = m_path[iPath-1][1];
		m_path[iPath][2] = m_path[iPath-1][2];
		m_path[iPath][3] = m_path[iPath-1][3]; //alt;
		m_path[iPath][4] = m_path[iPath-1][4];
		iPath++;

		// point 5, hover 5s after heave control
		t += 3;
		m_path[iPath][0] = t;
		m_path[iPath][1] = m_path[iPath-1][1];
		m_path[iPath][2] = m_path[iPath-1][2];
		m_path[iPath][3] = m_path[iPath-1][3];
		m_path[iPath][4] = m_path[iPath-1][4];
		iPath++;

		// point 6, 2D movement
		t += d/3;	// 3m/s cruise velocity
		m_path[iPath][0] = t;
		m_path[iPath][1] = x;
		m_path[iPath][2] = y;
		m_path[iPath][3] = m_path[iPath-1][3];
		m_path[iPath][4] = m_path[iPath-1][4];
		iPath++;

	}

	for (int i = 0; i<m_path.GetM(); i++) {
		printf("[CTL] GPS way points based path point %d: t %f, x %f, y %f, z %f, c %f\n", i, m_path[i][0], \
				m_path[i][1], m_path[i][2], m_path[i][3], m_path[i][4]);
	}

}
/*
{
	m_path.Reset(nPath+1, 11, (double *)m_data, TRUE);

	double t = 0;
	LOCATION cur = { from->latitude, from->longitude };
	int iPath = 0;

	for (int i=0; i<=nPath; i++) {
		if (i == 0) {
			m_path[iPath][0] = t = 0;
			m_path[iPath][1] = from->x;
			m_path[iPath][2] = from->y;
			m_path[iPath][3] = from->z;
			m_path[iPath++][4] = from->c;

			m_path[iPath][0] = t = 0.1;
			m_path[iPath][1] = from->x;
			m_path[iPath][2] = from->y;
			m_path[iPath][3] = from->z;
			m_path[iPath++][4] = from->c;

			continue;
		}

		LOCATION pos;
		COPYDOUBLE(&pos.latitude, (char *)pPath+24*(i-1));
		COPYDOUBLE(&pos.longitude, (char *)pPath+24*(i-1)+8);
		COPYDOUBLE(&pos.altitude, (char *)pPath+24*(i-1)+16);
		printf("[CTL]GPS current points: latitude %.7f, longitude %.7f, altitude %.2f\n", pos.latitude, pos.longitude, pos.altitude);

		//set t
		double dx = (pos.latitude - cur.latitude)*_radius;
		double dy = (pos.longitude - cur.longitude)*_radius*cos(pos.latitude);
		double d = sqrt(dx*dx+dy*dy);
		printf("dx %f, dy %f, d %f\n", dx, dy, d);
		t += d/3;				//3m/s

		cur = pos;

		//set x, y, z, c ref in NED
		double x = (pos.latitude - pPos0->latitude) * _radius;
		double y = (pos.longitude - pPos0->longitude) * _radius * cos(pPos0->latitude);
		double z = from->z;

		double c = atan2(dy, dx);

		m_path[iPath][0] = t;
		m_path[iPath][1] = x;
		m_path[iPath][2] = y;
		m_path[iPath][3] = z;
		m_path[iPath][4] = c;

		m_path[iPath-1][4] = c;

		iPath ++;

		if ( i != nPath ) {
			t += 0.1;				//heading angle change

			m_path[iPath][0] = t;
			m_path[iPath][1] = x;
			m_path[iPath][2] = y;
			m_path[iPath][3] = z;
			m_path[iPath++][4] = c;
		}

		printf("[CTL] gps path: t %f, x %f, y %f, z %f, c %f\n", m_path[iPath][0], m_path[iPath][1], \
				m_path[iPath][2], m_path[iPath][3], m_path[iPath][4]);
	}
}
*/

void clsTmpPath::CreatePath(clsMatrix &mtrx)
{
//	_ASSERT(m<=MAX_TMPPATHSIZE && n == 5);
	m_path.Reset(mtrx.GetM(), mtrx.GetN(), (double *)m_data, TRUE);
	m_path = mtrx;
}

void clsTmpPath::CreatePath(double mtrx[][5], int m)
{
//	_ASSERT(m<=MAX_TMPPATHSIZE);
	m_path.Reset(m, 5, (double *)m_data, TRUE);
	m_path = (double *)mtrx;
}

void clsTmpPath::CreateHoldingPath(double pos[4])
{
	m_data[0][0] = 0;
	m_data[0][1] = pos[0];
	m_data[0][2] = pos[1];
	m_data[0][3] = pos[2];
	m_data[0][4] = pos[3];

	m_path.Reset(1, 5, (double *)m_data, TRUE);
}

void clsTmpPath::AddPathPoint(double t, double pos[4])
{
	int m = m_path.GetM();

	if (m == MAX_TMPPATHSIZE) return;

	m_data[m][0] = t;
	m_data[m][1] = pos[0];
	m_data[m][2] = pos[1];
	m_data[m][3] = pos[2];
	m_data[m][4] = pos[3];

	m_path.Reset(m+1, 5, (double *)m_data, TRUE);
}

void clsTmpPath::AddPathPointRPT(double t, double pos[4], double vel[3], double acc[3])
{
	int m = m_path.GetM();

	if (m == MAX_TMPPATHSIZE) return;

	m_data[m][0] = t;
	m_data[m][1] = pos[0];
	m_data[m][2] = pos[1];
	m_data[m][3] = pos[2];
	m_data[m][4] = pos[3];

	if (vel != NULL)
	{
		m_data[m][5] = vel[0];
		m_data[m][6] = vel[1];
		m_data[m][7] = vel[2];
	}
	else
	{
		m_data[m][5] = 0;
		m_data[m][6] = 0;
		m_data[m][7] = 0;
	}

	if (acc != NULL)
	{
		m_data[m][8] = acc[0];
		m_data[m][9] = acc[1];
		m_data[m][10] = acc[2];
	}
	else
	{
		m_data[m][8] = 0;
		m_data[m][9] = 0;
		m_data[m][10] = 0;
	}

//	cout<<"t "<<m_data[m][0]<<" x "<<m_data[m][1]<<" y "<<m_data[m][2]<<" z "<<m_data[m][3]<<" c "<<m_data[m][4]<<endl;
//	cout<<"u "<<m_data[m][5]<<" v "<<m_data[m][6]<<" w "<<m_data[m][7]<<endl;
	m_path.Reset(m+1, 11, (double *)m_data, TRUE);
}

void clsTmpPath::CreateReturnPath(double pos[4])
{
	int i = 0;
	double t = 0;
	m_data[i][0] = t;
	m_data[i][1] = pos[0];
	m_data[i][2] = pos[1];
	m_data[i][3] = pos[2];
	m_data[i][4] = pos[3];

	//hovering for 5 seconds
	i ++; t += 5;
	m_data[i][0] = t;
	m_data[i][1] = pos[0];
	m_data[i][2] = pos[1];
	m_data[i][3] = pos[2];
	m_data[i][4] = pos[3];

	//begin turning to the back direction
	double cReturn = ::atan2(-pos[1], -pos[0]);
	double dc = cReturn - pos[3];
	INPI(dc);

	if (::fabs(dc) > 2*PI/3) {				//if dc > 2*PI/3, insert a medial path point for turning, dc is error of heading angle
		dc /= 2;
		i++; t += ::fabs(dc)/0.2;				//turn to return direction in 0.2 rad/s
		m_data[i][0] = t;
		m_data[i][1] = pos[0];
		m_data[i][2] = pos[1];
		m_data[i][3] = pos[2];
		m_data[i][4] = pos[3]+dc;
		INPI(m_data[i][4]);
	}

	if (::fabs(dc) > PI/180) {				//ignore turning if dc is no bigger than 1 degree
		i++; t += ::fabs(dc)/0.2;

		m_data[i][0] = t;
		m_data[i][1] = pos[0];
		m_data[i][2] = pos[1];
		m_data[i][3] = pos[2];
		m_data[i][4] = cReturn;
	}

	//return to starting position in 3m/s
	double distance = ::sqrt(pos[0]*pos[0]+pos[1]*pos[1]);
	i++; t += distance/3;
	m_data[i][0] = t;
	m_data[i][1] = 0;
	m_data[i][2] = 0;
	m_data[i][3] = pos[2];
	m_data[i][4] = cReturn;

	m_path.Reset(i+1, 5, (double *)m_data, TRUE);
}

void clsCTL::B2Para(double kx, double ky, double kz, double kc)
{
	B13_kx = B2_kx = kx;
	B13_ky = B2_ky = ky;
	B13_kz = B2_kz = kz;
	B2_kc = kc;
}

void clsCTL::B2Parai(double kxi, double kyi, double kzi, double kci)
{
	B2_kxi = B13_kxi = kxi;
	B2_kyi = B13_kyi = kyi;
	B2_kzi = B13_kzi = kzi;
	B2_kci = kci;
}

void clsCTL::B5Para(unsigned int nChoice)
{
	char szF[MAXSTR_VARIABLENAME], szG[MAXSTR_VARIABLENAME];
	::sprintf(szF, "_B5_F%d", nChoice);
	::sprintf(szG, "_B5_G%d", nChoice);
	_parser.GetVariable(szF, B5_F);
	_parser.GetVariable(szG, B5_G);
}

void clsTmpPath::CreateTakeoffPath(double pos0[4], double pose[4])
{
	double vup = 0.2;
//	double afast = 0.33;
	double tfast = 5;	// 5 seconds to take off for QuadLion
//	double vfast = vup+afast*tfast;

	int nfast = (int)(tfast/0.1);
//	m_path.Reset(nfast+2, 5, (double *)m_data, TRUE);
	m_path.Reset(nfast, 11, (double *)m_data, TRUE);

	double tAcc = 2;
	double tCruise = 1;
	double tDec = tfast - tAcc - tCruise;
	double vCruise = fabs(pose[2]-pos0[2]) / (tAcc + tCruise);
	double afast = vCruise / tAcc;
	printf("[clsTmpPath] afast %.3f \n", afast);

	for (int i=0; i<nfast; i++) {				//0-15s, 0.2 - 0.5 m/s up
		double t = i*0.1;
		m_path[i][0] = t;
		m_path[i][1] = pos0[0];
		m_path[i][2] = pos0[1];
		m_path[i][3] = pos0[2]-0.5*afast*t*t;
		m_path[i][4] = pos0[3];
		m_path[i][5] = 0;
		m_path[i][6] = 0;
		m_path[i][8] = 0;
		m_path[i][9] = m_path[i][10] = 0;

		if (i < int(tAcc/0.1) && i >= 0) m_path[i][7] = afast*t;
		else if ( i>=int(tAcc/0.1) && i<=int((tfast-tDec)/0.1) ) m_path[i][7] = vCruise;
		else if ( i>=int((tfast - tDec)/0.1) && i<nfast ) m_path[i][7] = vCruise - afast*t;
	}

//	m_path[nfast+1][0] = m_path[nfast][0] + 100/vfast;
//	m_path[nfast+1][1] = m_path[nfast][1];
//	m_path[nfast+1][2] = m_path[nfast][2];
//	m_path[nfast+1][3] = m_path[nfast][3] - 100;				//0.5 m/s up
//	m_path[nfast+1][4] = m_path[nfast][4];
}

void clsTmpPath::CreateLandingPath(double from[4], double target[4])
{
	//target - target landing place
	//from - position from where the helicopter start to land
	
	double h = target[2] - from[2];

	double vdown = 0.4;
	double hslow = 7;
	double aslow = 0.02;
	double tslow = 5;
	double vslow = vdown-aslow*tslow;

	int nslow = (int)(tslow/0.1);

	m_path.Reset(nslow+3, 5, (double *)m_data, TRUE);

	m_path[0][0] = 0;
	m_path[0][1] = target[0];
	m_path[0][2] = target[1];
	m_path[0][3] = target[2]-h;
	m_path[0][4] = from[3];

	double t1 = ::fabs(h-hslow)/vdown;
	if (t1 == 0) t1 = 1;

	for (int i=0; i<=nslow; i++) {				//down 15s, 0.5-0.2
		double t2 = i*0.1;
		double t = t1 + t2;
		m_path[i+1][0] = t;
		m_path[i+1][1] = target[0];
		m_path[i+1][2] = target[1];
		m_path[i+1][3] = target[2]-hslow+vdown*t2-0.5*aslow*t2*t2;
		m_path[i+1][4] = from[3];
	}

	m_path[nslow+2][0] = m_path[nslow+1][0] + 100/vslow;
	m_path[nslow+2][1] = m_path[nslow+1][1];
	m_path[nslow+2][2] = m_path[nslow+1][2];
	m_path[nslow+2][3] = m_path[nslow+1][3] + 100;
	m_path[nslow+1][4] = m_path[nslow+1][4];
}

void clsCTL::Init()
{
	/*Variables initialization for 2014 SAFMC by LPD, 2014-Mar-05*/
	m_bTakeOffFlag = false;
	m_bTransition_1 = false;
	m_bPath_1 = false;
	m_bVisionInitialization = false;
	m_bVisionGuidance = false;
	m_bVisionGuidance = false;
	m_bLandingFinishFlag = false;
	m_bTransition2 = false;
	m_bPath2 = false;
	m_bLandingFlag = false;
	B5_wg0 = 0;
	m_bSAFMCPathTotalTimeGetted = false;
	m_bSAFMCtargetDropped = false;
	/*End <-- Variables initialization for 2014 SAFMC by LPD, 2014-Mar-05*/

	B_acxr_ub = 0 ;
	B_acyr_vb = 0;
	B_aczr_wb = 0;
	B_lr = -0.08;

	SAMFC = FALSE;
	start = 1;
	B_tr = -1; //not open the triger yet
	xy_control = 0;
	z_control = 1;
	//B5_pPath->SAMFC = FALSE;
//	cout<<"z_control is"<<z_control;
	m_bFK = FALSE;
	m_innerloop = INNERLOOP_LQR;	// default:
	m_innerloop = INNERLOOP_RPT;
//	m_innerloop = INNERLOOP_GAINSCHEDULE;	// para(18), change to default in fast forward flight
	m_nHeading = 0;
	m_equ = _equ_Hover;
	::memset(&m_sig, 0, sizeof(m_sig));

// A1 equ and F, G are dynamically decided by A1_Lookup (gain schedule)

	A2_equ = _equ_Hover;
	A2_pVel = NULL;
	A2_bEnd = TRUE;
	A2_bbEnd = FALSE;
	A2_t0 = -1;

	 if ( _HELICOPTER == ID_QUADLION ) {
		m_xerrint = m_yerrint = m_zerrint = m_cerrint = 0;
		OUTER_P_QUADLION.Reset(4,7, (double *)_OUTER_P_QUADLION, TRUE);
		OUTER_D_QUADLION.Reset(4,7, (double *)_OUTER_D_QUADLION, TRUE);
		_parser.GetVariable("_OUTER_P_QUADLION", OUTER_P_QUADLION);
//		_parser.GetVariable("_OUTER_D_QUADLION", OUTER_D_QUADLION);

		Fxy.Reset(5, (double *)_Fxy, TRUE);
		Fz.Reset(5, (double *)_Fz, TRUE);
		Fc.Reset(3, (double *)_Fc, TRUE);
		_parser.GetVariable("_Fxy", Fxy);
		_parser.GetVariable("_Fz", Fz);
		_parser.GetVariable("_Fc", Fc);

		_parser.GetVariable("dc_ail2phi", &dc_ail2phi);
		_parser.GetVariable("dc_ele2tht", &dc_ele2tht);
		_parser.GetVariable("dc_thr2w", &dc_thr2w);
		_parser.GetVariable("dc_rud2r", &dc_rud2r);
		_parser.GetVariable("damping_u", &damping_u);
		_parser.GetVariable("damping_v", &damping_v);
		_parser.GetVariable("ratio_u", &ratio_u);
		_parser.GetVariable("ratio_v", &ratio_v);

		_parser.GetVariable("tLand", &tLand);
		B5_vax[0] = B5_vax[1] = B5_vay[0] = B5_vay[1] = B5_vaz[0] = B5_vaz[1] = B5_vac = 0;
		m_bAutoPath = TRUE;
		m_bIntegrator = FALSE;
		B5_t4 = -1;

		m_ReflexxesInitialized = false;

	    RML	=	new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
	    IP	=	new RMLPositionInputParameters(NUMBER_OF_DOFS);
	    OP	=	new RMLPositionOutputParameters(NUMBER_OF_DOFS);

	    IP->MaxVelocityVector->VecData			[0]	=	 2		;
	    IP->MaxVelocityVector->VecData			[1]	=	 2		;
	    IP->MaxVelocityVector->VecData			[2]	=	 0.5		;
	    IP->MaxVelocityVector->VecData          [3] =    0.2;

	    IP->MaxAccelerationVector->VecData		[0]	=	 0.7		;
	    IP->MaxAccelerationVector->VecData		[1]	=	 0.7		;
	    IP->MaxAccelerationVector->VecData		[2]	=	 0.3		;
	    IP->MaxAccelerationVector->VecData      [3] =    0.1        ;

	    IP->MaxJerkVector->VecData				[0]	=	 1.0		;
	    IP->MaxJerkVector->VecData				[1]	=	 1.0		;
	    IP->MaxJerkVector->VecData				[2]	=	 1.0		;
	    IP->MaxJerkVector->VecData              [3] =    1.0        ;

	    IP->SelectionVector->VecData[0]	=	true;
	    IP->SelectionVector->VecData[1]	=	true;
	    IP->SelectionVector->VecData[2]	=	true;
	    IP->SelectionVector->VecData[3] =   true;
	 }
	else if ( _HELICOPTER == ID_HELION || _HELICOPTER == ID_SHELION )
	{
		A2_F_RPT.Reset(4, 9 /*11*/, (double *)_A2_F_RPT, TRUE);
		A2_G_RPT.Reset(4, 4, (double *)_A2_G_RPT, TRUE);

		_parser.GetVariable("_A2_F_RPT", A2_F_RPT);
		_parser.GetVariable("_A2_G_RPT", A2_G_RPT);

		B5_F.Reset(3,6, (double *)_B5_F, TRUE);
		B5_G.Reset(3,9, (double *)_B5_G, TRUE);
		_parser.GetVariable("_B5_F", B5_F);
		_parser.GetVariable("_B5_G", B5_G);
	}

	B5_kx = -0.35;
	B5_ky = -0.35;
	B5_kz = -0.25;
	B5_kc = -0.35;

	B5_xref = B5_yref = B5_zref = B5_cref = 0;

	B5_adjustc = 0;
	B5_t1 = -1;
	B5_t2 = -1;
	B5_semiPsi = 0;
	B5_PsiErr = 0;
	B5_kxi = B5_kyi = B5_kzi = B5_kci = 0;				//no integral control involved


	B6_bChirp = FALSE;
//	B6_add = CHIRPADD_CONTROLSIGNAL;
	B6_add = CHIRPADD_OUTERLOOP;

	if (_HELICOPTER == ID_FEILION) {

		A1_equ.ea = A1_equ.ee = A1_equ.er = A1_equ.et = A1_equ.eu = 0;
		ref_x = ref_y = ref_z = 0;
		ref_u = ref_v = ref_w = 0;
		ref_acx = ref_acy = ref_acz = 0;

		// define A, B, C, G, K matrices
		A1_A.Reset(11, 11, (double *)_A1_A, TRUE);
		A1_B.Reset(11,  6, (double *)_A1_B, TRUE);
		A1_C.Reset( 3, 11, (double *)_A1_C, TRUE);
		A1_G.Reset( 3,  3, (double *)_A1_G, TRUE);
		A1_K.Reset(11,  3, (double *)_A1_K, TRUE);
		_parser.GetVariable("_A3_A", A1_A);
		_parser.GetVariable("_A3_B", A1_B);
		_parser.GetVariable("_A3_C", A1_C);
		_parser.GetVariable("_A3_G", A1_G);
		_parser.GetVariable("_A3_K", A1_K);

		B3_A.Reset( 6,  6, (double *)_B3_A, TRUE);
		B3_B.Reset( 6,  3, (double *)_B3_B, TRUE);
		B3_C.Reset( 3,  6, (double *)_B3_C, TRUE);
		B3_G.Reset( 3,  9, (double *)_B3_G, TRUE);
		B3_K.Reset( 6,  9, (double *)_B3_K, TRUE);
		_parser.GetVariable("_B3_A", B3_A);
		_parser.GetVariable("_B3_B", B3_B);
		_parser.GetVariable("_B3_C", B3_C);
		_parser.GetVariable("_B3_G", B3_G);
		_parser.GetVariable("_B3_K", B3_K);

		for (int i=0; i<11; i++) 	_A1_x[i] 		= 0;
		for (int i=0; i<11; i++) 	_A1_x_raw[i] 	= 0;
		for (int i=0; i<6; i++) 	_B3_x[i] 		= 0;


		// Position and heading references
		B2_xref = 0.0;
		B2_yref = 0.0;
		B2_zref = -1.2;
		B2_cref = 0;

		// Initialization for the integral term
		B2_x_I = 0;
		B2_y_I = 0;
		B2_z_I = 0;
		B2_c_I = 0;

		// PID parameters
		B2_kx_P = 0;//0.08;//0.1;
		B2_kx_I = 0;
		B2_kx_D = 0;//0.3;//1;

		B2_ky_P = 0;//0.08;//0.1;
		B2_ky_I = 0;
		B2_ky_D = 0;//0.3;//1; //0.4;

		B2_kz_P = 0.24;
		B2_kz_I = 0.0024;
		B2_kz_D = 0.24;

		B2_kc_P = 2.5; //0.2;
		B2_kc_I = 0; //0.005;//0.02;
		B2_kc_D = 0; //0.2;
	}

	Ave_equ.ea = 0;
	Ave_equ.ee = 0;
	Ave_equ.er = 0;
	Ave_equ.et = 0;
	Ave_equ.eu = 0;
	equ_count = 0;
	SetTrim = 0;

	m_sig.aileron = _equ_Hover.ea;
	m_sig.elevator = _equ_Hover.ee;
	m_sig.auxiliary = _equ_Hover.eu;
	m_sig.rudder = _equ_Hover.er;
	m_sig.throttle = _equ_Hover.et;
}

clsPlan::clsPlan()
{
	m_behavior.behavior = 0;
}

clsPlan::~clsPlan()
{
}

void clsCTL::SetPlan(int nPlan)
{
	if (nPlan == 1) m_pPlan = &m_plan1;
	else if (nPlan == 2) m_pPlan = &m_planTakeoff;
	else if (nPlan == 3) {
		m_pPlan = &m_planLand;
		_ctl.SetPathSmooth();
	}
	else if (nPlan == 4) m_pPlan = &m_plan2014SAFMC;

}

void clsTakeoffPlan::Reset()
{
	m_mode = READY;

	m_x = m_y = m_z = m_c = VALUE_VACANT;
}

int clsTakeoffPlan::Run()
{
	EVENT &event = _state.GetEvent();

	m_behavior.behavior = 0;

	switch (m_mode) {
	case READY:
		if (IsVacant(m_x)) m_x = m_state.x;
		if (IsVacant(m_y)) m_y = m_state.y;
		if (IsVacant(m_z)) m_z = m_state.z - 15;
		if (IsVacant(m_c)) m_c = m_state.c;

		m_mode = ENGINEUP;
		m_behavior.behavior = BEHAVIOR_ENGINEUP;
		break;

	case ENGINEUP:
		if (event.code == EVENT_BEHAVIOREND && (int &)event.info[0] == BEHAVIOR_ENGINEUP) {
			_state.ClearEvent();

			m_mode = ASCEND;

			double pos0[4] = { m_state.x, m_state.y, m_state.z, m_state.c };
			double pose[4] = { m_x, m_y, m_z, m_state.c };

			_pathTmp.CreateTakeoffPath(pos0, pose);

			_ctl.B2Para(-0.35, -0.35, -0.05, -0.7);

			m_behavior.behavior = BEHAVIOR_PATH;
			(int &)m_behavior.parameter[0] = -1;
			(int &)m_behavior.parameter[4] = PATHTRACKING_FIXED;

		}
		break;

	case ASCEND:
		if (::fabs(m_state.z - m_z) < 1) {
			_ctl.B2Para(-0.35, -0.35, -0.25, -0.7);				//resume regular parameters

			m_mode = HOLD;
			m_behavior.behavior = BEHAVIOR_HOLD;

			(double &)m_behavior.parameter[0] = m_x;
			(double &)m_behavior.parameter[8] = m_y;
			(double &)m_behavior.parameter[16] = m_z;
			(double &)m_behavior.parameter[24] = m_c;
		}
		break;

	case HOLD:
		break;
	}

	return m_mode != HOLD;
}

void clsLandPlan::Reset()
{
	m_mode = START;

	m_x = m_y = m_z = m_c = VALUE_VACANT;
}

int clsLandPlan::Run()
{
	EVENT &event = _state.GetEvent();

	m_behavior.behavior = 0;
	switch (m_mode) {
	case START:
		m_mode = DESCEND;
		m_behavior.behavior = BEHAVIOR_PATHA;
		break;
	case DESCEND:
		break;
	case GROUND:
		break;
	}

	return m_mode != GROUND;
}

/*
 * cls2014SAFMCPlan :: Task management for 2014 SAFMC by LPD, 2014 - March - 05
 */

void cls2014SAFMCPlan::Reset()
{
	m_mode = READY;
	printf("[2014 SAFMC Tasks Management] reset \n");
}

int cls2014SAFMCPlan::Run(){
	EVENT &event = _state.GetEvent();
	m_behavior.behavior = 0;

	switch (m_mode) {
		case READY:
		{
			m_mode = ENGINE_UP;
			_ctl.ResetIntegratorFlag();
			m_behavior.behavior = BEHAVIOR_ENGINEUP;
		}
		break;

		case ENGINE_UP:
			if (event.code == EVENT_BEHAVIOREND && (int &)event.info[0] == BEHAVIOR_ENGINEUP) {
				_state.ClearEvent();
				m_mode = TAKEING_OFF;
				_ctl.SetTakeOffFlag();
				m_behavior.behavior = BEHAVIOR_PATHA;
				(int &)m_behavior.parameter[0] = -1;
				(int &)m_behavior.parameter[4] = PATHTRACKING_FIXED;
			}
			break;

		case TAKEING_OFF:
			if ( (event.code == EVENT_BEHAVIOREND && (int &)event.info[0] == BEHAVIOR_PATHA) ) {
////				 Taking-off, then hold;
//				_state.ClearEvent();
//				m_mode = HOLD;
//				_ctl.ResetTakeOffFlag();
//				_ctl.SetIntegratorFlag();
//				m_behavior.behavior = BEHAVIOR_PATHA;

				//Fly to target point;
				_state.ClearEvent();
				m_mode = TRANSITION_1;
				_ctl.ResetTakeOffFlag();
				_ctl.SetIntegratorFlag();
				_ctl.SetTransition1Flag();
				m_behavior.behavior = BEHAVIOR_PATHA;

////				 Taking-off, hold 5s, then landing;
//				_state.ClearEvent();
//				m_mode = LANDING;
//				_ctl.ResetTakeOffFlag();
//				_ctl.SetIntegratorFlag();
//				_ctl.SetLandingFlag();
//				m_behavior.behavior = BEHAVIOR_PATHA;

//				// For vision guidance test;
//				_state.ClearEvent();
//				m_mode = VISION_INITIALIZATION;
//				_ctl.ResetTakeOffFlag();
//				_ctl.SetVisionInitializationFlag();
//				m_behavior.behavior = BEHAVIOR_PATHA;
			}
			break;

		case TRANSITION_1:
			if ( (event.code == EVENT_BEHAVIOREND && (int &)event.info[0] == BEHAVIOR_PATHA) ){
//				_state.ClearEvent();
//				m_mode = PATH_1;
//				_ctl.ResetTransition1Flag();
//				_ctl.SetPath1Flag();
//				m_behavior.behavior = BEHAVIOR_PATHA;

				_state.ClearEvent();
				m_mode = VISION_INITIALIZATION;
				_ctl.ResetTransition1Flag();
				_ctl.SetVisionInitializationFlag();
				m_behavior.behavior = BEHAVIOR_PATHA;
			}
			break;
		case PATH_1:
			if ( (event.code == EVENT_BEHAVIOREND && (int &)event.info[0] == BEHAVIOR_PATHA) ){
				// For competition purpose;
				_state.ClearEvent();
				m_mode = VISION_INITIALIZATION;
				_ctl.ResetPath1Flag();
				_ctl.SetVisionInitializationFlag();
				m_behavior.behavior = BEHAVIOR_PATHA;

//				// For vicon room test purpose, skip vision initialization, guidance phase;
//				_state.ClearEvent();
//				m_mode = TRANSITION_2;
//				_ctl.ResetPath1Flag();
//				_ctl.SetTransition2Flag();
//				m_behavior.behavior = BEHAVIOR_PATHA;
			}
			break;
		case VISION_INITIALIZATION:
			if ( (event.code == EVENT_BEHAVIOREND && (int &)event.info[0] == BEHAVIOR_PATHA) ){
				if (_ctl.GetSAFMCTargetDropped()){
					_state.ClearEvent();
					m_mode = TRANSITION_2;
					_ctl.SetIntegratorFlag();
					_ctl.ResetVisionInitializationFlag();
					_ctl.SetTransition2Flag();
					m_behavior.behavior = BEHAVIOR_PATHA;

//					_state.ClearEvent();
//					m_mode = LANDING;
//					_ctl.ResetVisionInitializationFlag();
//					_ctl.SetLandingFlag();
//					m_behavior.behavior = BEHAVIOR_PATHA;
				}
				else{
					_state.ClearEvent();
					m_mode = VISION_GUIDANCE;
					_ctl.SetIntegratorFlag();
					_ctl.ResetVisionInitializationFlag();
					_ctl.SetVisionGuidanceFlag();
					m_behavior.behavior = BEHAVIOR_PATHA;
				}
			}
			break;
		case VISION_GUIDANCE:
			if ( (event.code == EVENT_BEHAVIOREND && (int &)event.info[0] == BEHAVIOR_PATHA) ){
				_state.ClearEvent();
				_ctl.ResetVisionGuidanceFlag();
				_ctl.ResetIntegratorFlag();
				_ctl.SetlandingFinishFlag();
				m_mode = TASK;
				m_behavior.behavior = BEHAVIOR_ENGINEDOWN;
			}
			break;
		case TASK:
			if ( (event.code == EVENT_BEHAVIOREND && (int &)event.info[0] == BEHAVIOR_ENGINEDOWN) ){
				m_mode = ENGINE_UP_AFTER_TASK;
				_ctl.ResetIntegratorFlag();
				m_behavior.behavior = BEHAVIOR_ENGINEUP;
			}
			break;
	    case ENGINE_UP_AFTER_TASK:
			if (event.code == EVENT_BEHAVIOREND && (int &)event.info[0] == BEHAVIOR_ENGINEUP) {
				_state.ClearEvent();
				m_mode = TRANSITION_2;
				_ctl.SetIntegratorFlag();
				_ctl.SetTransition2Flag();
				m_behavior.behavior = BEHAVIOR_PATHA;
			}
			break;
		case TRANSITION_2:
			if ( (event.code == EVENT_BEHAVIOREND && (int &)event.info[0] == BEHAVIOR_PATHA) ){
//				_state.ClearEvent();
//				m_mode = PATH_2;
//				_ctl.ResetTransition2Flag();
//				_ctl.SetPath2Flag();
//				m_behavior.behavior = BEHAVIOR_PATHA;

				_state.ClearEvent();
				m_mode = LANDING;
				_ctl.ResetTransition2Flag();
				_ctl.SetLandingFlag();
				m_behavior.behavior = BEHAVIOR_PATHA;
			}
			break;
		case PATH_2:
			if ( event.code == EVENT_BEHAVIOREND && (int &)event.info[0] == BEHAVIOR_PATHA ){
				_state.ClearEvent();
				m_mode = LANDING;
				_ctl.ResetPath2Flag();
				_ctl.SetLandingFlag();
				m_behavior.behavior = BEHAVIOR_PATHA;
			}
			break;
		case LANDING:
			if(event.code == EVENT_BEHAVIOREND && (int &)event.info[0] == BEHAVIOR_PATHA ){
				_state.ClearEvent();
				_ctl.ResetLandingFlag();
				_ctl.ResetIntegratorFlag();
				m_mode = ENGINE_DOWN;
				m_behavior.behavior = BEHAVIOR_ENGINEDOWN;
			}
			break;
		case ENGINE_DOWN:
			if(event.code == EVENT_BEHAVIOREND && (int &)event.info[0] == BEHAVIOR_ENGINEDOWN ){
				_state.ClearEvent();
				m_mode = GROUND;
			}
			break;
		case GROUND:
			break;
		case HOLD:
			break;
		}

	return m_mode != GROUND;
}


clsPath *clsCTL::GetPath(int nPath)
{
	if (nPath == -1) return &_pathTmp;
	else if (nPath >= 1 && nPath <= _nPath) return &_path[nPath-1];
	else return NULL;
}


void clsTmpPath::CreateFromGPSPath(clsPath *pPath, double coor[2], double pos[4])
{
	//coor[2] store the longitude and latitude of origin, pos is the default position
	clsMatrix &mtrx = pPath->GetMatrix();
	int m = mtrx.GetM();

	m_data[0][0] = 0;
	m_data[0][1] = pos[0];
	m_data[0][2] = pos[1];
	m_data[0][3] = pos[2];
	m_data[0][4] = pos[3];

	//put gps path pPath to m_data
	double tTransition = 0;
	for (int i=1; i<=m; i++) {
		double t = mtrx[i-1][0];
		double longitude = mtrx[i-1][1]*PI/180;
		double latitude = mtrx[i-1][2]*PI/180;
		double h = mtrx[i-1][3];
		double c = mtrx[i-1][4]*PI/180;

		double x = (latitude-coor[1])*_radius;
		double y = (longitude-coor[0])*_radius;
		double z = -h;

		if (i==1) {				//first point, calculate the transition time
			double dx = x - pos[0];
			double dy = y - pos[1];
			double dz = z - pos[2];
			double dc = c - pos[3];
			INPI(dc);

			double distance = ::sqrt(dx*dx+dy*dy+dz*dz);
			double tTransition = max(distance/2, dc/0.2);				//go to startig point in velocity up to 2 m/s, or turning to starting heading angle in rate up to 0.2 rad/s
			if (tTransition < 5) tTransition = 5;				//at least 5 seconds of transition
		}

		m_data[i][0] = tTransition + t;
		m_data[i][1] = x;
		m_data[i][2] = y;
		m_data[i][3] = z;
		m_data[i][4] = c;

		INPI(m_data[i][4]);
	}

	//set path to m_data
	m_path.Reset(m+1, 5, (double *)m_data, TRUE);
}

void clsCTL::FormationKalman(double pos[4], double vel[3], double acc[3], double psider[2])
{
	if (!m_bFK)	{
		F_F.Reset(3,3, (double *)_F_F, TRUE);
		_parser.GetVariable("_F_F", F_F);
		
		F_H.Reset(1,3, (double *)_F_H, TRUE);
		_parser.GetVariable("_F_H", F_H);
		
		F_H2.Reset(2,3, (double *)_F_H2, TRUE);
		_parser.GetVariable("_F_H2", F_H2);
		
		F_Q.Reset(3,3, (double *)_F_Q, TRUE);
		_parser.GetVariable("_F_Q", F_Q);
		
		F_R.Reset(1,1, (double *)_F_R, TRUE);
		_parser.GetVariable("_F_R", F_R);
		
		F_R2.Reset(2,2, (double *)_F_R2, TRUE);
		_parser.GetVariable("_F_R2", F_R2);
		
		F_P.Reset(3,3, (double *)_F_P, TRUE);
		_parser.GetVariable("_F_P", F_P);
		
		F_P2.Reset(3,3, (double *)_F_P2, TRUE);
		_parser.GetVariable("_F_P2", F_P2);
		
		m_ldHeading = _B_psi[0] = pos[3];
		_B_psi[1] = _B_psi[2] = 0;
		
		::memset(_pva, 0, 9*sizeof(double));
		_pva[0][0] = pos[0]; _pva[0][1] = vel[0]; 
		_pva[1][0] = pos[1]; _pva[1][1] = vel[1];
		_pva[2][0] = pos[2]; _pva[2][1] = vel[2];
		m_bFK = TRUE;
		cout<<"m_bFK set once!"<<endl;
	}
	else {
		double dHeading = pos[3] - m_ldHeading;
		if (dHeading < -PI) {
			m_nHeading ++;
		} 
		else if (dHeading > PI) {
			m_nHeading --;
		}
		m_ldHeading = pos[3];
		pos[3] = m_ldHeading + m_nHeading*2*PI;
		// ------ prediction ------
		// for psi derivative
		double _Fx[3];
		clsVector Fx(3, (double *)_Fx, TRUE);
		clsVector B_psi(3,(double *)_B_psi, TRUE);
		clsMatrix::X(F_F, B_psi, Fx);
		B_psi = Fx;
		
		if (m_nCount % 50 == 0) {
			cout<<"_B_psi "<<_B_psi[0]<<" "<<_B_psi[1]<<" "<<_B_psi[2]<<endl;
		}
		
		double _F_Ft[3][3]; clsMatrix F_Ft;
		F_Ft.Reset(3, 3, (double *)_F_Ft, TRUE);
		clsMatrix::T(F_F, F_Ft);
		
		double _FP[3][3]; clsMatrix FP;
		FP.Reset(3,3,(double *)_FP, TRUE);
		clsMatrix::X(F_F, F_P, FP);
		
		double _FPFt[3][3]; clsMatrix FPFt;
		FPFt.Reset(3, 3, (double *)_FPFt, TRUE);
		clsMatrix::X(FP, F_Ft, FPFt);
		
		FPFt += F_Q;
		F_P = FPFt;
		
		// for pva
		for (int i=0; i<3; i++) {
			clsVector pva(3,_pva[i], TRUE);
//			_pva[i][0] = pos[i]; _pva[i][1] = vel[i]; _pva[i][2] = acc[i];
			double _Fpva[3]; clsVector Fpva(3, (double *)_Fpva, TRUE);
			clsMatrix::X(F_F, pva, Fpva);
			pva = Fpva;
			
/*			if (m_nCount % 50 ==0) {
				cout<<"input: "<<pos[i]<<" "<<vel[i]<<" "<<acc[i]<<endl;
				cout<<"pva: "<<_pva[i][0]<<" "<<_pva[i][1]<<" "<<_pva[i][2]<<endl;
			}*/
		}
		
		double _F_Ft2[3][3]; clsMatrix F_Ft2;
		F_Ft2.Reset(3, 3, (double *)_F_Ft2, TRUE);
		clsMatrix::T(F_F, F_Ft2);
		
		double _FP2[3][3]; clsMatrix FP2;
		FP2.Reset(3,3,(double *)_FP2, TRUE);
		clsMatrix::X(F_F, F_P2, FP2);
		
		double _FPFt2[3][3]; clsMatrix FPFt2;
		FPFt2.Reset(3, 3, (double *)_FPFt2, TRUE);
		clsMatrix::X(FP2, F_Ft2, FPFt2);
		
		FPFt2 += F_Q;
		F_P2 = FPFt2;
		// ------ correction ------
		double _HP[1][3]; clsMatrix HP;
		HP.Reset(1,3, (double *)_HP, TRUE);
		clsMatrix::X(F_H,F_P,HP);
		
		double _F_Ht[3][1]; clsMatrix FHt; 
		FHt.Reset(3,1, (double *)_F_Ht, TRUE);
		clsMatrix::T(F_H,FHt);
		
		double _HPHt[1][1]; clsMatrix HPHt;
		HPHt.Reset(1,1, (double *)_HPHt, TRUE);
		clsMatrix::X(HP,FHt,HPHt);
		HPHt += F_R;
		
		double _invMatrix[1][1]; clsMatrix invMatrix;
		invMatrix.Reset(1,1,(double *)_invMatrix, TRUE);
		clsMatrix::R(HPHt, invMatrix);
		
		double _PHt[3][1]; clsMatrix PHt;
		PHt.Reset(3,1,(double *)_PHt, TRUE);
		clsMatrix::X(F_P, FHt, PHt); 
		
		double _K[3][1]; clsMatrix K;
		K.Reset(3,1,(double *)_K, TRUE);
		clsMatrix::X(PHt, invMatrix, K); 
		
		double _phi[1]; clsVector phi(1, (double *)_phi, TRUE);
		_phi[0] = pos[3];
		
		double _Hx[1]; clsVector Hx(1,(double *)_Hx, TRUE);
		clsMatrix::X(F_H, B_psi, Hx); 
		phi -= Hx;
		
//		_B_psi[0] = _K[0][0] * _phi[0]; _B_psi[1] = _K[1][0] * _phi[0]; _B_psi[2] = _K[2][0] * _phi[0];
		double _Kphi[3]; clsVector Kphi(3, (double *)_Kphi, TRUE);
		clsMatrix::X(K, phi, Kphi);
		B_psi += Kphi;
		
		double _KH[3][3]; clsMatrix KH;
		KH.Reset(3,3,(double *)_KH, TRUE);
		clsMatrix::X(K,F_H, KH); 
		
//		_KH[0][0] = _K[0][0]*_F_H[0][0]; _KH[0][1] = _K[0][0]*_F_H[0][1]; _KH[0][2] = _K[0][0]*_F_H[0][2]; 
//		_KH[1][0] = _K[1][0]*_F_H[0][0]; _KH[1][1] = _K[1][0]*_F_H[0][1]; _KH[1][2] = _K[1][0]*_F_H[0][2]; 
//		_KH[2][0] = _K[2][0]*_F_H[0][0]; _KH[2][1] = _K[2][0]*_F_H[0][1]; _KH[2][2] = _K[2][0]*_F_H[0][2];
		
		double _EYE[3][3] = {
				{1,0,0}, {0,1,0}, {0,0,1}
		};
		clsMatrix EYE; EYE.Reset(3,3,(double *)_EYE, TRUE);
		EYE -= KH;
		
		double _F_Ptmp[3][3]; clsMatrix F_Ptmp;
		F_Ptmp.Reset(3,3,(double *)_F_Ptmp, TRUE);
		clsMatrix::X(EYE, F_P, F_Ptmp);
		F_P = F_Ptmp;
		
		// correction for pva
		// Kk
		double _HP2[2][3]; clsMatrix HP2;
		HP2.Reset(2,3, (double *)_HP2, TRUE);
		clsMatrix::X(F_H2,F_P,HP2);
		
		double _F_Ht2[3][2]; clsMatrix FHt2; 
		FHt2.Reset(3,2, (double *)_F_Ht2, TRUE);
		clsMatrix::T(F_H2,FHt2);
		
		double _HPHt2[2][2]; clsMatrix HPHt2;
		HPHt2.Reset(2,2, (double *)_HPHt2, TRUE);
		clsMatrix::X(HP2,FHt2,HPHt2);
		HPHt2 += F_R2;
		
		double _invMatrix2[2][2]; clsMatrix invMatrix2;
		invMatrix2.Reset(2,2,(double *)_invMatrix2, TRUE);
		clsMatrix::R(HPHt2, invMatrix2);
		
		double _PHt2[3][2]; clsMatrix PHt2;
		PHt2.Reset(3,2,(double *)_PHt2, TRUE);
		clsMatrix::X(F_P2, FHt2, PHt2);
		
		double _K2[3][2]; clsMatrix K2;
		K2.Reset(3,2,(double *)_K2, TRUE);
		clsMatrix::X(PHt2, invMatrix2, K2);

		// Pk
		double _EYE2[3][3] = {
				{1,0,0}, {0,1,0}, {0,0,1}
		};
		clsMatrix EYE2; EYE2.Reset(3,3,(double *)_EYE2, TRUE);
		
		double _KH2[3][3]; clsMatrix KH2;
		KH2.Reset(3,3,(double *)_KH2, TRUE);
		clsMatrix::X(K2,F_H2, KH2);
		EYE2 -= KH2;
		
		clsMatrix::X(EYE2, F_P2, F_Ptmp);
		F_P2 = F_Ptmp;
		
		// Xk
		double _zk[3][2]; 
		for (int i=0; i<3; i++) {
			clsVector zk(2,(double *)_zk[i], TRUE);
			_zk[i][0] = pos[i]; _zk[i][1] = vel[i];
			double _Hx2[2]; clsVector Hx2(2,(double *)_Hx2, TRUE);
			clsVector pva(3, _pva[i], TRUE);
			clsMatrix::X(F_H2, pva, Hx2);
			zk -= Hx2;
			double _Kzk2[3]; clsVector Kzk2(3, (double *)_Kzk2, TRUE);
			clsMatrix::X(K2, zk, Kzk2);
			pva += Kzk2;
			pos[i] = _pva[i][0]; vel[i] = _pva[i][1]; acc[i] = _pva[i][2];
			

		}
	}
}

/* Implementations of defined functions for 2014 SAFMC by Liu Peidong, 2014-Mar-05 */
void clsCTL::ConstructTakeOffPath(UAVSTATE state, double height, double pnr[4], double vnr[3], double anr[3]){
	double tStart = _ctl.GetPathStartTime();
	double tElapse = ::GetTime() - tStart;

	IP->MaxVelocityVector->VecData			[0]	=	 2;
	IP->MaxVelocityVector->VecData			[1]	=	 2;
	IP->MaxVelocityVector->VecData			[2]	=	 1.5;
	IP->MaxAccelerationVector->VecData		[0]	=	 0.7;
	IP->MaxAccelerationVector->VecData		[1]	=	 0.7;
	IP->MaxAccelerationVector->VecData		[2]	=	 0.5;

	QROTOR_REF temp_takeOffFinalRef;
	memset(&temp_takeOffFinalRef, 0, sizeof(QROTOR_REF));
	temp_takeOffFinalRef.p_x_r = B5_x0;
	temp_takeOffFinalRef.p_y_r = B5_y0;
	temp_takeOffFinalRef.p_z_r = height;
	temp_takeOffFinalRef.psi_r = B5_c0;

	static double pathTotalTime;

	if (!m_bSAFMCPathTotalTimeGetted){
		m_ReflexxesInitialized = false;
		pathTotalTime = ReflexxesPathPlanning(state, temp_takeOffFinalRef, pnr, vnr, anr);
		m_bSAFMCPathTotalTimeGetted = true;
	}

	ReflexxesPathPlanning(state, temp_takeOffFinalRef, pnr, vnr, anr);

	static int count = 0;
	count++;
	if (count > 1e7) count = 0;
	if(count%100 == 0){
		printf("[ctl:TakeOffPath] totalPathTime %.2f; pnr: %.2f %.2f %.2f %.2f; vnr: %.2f %.2f %.2f\n", pathTotalTime, pnr[0], pnr[1], pnr[2], pnr[3], vnr[0], vnr[1], vnr[2]);
	}
	if (tElapse > pathTotalTime){
		_ctl.SetIntegratorFlag();
	}
	if (tElapse > pathTotalTime){
		B5_t2 = -1;
		m_bSAFMCPathTotalTimeGetted = false;
		_state.SetEvent(EVENT_BEHAVIOREND, BEHAVIOR_PATHA);
	}
}

void clsCTL::ConstructTransition1PathRef(double outerRefPos[4], double outerRefVel[3], double outerRefAcc[3]){
		double tStart = _ctl.GetPathStartTime();
		double tElapse = ::GetTime() - tStart;

		static double final_pos[4] = {0};
		static double final_vel[3] = {0};
		static double final_acc[3] = {0};

		//update the roof location regularly in 0.1 Hz;
		static bool firstTimeEnter = true;
		if( firstTimeEnter || m_nCount%500 == 0){
			double roof_latitude, roof_longitude, roof_heading;
			_parser.GetVariable("_ROOF_LATITUDE", &roof_latitude);
			_parser.GetVariable("_ROOF_LONGTITUDE", &roof_longitude);
			_parser.GetVariable("_ROOF_HEADING", &roof_heading);

			roof_latitude = roof_latitude*PI/180.0;
			roof_longitude = roof_longitude*PI/180.0;
			roof_heading = roof_heading*PI/180.0;

			double delta[2]={0};
			_state.CalculateRelativePositionViaGPS(roof_latitude, roof_longitude, delta);

			printf("[ctl:Transition-1] delta: %.2f %.2f\n", delta[0], delta[1]);

			final_pos[0] = outerRefPos[0] + delta[0];
			final_pos[1] = outerRefPos[1] + delta[1];
			final_pos[2] = outerRefPos[2];
			final_pos[3] = roof_heading;

			final_vel[0] = 0; final_vel[1] = 0; final_vel[2] = 0;
			final_acc[0] = 0; final_acc[1] = 0; final_acc[2] = 0;

			if( sqrt(delta[0]*delta[0] + delta[1]*delta[1]) < 1 ){
				//TODO: Reached the roof top;
				B5_t2 = -1;
				m_bSAFMCPathTotalTimeGetted = false;
				_state.SetEvent(EVENT_BEHAVIOREND, BEHAVIOR_PATHA);
				printf("[ctl:plan 4]: Roof top reached!!\n");
			}
			firstTimeEnter = false;
		}

		//m_pLoadedTextPath->GetPosVelAcc(0.02, final_pos, final_vel, final_acc);

		QROTOR_REF temp_TransitionFinalRef;
		memset(&temp_TransitionFinalRef, 0, sizeof(QROTOR_REF));
		temp_TransitionFinalRef.p_x_r = final_pos[0];
		temp_TransitionFinalRef.p_y_r = final_pos[1];
		temp_TransitionFinalRef.p_z_r = final_pos[2];
		temp_TransitionFinalRef.psi_r = final_pos[3];

		temp_TransitionFinalRef.v_x_r = final_vel[0];
		temp_TransitionFinalRef.v_y_r = final_vel[1];
		temp_TransitionFinalRef.v_z_r = final_vel[2];

		temp_TransitionFinalRef.agx_r = final_acc[0];
		temp_TransitionFinalRef.agy_r = final_acc[1];
		temp_TransitionFinalRef.agz_r = final_acc[2];

		static double pathTotalTime;

		if (!m_bSAFMCPathTotalTimeGetted){
			printf("[ctl:Transition-1] final_pos: %.2f %.2f %.2f\n", final_pos[0], final_pos[1], final_pos[2]);
//			m_ReflexxesInitialized = false;
			pathTotalTime = ReflexxesPathPlanning(_state.GetState(), temp_TransitionFinalRef, outerRefPos, outerRefVel, outerRefAcc);
			//printf("[ctl:Transition-1] Pos: %.2f %.2f %.2f %.2f; Vel: %.2f %.2f %.2f\n", final_pos[0], final_pos[1], final_pos[2], final_pos[3], final_vel[0], final_vel[1], final_vel[2]);
			m_bSAFMCPathTotalTimeGetted = true;
		}

		ReflexxesPathPlanning(_state.GetState(), temp_TransitionFinalRef, outerRefPos, outerRefVel, outerRefAcc);

		if(m_nCount%100 == 0){
			printf("[ctl:Transition-1 Path] totalPathTime %.2f; pnr: %.2f %.2f %.2f %.2f; vnr: %.2f %.2f %.2f\n", pathTotalTime, outerRefPos[0], outerRefPos[1], outerRefPos[2], outerRefPos[3], outerRefVel[0], outerRefVel[1], outerRefVel[2]);
		}
}
void clsCTL::ConstructPath1Ref(double outerRefPos[4], double outerRefVel[3], double outerRefAcc[3]){
	double tStart = _ctl.GetPathStartTime();
	double tElapse = ::GetTime() - tStart;

	if (tElapse < m_pLoadedTextPath->GetEndTime()){
		m_pLoadedTextPath->GetPosVelAcc(tElapse, outerRefPos, outerRefVel, outerRefAcc);
	}
	else{
		B5_t2 = -1;
		_state.SetEvent(EVENT_BEHAVIOREND, BEHAVIOR_PATHA);
	}

	static int count = 0;
	count++;
	if (count > 1e7) count = 0;
	if(count%50 == 0){
		printf("[ctl:Path-1] pnr: %.2f %.2f %.2f %.2f; vnr: %.2f %.2f %.2f\n", outerRefPos[0], outerRefPos[1], outerRefPos[2], outerRefPos[3], outerRefVel[0], outerRefVel[1], outerRefVel[2]);
	}
}

void clsCTL::ConstructVisionInitializationPathref(double outerRefPos[4], double outerRefVel[3], double outerRefAcc[3]){
	///calculate the searching area waypoint
	///depends on the search radius and step size;
	int stepSize = 7; // 7 meters
	int totalNumWayPoint = 9;
	double wayPoint[18] = {
			0, 0,
			stepSize, 0,
			0, stepSize,
			-1*stepSize, 0,
			-1*stepSize, 0,
			0, -1*stepSize,
			0, -1*stepSize,
			stepSize, 0,
			stepSize, 0
	};

	/// Generate the path references for the search path;
	static int passedWayPointCount = 0;
	static bool local_QROTOR_REF_initialized = false;
	static QROTOR_REF temp_SearchPathRef;
	static double tStartHover = 0;
	static bool local_tStartInitialized = false;

	if (passedWayPointCount < totalNumWayPoint){
		if (!local_QROTOR_REF_initialized){
			//Limit the max velocity and acceleration;
			IP->MaxVelocityVector->VecData			[0]	=	 2;
			IP->MaxVelocityVector->VecData			[1]	=	 2;
			IP->MaxAccelerationVector->VecData		[0]	=	 0.5;
			IP->MaxAccelerationVector->VecData		[1]	=	 0.5;

			memset(&temp_SearchPathRef, 0, sizeof(QROTOR_REF));
			temp_SearchPathRef.p_x_r = outerRefPos[0] + wayPoint[passedWayPointCount*2];
			temp_SearchPathRef.p_y_r = outerRefPos[1] + wayPoint[passedWayPointCount*2+1];
			temp_SearchPathRef.p_z_r = outerRefPos[2];
			temp_SearchPathRef.psi_r = outerRefPos[3];
			local_QROTOR_REF_initialized = true;
		}
		if (ReflexxesPathPlanning(_state.GetState(), temp_SearchPathRef, outerRefPos, outerRefVel, outerRefAcc) < 0.2){
			/// Arrive at the waypoint, hover there for 30 s
			if (!local_tStartInitialized){
				local_tStartInitialized = true;
				tStartHover = ::GetTime();
			}

			if (::GetTime() - tStartHover > 30){
				local_QROTOR_REF_initialized = false;
				passedWayPointCount++;
				local_tStartInitialized = false;
			}
		}
	}
	// End of the reference generation

	///If can see target clearly during searching, GOTO->Guidance mode directly;
	static int local_targetDetectedCount = 0;
	if (_cam.GetVisionTargetInfo().flags[0]){ // flag[0] --> target detected (true);
		local_targetDetectedCount++;
		printf("[visionInitialization] local_targetDetectedCount: %d\n", local_targetDetectedCount);
	}
	else{
		local_targetDetectedCount = 0;
	}

	//printf("_cam.GetVisionTargetInfo().flags[0]: %d\n", _cam.GetVisionTargetInfo().flags[0]);

	if (local_targetDetectedCount > 10){
		//vision has a good viewPoint, can detect the target stably;
		//Finish the vision initialization phase, enter the vision guidance phase;
		B5_t2 = -1;
		_state.SetEvent(EVENT_BEHAVIOREND, BEHAVIOR_PATHA);
		printf("[visionInitialization] Good viewPoint, enter vision guidance\n");
	}
	// End;

	/// If still cannot detect the target after searching path, return home immediately;
	if (passedWayPointCount == totalNumWayPoint && !local_tStartInitialized){
			/* If still cannot detect the target after
			 * the searching path, then return home;*/
			_im9.SetDropSAFMC2014Target();
			_ctl.SetSAFMCTargetDropped();
			B5_t2 = -1;
			_state.SetEvent(EVENT_BEHAVIOREND, BEHAVIOR_PATHA);
			printf("[VisionInitialization] Search path finish, didnt see the target, return home now\n");
	}
	// End;
}

void clsCTL::ConstructVisionGuidancePathRef(double outerRefPos[4], double outerRefVel[3], double outerRefAcc[3]){
	double local_outerXYRefPos[4];	double local_outerXYRefVel[3]; double local_outerXYRefAcc[3];
	memcpy(local_outerXYRefPos, outerRefPos, sizeof(double)*4);
	memcpy(local_outerXYRefVel, outerRefVel, sizeof(double)*3);
	memcpy(local_outerXYRefAcc, outerRefAcc, sizeof(double)*3);

	static QROTOR_REF local_visionGuidanceXYRef;
	static bool local_QROTOR_REF_initialized = false;
	static bool local_startLandingFlag = false;

	if (!local_QROTOR_REF_initialized){
		local_QROTOR_REF_initialized = true;
		IP->MaxVelocityVector->VecData			[0]	=	 2;
		IP->MaxVelocityVector->VecData			[1]	=	 2;
		IP->MaxVelocityVector->VecData			[2]	=	 0.3;
		IP->MaxAccelerationVector->VecData		[0]	=	 0.7;
		IP->MaxAccelerationVector->VecData		[1]	=	 0.7;
		IP->MaxAccelerationVector->VecData		[2]	=	 0.5;

		memset(&local_visionGuidanceXYRef, 0, sizeof(QROTOR_REF));

		// Initialize the horizontal direction;
		local_visionGuidanceXYRef.p_x_r = outerRefPos[0];
		local_visionGuidanceXYRef.p_y_r = outerRefPos[1];
		local_visionGuidanceXYRef.p_z_r = outerRefPos[2];
		local_visionGuidanceXYRef.psi_r = outerRefPos[3];
	}
	// Vision guidance phase, guide the UAV to the target area;
	if (_cam.GetVisionTargetInfo().flags[0] && m_nCount%10 == 0){
		local_visionGuidanceXYRef.p_x_r = outerRefPos[0] + _cam.GetVisionTargetInfo().nedFrame_dvec[0];
		local_visionGuidanceXYRef.p_y_r = outerRefPos[1] + _cam.GetVisionTargetInfo().nedFrame_dvec[1];
		if (sqrt(pow(_cam.GetVisionTargetInfo().nedFrame_dvec[0], 2) + pow(_cam.GetVisionTargetInfo().nedFrame_dvec[1], 2)) < 2){
			local_startLandingFlag = true;
		}
	}

	local_outerXYRefPos[2] = local_visionGuidanceXYRef.p_z_r;
	ReflexxesPathPlanning(_state.GetState(), local_visionGuidanceXYRef, local_outerXYRefPos, local_outerXYRefVel, local_outerXYRefAcc);

//	if(m_nCount%50 == 0){
//			printf("[visionGuidance] local_outerXYRefPos: %.2f %.2f %.2f %.2f; B5_vnr: %.2f %.2f %.2f\n", local_outerXYRefPos[0], local_outerXYRefPos[1], local_outerXYRefPos[2], local_outerXYRefPos[3], local_outerXYRefVel[0], local_outerXYRefVel[1], local_outerXYRefVel[2]);
//	}

	outerRefPos[0] = local_outerXYRefPos[0]; outerRefPos[1] = local_outerXYRefPos[1]; outerRefPos[3] = local_outerXYRefPos[3];
	outerRefVel[0] = local_outerXYRefVel[0]; outerRefVel[1] = local_outerXYRefVel[1];
	outerRefAcc[0] = local_outerXYRefAcc[0]; outerRefAcc[1] = local_outerXYRefAcc[1];

	/// Generate references for z direction;
	if (local_startLandingFlag){
		static bool local_startTimeInitialized = false;
		static double tStart = 0;
		if (!local_startTimeInitialized){
			local_startTimeInitialized = true;
			tStart = ::GetTime();
		}
		double tElapse = ::GetTime() - tStart;
		double velDown = 0.5;
		double accDown = 0.25;
		double velSlow = 0.2;
		double tRamp = velDown / accDown;
		double dt = 0.02;

		if( fabs(_state.GetState().z) > 5){ // >5m
			outerRefAcc[2] = 0;
			if(tElapse < tRamp){
				outerRefAcc[2] = accDown;
				outerRefVel[2] += outerRefAcc[2]*dt;
				outerRefPos[2] += outerRefVel[2]*dt;
			}
			else{
				outerRefVel[2] += outerRefAcc[2]*dt;
				outerRefPos[2] += outerRefVel[2]*dt;
			}
		}
		else if( fabs(_state.GetState().z) < 5  ){ //8cm - 5m, decrease to 0.2m/s
			outerRefAcc[2] = 0;
			if(outerRefVel[2] > velSlow){
				outerRefAcc[2] = -accDown;
				outerRefVel[2] += outerRefAcc[2]*dt;
				outerRefPos[2] += outerRefVel[2]*dt;
			}
			else{
				outerRefVel[2] = velSlow;
				outerRefPos[2] += outerRefVel[2]*dt;
			}
		}
	}
	/// end
	if(m_nCount%50 == 0){
		printf("[visionGuidance] B5_pnr: %.2f %.2f %.2f %.2f; B5_vnr: %.2f %.2f %.2f\n", outerRefPos[0], outerRefPos[1], outerRefPos[2], outerRefPos[3], outerRefVel[0], outerRefVel[1], outerRefVel[2]);
	}
	// finish the vision guidance phase;

	if ( outerRefPos[2] > 0.5 ){
		//UAV is in the target area, can drop the payload, then finish the vision guidance phase;
		_im9.SetDropSAFMC2014Target();
		_ctl.SetSAFMCTargetDropped();
		B5_t2 = -1;
		m_bSAFMCPathTotalTimeGetted = false;
		_state.SetEvent(EVENT_BEHAVIOREND, BEHAVIOR_PATHA);
		printf("[visionGuidance] landing finished\n");
	}
}

void clsCTL::ConstructTransition2PathRef(double outerRefPos[4], double outerRefVel[3], double outerRefAcc[3]){
			double tStart = _ctl.GetPathStartTime();
			double tElapse = ::GetTime() - tStart;

			static double final_pos[4] = {0};
			static double final_vel[3] = {0};
			static double final_acc[3] = {0};
			static bool bInitializedTransitPosition = false;
//			m_pLoadedTextPath->GetPosVelAcc(0.02, final_pos, final_vel, final_acc);
			if (!bInitializedTransitPosition){
				IP->MaxVelocityVector->VecData			[0]	=	 2;
				IP->MaxVelocityVector->VecData			[1]	=	 2;
				IP->MaxVelocityVector->VecData			[2]	=	 1.5;
				IP->MaxAccelerationVector->VecData		[0]	=	 0.7;
				IP->MaxAccelerationVector->VecData		[1]	=	 0.7;
				IP->MaxAccelerationVector->VecData		[2]	=	 0.5;
				IP->MaxJerkVector->VecData				[0]	=	 1.0		;
				IP->MaxJerkVector->VecData				[1]	=	 1.0		;
				IP->MaxJerkVector->VecData				[2]	=	 1.0		;

				final_pos[0] = outerRefPos[0];
				final_pos[1] = outerRefPos[1];
				final_pos[2] = -10;
				final_pos[3] = outerRefPos[3];

				final_vel[0] = 0; final_vel[1] = 0; final_vel[2] = 0;
				final_acc[0] = 0; final_acc[1] = 0; final_acc[2] = 0;
				bInitializedTransitPosition = true;
			}
			static bool b2ndInitialized = false;
			if (!b2ndInitialized && outerRefPos[2] < -3){
				final_pos[0] = 0;
				final_pos[1] = 0;
				b2ndInitialized = true;
			}

			QROTOR_REF temp_TransitionFinalRef;
			memset(&temp_TransitionFinalRef, 0, sizeof(QROTOR_REF));
			temp_TransitionFinalRef.p_x_r = final_pos[0];
			temp_TransitionFinalRef.p_y_r = final_pos[1];
			temp_TransitionFinalRef.p_z_r = final_pos[2];
			temp_TransitionFinalRef.psi_r = final_pos[3];

			temp_TransitionFinalRef.v_x_r = final_vel[0];
			temp_TransitionFinalRef.v_y_r = final_vel[1];
			temp_TransitionFinalRef.v_z_r = final_vel[2];

			temp_TransitionFinalRef.agx_r = final_acc[0];
			temp_TransitionFinalRef.agy_r = final_acc[1];
			temp_TransitionFinalRef.agz_r = final_acc[2];

			static double pathTotalTime = 0;
			static bool bTotalPathTimeGetted = false;

//			if (!bTotalPathTimeGetted){
//				m_ReflexxesInitialized = false;
//				pathTotalTime = ReflexxesPathPlanning(_state.GetState(), temp_TransitionFinalRef, outerRefPos, outerRefVel, outerRefAcc);
//				bTotalPathTimeGetted = true;
//				printf("[ctl:Transition2] pathTotalTime: %.2f t: %.2f\n", pathTotalTime, tElapse);
//			}
//			ReflexxesPathPlanning(_state.GetState(), temp_TransitionFinalRef, outerRefPos, outerRefVel, outerRefAcc);

			if(m_nCount%100 == 0){
				printf("[ctl:Transition-2] totalPathTime %.2f; t: %.2f; pnr: %.2f %.2f %.2f %.2f; vnr: %.2f %.2f %.2f\n", pathTotalTime, tElapse, outerRefPos[0], outerRefPos[1], outerRefPos[2], outerRefPos[3], outerRefVel[0], outerRefVel[1], outerRefVel[2]);
			}
			if (ReflexxesPathPlanning(_state.GetState(), temp_TransitionFinalRef, outerRefPos, outerRefVel, outerRefAcc) < 0.2){
				B5_t2 = -1;
				m_bSAFMCPathTotalTimeGetted = false;
				_state.SetEvent(EVENT_BEHAVIOREND, BEHAVIOR_PATHA);
			}
}

void clsCTL::ConstructPath2Ref(double outerRefPos[4], double outerRefVel[3], double outerRefAcc[3]){
		double tStart = _ctl.GetPathStartTime();
		double tElapse = ::GetTime() - tStart;

		if (tElapse < m_pLoadedTextPath->GetEndTime()){
			m_pLoadedTextPath->GetPosVelAcc(tElapse, outerRefPos, outerRefVel, outerRefAcc);
		}
		else{
			B5_t2 = -1;
			_state.SetEvent(EVENT_BEHAVIOREND, BEHAVIOR_PATHA);
		}

		static int count = 0;
		count++;
		if (count > 1e7) count = 0;
		if(count%50 == 0){
			printf("[ctl:Path-2] pnr: %.2f %.2f %.2f %.2f; vnr: %.2f %.2f %.2f\n", outerRefPos[0], outerRefPos[1], outerRefPos[2], outerRefPos[3], outerRefVel[0], outerRefVel[1], outerRefVel[2]);
		}
}

void clsCTL::ConstructLandingPath(UAVSTATE state, double pnr[4], double vnr[3], double anr[3]){
		double tStart = _ctl.GetPathStartTime();
		double tElapse = ::GetTime() - tStart;

		QROTOR_REF temp_LandingFinalRef;
		memset(&temp_LandingFinalRef, 0, sizeof(QROTOR_REF));
		temp_LandingFinalRef.p_x_r = B5_pnr[0];
		temp_LandingFinalRef.p_y_r = B5_pnr[1];
		temp_LandingFinalRef.p_z_r = 2;
		temp_LandingFinalRef.psi_r = B5_pnr[3];

		static double pathTotalTime;

		if (!m_bSAFMCPathTotalTimeGetted){
			IP->MaxVelocityVector->VecData			[0]	=	 2;
			IP->MaxVelocityVector->VecData			[1]	=	 2;
			IP->MaxVelocityVector->VecData			[2]	=	 0.4;
			IP->MaxAccelerationVector->VecData		[0]	=	 0.7;
			IP->MaxAccelerationVector->VecData		[1]	=	 0.7;
			IP->MaxAccelerationVector->VecData		[2]	=	 0.5;
			printf("[ctl:Landing] state: %.2f %.2f %.2f %.2f %.2f %.2f\n", state.x, state.y, state.z, state.ug, state.vg, state.wg);
			pathTotalTime = ReflexxesPathPlanning(state, temp_LandingFinalRef, pnr, vnr, anr);
			m_bSAFMCPathTotalTimeGetted = true;
		}

		ReflexxesPathPlanning(state, temp_LandingFinalRef, pnr, vnr, anr);

		static int count = 0;
		count++;
		if (count > 1e7) count = 0;

		double local_cutEnginHeight = 0;
		if (_cmm.GetViconFlag()) local_cutEnginHeight = LANDING_CUTENGINE_HEIGHT_VICON;
				else local_cutEnginHeight = LANDING_CUTENGINE_HEIGHT_OUTDOOR;

		if(count%100 == 0){
				//printf("[ctl:landing] local_cutEnginHeight: %.2f\n", local_cutEnginHeight);
				printf("[ctl:Landing] tElapse %.2f; totalPathTime %.2f; pnr: %.2f %.2f %.2f %.2f; vnr: %.2f %.2f %.2f\n", tElapse, pathTotalTime, pnr[0], pnr[1], pnr[2], pnr[3], vnr[0], vnr[1], vnr[2]);
		}

		if ( pnr[2] > temp_LandingFinalRef.p_z_r - 0.5){
				B5_t2 = -1;
				m_bSAFMCPathTotalTimeGetted = false;
				m_behavior.behavior = BEHAVIOR_ENGINEDOWN;
				_ctl.ResetPathSmooth();
				printf("[ctl:Landing] Landing finished!\n");
		}
}

void clsCTL::ConstructLandingPath2(UAVSTATE state, double pnr[4], double vnr[3], double anr[3]){
		double tStart = _ctl.GetPathStartTime();
		double tElapse = ::GetTime() - tStart;

		QROTOR_REF temp_LandingFinalRef;
		memset(&temp_LandingFinalRef, 0, sizeof(QROTOR_REF));
		temp_LandingFinalRef.p_x_r = B5_pnr[0];
		temp_LandingFinalRef.p_y_r = B5_pnr[1];
		temp_LandingFinalRef.p_z_r = 1;
		temp_LandingFinalRef.psi_r = B5_pnr[3];

		static double pathTotalTime;

		if (!m_bSAFMCPathTotalTimeGetted){
			printf("[ctl:LandingTmp] B5_pnr: %.2f %.2f %.2f \n", B5_pnr[0], B5_pnr[1], B5_pnr[2]);
			m_ReflexxesInitialized = false;
			pathTotalTime = ReflexxesPathPlanning(state, temp_LandingFinalRef, pnr, vnr, anr);
			m_bSAFMCPathTotalTimeGetted = true;
		}

		ReflexxesPathPlanning(state, temp_LandingFinalRef, pnr, vnr, anr);

		static int count = 0;
		count++;
		if (count > 1e7) count = 0;

		double local_cutEnginHeight = 0;
		if (_cmm.GetViconFlag()) local_cutEnginHeight = LANDING_CUTENGINE_HEIGHT_VICON;
		else local_cutEnginHeight = LANDING_CUTENGINE_HEIGHT_OUTDOOR;

		if(count%50 == 0){
			printf("[ctl:landing] local_cutEnginHeight: %.2f\n", local_cutEnginHeight);
			printf("[ctl:LandingTmp] tElapse %.2f; totalPathTime %.2f; pnr: %.2f %.2f %.2f %.2f; vnr: %.2f %.2f %.2f\n", tElapse, pathTotalTime, pnr[0], pnr[1], pnr[2], pnr[3], vnr[0], vnr[1], vnr[2]);
		}

		if ( state.z > local_cutEnginHeight){
			B5_t2 = -1;
			m_bSAFMCPathTotalTimeGetted = false;
			m_behavior.behavior = BEHAVIOR_ENGINEDOWN;
			_ctl.ResetPathSmooth();
		}
}

/* End <-- Implementations of defined functions for 2014 SAFMC by Liu Peidong, 2014-Mar-05 */
double clsCTL::ReflexxesPathPlanning(UAVSTATE state, QROTOR_REF ref, double pnr[4], double vnr[3], double anr[3])
{
	if ( !m_ReflexxesInitialized ) {
	    IP->CurrentPositionVector->VecData		[0]	= state.x;
	    IP->CurrentPositionVector->VecData		[1]	= state.y;
	    IP->CurrentPositionVector->VecData		[2]	= state.z;
        IP->CurrentPositionVector->VecData      [3] = state.c;

	    IP->CurrentVelocityVector->VecData		[0]	= state.ug	;
	    IP->CurrentVelocityVector->VecData		[1]	= state.vg	;
	    IP->CurrentVelocityVector->VecData		[2]	= state.wg	;
	    IP->CurrentVelocityVector->VecData      [3] = state.r    ;

	    IP->CurrentAccelerationVector->VecData	[0]	= 0.0		;
	    IP->CurrentAccelerationVector->VecData	[1]	= 0.0		;
	    IP->CurrentAccelerationVector->VecData	[2]	= 0.0		;
	    IP->CurrentAccelerationVector->VecData  [3] = 0.0        ;

	    m_ReflexxesInitialized = true;
	}


    IP->TargetPositionVector->VecData		[0]	=	ref.p_x_r		;
    IP->TargetPositionVector->VecData		[1]	=	ref.p_y_r		;
    IP->TargetPositionVector->VecData		[2]	=	ref.p_z_r		;
    IP->TargetPositionVector->VecData       [3] =   ref.psi_r       ;

    IP->TargetVelocityVector->VecData		[0]	= 0.0		;
    IP->TargetVelocityVector->VecData		[1]	= 0.0		;
    IP->TargetVelocityVector->VecData		[2]	= 0.0		;
    IP->TargetVelocityVector->VecData       [3] = 0.0       ;

    // Calling the Reflexxes OTG algorithm
    int ResultValue	= RML->RMLPosition(*IP, OP, Flags);
    if (ResultValue < 0)
    {
        printf("An error occurred (%d).\n", ResultValue	);
        printf("%s\n", OP->GetErrorString());
    }

    if (ResultValue == 1) // Target position reached;
    {
    	pnr[0] = ref.p_x_r; pnr[1] = ref.p_y_r; pnr[2] = ref.p_z_r; pnr[3] = ref.psi_r;
    	vnr[0] = ref.v_x_r; vnr[1] = ref.v_y_r; vnr[2] = ref.v_z_r;
    	anr[0] = ref.agx_r; anr[1] = ref.agy_r; anr[2] = ref.agz_r;
    	return OP->SynchronizationTime;
    }

//    printf("New position/pose vector                  : ");
    for (int j = 0; j < NUMBER_OF_DOFS; j++)
    {
//        printf("%10.3lf ", OP->NewPositionVector->VecData[j]);
        pnr[j] = OP->NewPositionVector->VecData[j];
    }
//    printf("\n");
//    printf("New velocity vector                       : ");
    for (int j = 0; j < NUMBER_OF_DOFS - 1; j++)
    {
//        printf("%10.3lf ", OP->NewVelocityVector->VecData[j]);
        vnr[j] = OP->NewVelocityVector->VecData[j];
    }
//    printf("\n");
//    printf("New acceleration vector                   : ");
    for (int j = 0; j < NUMBER_OF_DOFS - 1; j++)
    {
//        printf("%10.3lf ", OP->NewAccelerationVector->VecData[j]);
        anr[j] = OP->NewAccelerationVector->VecData[j];
    }

    // Feed the output values of the current control cycle back to
    // input values of the next control cycle

    *IP->CurrentPositionVector		=	*OP->NewPositionVector		;
    *IP->CurrentVelocityVector		=	*OP->NewVelocityVector		;
    *IP->CurrentAccelerationVector	=	*OP->NewAccelerationVector	;

    return OP->SynchronizationTime;
}
