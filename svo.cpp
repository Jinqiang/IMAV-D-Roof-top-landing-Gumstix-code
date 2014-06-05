//svo.cpp
//implementation file for servo related class & functions

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <iostream.h>

#include "matrix.h"

#include "uav.h"
#include "svo.h"
#include "state.h"
#include "ctl.h"

extern clsState _state;
extern EQUILIBRIUM _equ_Hover;
extern clsIM9	_im9;

BOOL clsSVO::InitThread()
{
	if (_HELICOPTER == ID_HELION || _HELICOPTER == ID_SHELION || _HELICOPTER == ID_QUADLION) {
	    m_nsSVO = open("/dev/ser2", O_RDWR);
	    if (m_nsSVO == -1) { printf("[SVO] Open servo serial port (/dev/ser2) failed.\n"); return FALSE; }

	    termios termSVO;
	    tcgetattr(m_nsSVO, &termSVO);

		cfsetispeed(&termSVO, SVO_BAUDRATE);				//input and output baudrate
		cfsetospeed(&termSVO, SVO_BAUDRATE);

		termSVO.c_cflag = CS8 | CLOCAL | CREAD;
	//	termSVO.c_iflag = IGNBRK | IGNCR | IGNPAR;

		tcsetattr(m_nsSVO, TCSANOW, &termSVO);
		tcflush(m_nsSVO, TCIOFLUSH);
		printf("[SVO] UAV100 Start\n");
	}

	if (_HELICOPTER == ID_FEILION) {
	    m_trim_aileron 		= 2983; // + means fly left
	    m_trim_elevator 	= 3004; // + means fly backward
	    m_trim_throttle 	= 3339; // + means fly upward
	    m_trim_rudder 		= 2944; // + means turn left
	    m_trim_auxiliary 	= 3000; // not used

		printf("[SVO] On FeiLion Start\n");
		return TRUE;
	}

	usleep(10000);				//wait for 10 ms

	if (_HELICOPTER == ID_GREMLION || _HELICOPTER == ID_QUADLION) {	// for GremLion
		WriteCommand1();
	}
	else if (_HELICOPTER == ID_HELION || _HELICOPTER == ID_SHELION) {	// for HeLion, SheLion
		WriteCommand();
	}

	usleep(10000);				//wait for 10 ms

	//init variables
	m_tSVO0 = -1;
	m_nSVO = 0;
	m_tTrimvalue = -1;
	m_trimT0 = -1;

//	::memcpy(&m_svo0, 0, sizeof(m_svo0));
//	::memcpy(&m_svo1, 0, sizeof(m_svo1));

	m_bManualTrimFlag = FALSE;

	m_avgTrimvalue.aileron = m_svoEqu.aileron = _equ_Hover.ea;
	m_avgTrimvalue.elevator = m_svoEqu.elevator = _equ_Hover.ee;
	m_avgTrimvalue.auxiliary =  m_svoEqu.auxiliary = _equ_Hover.eu;
	m_avgTrimvalue.rudder = m_svoEqu.rudder = _equ_Hover.er;
	m_avgTrimvalue.throttle = m_svoEqu.throttle = _equ_Hover.et;

	m_svoSet = m_svoEqu;

	m_svoManualTrimRaw.aileron = m_svoManualTrimRaw.elevator = m_svoManualTrimRaw.auxiliary = m_svoManualTrimRaw.rudder = m_svoManualTrimRaw.throttle = 15000;

	m_limit = 0.02;
	/* below for the online trimvalue updating */
	m_bTrimvalue = FALSE;
	m_nTrimCount = 0;


	char devPort[] = "\/dev\/ser2";
	SetSvoConfig(devPort, sizeof(devPort), 1, SVO_BAUDRATE);

	printf("[SVO] Start\n");

	return TRUE;
}

void clsSVO::PutCommand(COMMAND *pCmd)
{
	pthread_mutex_lock(&m_mtxCmd);
	m_cmd = *pCmd;
	pthread_mutex_unlock(&m_mtxCmd);
}

void clsSVO::GetCommand(COMMAND *pCmd)
{
	pthread_mutex_lock(&m_mtxCmd);

	if(m_cmd.code == 0)
		pCmd->code = 0;
	else
	{
		*pCmd = m_cmd;
		m_cmd.code = 0;
	}

	pthread_mutex_unlock(&m_mtxCmd);
}

void clsSVO::SetTrimvalue()
{
	m_svoEqu.aileron = 0.5*_equ_Hover.ea + 0.5*m_avgTrimvalue.aileron;
	m_svoEqu.elevator = 0.5*_equ_Hover.ee + 0.5*m_avgTrimvalue.elevator;
	m_svoEqu.auxiliary = 0.5*_equ_Hover.eu + 0.5*m_avgTrimvalue.auxiliary;
	m_svoEqu.rudder = 0.5*_equ_Hover.er + 0.5*m_avgTrimvalue.rudder;
	m_svoEqu.throttle = 0.5*_equ_Hover.et + 0.5*m_avgTrimvalue.throttle;

	m_svoSet = m_svoEqu;	// put the servo deflections to the new trimvalues

	m_tTrimvalue = ::GetTime();
}

BOOL clsSVO::ProcessCommand(COMMAND *pCmd)
{
	COMMAND &cmd = *pCmd;
//	char *paraCmd = cmd.parameter;
	BOOL bProcess = TRUE;

	switch(cmd.code)
	{
	case COMMAND_GETTRIM:
//		m_bTrimvalue = TRUE;
		/* set all variables concering get trimvalue to zeros */
		m_nTrimCount = 0;
		m_avgTrimvalue.aileron = _equ_Hover.ea;
		m_avgTrimvalue.auxiliary = _equ_Hover.eu;
		m_avgTrimvalue.elevator = _equ_Hover.er;
		m_avgTrimvalue.rudder = _equ_Hover.er;
		m_avgTrimvalue.throttle = _equ_Hover.et;
		break;

	case COMMAND_HOLD:
		m_bTrimvalue = TRUE;
		m_trimT0 = ::GetTime();
		break;

	default:
		bProcess = FALSE;
		break;
	}
	return bProcess;
}

BOOL clsSVO::ValidateTrimvalue()
{
	UAVSTATE &state = _state.GetState();
	BOOL bValid =
		::fabs(state.u) <= THRESHOLDHIGH_U &&
		::fabs(state.v) <= THRESHOLDHIGH_V &&
		::fabs(state.w) <= THRESHOLDHIGH_W &&
		::fabs(state.p) <= THRESHOLDHIGH_P &&
		::fabs(state.q) <= THRESHOLDHIGH_Q &&
		::fabs(state.r) <= THRESHOLDHIGH_R;

	return bValid;
}

int clsSVO::EveryRun()
{
	 double t2 = GetTime();
	/// get autocontrol signal from innerloop
//	printf("[SVO] count %d \n", m_nCount);
	HELICOPTERRUDDER sig;
	_state.GetSIG(&sig);

	if (_HELICOPTER == ID_FEILION) {
		IM9PACK im9pack = _im9.GetIM9Pack();
		m_svo0.aileron = im9pack.ail;
		m_svo0.elevator = im9pack.ele;
		m_svo0.rudder = im9pack.rud;
		m_svo0.throttle = im9pack.thr;
		m_svo0.sv6 = im9pack.tog;

		SetRudder_FeiLion(&sig);

		pthread_mutex_lock(&m_mtxSVO);
		if (m_nSVO != MAX_SVO) {
			m_tSVO[m_nSVO] = ::GetTime();
			m_svoRaw[m_nSVO] = m_svoRaw0;
			m_svo[m_nSVO++] = m_svo0;
		}
		pthread_mutex_unlock(&m_mtxSVO);

		return TRUE;		// finish the svo loop
	}

	/// send to servo board
	int nMode = MODE_NAVIGATION; //_ctl.GetMode();
//	BOOL bThrottleBypass = _ctl.GetThrottleByPassFlag();
	BOOL bThrottleBypass = TRUE;
	if (/*nMode == MODE_SEMIAUTO &&*/ bThrottleBypass) {
		m_svoSet.aileron = sig.aileron;
		m_svoSet.elevator = sig.elevator;
		m_svoSet.auxiliary = sig.auxiliary;
		m_svoSet.rudder = sig.rudder;
		m_svoSet.throttle = sig.throttle;	// bypass throttle manual signal to servo board
		m_svoSet.triger = sig.triger;
		m_svoSet.laser = sig.laser;
	}
	else if (nMode == MODE_SEMIAUTO && !bThrottleBypass) {
		m_svoSet.aileron = sig.aileron;
		m_svoSet.elevator = sig.elevator;
		m_svoSet.auxiliary = sig.auxiliary;
		m_svoSet.rudder = sig.rudder;
		m_svoSet.throttle = sig.throttle;	// semi-auto throttle sig to servo board
		m_svoSet.triger = sig.triger;
		m_svoSet.laser = sig.laser;
	}
	else if (nMode == MODE_ALLAUTO) {
		m_svoSet.aileron = sig.aileron;
		m_svoSet.elevator = sig.elevator;
		m_svoSet.auxiliary = sig.auxiliary;
		m_svoSet.rudder = sig.rudder;
		m_svoSet.throttle = sig.throttle;
		m_svoSet.triger = sig.triger;
		m_svoSet.laser = sig.laser;
	}
	else if (nMode == MODE_NAVIGATION) {
		m_svoSet.aileron = sig.aileron;
		m_svoSet.elevator = sig.elevator;
		m_svoSet.auxiliary = sig.auxiliary;
		m_svoSet.rudder = sig.rudder;
		m_svoSet.throttle = sig.throttle;
		m_svoSet.triger = sig.triger;
		m_svoSet.laser = sig.laser;
	}
/*	if (m_nCount % _DEBUG_COUNT == 0) {
		printf("[SVO] auto data, ea %.3f, ee %.3f, eu %.3f, er %.3f\n",
		m_svoSet.aileron, m_svoSet.elevator, m_svoSet.auxiliary, m_svoSet.rudder);
	}*/



	SetRudder(&m_svoSet);



#ifdef _CAMERA
	double camera = _state.GetCamera();
	SetCamera(camera);
#endif

	/// read manual servo data requested in the last loop
	SVORAWDATA svoraw = {0};
	BOOL bGetData = GetData(&svoraw);

	/// send request for manual servo data
	if (_HELICOPTER == ID_GREMLION || _HELICOPTER == ID_QUADLION) {	// for GremLion
		WriteCommand1();
	}
	else {	// for HeLion, SheLion
		WriteCommand();
	}



	if (!bGetData) return TRUE;
/*	if (m_nCount % _DEBUG_COUNT == 0) {
		printf("[SVO] manual data, ea %d, ee %d, eu %d, er %d, et %d, toggle %d\n",
		svoraw.aileron, svoraw.elevator, svoraw.auxiliary, svoraw.rudder, svoraw.throttle, svoraw.sv6);
	}*/

	/// check manual trim value update function is to be done or not
	SVODATA svo = {0};
	if (_HELICOPTER == ID_GREMLION || _HELICOPTER == ID_QUADLION) {	// for GremLion
		BOOL bManualTrim = GetManualTrimFlag();
		SVORAWDATA &svoRawTrim = GetManualTrimRawData();
		Translate_GremLion(&svoraw, &svo, &svoRawTrim, bManualTrim);
	}
	else {	// for HeLion, SheLion
		Translate_HeLion(&svoraw, &svo);
	}

#if (_DEBUG & DEBUGFLAG_SVO)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[SVO] manual data, ea %.3f, ee %.3f, eu %.3f, er %.3f, et %.3f\n",
	svo.aileron, svo.elevator, svo.auxiliary, svo.rudder, svo.throttle);
}
#endif

	m_tSVO0 = m_tRetrieve;
	m_svoRaw0 = svoraw;

	if (svo.aileron >= -1 && svo.aileron <= 1) {
		m_svo0.aileron = svo.aileron;
	}

	if (svo.elevator >= -1 && svo.elevator <= 1) {
		m_svo0.elevator = svo.elevator;
	}

	if (svo.auxiliary >= -1 && svo.auxiliary <= 1) {
		m_svo0.auxiliary = svo.auxiliary;
	}

	if (svo.rudder >= -1 && svo.rudder <= 1) {
		m_svo0.rudder = svo.rudder;
	}


	if (svo.throttle >= -1 && svo.throttle <= 1) {
		m_svo0.throttle = svo.throttle;
	}

	m_svo0.sv6 = svo.sv6;
//	m_svo0 = svo;

	/// copy into memory servo time, raw data and translated data
	pthread_mutex_lock(&m_mtxSVO);
	if (m_nSVO != MAX_SVO) {
		m_tSVO[m_nSVO] = m_tSVO0;
		m_svoRaw[m_nSVO] = m_svoRaw0;
		m_svo[m_nSVO++] = m_svo0;
	}
	pthread_mutex_unlock(&m_mtxSVO);

	double t3 = GetTime();
  // cout<<"[SVO] time is"<<t3-t2<<endl;
	return TRUE;
}

BOOL clsSVO::CalcNewTrim()
{
	if( ValidateTrimvalue() )
	{
		m_avgTrimvalue.aileron = SVO_WEIGHT1*m_svo0.aileron + (1-SVO_WEIGHT1)*m_avgTrimvalue.aileron;
		m_avgTrimvalue.elevator = SVO_WEIGHT1*m_svo0.elevator + (1-SVO_WEIGHT1)*m_avgTrimvalue.elevator;
		m_avgTrimvalue.auxiliary = SVO_WEIGHT1*m_svo0.auxiliary + (1-SVO_WEIGHT1)*m_avgTrimvalue.auxiliary;
		m_avgTrimvalue.rudder = SVO_WEIGHT1*m_svo0.rudder + (1-SVO_WEIGHT1)*m_avgTrimvalue.rudder;
		m_avgTrimvalue.throttle = SVO_WEIGHT1*m_svo0.throttle + (1-SVO_WEIGHT1)*m_avgTrimvalue.throttle;
		return TRUE;
	}
	return FALSE;
}

void clsSVO::ExitThread()
{
	::close(m_nsSVO);
	printf("[SVO] quit\n");
}

void clsSVO::SetRudder(HELICOPTERRUDDER *pRudder)
{
	HELICOPTERRUDDER rudder = *pRudder;
	rudder.aileron = range(rudder.aileron, -1, 1);
	rudder.elevator = range(rudder.elevator, -1, 1);
	rudder.auxiliary = range(rudder.auxiliary, -0.85, 0.85);
	rudder.rudder = range(rudder.rudder, -0.85, 0.85);
	rudder.throttle = range(rudder.throttle, -0.85, 0.85);

	char szSVOCommand[32]; int nPosition;
	nPosition = 15000+(int)(5000*rudder.aileron);
	sprintf(szSVOCommand, "SV0 M%d\r", nPosition);
	write(m_nsSVO, szSVOCommand, 11);
	sprintf(szSVOCommand, " M%d\r", nPosition);
	write(m_nsSVO, szSVOCommand, 11);

	nPosition = 15000+(int)(5000*rudder.elevator);
//	nPosition = 15000+::sin(0.2*PI*::GetTime())*1000;
	sprintf(szSVOCommand, "SV4 M%d\r", nPosition);
	write(m_nsSVO, szSVOCommand, 11);
	sprintf(szSVOCommand, " M%d\r", nPosition);
	write(m_nsSVO, szSVOCommand, 11);

	nPosition = 15000+(int)(5000*rudder.throttle);
	sprintf(szSVOCommand, "SV2 M%d\r", nPosition);
	write(m_nsSVO, szSVOCommand, 11);
	sprintf(szSVOCommand, " M%d\r", nPosition);
	write(m_nsSVO, szSVOCommand, 11);

	nPosition = 15000+(int)(5000*rudder.rudder);
//	printf("Rudder: %d\n", nPosition);
//	nPosition = 15000+::sin(0.2*PI*::GetTime())*2000;
	sprintf(szSVOCommand, "SV1 M%d\r", nPosition);
	write(m_nsSVO, szSVOCommand, 11);
	sprintf(szSVOCommand, " M%d\r", nPosition);
	write(m_nsSVO, szSVOCommand, 11);

	if(rudder.triger == 1)
	nPosition = 15000+(int)(5000*(1));
	else
    nPosition = 15000+(int)(5000*(-1));//keep it close
//	printf("Rudder: %d\n", nPosition);
//	nPosition = 15000+::sin(0.2*PI*::GetTime())*2000;
	sprintf(szSVOCommand, "SV5 M%d\r", nPosition);
	write(m_nsSVO, szSVOCommand, 11);
	sprintf(szSVOCommand, " M%d\r", nPosition);
	write(m_nsSVO, szSVOCommand, 11);


	nPosition = 15000+(int)(5000*(-rudder.laser));
//	printf("Rudder: %d\n", nPosition);
//	nPosition = 15000+::sin(0.2*PI*::GetTime())*2000;
	sprintf(szSVOCommand, "SV6 M%d\r", nPosition);
	write(m_nsSVO, szSVOCommand, 11);
	sprintf(szSVOCommand, " M%d\r", nPosition);
	write(m_nsSVO, szSVOCommand, 11);
/*	nPosition = 10000+(int)(10000*rudder.throttle);
	sprintf(szSVOCommand, "SV0 M%d\r", nPosition);
	write(m_nsSVO, szSVOCommand, 11);
	sprintf(szSVOCommand, " M%d\r", nPosition);
	write(m_nsSVO, szSVOCommand, 11);*/

}

void clsSVO::SetRudder_FeiLion(HELICOPTERRUDDER *pRudder)
{
	HELICOPTERRUDDER rudder_ = *pRudder;

	int nAilPos = (int)(rudder_.aileron*834);
	int nElePos = (int)(rudder_.elevator*834);
	int nThrPos = (int)(rudder_.throttle*834);
	int nRudPos	= (int)(rudder_.rudder*834);

	// Cap the control inputs to protect the servos and motors
	nAilPos = range(nAilPos, -500, 500);
	nElePos = range(nElePos, -500, 500);
	nThrPos = range(nThrPos, -500, 500);
	nRudPos	= range(nRudPos, -500, 500);

//	nAilPos = m_trim_aileron;
	nAilPos = m_trim_aileron + nAilPos;
//	nAilPos = m_trim_aileron + ::sin(0.2*PI*GetTime())*300;

//	nElePos = m_trim_elevator;
	nElePos = m_trim_elevator + nElePos;
//	nElePos = m_trim_elevator + ::sin(0.2*PI*GetTime())*300;
//	nElePos = m_trim_elevator + floor(::sin(0.5*PI*t))*700;

//	nThrPos = m_trim_throttle;
//	nThrPos = m_trim_throttle + nThrPos;
//	nThrPos = m_trim_throttle + ::sin(0.2*PI*t)*300;
//	nThrPos = 2900 + floor(::sin(0.5*PI*t))*400;
	nThrPos = 600;

	nRudPos = m_trim_rudder;
//	nRudPos = m_trim_rudder + nRudPos;
//	nRudPos = m_trim_rudder + ::sin(0.2*PI*t)*300;

	if (m_nCount % 50 == 0 )
	{
		cout<<"Ail: "<<nAilPos<<endl;
		cout<<"Ele: "<<nElePos<<endl;
		cout<<"Thr: "<<nThrPos<<endl;
		cout<<"Rud: "<<nRudPos<<endl;
		cout<<endl;
	}

	// Send to servo controller
    char ail[6], ele[6], thr[6], rud[6];
    ail[0] = ele[0] = thr[0] = rud[0] = 0x80;
    ail[1] = ele[1] = thr[1] = rud[1] = 0x01;
    ail[2] = ele[2] = thr[2] = rud[2] = 0x04;

    // Channel numbers for FeiLion
    ail[3] = 0x00;
    ele[3] = 0x01;
    thr[3] = 0x02;
    rud[3] = 0x03;

	ail[4] = nAilPos / 128;
	ail[5] = nAilPos % 128;
	ele[4] = nElePos / 128;
	ele[5] = nElePos % 128;
	thr[4] = nThrPos / 128;
	thr[5] = nThrPos % 128;
	rud[4] = nRudPos / 128;
	rud[5] = nRudPos % 128;

	write(_im9.m_nsIM9, ail, 6);
	write(_im9.m_nsIM9, ele, 6);
	write(_im9.m_nsIM9, thr, 6);
	write(_im9.m_nsIM9, rud, 6);
}

void clsSVO::SetCamera(double camera)
{
	char szCmd[32];

	//Assert the camera is installed PI/4 down
	double angle = camera + PI/4;
	angle = range(angle, -PI/4, PI/4);

	int nPosition = 15000 + (int)(5000*angle/(PI/4));
	sprintf(szCmd, "SV5 M%d\r", nPosition);
	write(m_nsSVO, szCmd, 11);
}

void clsSVO::WriteCommand()
{
//	tcflush(m_nsSVO, TCIOFLUSH);

	write(m_nsSVO, "M?9\r", 4);
	write(m_nsSVO, "M?10\r",5);
	write(m_nsSVO, "M?11\r",5);
	write(m_nsSVO, "M?12\r",5);
	write(m_nsSVO, "M?13\r",5);
	write(m_nsSVO, "M?8\r", 4);

	m_tRequest = GetTime();
}

// GremLion setting
void clsSVO::WriteCommand1()
{
//	tcflush(m_nsSVO, TCIOFLUSH);

/*	write(m_nsSVO, "M?10\r",5);
	write(m_nsSVO, "M?8\r",4);
	write(m_nsSVO, "M?11\r",5);
	write(m_nsSVO, "M?9\r",4);
	write(m_nsSVO, "M?13\r",5);
	write(m_nsSVO, "M?14\r",5);*/

	write(m_nsSVO, "M?12\r",5);		// elevator
	write(m_nsSVO, "M?10\r",5);		// throttle
	write(m_nsSVO, "M?8\r",4);		// aileron
	write(m_nsSVO, "M?10\r",5);		// auxiliary
	write(m_nsSVO, "M?9\r",4);		// rudder
	write(m_nsSVO, "M?15\r",5);		// toggle
	m_tRequest = GetTime();
}

BOOL clsSVO::GetData(SVORAWDATA *pData)
{
	char szSVO[256];

	SVORAWDATA svo;

	int nRead = read(m_nsSVO, szSVO, 256);
	szSVO[nRead] = '\0';

	unsigned short check;
	int nScanf = sscanf(szSVO, "%hd%hd%hd%hd%hd%hd%hd",
		&svo.elevator, &svo.throttle, &svo.aileron, &svo.auxiliary, &svo.rudder, &svo.sv6, &check);

	if (nScanf != 6) {
		return FALSE;
	}

	*pData = svo;
	m_tRetrieve = m_tRequest + 0.004;

	return TRUE;
}

clsSVO::clsSVO()
{
	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutex_init(&m_mtxSVO, &attr);
}

clsSVO::~clsSVO()
{
	pthread_mutex_destroy(&m_mtxSVO);
}

void clsSVO::Translate_HeLion(SVORAWDATA *pSVORAW, SVODATA *pSVO)
{
	pSVO->aileron = (double)(pSVORAW->aileron-15000)/5000;		// normalized to -1 ~ 1
	pSVO->elevator = (double)(pSVORAW->elevator-15000)/5000;	// normalized to -1 ~ 1
	pSVO->auxiliary = (double)(pSVORAW->auxiliary-15000)/5000;	// normalized to -1 ~ 1
	pSVO->rudder = (double)(pSVORAW->rudder-15000)/5000;		// normalized to -1 ~ 1
	pSVO->throttle = (double)(pSVORAW->throttle-10000)/10000;	// normalized to  0 ~ 1

	pSVO->sv6 = (double)(pSVORAW->sv6-15000)/5000;
}

void clsSVO::Translate_GremLion(SVORAWDATA *pSVORAW, SVODATA *pSVO)
{
	pSVO->aileron = (double)(pSVORAW->aileron-15290)/5000;		// normalized to -1 ~ 1
	pSVO->elevator = (double)(pSVORAW->elevator-15516)/5000;	// normalized to -1 ~ 1
	pSVO->auxiliary = (double)(pSVORAW->auxiliary-15000)/5000;	// normalized to -1 ~ 1
	pSVO->rudder = (double)(pSVORAW->rudder-14685)/5000;		// normalized to -1 ~ 1
	pSVO->throttle = (double)(pSVORAW->throttle-15000)/5000;	// normalized to -1 ~ 1
	pSVO->sv6 = (double)(pSVORAW->sv6-15000)/5000;
}

void clsSVO::Translate_GremLion(SVORAWDATA *pSVORAW, SVODATA *pSVO, SVORAWDATA *pSVOTrim, BOOL bManualTrim)
{
	if ( bManualTrim ) {
		pSVO->aileron = (double)(pSVORAW->aileron-pSVOTrim->aileron)/5000;		// normalized to -1 ~ 1
		pSVO->elevator = (double)(pSVORAW->elevator-pSVOTrim->elevator)/5000;		// normalized to -1 ~ 1
		pSVO->auxiliary = (double)(pSVORAW->auxiliary-15000)/5000;	// normalized to -1 ~ 1
		pSVO->rudder = (double)(pSVORAW->rudder-pSVOTrim->rudder)/5000;			// normalized to -1 ~ 1
		pSVO->throttle = (double)(pSVORAW->throttle- 15000 /*pSVOTrim->throttle*/)/5000;		// normalized to -1 ~ 1
		pSVO->sv6 = (double)(pSVORAW->sv6-15000)/5000;
	} else {
		pSVO->aileron = (double)(pSVORAW->aileron-15000)/5000;		// normalized to -1 ~ 1
		pSVO->elevator = (double)(pSVORAW->elevator-15000)/5000;		// normalized to -1 ~ 1
		pSVO->auxiliary = (double)(pSVORAW->auxiliary-15000)/5000;	// normalized to -1 ~ 1
		pSVO->rudder = (double)(pSVORAW->rudder-15000)/5000;			// normalized to -1 ~ 1
		pSVO->throttle = (double)(pSVORAW->throttle-15000)/5000;		// normalized to -1 ~ 1
		pSVO->sv6 = (double)(pSVORAW->sv6-15000)/5000;
	}
}

void clsSVO::SetManualTrimRawData(SVORAWDATA svoraw)
{
	m_svoManualTrimRaw.aileron = svoraw.aileron;
	m_svoManualTrimRaw.elevator = svoraw.elevator;
	m_svoManualTrimRaw.auxiliary = svoraw.auxiliary;
	m_svoManualTrimRaw.rudder = svoraw.rudder;
	m_svoManualTrimRaw.throttle = svoraw.throttle;

}

void clsSVO::SetSvoConfig(char *devPort, short size, short flag, int baudrate)
{
	memcpy(m_svoConfig.devPort, devPort, size);
	m_svoConfig.flag = flag;
	m_svoConfig.baudrate = baudrate;
}
