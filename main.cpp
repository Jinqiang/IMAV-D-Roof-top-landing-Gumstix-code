/*
 * main.cpp
 *  Created on: Mar 15, 2011
 *
 *  Implement file for clsMain, managing task threads
 */

#include <stdio.h>
#include <pthread.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "uav.h"
#include "cmm.h"
#include "im6.h"
#include "im7.h"
#include "im8.h"
#include "im9.h"
#include "laser.h"
#include "daq.h"
#include "cam.h"
#include "ctl.h"
#include "svo.h"
#include "dlg.h"
#include "net.h"
#include "coop.h"
#include "user.h"
#include "parser.h"
#include "state.h"
#include "ptu.h"

extern clsParser 	_parser;
extern clsCMM 		_cmm;
extern clsState 	_state;
extern clsURG		_urg;
extern clsPTU 		_ptu;
extern EQUILIBRIUM 	_equ_Hover;

////////////////////////////
const char *_configfile[] = { "quadlion.txt", NULL };

int main(int argc, char *argv[])
{
     printf("[main] begin\n");

    //record the starting time
	_cycle0 = ClockCycles();

	//query working directory
    char path[256];
    ::getcwd(path, sizeof(path));
    printf("[main] current work directory: %s\n", path);

    // change to working directory
    chdir("/fs/sd");
    //load configuration from any available configuration file first
	int i=0;
	while (_configfile[i] != NULL) {
		if (_parser.Load(_configfile[i])) {
			printf("[main] Load configuration file - %s\n", _configfile[i]);
			break;
		}
		i++;
	}

	if (_configfile[i] == NULL) {
		printf("[main] No configuration file found, program quit!\n");
		return -1;
	}

	_parser.Parse2();
	_main.Init();

	printf("[main] Helicopter %d\n", _HELICOPTER);

	//launch keyboard input program
    _user.StartUserThread(PRIORITY_USER);
    // lauch UART listening (from vision computer)
	_cam.StartInputThread(PRIORITY_CAM);
	
	//open and launch communication & input through wireless modem (RS232)
	if (!_cmm.Open()) {
		printf("[main] CMM open failed\n");
	}
	else if (!_cmm.StartInputThread(PRIORITY_CMM)) {
		printf("[main] CMM start input thread failed\n");
	}

	if (!_cmm.InitSocket()) {
		printf("[main] CMM open socket failed\n");
	}
	else if (!_cmm.StartListenThread(PRIORITY_CMM)) {
		printf("[main] CMM start listen failed\n");
	}

	_ctl.Init();
	_state.Init();
	_cam.Init();

	sched_param param;
	param.sched_priority = 10;
	pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

	//loading pathes
	for (i=0; i<=MAX_PATH-1; i++)
	{
		char szFile[64];
		sprintf(szFile, "%d.txt", i+1);
		if (!_path[i].LoadPath(szFile)) break;
	}
	_nPath = i;

    //loading data for simulation
	if (_state.LoadData("a.dat"))
	{
		::printf("[main] Data loaded from 'a.dat'\n");
	}
	else
	{
		::printf("[main] Load data 'a.dat' failed\n");
	}

	/*
	for (int i=0; i<10; i++) {
		_cmm.SendRawMessage("Test information");
//		_cmm.SendMessage("Test information");

	}
	*/
	_cmm.SendMessage("System started, type \"run\" to launch program, \"quit\" to exit");

	while (1) {
		COMMAND cmd;
		_user.GetCommand(&cmd);
		if (cmd.code == 0) _cmm.GetCommand(&cmd);

		if (cmd.code == COMMAND_QUIT) break;
		else if (cmd.code == COMMAND_MAGTEST) {
			_main.MAGTest();
			printf("[main] Magnetic tested\n");
			_cmm.SendMessage("Magnetic tested");
		}
		else if (cmd.code == COMMAND_TEST) {
		}
		else if (cmd.code == COMMAND_RUN) {
			int nOption = GETLONG(cmd.parameter);

			int nSimulationType = 0;				//no simulation
			if (nOption == 1) nSimulationType = SIMULATIONTYPE_MODEL;
			else if (nOption == 2) nSimulationType = SIMULATIONTYPE_STATE;
			else if (nOption == 3) nSimulationType = SIMULATIONTYPE_DATA;

			printf("[main] Program run, option %d\n", nOption);

			_state.SetSimulationType(nSimulationType);
			_state.SetSimulationStartTime(GetTime());

			_main.Run();

			_cmm.SendMessage("Program quit");
		}
		sleep(1);
	}

	_cmm.SendMessage("System quit");
	_cmm.Close();
	printf("[main] quit\n");

	return 1;
}

void clsMain::Run()
{
	if (m_fThread & THREADFLAG_CMM) _cmm.StartThread(PRIORITY_CMM);
	if (m_fThread & THREADFLAG_SVO) _svo.StartThread(PRIORITY_SVO);
	if (m_fThread & THREADFLAG_IM6) _im6.StartThread(PRIORITY_IM6);
	if (m_fThread & THREADFLAG_IM7) _im7.StartThread(PRIORITY_IM7);
	if (m_fThread & THREADFLAG_IM8)	_im8.StartThread(PRIORITY_IM8);
	if (m_fThread & THREADFLAG_IM9)	_im9.StartThread(PRIORITY_IM9);
	if (m_fThread & THREADFLAG_PTU) _ptu.StartThread(PRIORITY_PTU);
	if (m_fThread & THREADFLAG_URG)	_urg.StartThread(PRIORITY_URG);
	if (m_fThread & THREADFLAG_DAQ) _daq.StartThread(PRIORITY_DAQ);
	if (m_fThread & THREADFLAG_CAM) _cam.StartThread(PRIORITY_CAM);
	if (m_fThread & THREADFLAG_CTL) _ctl.StartThread(PRIORITY_CTL);
	if (m_fThread & THREADFLAG_DLG) _dlg.StartThread(PRIORITY_DLG);
//	if (m_fThread & THREADFLAG_COOP) _coop.StartThread(PRIORITY_COOP);
//	if (m_fThread & THREADFLAG_NET) _net.StartThread(PRIORITY_NET);

    m_chMain = ChannelCreate(0);

    sigevent event;
    event.sigev_notify = SIGEV_PULSE;
    event.sigev_coid = ConnectAttach(ND_LOCAL_NODE, 0, m_chMain, _NTO_SIDE_CHANNEL, 0);
    event.sigev_priority = getprio(0);
    event.sigev_code = PULSECODE_MAIN;

    itimerspec timer; timer_t idTimer;
    timer_create(CLOCK_REALTIME, &event, &idTimer);

 	timer.it_value.tv_sec = 0;
    timer.it_value.tv_nsec = PERIOD_MAIN*MILISECOND;				//wait for threads' initializaton work
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_nsec = PERIOD_MAIN*MILISECOND;

#if (_DEBUG)
#if (_DEBUG_PERIOD >= 1000)
	timer.it_value.tv_sec = 1;
    timer.it_value.tv_nsec = 0;
    timer.it_interval.tv_sec = 1;
    timer.it_interval.tv_nsec = 0;
#else
	timer.it_value.tv_sec = 0;
    timer.it_value.tv_nsec = _DEBUG_PERIOD*MILISECOND;
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_nsec = _DEBUG_PERIOD*MILISECOND;
#endif
#endif

    timer_settime(idTimer, 0, &timer, NULL);

    //for timeout setting, the timeout will be used in waiting task thread
    //the timeout waiting is in milli-second precise and begin from a minimum value of 2 ms
    //e.g. a nsec = n*1000000 timeout setting will yeild a 2+n ms waiting
	uint64_t nsecIM6 = PERIOD_IM6*MILISECOND;
	uint64_t nsecIM7 = PERIOD_IM7*MILISECOND;
	uint64_t nsecIM8 = PERIOD_IM8*MILISECOND;
	uint64_t nsecIM9 = PERIOD_IM9*MILISECOND;
	uint64_t nsecDAQ = PERIOD_DAQ*MILISECOND;
	uint64_t nsecCAM = PERIOD_CAM*MILISECOND;
	uint64_t nsecCTL = PERIOD_CTL*MILISECOND;
	uint64_t nsecSVO = PERIOD_SVO*MILISECOND;
	uint64_t nsecCMM = PERIOD_CMM*MILISECOND;
	uint64_t nsecDLG = PERIOD_DLG*MILISECOND;
	uint64_t nsecURG = PERIOD_URG*MILISECOND;
	uint64_t nsecPTU = PERIOD_PTU*MILISECOND;

//	uint64_t nsecServer = PERIOD_SERVER * MILISECOND;
//	uint64_t nsecClient = PERIOD_CLIENT * MILISECOND;
//	uint64_t nsecCoop = PERIOD_COOP*MILISECOND;
//	uint64_t nsecNET= PERIOD_NET*MILISECOND;

	_pulse pulse;
	for (m_nCount=0; ; m_nCount++) {
        MsgReceivePulse(m_chMain, &pulse, sizeof(_pulse), NULL);

		TIMECOST tc = {0};
		double ts = ::GetTime();

		COMMAND cmd;
		_user.GetCommand(&cmd);
		if (cmd.code == 0) _cmm.GetCommand(&cmd);

		if (cmd.code != 0) {
			if (cmd.code == COMMAND_QUIT) break;
			if (!ProcessCommand(&cmd))
			{
				if (cmd.code == COMMAND_FORMATION || cmd.code == COMMAND_COOP_PKT)
					_coop.PutCoopPkt(cmd);
				else _ctl.PutCommand(&cmd);
//				_svo.PutCommand(&cmd);
			}
		}

		if ((m_fThread & THREADFLAG_IM6) && _im6.IsReady()) {
	        double t1 = ::GetTime();
			_im6.SendPulse(PRIORITY_IM6, PULSECODE_DATA);
			_im6.WaitForEvent(nsecIM6);
			tc.tIMU = ::GetTime()-t1;
		}

		if ((m_fThread & THREADFLAG_IM7) && _im7.IsReady()) {
	        double t1 = ::GetTime();
			_im7.SendPulse(PRIORITY_IM7, PULSECODE_DATA);
			_im7.WaitForEvent(nsecIM7);
			tc.tIMU = ::GetTime()-t1;
		}

		if ((m_fThread & THREADFLAG_IM8) && _im8.IsReady()) {
	        double t1 = ::GetTime();
			_im8.SendPulse(PRIORITY_IM8, PULSECODE_DATA);
			_im8.WaitForEvent(nsecIM8);
			tc.tIMU = ::GetTime()-t1;
		}

		if ((m_fThread & THREADFLAG_IM9) && _im9.IsReady()) {
	        double t1 = ::GetTime();
			_im9.SendPulse(PRIORITY_IM9, PULSECODE_DATA);
			_im9.WaitForEvent(nsecIM9);
			tc.tIMU = ::GetTime()-t1;
		}


		if ((m_fThread & THREADFLAG_DAQ) && _daq.IsReady()) {
	        double t1 = ::GetTime();
			_daq.SendPulse(PRIORITY_DAQ, PULSECODE_DATA);
			_daq.WaitForEvent(nsecDAQ);
			tc.tDAQ = ::GetTime()-t1;
		}

		if ((m_fThread & THREADFLAG_CAM) && _cam.IsReady()) {
	        double t1 = ::GetTime();
			_cam.SendPulse(PRIORITY_CAM, PULSECODE_DATA);
			_cam.WaitForEvent(nsecCAM);
			tc.tCAM = ::GetTime()-t1;
		}

		if ((m_fThread & THREADFLAG_PTU) && _ptu.IsReady()) {
			double t1 = ::GetTime();
			_ptu.SendPulse(PRIORITY_PTU, PULSECODE_DATA);
			_ptu.WaitForEvent(nsecPTU);
		}

		if ((m_fThread & THREADFLAG_CTL) && _ctl.IsReady()) {
	        double t1 = ::GetTime();
			_ctl.SendPulse(PRIORITY_CTL, PULSECODE_DATA);
			_ctl.WaitForEvent(nsecCTL);
			tc.tCTL = ::GetTime()-t1;
		}

		if ((m_fThread & THREADFLAG_SVO) && _svo.IsReady()) {
	        double t1 = ::GetTime();
			_svo.SendPulse(PRIORITY_SVO, PULSECODE_DATA);
			_svo.WaitForEvent(nsecSVO);
			tc.tSVO = ::GetTime()-t1;
		}

		/* only sending data in this thread */
//		if ( (m_fThread & THREADFLAG_NET) && _net.IsReady() )
//		{
//			_net.SendPulse(PRIORITY_NET, PULSECODE_DATA);
//			_net.WaitForEvent(nsecNET);
//		}
		if ((m_fThread & THREADFLAG_URG) && _urg.IsReady()) {
	        double t1 = ::GetTime();
			_urg.SendPulse(PRIORITY_URG, PULSECODE_DATA);
			_urg.WaitForEvent(nsecURG);
//			tc.tIMU = ::GetTime()-t1;
		}


		if ((m_fThread & THREADFLAG_CMM) && _cmm.IsReady()) {
	        double t1 = ::GetTime();
			_cmm.SendPulse(PRIORITY_CMM, PULSECODE_DATA);
			_cmm.WaitForEvent(nsecCMM);
			tc.tCMM = ::GetTime()-t1;
		}

		if ((m_fThread & THREADFLAG_DLG) && _dlg.IsReady()) {
	        double t1 = ::GetTime();
//	        cout<<t1<<endl;
			_dlg.SendPulse(PRIORITY_DLG, PULSECODE_DATA);
			_dlg.WaitForEvent(nsecDLG);
			tc.tDLG = ::GetTime()-t1;
		}
		tc.tMain = ::GetTime()-ts;

		uint64_t nsecTC = 2*MILISECOND;
	    TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_CONDVAR | _NTO_TIMEOUT_MUTEX, NULL, &nsecTC, NULL);
		if (pthread_mutex_lock(&m_mtxTC) == 0) {
			if (m_nTC != MAX_TC) {
				m_tTC[m_nTC] = ts;
				m_tc[m_nTC++] = tc;
			}
			pthread_mutex_unlock(&m_mtxTC);
		}

#if (_DEBUG & DEBUGFLAG_MAIN)
if (m_nCount % _DEBUG_COUNT == 0) {
		printf("[main] IMU %.3f ms\n[main] DAQ %.3f ms\n[main] CAM %.3f ms\n[main] CTL %.3f ms\n[main] SVO %.3f ms\n[main] CMM %.3f ms\n[main] DLG %.3f ms\n[main] SERVER %.3f ms\n[main] CLENT %.3f\n[main] main %.3f ms\n,",
			tc.tIMU*1000, tc.tDAQ*1000, tc.tCAM*1000, tc.tCTL*1000, tc.tSVO*1000, tc.tCMM*1000, tc.tDLG*1000, tc.tServer*1000, tc.tClient*1000, tc.tMain*1000);
}
#endif
}	// for loop ends here

	uint64_t nsec = (uint64_t)2000*MILISECOND;

	if (m_fThread & THREADFLAG_IM6) {
		_im6.SendPulse(PRIORITY_IM6, PULSECODE_EXIT);
		_im6.WaitForEvent(nsec);
	}

	if (m_fThread & THREADFLAG_IM7) {
		_im7.SendPulse(PRIORITY_IM7, PULSECODE_EXIT);
		_im7.WaitForEvent(nsec);
	}

	if (m_fThread & THREADFLAG_IM8) {
		_im8.SendPulse(PRIORITY_IM8, PULSECODE_EXIT);
		_im8.WaitForEvent(nsec);
	}

	if (m_fThread & THREADFLAG_IM9) {
		_im9.SendPulse(PRIORITY_IM9, PULSECODE_EXIT);
		_im9.WaitForEvent(nsec);
	}

	if (m_fThread & THREADFLAG_PTU) {
			_ptu.SendPulse(PRIORITY_PTU, PULSECODE_EXIT);
			_ptu.WaitForEvent(nsec);
	}

	if (m_fThread & THREADFLAG_DAQ) {
		_daq.SendPulse(PRIORITY_DAQ, PULSECODE_EXIT);
		_daq.WaitForEvent(nsec);
	}


/*	if (m_fThread & THREADFLAG_COOP)
	{
		_coop.SendPulse(PRIORITY_COOP, PULSECODE_EXIT);
		_coop.WaitForEvent(nsec);
	}*/

	if (m_fThread & THREADFLAG_CTL) {
		_ctl.SendPulse(PRIORITY_CTL, PULSECODE_EXIT);
		_ctl.WaitForEvent(nsec);
	}

	if (m_fThread & THREADFLAG_SVO) {
		_svo.SendPulse(PRIORITY_SVO, PULSECODE_EXIT);
		_svo.WaitForEvent(nsec);
	}

/*	if (m_fThread & THREADFLAG_NET) {
		_net.SendPulse(PRIORITY_NET, PULSECODE_EXIT);
		_net.WaitForEvent(nsec);
	}*/

	if (m_fThread & THREADFLAG_CMM) {
		_cmm.SendPulse(PRIORITY_CMM, PULSECODE_EXIT);
		_cmm.WaitForEvent(nsec);
	}

	if (m_fThread & THREADFLAG_URG) {
		_urg.SendPulse(PRIORITY_URG, PULSECODE_EXIT);
		_urg.WaitForEvent(nsec);
	}


	if (m_fThread & THREADFLAG_DLG) {
		_dlg.SendPulse(PRIORITY_DLG, PULSECODE_EXIT);
		_dlg.WaitForEvent(nsec);
	}
}

BOOL clsMain::ProcessCommand(COMMAND *pCommand)
{
	char code = pCommand->code;
//	char *para = pCommand->parameter;

	BOOL bProcess = TRUE;
	switch (code) {
	case COMMAND_FILTER: {
		_state.ToggleFilter();
		break; }
	default: bProcess = FALSE;
	}

	return bProcess;
}

BOOL clsMain::MAGTest()
{
    char bufQuiet[] = {0x55,0x55,0x53,0x46,0x01,0x00,0x01,0x00,0x00,0x00,0x9B};
//  char bufChBaud[]= {0x55,0x55,0x57,0x46,0x01,0x00,0x02,0x00,0x03,0x00,0xA3};
	char bufMagOK[] =  {0x55,0x55,0x57,0x43,0x00,0x0F,0x00,0xA9};
    termios term;

    // Open the com port
    int m_nsIM6 = open("/dev/ser1", O_RDWR |O_NONBLOCK );
    if (m_nsIM6 == -1) return FALSE;

    // Setup the COM port
    tcgetattr(m_nsIM6, &term);

    cfsetispeed(&term, IM6_BAUDRATE);				//input and output baudrate
    cfsetospeed(&term, IM6_BAUDRATE);

    term.c_cflag = CS8 | CLOCAL | CREAD;				//communication flags
//    term.c_iflag = IGNBRK | IGNCR | IGNPAR;

    tcsetattr(m_nsIM6, TCSANOW, &term);

    //now make IM6 quiet
    if (write(m_nsIM6, bufQuiet, sizeof(bufQuiet))!= 11) {
		close(m_nsIM6);
    	return FALSE;
    }

	char bufMagIni[] = {0x55,0x55,0x53,0x46,0x01,0x00,0x01,0x00,0x00,0x00,0x9B};
    char bufBegCal[] = {0x55,0x55,0x57,0x43,0x00,0x0C,0x00,0xA6};
    char bufResp[10], bufRslt[20];
    long Bias_X, Bias_Y, Scl_Ratio;
    double Real_BiasX, Real_BiasY, Real_SclRatio;

	tcflush(m_nsIM6, TCIOFLUSH);

	if (write(m_nsIM6, bufMagIni, sizeof(bufMagIni))!= 11) {
		close(m_nsIM6);
		return FALSE;
	}
	printf("[MAGTest] Calibration begin, waiting for response ...\n");
//	_cmm.SendMessage("Calibration begin, waiting for response ...");
	usleep(10000);

    if (write(m_nsIM6, bufBegCal, sizeof(bufBegCal))!= 8) {
		close(m_nsIM6);
    	return FALSE;
    }
	sleep(5);

	int nRead = read(m_nsIM6, bufResp,6);
	if (nRead!=6 || bufResp[5]!= 0x63) {
		close(m_nsIM6);
		return FALSE;
	}
	printf("[MAGTest] Response received, please turn the IM6 ... (in one minute)\n");
//	_cmm.SendMessage("Response received, please turn the IM6 ...");
	sleep(60);

	nRead = read(m_nsIM6, bufRslt,17);
//	for (int i=0; i<=nRead-1; i++) {
//		printf("%x  ", (unsigned short)(unsigned char)bufRslt[i]);
//	}
//	printf("\n");

	if (nRead != 17) {
//		_cmm.SendMessage("Receive data failed, please do this test again");
		close(m_nsIM6);
		return FALSE;
	}
    //change order

    char ch;
    for(int i = 3;i <= 11;i += 4) {				//3, 7, 11
    	ch = bufRslt[i]; bufRslt[i] = bufRslt[i+3]; bufRslt[i+3] = ch;
	    ch = bufRslt[i+1]; bufRslt[i+1] = bufRslt[i+2]; bufRslt[i+2] = ch;
    }
    ch = bufRslt[15]; bufRslt[15] = bufRslt[16]; bufRslt[16] = ch;

    //verification
    unsigned short Cal_checksum = 0;
	for (int i = 2; i <= 14; i++) {
		Cal_checksum += (unsigned char)bufRslt[i];
	}
	unsigned short Real_checksum = *(unsigned short *)(bufRslt+15);

    //record value
    if (Cal_checksum != Real_checksum) { close(m_nsIM6); return FALSE; }
    Bias_X = *(long *)(bufRslt+3);
    Bias_Y = *(long *)(bufRslt+7);
    Scl_Ratio = *(long *)(bufRslt+11);

    unsigned long b27 = 0x08000000;
    Real_BiasX = (double)Bias_X / b27;
    Real_BiasY = (double)Bias_Y / b27;
    Real_SclRatio = (double)Scl_Ratio / b27;

	char szReply[128];
	printf("[MAGTest] Calibration checked, Bias_X %f, Bias_Y %f, Bias_Z %f\n", Real_BiasX, Real_BiasY, Real_SclRatio);
	sprintf(szReply, "[MAGTest] Calibration checked, Bias_X %f, Bias_Y %f, Bias_Z %f", Real_BiasX, Real_BiasY, Real_SclRatio);
	_cmm.SendMessage(szReply);

	//magset
	int iCommit = write(m_nsIM6, bufMagOK, sizeof(bufMagOK));
	printf("[MAGTest] Commit writen %d\n", iCommit);
	if (iCommit != 8){
		close(m_nsIM6);
		return FALSE;
	}
	printf("[MAGTest] Calibration set\n");
//	_cmm.SendMessage("Calibration set");

	close(m_nsIM6);
	return TRUE;
}

clsMain::clsMain()
{
	m_nTC = 0;

	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutex_init(&m_mtxTC, &attr);
}

clsMain::~clsMain()
{
}

void clsMain::Init()
{
	double helicopter;
	_parser.GetVariable("_HELICOPTER", &helicopter);
	_HELICOPTER = (int)helicopter;

	_parser.GetVariable("_equ_Hover", &_equ_Hover, sizeof(_equ_Hover));
	_parser.GetVariable("_gravity", &_gravity);
	_parser.GetVariable("_radius", &_radius);

	m_fThread = THREADFLAG_ALL & ~THREADFLAG_CAM; // no camera for general flight test

	if (_HELICOPTER == ID_HELION) {		// HeLion(&)
		m_fThread &= ~(THREADFLAG_IM9| THREADFLAG_IM7 | THREADFLAG_IM8 | THREADFLAG_URG);
	}
	else if (_HELICOPTER == ID_SHELION) {	// SheLion(&)
		m_fThread &= ~(THREADFLAG_IM9 | THREADFLAG_IM6 | THREADFLAG_IM8 | THREADFLAG_URG);
		m_fThread |= THREADFLAG_CAM;	//for shelion, turn on this if camera need to be used
	}
	else if (_HELICOPTER == 8) {	// GremLion(&)
		m_fThread &= ~(THREADFLAG_IM9 | THREADFLAG_IM6 | THREADFLAG_IM7 | THREADFLAG_URG);
//		m_fThread |= THREADFLAG_CAM;	//for grelion, turn on this if camera need to be used
	}
	else if (_HELICOPTER == ID_GREMLION) { //just for test on gumstix chip
		m_fThread &= ~(THREADFLAG_IM9 | THREADFLAG_IM7 | THREADFLAG_IM6 | THREADFLAG_URG);
		m_fThread |= THREADFLAG_CAM;
	}
	else if (_HELICOPTER == ID_FEILION) {
		m_fThread &= ~(THREADFLAG_IM8 | THREADFLAG_IM7 | THREADFLAG_IM6 | THREADFLAG_DAQ);
		m_fThread |= THREADFLAG_CAM;
	}
	else if (_HELICOPTER == ID_QUADLION) {
		m_fThread &= ~(/*THREADFLAG_IM8 |*/ THREADFLAG_IM7 | THREADFLAG_IM6 | THREADFLAG_SVO | THREADFLAG_DAQ);
		m_fThread |= THREADFLAG_CAM;
	}
}

