//daq.cpp
// created on 2011-03-15
// implementation file for clsDAQ, in charge of data acquisition
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>

#include "uav.h"
#include "daq.h"

void clsDAQ::Translate(DAQRAWDATA *pDAQRAW, DAQDATA *pDAQ)
{
#ifndef _GUMSTIX
	DAQRAWDATA &daqraw = *pDAQRAW;
	DAQDATA &daq = *pDAQ;

	unsigned short b16 = 0x8000;

	//change format
	daq.level = (double)daqraw.level*10/b16;
	daq.height = (double)daqraw.height*800/b16 - HEIGHT_CORRECTION;
	daq.height /= 1000;				//translated into meter
//*** modify_5
//	daq.elevator = (double)daqraw.Elevator_Svo*10/b16;

	if ((daq.bRPM = daqraw.bRPM)) {
		daq.status = daqraw.status;
		daq.rpm = (double)daqraw.num*PI*_cps/daqraw.cycleElapse;
	}
#endif
}

clsDAQ::clsDAQ()
{
	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutex_init(&m_mtxDAQ, &attr);
}

clsDAQ::~clsDAQ()
{
	pthread_mutex_destroy(&m_mtxDAQ);
}

BOOL clsDAQ::InitThread()
{
	//initialization
	m_nDAQ = 0;
	m_tDAQ0 = -1;

	m_nsSNA = ::open("/dev/serusb6", O_RDWR | O_NOCTTY );//O_NONBLOCK
//	m_nsSNA = ::open("/dev/ser2", O_RDWR | O_NOCTTY );//O_NONBLOCK
	if (m_nsSNA == -1) {
		printf("[DAQ] Sonar usb serial port cannot be opened!\n");
		return FALSE;
	}
    termios term;
    memset(&term,0,sizeof(term));
    tcgetattr(m_nsSNA, &term);
    cfsetispeed(&term, 9600);			//input and output baudrate
    cfsetospeed(&term, 9600);
    term.c_cflag &= ~(PARENB | CSTOPB | CSIZE); // Note: must do ~CSIZE before CS8
    term.c_cflag |= CS8 | CLOCAL | CREAD;
    term.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Non-canonical interpretation
    term.c_iflag = 0; // Raw mode for input
    term.c_oflag = 0; // Raw mode for output
    term.c_cc[VMIN] = 1;
	tcflush(m_nsSNA,TCIOFLUSH);
	tcsetattr(m_nsSNA, TCSANOW, &term);

	m_nBuffer = 0;

//	m_nSNA = 0;
	m_sna = 0;

	printf("[DAQ] Sonar Start\n");

	return TRUE;
}

int clsDAQ::EveryRun()
{
	//	int nRead = read(m_nsSNA, m_buffer+m_nBuffer, BUFFERSIZE_SNA - m_nBuffer);
	if (m_nCount % 5 != 0) return TRUE;
	int nRead = read(m_nsSNA, m_buffer, 256 /*BUFFERSIZE_SNA*/);

	if (nRead != 6)  return TRUE;

	m_sna = ::atoi(m_buffer+1);

//	printf("[SNA] %s, sona = %d, bytes = %d\n", m_buffer, m_sna, nRead);

	m_tDAQ0 = ::GetTime();

	m_daq0.level = 0;
	m_daq0.height = m_sna;
//	m_daq0.status = 0;
	m_daq0.rpm = 0;

	pthread_mutex_lock(&m_mtxDAQ);
	if (m_nDAQ != MAX_DAQ) {
		m_tDAQ[m_nDAQ] = m_tDAQ0;
//		m_daqRaw[m_nDAQ] = m_daqRaw0;
		m_daq[m_nDAQ++] = m_daq0;
	}

	pthread_mutex_unlock(&m_mtxDAQ);
	return TRUE;

#ifndef _GUMSTIX
	DSCSAMPLE sample[4];

	if (dscADScan(m_dscb, &m_dscadscan, sample) != DE_NONE) return TRUE;

	double tDAQ = GetTime();

	DAQRAWDATA daqraw;
	daqraw.level    = m_dscadscan.sample_values[0];
	daqraw.height = m_dscadscan.sample_values[1];
// *** (modify_2)
//    daqraw.Elevator_Svo = m_dscadscan.sample_values[2];

	//RPM
	BOOL bRPM = m_nCount%COUNT_DAQ == 0;

	if ((daqraw.bRPM = bRPM)) {
		dscCounterRead(m_dscb, &m_dsccr);
		uint64_t cycleNew = ClockCycles();

		daqraw.status = m_dsccr.counter0.status;
		unsigned short numNew = m_dsccr.counter0.value;

		unsigned short num = m_num - numNew;
		daqraw.num = num < 2000 ? num : 0;
		daqraw.cycleElapse = cycleNew - m_cycle;

		m_num = numNew;
		m_cycle = cycleNew;

		if(m_num<1000){
			dscCounterDirectSet(m_dscb, 0x38, 0xffff,0);
			numNew = 0xffff;
		}
	}

	DAQDATA daq;
	Translate(&daqraw, &daq);

#if (_DEBUG & DEBUGFLAG_DAQ)
if (m_nCount % _DEBUG_COUNT == 0) {
	printf("[DAQ] Raw data, level %d, height %d, rpm %d\n",
		daqraw.level, daqraw.height, daqraw.num);
	printf("[DAQ] Data, level %.3f, height %.3f, rpm %d\n",
		daq.level, daq.height, daqraw.num);
}
#endif

	pthread_mutex_lock(&m_mtxDAQ);

	m_tDAQ0 = tDAQ;
	m_daqRaw0.level = daqraw.level;
	m_daqRaw0.height = daqraw.height;

//*** modify_3
//        dataDAQRAW0.Elevator_Svo = daqraw.Elevator_Svo;

	if ((m_daqRaw0.bRPM = daqraw.bRPM)) {
		m_daqRaw0.status = daqraw.status;
		m_daqRaw0.num = daqraw.num;
		m_daqRaw0.cycleElapse = daqraw.cycleElapse;				//the clock cycle elapsed
	}

	m_daq0.level = daq.level;
	m_daq0.height = daq.height;

//*** modify_4
// 		dataDAQ0.elevator = daq.elevator;

	if ((m_daq0.bRPM = daq.bRPM)) {
		m_daq0.status = daq.status;
		m_daq0.rpm = daq.rpm;
	}

	if (m_nDAQ != MAX_DAQ) {
		m_tDAQ[m_nDAQ] = m_tDAQ0;
		m_daqRaw[m_nDAQ] = m_daqRaw0;
		m_daq[m_nDAQ++] = m_daq0;
	}

	pthread_mutex_unlock(&m_mtxDAQ);
#endif
	return TRUE;
}

void clsDAQ::ExitThread()
{
	::close(m_nsSNA);
	printf("[DAQ] Sonar quit\n");
}

