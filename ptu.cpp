/*
 * clsPTU.cpp
 *
 *  Created on: Apr 8, 2013
 *      Author: nusuav
 */
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include "ptu.h"
#include "uav.h"
#include "state.h"
#include "im8.h"
#include <stdio.h>

extern clsState _state;
extern clsIM8 _im8;

#define PAN_TRIM 3200
#define TILT_TRIM 2600
#define SVO_ANGLE_PAN_SCALING (18.1594) //angle in degrees
#define SVO_ANGLE_TILT_SCALING (-18.1594)

clsPTU::clsPTU() {
	// TODO Auto-generated constructor stub
	m_panAngleOffset = 0;
	m_tiltAngleOffset = 0;
}

clsPTU::~clsPTU() {
	// TODO Auto-generated destructor stub
}

BOOL clsPTU::InitThread()
{
	m_nsPTU = ::open("/dev/serusb2", O_RDWR|O_NONBLOCK );

	if (m_nsPTU == -1) {
		printf("[PTU] Open PTU serial port (/dev/serusb2) failed!\n");
		return FALSE;
	}

	termios term;
	tcgetattr(m_nsPTU, &term);
	bzero(&term, sizeof(termios));

	cfsetispeed(&term, 38400);				//input and output baudrate
	cfsetospeed(&term, 38400);

	term.c_cflag = CS8 | CLOCAL | CREAD;				//communication flags
	tcsetattr(m_nsPTU, TCSANOW, &term);
	tcflush(m_nsPTU, TCIOFLUSH);

	m_panAngle = 0; m_tiltAngle = 0;

	printf("[PTU] Start\n");
	return TRUE;
}

int clsPTU::EveryRun()
{
		// channel 0 (channel 0 start) for camera PTU control
		// channel 0 -> PAN; channel 1 -> TILT
		char pan[6], tilt[6];
		pan[0] = tilt[0] = 0x80;
		pan[1] = tilt[1] = 0x01;
		pan[2] = tilt[2] = 0x04;
		pan[3] = 0;	// servo channel 0
		tilt[3] = 1; // servo channel 1


		PanTiltConpensateUAV(_state.GetState().a + m_panAngleOffset, _state.GetState().b + m_tiltAngleOffset,  &m_panAngle, &m_tiltAngle);

		int nPan = int(PAN_TRIM + SVO_ANGLE_PAN_SCALING*m_panAngle*180/PI);
		int nTilt = int(TILT_TRIM + SVO_ANGLE_TILT_SCALING*m_tiltAngle*180/PI);

		pan[4] = nPan / 128; pan[5] = nPan % 128;
		tilt[4] = nTilt / 128; tilt[5] = nTilt % 128;
		int nWrite = write(m_nsPTU, pan, 6);
		write(m_nsPTU, tilt, 6);
//		printf("[PTU] Write %d bytes to pan\n", nWrite);
		return TRUE;
}

void clsPTU::ExitThread()
{
	::close(m_nsPTU);
	printf("[PTU] quit\n");
}

void clsPTU::PanTiltConpensateUAV(double phiUAV, double thtUAV, double* phiPT, double* thtPT)
{
          *phiPT=asin(-1*sin(phiUAV)*cos(thtUAV));
          *thtPT=atan(-sin(thtUAV) / (cos(phiUAV)*cos(thtUAV)));
}