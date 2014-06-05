/*
 * laser.cpp
 *
 *  Created on: Aug 27, 2012
 *      Author: nusuav
 */
#include <pthread.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream.h>

#include "uav.h"
#include "im8.h"
#include "state.h"
#include "laser.h"
#include "state.h"
#include "hokuyo.h"
#include "slam.h"
#include "height.h"

extern clsState _state;
extern clsCAM _cam;

clsURG::clsURG()
{
	m_nCount = 0;
	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutex_init(&m_mtxURG, &attr);
}

clsURG::~clsURG()
{
	pthread_mutex_destroy(&m_mtxURG);
}

BOOL clsURG::InitThread()
{
	m_nURG = 0;
	start_localisation = 0;
	goodMeasurement = 0; // added by Jin for laser jump signal check
	jumpCount = 0;       // Jin
	firstMeasurement = 1;
//	Init_URG_4();
//	cout << "[URG] 4 initialized" << endl;
//	Init_URG_30();
	if (!urg_init()) {
		printf("[URG] init fail.\n");
		return FALSE;
	}
	cout << "[URG] 30 initialized" << endl;

	infile30 = fopen("test1.log", "r");

	infile4 = fopen("laser4_1.log", "r");
	outfile4 = fopen("laser4.log", "w");

	SAFMC.init();
	SAFMC_height.init();
	::memset(&m_info, 0, sizeof(m_info));
	m_info.on_off = 1;

    printf("[URG] Start\n");
	return true;
}

void clsURG::ExitThread()
{
	char cmd[5] = {"QT"}; cmd[2] = 13;
	//write(_urg4, cmd, 3);
	write(_urg30, cmd, 3);

	//close(_urg4);
	close(_urg30);

	printf("[URG] quit\n");

	fclose(outfile4);
}

void clsURG::set_start_localisation(int i)
{
	start_localisation = i;
}

void clsURG::request_urg4(long int range_4[])
{
	int size = 1612;
	int nrecv = 0;
	char buf[4000];
	while (nrecv < size)
	{
		int nread = read(_urg4, buf+nrecv, size-nrecv);
		cout << "[URG]  4 nread is " << nread << endl;
		nrecv += nread;
	}

	char data[20] = {"GS0000076801"}; data[12]= 13;
	write(_urg4, data, 13);

	Process_URG4(range_4, buf, size);
}

void clsURG::Process_URG4(long int range_4[],char buf[],int size)
{
	int i = 23;
	int j = 0;
	int m = 0;
	int k = 0;
	int N = floor(MAX_URG_4*2/64); //24;  //sets of data N = number_point*2/64
	int temp;
	int rest = MAX_URG_4*2-N*64; //2;  //rest = number_point*2%64

	if (buf[0] == 'G')
	{
		for (j=0; j<N; j++)
		{
			i = 23+j*66;
			for (m=0; m<64; m=m+2)
			{
				temp = buf[i+m]-48;
				temp = temp*64;
				temp = temp+(buf[i+m+1]-48);
				range_4[k] = temp;
				k++;
			}
		}

		i = 23+N*66;
		for (m=0; m<rest; m=m+2)
		{
			temp = buf[i+m]-48;
			temp = temp*64;
			temp = temp+(buf[i+m+1]-48);
			range_4[k] = temp;
			k++;
		}
	}
	else
		cout << "Something wrong with decoding 4m laser scanner!\n" << endl;
}

void clsURG::request_urg30(long int range_30[])
{
	int size = 2048;
	int nrecv = 0;
	char buf[4000];
	while (nrecv < size)
	{
		int nread = read(_urg30, buf+nrecv, size-nrecv);
		cout << "[URG] 30 nread is "<< nread << endl;
		nrecv += nread;
	}

	char data[20] = {"GS0000108001"}; data[12]= 13;
	write(_urg30, data, 13);

	Process_URG30(range_30, buf, size);
}

void clsURG::Process_URG30(long int range_30[],char buf[],int size)
{
	int i = 23;
	int j = 0;
	int m = 0;
	int k = 0;
	int N = floor(MAX_URG_PT*2/64); //24;  //sets of data N = number_point*2/64
	int temp;
	int rest = MAX_URG_PT*2-N*64; //2;  //rest = number_point*2%64

	if (buf[0] == 'G')
	{
		for (j=0; j<N; j++)
		{
			i = 23+j*66;
			for (m=0; m<64; m=m+2)
			{
				temp = buf[i+m]-48;
				temp = temp*64;
				temp = temp+(buf[i+m+1]-48);
				range_30[k] = temp;
				k++;
			}
		}

		i = 23+N*66;
		for (m=0; m<rest; m=m+2)
		{
			temp = buf[i+m]-48;
			temp = temp*64;
			temp = temp+(buf[i+m+1]-48);
			range_30[k] = temp;
			k++;
		}
//		cout << "30m last range: " << range_30[k-1] << endl;
	}
	else
		cout << "Something wrong with decoding 30m laser scanner!\n" << endl;
}

int clsURG::EveryRun()
{
	UAVSTATE state = _state.GetState();
	double t1 = ::GetTime();
	m_tURG0 = ::GetTime(); //DONNOT delete this line if you want to transimit data back to GCS;

//    if (m_nCount%5 == 1 || m_nCount%5 == 2 || m_nCount%5 == 4)
    if (m_nCount%5 !=0)
    	return TRUE;

//    else if (m_nCount%5 == 0)
//    {
    	double tPack = ::GetTime();

//    	double temp;
//		for(int i=0; i<1088; i++)
//		{
//			if (i<1081)
//			{
//				fscanf(infile30, "%lf", &temp);
//				range[i-8] = (long)temp;
//			}
//			else
//				fscanf(infile30, "%lf", &temp);
//		}

    	urg_requestdata(range); // Hokuyo default way
//    	request_urg30(range);  // wangfei modified

		// Get the UAV height with unit as "-meter"
		// The result stored in m_info.z;
		GetHeight();

		position cur = SAFMC.localization(range);

		_state.SetbMeasurementUpdate();

		if (cur.flag == 1)
		{
			m_info.y = cur.x / 1000;
			m_info.x = cur.y / 1000;

			m_info.psi = -cur.theta;
			INPI(m_info.psi);
		}

		pthread_mutex_lock(&m_mtxURG);
		if (m_nURG < MAX_IM9PACK)
		{
			m_tURG[m_nURG] = tPack;
			m_tURG0 = tPack;

			for (int i=0; i<MAX_URG_PT; i++) {
				ranges[m_nURG][i] = range[i];
			}
//			for (int j=0; j<MAX_URG_4; j++) {
//				ranges4[m_nURG][j] = range_4[j];
//			}

			x_log[m_nURG] 	= m_info.x;
			y_log[m_nURG] 	= _cam.GetVisionTargetInfo().nedFrame_dvec[2];
			z_log[m_nURG] 	= m_info.z;
			phi_log[m_nURG] = state.a;
			tht_log[m_nURG] = state.b;
			psi_log[m_nURG] = state.c;
			m_nURG++;
		}
		pthread_mutex_unlock(&m_mtxURG);
//    }

//    else if (m_nCount%10 == 3)
//    {
//    	return TRUE;
//
//        request_urg4(range_4);
//        m_info.z = SAFMC_height.localization(range_4);
//        cout << "Laser height: " << m_info.z << endl;
//
////        for (int i=0; i<MAX_URG_4; i++)
////			fprintf(outfile4, "%d\t", range_4[i]);
////        fprintf(outfile4, "%f\t", state.x);
////        fprintf(outfile4, "%f\t", state.y);
////        fprintf(outfile4, "%f\t", state.z);
////        fprintf(outfile4, "%f\t", state.a);
////        fprintf(outfile4, "%f\t", state.b);
////        fprintf(outfile4, "%f\t", state.c);
////        fprintf(outfile4, "%f\t", t1);
////        fprintf(outfile4, "\n");
//    }

	return TRUE;
}


void clsURG::SetLaserConfig(char *laserDevice, short size, short flag)
{
	::memcpy(m_laserConfig.laserDevice, laserDevice, size);
	m_laserConfig.flag = flag;
}

// Init port for 30m laser scanner
void clsURG::Init_URG_30() {
	_urg30 = ::open("/dev/serusb2", O_RDWR);
	if (_urg30 == -1) {
		printf("[URG] Open URG30 port failed!\n");
		return;
	}

	termios term;
	tcgetattr(_urg30, &term);
	cfsetispeed(&term, 115200);
	cfsetospeed(&term, 115200);
	term.c_cflag = CS8 | CLOCAL | CREAD;
	tcsetattr(_urg30, TCSANOW, &term);
	tcflush(_urg30, TCIOFLUSH);
	usleep(5000);
	char cmd[5] = {"BM"}; cmd[2] = 13;
	write(_urg30, cmd, 3);
	usleep(100000);
	char buf[10];
	int count = read(_urg30, buf, 8);
	cout << "Read " << count << " of bytes from URG30" << endl;
	char data[20] = {"GS0000108001"}; data[12]= 13;
	write(_urg30, data, 13);
	usleep(100000);
}

// Init port for 4m laser scanner
void clsURG::Init_URG_4() {
	_urg4 = ::open("/dev/serusb1", O_RDWR);
	if (_urg4 == -1) {
		printf("[URG] Open URG4 port failed!\n");
		return;
	}

    termios term;
    tcgetattr(_urg4, &term);
    cfsetispeed(&term, 115200);
    cfsetospeed(&term, 115200);
    term.c_cflag = CS8 | CLOCAL | CREAD;
	tcsetattr(_urg4, TCSANOW, &term);
	tcflush(_urg4, TCIOFLUSH);
	usleep(5000);
    char cmd[5] = {"BM"}; cmd[2] = 13;
    write(_urg4, cmd, 3);
    usleep(100000);
	char buf[10];
	int count = read(_urg4, buf, 8);
	cout << "Read " << count << " of bytes from URG4" << endl;
	char data[20] = {"GS0000076801"}; data[12]= 13;
	write(_urg4, data, 13);
	usleep(100000);
}

double clsURG::GetHeight(){
	    long temp[NUSUAV_LASERH_HEIGHT_END_INDEX - NUSUAV_LASERH_HEIGHT_START_INDEX + 1];
		long swap;

		UAVSTATE &state = _state.GetState();

		for (int i = NUSUAV_LASERH_HEIGHT_START_INDEX; i <= NUSUAV_LASERH_HEIGHT_END_INDEX; i++)
		{
			temp[i - NUSUAV_LASERH_HEIGHT_START_INDEX] = range[i];
		}

		// Bubble sort for medium filter
		bool sorted = false;
	    while(!sorted)
		{
			sorted = true;
			for (int i=0; i < NUSUAV_LASERH_HEIGHT_END_INDEX - NUSUAV_LASERH_HEIGHT_START_INDEX + 1; i++)
			{
				if ( temp[i]<temp[i+1] )
				{
					swap = temp[i];
					temp[i] = temp[i+1];
					temp[i+1] = swap;
					sorted = false;
				}
			}
		}

	    if (temp[ (NUSUAV_LASERH_HEIGHT_END_INDEX - NUSUAV_LASERH_HEIGHT_START_INDEX)/2 + 1] <= 2000 && temp[ (NUSUAV_LASERH_HEIGHT_END_INDEX - NUSUAV_LASERH_HEIGHT_START_INDEX)/2 + 1 ]>= 80)
	    {
	    	m_info.z = (double)(temp[ (NUSUAV_LASERH_HEIGHT_END_INDEX - NUSUAV_LASERH_HEIGHT_START_INDEX)/2 + 1 ] - NUSUAV_LASERH_OFFSET_TO_GROUND)*cos(state.b)*cos(state.a)/(-1000.0); //in m
//	    	m_info.x = (double)(temp[ (NUSUAV_LASERH_HEIGHT_END_INDEX - NUSUAV_LASERH_HEIGHT_START_INDEX)/2 + 1 ] - NUSUAV_LASERH_OFFSET_TO_GROUND)/(-1000.0);
//	    	printf("[urg] m_info.z : %.2f\n", m_info.z);
	    }
//	    cout << "Current m_info.z" << m_info.z << endl;
	    CheckHeightJump(); // comment if no jump signal check
}

void clsURG::CheckHeightJump() //Jinqiang:  added on 2014 03 13 for height jump check
{
	if(firstMeasurement) // if first measurement, inintialize the measurement
	{
		goodMeasurement = m_info.z;
		firstMeasurement = 0;
	}
	double diff;
	diff = fabs(m_info.z - goodMeasurement);

	if(diff < 0.2)
	{
		goodMeasurement = m_info.z;
		jumpCount = 0;
	}
	else
	{
		jumpCount++;
//		cout<< "Measurement jumping  "<< jumpCount << endl;
		if(jumpCount > 2)  // number need to be trimed. for short jumps with 2 points, this work. the third one will update to KF.
		{
			goodMeasurement = m_info.z;
			cout<< "Measurement reset"<< endl;
		}
	}
//    cout << "Current Good measurement" << goodMeasurement << endl;

	m_info.z = goodMeasurement;
}
