//dlg.cpp
//this is the implementation file for data logging
#include <stdio.h>
#include <pthread.h>
#include <string.h>

#include "uav.h"
#include "dlg.h"
#include "state.h"
#include "laser.h"

extern clsState _state;
extern clsURG	_urg;

clsDLG::clsDLG()
{
}

clsDLG::~clsDLG()
{
}

BOOL clsDLG::InitThread()
{
	m_nIM6 = 0;
	m_nIM7 = 0;
	m_nURG = 0;

	m_nState = 0;
	m_nDAQ = 0;

	m_nCTL = 0;
	m_nSIG = 0;

	m_nSVO = 0;

	m_nTC = 0;

	m_pfLoga = m_pfLog = m_pfLogb = NULL;

	m_bBlackBox = FALSE;//TRUE;				//default
//	if (_HELICOPTER == 6) m_bBlackBox = FALSE;				//close black back for the SheLion, to decrease cpu occupancy

	char szTime[256];

	time_t t = time(NULL);
	tm *ptm = localtime(&t);
	sprintf(szTime, "%02d%02d%02d%02d%02d", ptm->tm_year, ptm->tm_mon, ptm->tm_mday, ptm->tm_hour, ptm->tm_min);
	::sprintf(m_szFile, "%s.log", szTime);
	::sprintf(m_szFileb, "%sb.log", szTime);
	::sprintf(m_szFilea, "laser_%sb.log", szTime);

	if (!m_bBlackBox) {
		m_pfLog = fopen(m_szFile, "wb");
		if (m_pfLog == NULL) return FALSE;
	}
	else {
		m_pfLogb = fopen(m_szFileb, "wb");
		if (m_pfLogb == NULL) return FALSE;
	}

	//m_pfLoga = fopen(m_szFilea, "wb");
	//if (m_pfLoga == NULL) return FALSE;

	//write into file header
	char header[] = { 0x55, 0x55, 0x56, 0x07 };

	char version[] = { 0x55, 0x55, DATA_VERSION };
	int nVersion = _VERSION;

	if (m_pfLog != NULL) {
		fwrite(header, 1, 4, m_pfLog);
		fwrite(version, 1, 3, m_pfLog);
		fwrite(&nVersion, sizeof(int), 1, m_pfLog);
	}

	if (m_pfLogb != NULL) {
		fwrite(header, 1, 4, m_pfLogb);
		fwrite(version, 1, 3, m_pfLogb);
		fwrite(&nVersion, sizeof(int), 1, m_pfLogb);
		fclose(m_pfLogb);
		m_pfLogb = NULL;
	}

	fclose(m_pfLoga); m_pfLoga = NULL;

	printf("[DLG] Start\n");

	return TRUE;
}

int clsDLG::EveryRun()
{
//	cout<<::GetTime()<<endl;
	if (m_nCount % COUNT_DLG != 0) return TRUE;

//	cout<<::GetTime()<<endl;
	//move data to data log storage
	if (_im6.m_nIM6 != 0) {
//		cout<<"m_nIM6 "<<m_nIM6<<endl;
		pthread_mutex_lock(&_im6.m_mtxIM6);
		m_nIM6 = _im6.m_nIM6;
		memcpy(m_tIM6, _im6.m_tIM6, m_nIM6*sizeof(double));
		memcpy(m_im6Raw, _im6.m_im6Raw, m_nIM6*sizeof(IM6RAWPACK));
		memcpy(m_im6, _im6.m_im6, m_nIM6*sizeof(IM6PACK));
		_im6.m_nIM6 = 0;
		pthread_mutex_unlock(&_im6.m_mtxIM6);
	}

	if (_im7.m_nIM7 != 0) {
//		cout<<"m_nIM7 "<<m_nIM7<<endl;
		pthread_mutex_lock(&_im7.m_mtxIM7);
		m_nIM7 = _im7.m_nIM7;
		::memcpy(m_tIM7, _im7.m_tIM7, m_nIM7*sizeof(double));
		::memcpy(m_im7Raw, _im7.m_im7Raw, m_nIM7*sizeof(IM7RAWPACK));
		::memcpy(m_im7, _im7.m_im7, m_nIM7*sizeof(IM7PACK));
		_im7.m_nIM7 = 0;
		pthread_mutex_unlock(&_im7.m_mtxIM7);
	}

	if (_state.m_nState != 0) {
//		cout<<"m_nState "<<m_nState<<endl;
		pthread_mutex_lock(&_state.m_mtxState);

		m_nState = _state.m_nState;
		::memcpy(m_tState, _state.m_tState, m_nState*sizeof(double));
		::memcpy(m_state, _state.m_state, m_nState*sizeof(UAVSTATE));
		_state.m_nState = 0;

		pthread_mutex_unlock(&_state.m_mtxState);
	}

	if (_daq.m_nDAQ != 0) {
//		cout<<"m_nDAQ "<<m_nDAQ<<endl;
		pthread_mutex_lock(&_daq.m_mtxDAQ);
		m_nDAQ = _daq.m_nDAQ;
		::memcpy(m_tDAQ, _daq.m_tDAQ, m_nDAQ*sizeof(double));
//		::memcpy(m_daqRaw, _daq.m_daqRaw, m_nDAQ*sizeof(DAQRAWDATA));
		::memcpy(m_daq, _daq.m_daq, m_nDAQ*sizeof(DAQDATA));
		_daq.m_nDAQ = 0;
		pthread_mutex_unlock(&_daq.m_mtxDAQ);
	}

	if (_ctl.m_nCTL != 0) {
	//	cout<<"m_nCTL "<<_ctl.m_nCTL<<endl;
		pthread_mutex_lock(&_ctl.m_mtxCTL);
		m_nCTL = _ctl.m_nCTL;
		::memcpy(m_tCTL, _ctl.m_tCTL, m_nCTL*sizeof(double));
		::memcpy(m_ctl, _ctl.m_ctl, m_nCTL*sizeof(CONTROLSTATE));
		_ctl.m_nCTL = 0;
		pthread_mutex_unlock(&_ctl.m_mtxCTL);
	}

	if (_state.m_nSIG != 0) {
//		cout<<"m_nSIG "<<m_nSIG<<endl;
		pthread_mutex_lock(&_state.m_mtxSIG);
		m_nSIG = _state.m_nSIG;
		::memcpy(m_tSIG, _state.m_tSIG, m_nSIG*sizeof(double));
		::memcpy(m_sig, _state.m_sig, m_nSIG*sizeof(HELICOPTERRUDDER));
		_state.m_nSIG = 0;
		pthread_mutex_unlock(&_state.m_mtxSIG);
	}

	if (_svo.m_nSVO != 0) {
//		cout<<"m_nSVO "<<m_nSVO<<endl;
		pthread_mutex_lock(&_svo.m_mtxSVO);
		m_nSVO = _svo.m_nSVO;
		memcpy(m_tSVO, _svo.m_tSVO, m_nSVO*sizeof(double));
		memcpy(m_svoRaw, _svo.m_svoRaw, m_nSVO*sizeof(SVORAWDATA));
		memcpy(m_svo, _svo.m_svo, m_nSVO*sizeof(SVODATA));
		_svo.m_nSVO = 0;
		pthread_mutex_unlock(&_svo.m_mtxSVO);
	}

	if (_main.m_nTC != 0) {
		pthread_mutex_lock(&_main.m_mtxTC);
		m_nTC = _main.m_nTC;
		memcpy(m_tTC, _main.m_tTC, m_nTC*sizeof(double));
		memcpy(m_tc, _main.m_tc, m_nTC*sizeof(TIMECOST));
		_main.m_nTC = 0;
		pthread_mutex_unlock(&_main.m_mtxTC);
	}

	if (_urg.m_nURG != 0) {
		pthread_mutex_lock(&_urg.m_mtxURG);
		m_nURG = _urg.m_nURG;
		::memcpy(m_tURG, _urg.m_tURG, m_nURG*sizeof(double));
		::memcpy(laser_ranges, _urg.ranges, MAX_URG_PT*m_nURG*sizeof(long));
		::memcpy(laser_x, _urg.x_log, m_nURG*sizeof(double));
		::memcpy(laser_y, _urg.y_log, m_nURG*sizeof(double));
		::memcpy(laser_z, _urg.z_log, m_nURG*sizeof(double));
		::memcpy(laser_phi, _urg.phi_log, m_nURG*sizeof(double));
		::memcpy(laser_tht, _urg.tht_log, m_nURG*sizeof(double));
		::memcpy(laser_psi, _urg.psi_log, m_nURG*sizeof(double));
		_urg.m_nURG = 0;
		pthread_mutex_unlock(&_urg.m_mtxURG);
	}

#if (_DEBUG & DEBUGFLAG_DLG)
	printf("[DLG] pack to save");
	if (m_nIM6 != 0) printf(", IM6 %d", m_nIM6);
	if (m_nIM7 != 0) printf(", IM7 %d", m_nIM7);
	if (m_nState != 0) printf(", State %d", m_nState);
	if (m_nDAQ != 0) printf(", DAQ %d", m_nDAQ);
	if (m_nCTL != 0) printf(", CTL %d", m_nCTL);
	if (m_nSVO != 0) printf(", SVO %d", m_nSVO);
	if (m_nSIG != 0) printf(", SIG %d", m_nSIG);
	if (m_nTC != 0) printf(", TC %d", m_nTC);
	printf("\n");
#endif

	//begin writing data
	if (m_bBlackBox) m_pfLogb = ::fopen(m_szFileb, "a");

	//m_pfLoga = ::fopen(m_szFilea, "a");

	int i; //char strData[512];
//	for (i=0; i<=50; i++)
//	{
//		DLGHEADER header = { 0x5555, 'A', 1.00 };
//		if (m_pfLog != NULL)
//		{
//			fwrite(&header, sizeof(DLGHEADER), 1, m_pfLog);
//		}
//	}

	for (i=0; i<=m_nIM6-1; i++) {
		char type = m_im6[i].ah6.type;
		double t = m_tIM6[i];

		DLGHEADER header = { 0x5555, type, t };

		if (m_pfLog != NULL) {
			fwrite(&header, sizeof(DLGHEADER), 1, m_pfLog);
			fwrite(&m_im6[i], sizeof(IM6PACK), 1, m_pfLog);
		}
		if (m_pfLogb != NULL) {
			fwrite(&header, sizeof(DLGHEADER), 1, m_pfLogb);
			fwrite(&m_im6[i], sizeof(IM6PACK), 1, m_pfLogb);
		}
	}
	m_nIM6 = 0;

	for (i=0; i<=m_nIM7-1; i++) {
		char type = m_im7[i].sc7.type;
		double t = m_tIM7[i];

		DLGHEADER header = { 0x5555, type, t };

		if (m_pfLog != NULL) {
			fwrite(&header, sizeof(DLGHEADER), 1, m_pfLog);
			fwrite(&m_im7[i], sizeof(IM7PACK), 1, m_pfLog);
		}
		if (m_pfLogb != NULL) {
			fwrite(&header, sizeof(DLGHEADER), 1, m_pfLogb);
			fwrite(&m_im7[i], sizeof(IM7PACK), 1, m_pfLogb);
		}

		char typeRaw = type == DATA_SC7 ? DATA_SC7RAW : DATA_GP7RAW;
		header.code = typeRaw;

		if (m_pfLog != NULL) {
			fwrite(&header, sizeof(DLGHEADER), 1, m_pfLog);
			fwrite(&m_im7Raw[i], sizeof(IM7RAWPACK), 1, m_pfLog);
		}
		if (m_pfLogb != NULL) {
			fwrite(&header, sizeof(DLGHEADER), 1, m_pfLogb);
			fwrite(&m_im7Raw[i], sizeof(IM7RAWPACK), 1, m_pfLogb);
		}
	}
	m_nIM7 = 0;

	for (i=0; i<=m_nState-1; i++) {
		double t = m_tState[i];
		DLGHEADER header = { 0x5555, DATA_STATE, t };

		if (m_pfLog != NULL) {
			fwrite(&header, sizeof(DLGHEADER), 1, m_pfLog);
			fwrite(&m_state[i], sizeof(UAVSTATE), 1, m_pfLog);
		}
		if (m_pfLogb != NULL) {
			fwrite(&header, sizeof(DLGHEADER), 1, m_pfLogb);
			fwrite(&m_state[i], sizeof(UAVSTATE), 1, m_pfLogb);
		}
	}
	m_nState = 0;

	for (i=0; i<=m_nDAQ-1; i++) {
		DLGHEADER header = { 0x5555, DATA_DAQ, m_tDAQ[i] };

		if (m_pfLog != NULL) {
			fwrite(&header, sizeof(DLGHEADER), 1, m_pfLog);
			fwrite(&m_daq[i], sizeof(DAQDATA), 1, m_pfLog);
		}
		if (m_pfLogb != NULL) {
			fwrite(&header, sizeof(DLGHEADER), 1, m_pfLogb);
			fwrite(&m_daq[i], sizeof(DAQDATA), 1, m_pfLogb);
		}
	}
	m_nDAQ = 0;

	for (i=0; i<=m_nCTL-1; i++) {
		DLGHEADER header = { 0x5555, DATA_CTL, m_tCTL[i] };

		if (m_pfLog != NULL) {
			fwrite(&header, sizeof(DLGHEADER), 1, m_pfLog);
			fwrite(&m_ctl[i], sizeof(CONTROLSTATE), 1, m_pfLog);
		}
		if (m_pfLogb != NULL) {
			fwrite(&header, sizeof(DLGHEADER), 1, m_pfLogb);
			fwrite(&m_ctl[i], sizeof(CONTROLSTATE), 1, m_pfLogb);
		}
	}
	m_nCTL = 0;

	for (i=0; i<=m_nSIG-1; i++) {
		DLGHEADER header = { 0x5555, DATA_SIG, m_tSIG[i] };

		if (m_pfLog != NULL) {
			fwrite(&header, sizeof(DLGHEADER), 1, m_pfLog);
			fwrite(&m_sig[i], sizeof(HELICOPTERRUDDER), 1, m_pfLog);
		}
		if (m_pfLogb != NULL) {
			fwrite(&header, sizeof(DLGHEADER), 1, m_pfLogb);
			fwrite(&m_sig[i], sizeof(HELICOPTERRUDDER), 1, m_pfLogb);
		}
	}
	m_nSIG = 0;

	for (i=0; i<=m_nSVO-1; i++) {
		DLGHEADER header = { 0x5555, DATA_SVO, m_tSVO[i] };

		if (m_pfLog != NULL) {
			fwrite(&header, sizeof(DLGHEADER), 1, m_pfLog);
			fwrite(&m_svo[i], sizeof(SVODATA), 1, m_pfLog);
		}
		if (m_pfLogb != NULL) {
			fwrite(&header, sizeof(DLGHEADER), 1, m_pfLogb);
			fwrite(&m_svo[i], sizeof(SVODATA), 1, m_pfLogb);
		}
	}
	m_nSVO = 0;

	for (i=0; i<=m_nTC-1; i++) {
		DLGHEADER header = { 0x5555, DATA_TC, m_tTC[i] };

		if (m_pfLog != NULL) {
			fwrite(&header, sizeof(DLGHEADER), 1, m_pfLog);
			fwrite(&m_tc[i], sizeof(TIMECOST), 1, m_pfLog);
		}
		if (m_pfLogb != NULL) {
			fwrite(&header, sizeof(DLGHEADER), 1, m_pfLogb);
			fwrite(&m_tc[i], sizeof(TIMECOST), 1, m_pfLogb);
		}
	}
	m_nTC = 0;

	// Log laser range finder data to "laser.txt"
	for (int i=0; i<m_nURG; i++)
	{
		if (m_pfLoga != NULL)
		{
//			for (int j=0; j<MAX_URG_PT; j++)
//			{
//				fprintf(m_pfLoga, "%d\t", laser_ranges[i][j]);
//			}
			fprintf(m_pfLoga, "%.3f\t", laser_x[i]);
			fprintf(m_pfLoga, "%.3f\t", laser_y[i]);
			fprintf(m_pfLoga, "%.3f\t", laser_z[i]);
			fprintf(m_pfLoga, "%.3f\t", laser_phi[i]);
			fprintf(m_pfLoga, "%.3f\t", laser_tht[i]);
			fprintf(m_pfLoga, "%.3f\t", laser_psi[i]);
			fprintf(m_pfLoga, "%.3f\n", m_tURG[i]);
		}
	}
	m_nURG = 0;


	if (m_pfLoga) {
		::fclose(m_pfLoga);
		m_pfLoga = NULL;
	}

	if (m_pfLogb) {
		::fclose(m_pfLogb);
		m_pfLogb = NULL;
	}

	return TRUE;
}

void clsDLG::ExitThread()
{
	if (m_pfLog != NULL) ::fclose(m_pfLog);
	printf("[DLG] quit\n");
}

