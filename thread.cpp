/*
 * thread.cpp
 *
 *  Created on: Mar 15, 2011

 */
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include <pthread.h>
#include <errno.h>

#include "thread.h"

clsThread::clsThread()
{
	m_idThread = 0;

	m_chThread = ChannelCreate(0);
    m_coidThread = ConnectAttach(ND_LOCAL_NODE, 0, m_chThread, _NTO_SIDE_CHANNEL, 0);

	pthread_condattr_t attrCond;
	pthread_condattr_init(&attrCond);

	pthread_cond_init(&m_condThread, &attrCond);

	pthread_mutexattr_t attrMtx;
	pthread_mutexattr_init(&attrMtx);

	pthread_mutex_init(&m_mtxThread, &attrMtx);
	pthread_mutex_lock(&m_mtxThread);

	m_bReady = FALSE;

	m_nCount = 0;
}

clsThread::~clsThread()
{
	ConnectDetach(m_coidThread);
	ChannelDestroy(m_chThread);

	pthread_cond_destroy(&m_condThread);
	pthread_mutex_destroy(&m_mtxThread);
}

BOOL clsThread::StartThread(int priority)
{
    pthread_attr_t attribute;
    pthread_attr_init(&attribute);
    pthread_attr_setdetachstate(&attribute, PTHREAD_CREATE_DETACHED);
    pthread_attr_setinheritsched(&attribute, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attribute, SCHED_RR);

//    attribute.param.sched_priority = priority;

    sched_param_t param;
    pthread_attr_getschedparam(&attribute, &param);
    param.sched_priority = priority;
    pthread_attr_setschedparam(&attribute, &param);

	int nCreate = pthread_create(&m_idThread, &attribute, &clsThread::Thread, this);

	return nCreate == EOK;
}

void *clsThread::Thread(void *pParameter)
{
	clsThread *pThread = (clsThread *)pParameter;
	pThread->Run();

	return NULL;
}

void clsThread::SendPulse(int priority, int code, int value)
{
	MsgSendPulse(m_coidThread, priority, code, value);
}

void clsThread::SetEvent()
{
	pthread_cond_signal(&m_condThread);
}

int clsThread::WaitForEvent(uint64_t nsec)
{
	uint64_t nWait = nsec;

    TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_CONDVAR | _NTO_TIMEOUT_MUTEX, NULL, &nWait, NULL);
	int nReturn = pthread_cond_wait(&m_condThread, &m_mtxThread);

	return nReturn;
}

int clsThread::Run()
{
	if (!InitThread()) return -1;

	_pulse pulse;
	for (m_nCount = 0; ; m_nCount ++) {

		m_bReady = TRUE;
		ReceivePulse(&pulse);
		m_bReady = FALSE;				//begin working

		if (pulse.code == PULSECODE_EXIT) break;

		if (!EveryRun()) break;

		SetEvent();
	}

	ExitThread();
	SetEvent();
	return 1;
}

int clsThread::ReceivePulse(_pulse *pPulse)
{
	return MsgReceivePulse(m_chThread, pPulse, sizeof(_pulse), NULL);
}

BOOL clsThread::InitThread()
{
	return TRUE;
}

int clsThread::EveryRun()
{
	return 1;
}

void clsThread::ExitThread()
{
}
