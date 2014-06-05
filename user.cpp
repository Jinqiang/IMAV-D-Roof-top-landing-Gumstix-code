/*
 * user.cpp
 *
 *  Created on: Mar 15, 2011

 */
#include <pthread.h>
#include <errno.h>
#include <stdio.h>

#include "user.h"

clsUser::clsUser()
{
	m_cmd.code = 0;

	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutex_init(&m_mtxCmd, &attr);
}

clsUser::~clsUser()
{
	pthread_mutex_destroy(&m_mtxCmd);
}

BOOL clsUser::StartUserThread(int priority)
{
    pthread_attr_t attribute;
    pthread_attr_init(&attribute);
    pthread_attr_setdetachstate(&attribute, PTHREAD_CREATE_DETACHED);
    pthread_attr_setinheritsched(&attribute, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attribute, SCHED_RR);

    sched_param_t param;
    pthread_attr_getschedparam(&attribute, &param);
    param.sched_priority = priority;
    pthread_attr_setschedparam(&attribute, &param);

	int nCreate = pthread_create(&m_idThread, &attribute, &clsUser::UserThread, this);

	return nCreate == EOK;
}

void *clsUser::UserThread(void *pParameter)
{
	clsUser *pUser = (clsUser *)pParameter;

	pUser->Run();

	return NULL;
}

int clsUser::Run()
{
	printf("[User] start\n");

	char chGet;

	while (1) {
		chGet = getchar();
		switch (chGet) {
		case 'q': m_cmd.code = COMMAND_QUIT; break;
		case 'r':
			PUTLONG(m_cmd.parameter, 0);
			m_cmd.code = COMMAND_RUN;
			break;
		case '1':
			PUTLONG(m_cmd.parameter, 1);
			m_cmd.code = COMMAND_RUN;
			break;
		}
	}

	return TRUE;
}

void clsUser::GetCommand(COMMAND *pCmd)
{
	pthread_mutex_lock(&m_mtxCmd);

	if (m_cmd.code == 0) pCmd->code = 0;
	else {
		*pCmd = m_cmd;
		m_cmd.code = 0;
	}

	pthread_mutex_unlock(&m_mtxCmd);
}

