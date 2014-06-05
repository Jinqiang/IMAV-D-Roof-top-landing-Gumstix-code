//	Plan.cpp
//	This is the implementation file of the plan compiling & implementation
//
#include <stdio.h>
#include <inttypes.h>
#include <pthread.h>
#include <sys/neutrino.h>
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "dscud.h"

#include "matrix.h"
#include "plan.h"
#include "uav.h"

/*
clsPlan::clsPlan()
{
//	m_pStack = new char[STACKSIZE];
//	m_pData = new char[STACKSIZE];
}

clsPlan::~clsPlan()
{
//	delete[] m_pStack;
//	delete[] m_pData;
}

BOOL clsPlan::Load(char *pszPlan)
{
	return FALSE;
}

BOOL clsPlan::LoadFromFile(char *pszFile)
{
	FILE *pfPlan = ::fopen(pszFile, "r");
	if (pfPlan == NULL) return NULL;

	::fseek(pfPlan, 0, SEEK_END);
	int nLen = ::ftell(pfPlan);

	::rewind(pfPlan);

	char *pszScript = new char[nLen+1];
	int nRead = ::fread(pszScript, 1, nLen, pfPlan);
	pszScript[nRead] = '\0';

	::fclose(pfPlan);

	BOOL bLoad = Load(pszScript);
	delete pszScript;

	return bLoad;
	return FALSE;
}

int clsPlan::Call()
{
	return 0;
}

int clsPlan::Call0(char *pszFunc, BOOL bStart)
{
	return 0;
}

void clsPlan::PutState(STATE *pState)
{
//	::memcpy(m_pData+STACK_STATE, pState, sizeof(STATE));
}

void clsPlan::PutCommand(int nCommand, void *pParameter, int nSize)
{
	*(int *)(m_pData+STACK_COMMAND) = nCommand;

	if (pParameter != NULL && nSize != 0)
		::memcpy(m_pData+STACK_COMMAND+sizeof(int), pParameter, nSize);
}
*/