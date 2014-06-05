//	Plan.h
//	This file is to define classes for plan framework
//	
//	Control & Simulation Laboratory
//	2005-08-22

#ifndef _PLAN_H_20050822
#define	_PLAN_H_20050822

#ifndef BOOL
#define BOOL	int
#define TRUE	1
#define FALSE	0
#endif

// data stack structure 
#define STACK_STATE		0
#define STACK_COMMAND	1024
#define STACK_BEHAVIOR	2048

#define STACKSIZE		64*1024

struct STATE {
	double x, y, z;
	double u, v, w;
	double a, b, c;
	double p, q, r;
};

struct BEHAVIOR;
/*
class clsPlan {
public:
	clsPlan();
	~clsPlan();

public:
	BOOL Load(char *pszPlan);
	BOOL LoadFromFile(char *pszFile);

protected:
	char *m_pStack, *m_pData;

public:
	void SetStack(char *pStack, char *pData) { m_pStack = pStack; m_pData = pData; }

public:
	int Call();
	int Call0(char *pszFunc = NULL, BOOL bStart = TRUE);				//call specific function

public:
//	BEHAVIOR *GetBehavior() { return (BEHAVIOR *)(m_pData+STACK_BEHAVIOR); }				//return pointer to behavior and parameter
//	void GetBehavior(BEHAVIOR *pBehavior) { *pBehavior = *GetBehavior(); }
	
	BEHAVIOR *GetBehavior() { return NULL; }				//empty so far
	void GetBehavior(BEHAVIOR *pBehavior) {}				//empty so far

	void PutCommand(int nCommand, void *pParameter = NULL, int nSize = 0);				//universal form
	void PutState(STATE *pState);
};
*/

#endif