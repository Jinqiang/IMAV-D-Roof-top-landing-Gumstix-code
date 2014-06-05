/*
 * im9.h
 *
 *  Created on: Aug 27, 2012
 *      Author: nusuav
 */

#ifndef IM9_H_
#define IM9_H_

#include "svo.h"
#include "thread.h"

#define MAX_IM9PACK 128

struct  IM9PACK {
	double a, b, c;
	double p, q, r;
	double acx, acy, acz;
	double ail, ele, thr, rud, tog;
};

class clsIM9 : public clsThread {
public:
	clsIM9();
	virtual ~clsIM9();

public:
	int m_nsIM9;

protected:
	int m_nIM9;
	bool m_bSAFMC2014DropTarget;
	//to store received packs
	IM9PACK m_im90;
	double m_tIM9[MAX_IM9PACK];

//	HELICOPTERRUDDER m_MAN[MAX_IM9PACK];
	SVODATA m_MAN[MAX_IM9PACK];

	pthread_mutex_t m_mtxIM9;

public:
	void PX4Update(IM9PACK& pack);
	void SetDropSAFMC2014Target(){
		m_bSAFMC2014DropTarget = true;
	}

public:
	IM9PACK GetIM9Pack() const { return m_im90;}
	virtual BOOL InitThread();
	virtual int EveryRun();
	virtual void ExitThread();

	friend class clsURG;
	friend class clsDLG;
};

#endif /* IM9_H_ */
