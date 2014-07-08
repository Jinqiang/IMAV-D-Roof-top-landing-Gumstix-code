/*
 * clsPTU.h
 *
 *  Created on: Apr 8, 2013
 *      Author: nusuav
 */

#ifndef PTU_H_
#define PTU_H_

#include "thread.h"

class clsPTU: public clsThread {
public:
	clsPTU();
	virtual ~clsPTU();
protected:
	int m_nsPTU;
	double m_panAngle, m_tiltAngle;
	double m_PTU2_panAngle, m_PTU2_tiltAngle;
	double m_panAngleOffset, m_tiltAngleOffset;
	double m_PTU2_panAngleOffset, m_PTU2_tiltAngleOffset;

public:
	virtual BOOL InitThread();
	virtual int EveryRun();
	virtual void ExitThread();
	void PanTiltConpensateUAV(double phiUAV, double thtUAV, double* phiPT, double* thtPT);
	void SetTiltAngle(double degree){
		m_tiltAngleOffset = degree*3.1415926/180.0;
	}
	void SetPanAngle(double degree){
		m_panAngleOffset = degree*3.1415926/180.0;
	}
};

#endif /* NAV_100_H_ */
