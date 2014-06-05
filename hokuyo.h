/*
 * hokuyo.h
 *
 *  Created on: 2012-1-7
 *      Author: Dong
 */

#ifndef HOKUYO_H_
#define HOKUYO_H_

#ifndef BOOL
#define BOOL	int
#define TRUE	1
#define FALSE	0
#endif

BOOL urg_init();
int urg_requestdata(long *data);
void urg_exit();

void urg_test();

#endif /* HOKUYO_H_ */
