/////////////////////////////////////////////////////////////////////////////
// Matrix DLL (matrix operation dll)

#ifndef _MATRIX_20051024
#define _MATRIX_20051024

#include <math.h>
#include <string.h>

// type defines for ANSI C
#ifndef BOOL
#define BOOL	int
#define TRUE	1
#define FALSE	0
#endif

#ifndef NULL
#define NULL	0
#endif

///////////////////////////////////////////////////////////////////////
//  Length of string
#define LEN_LINE_MATRIXIMPORT	512

#define MAX_SIZE_MATRIX		4*1024*1024
#define MAX_LINES			256*1024
#define MAX_COLUMNS			256*1024

#define MATRIX_BYTE		1				//
#define MATRIX_TEXT		2

#define OK 1

#define LIM_M_MAX 1000000000
#define LIM_M_MIN 0.0000000001

#define GETRGN_COLUMN	1
#define GETRGN_LINE		2

#define MAX_TEMPMATRIXS		8				//
#define MAX_TEMPVECTORS		8				//

#define MEMORYLIMIT_NEW		4*1024*1024
#define MEMORYLIMIT_ALLOC	64*1024*1024

#define ERR_M_DEVIDEDBYZERO	-5
#define ERR_M_COFFLIM		-7

class clsVector;				//
class clsMatrix;				//

class clsVector {
public:
	clsVector();
	clsVector(int m, double *p = NULL, BOOL bAssign = FALSE);
	clsVector(clsVector &v);

	~clsVector();

protected:
	double *pV;
	int m;

	BOOL m_bAutoDelete;

public:
// Attributes
	double &operator [] (int i) { return pV[i]; }
	BOOL IsEmpty() { return (pV == NULL); }

// Operations
	void Reset();
	void Reset(clsVector &V);
	void Reset(int m, double *pV = NULL, BOOL bAssign = FALSE);

	clsVector & operator = (clsVector &V);
	clsVector & operator = (double *pV);
	clsVector & operator = (double x);
	clsVector & operator += (clsVector &V);
	clsVector & operator += (double *pV);
	clsVector & operator -= (clsVector &V);
	clsVector & operator -= (double *pV);
	clsVector & operator *= (double a);
	clsVector & operator /= (double a);

	double operator*(clsVector &V);

	static void X3(clsVector &V1, clsVector &V2, clsVector &V);				//only for clsVector(3)
	static void X3(clsVector &V1, clsMatrix &M2, clsMatrix &M);				//
	static void X(clsVector &V1, clsVector &V2, clsVector &V);				//<x1*x2, y1*y2, z1*z2,...>
	static void X(clsVector &V1, clsVector &V2, clsMatrix &M);				//Mij = V1i*V2j

	BOOL EaqualSize(clsVector &v){ return (m == v.m); }
	int GetM() { return m; }
	double *GetP() { return pV; }

	double e2();
	double e2s();				//e2*e2
	double ei();

	friend class clsMatrix;
};

class clsMatrix {
protected:
	double *pM;
	int m;
	int n;

	BOOL m_bAutoDelete;
	//m_bAutoDeletenewbAutoDeleteTRUEbAutoDeleteFALSE

public:
	clsMatrix();
	clsMatrix(int m, int n, double *p = NULL, BOOL bAssign = FALSE);

	~clsMatrix();

	void Reset();
	void Reset(int i, int j, double *p = NULL, BOOL bAssign = FALSE);

//	Attributes
	int GetM() { return m; }
	int GetN() { return n; }
	double *GetP() { return pM; }

	BOOL EqualSize(clsMatrix &M) { return ((m == M.m) && (n == M.n)); }

public:
//	Operators
	double * operator [] (int i) { return pM+i*n; }

	clsMatrix & operator = (clsMatrix &M);
	clsMatrix & operator = (double *p);
	clsMatrix & operator = (double x);
	clsMatrix & operator += (clsMatrix &M);
	clsMatrix & operator += (double *p);
	clsMatrix & operator -= (clsMatrix &M);
	clsMatrix & operator -= (double *p);
	clsMatrix & operator *= (clsMatrix &mtrx);				//
	clsMatrix & operator *= (double *p);
	clsMatrix & operator *= (double a);
	clsMatrix & operator /= (double a);

	static void X(clsMatrix &mtrx1, clsMatrix &mtrx2, clsMatrix &mtrx);				//multiple
	static void X(clsMatrix &mtrx1, clsVector &vctr2, clsVector &vctr);
	static void X(clsVector &vctr1, clsMatrix &mtrx2, clsVector &vctr);
	static void T(clsMatrix &mtrx1, clsMatrix &mtrx);				//transpose
	static void R(clsMatrix &mtrx1, clsMatrix &mtrx);				//inverse
	static void S(clsMatrix &mtrx1, int m0, int m1, int n0, int n1, clsMatrix &mtrx, int m = 0, int n = 0);
	static void S(clsVector &vctr1, int m0, int m1, clsVector &vctr, int m = 0);

	double e();
	double e2();
	double ei();

public:
	BOOL LoadT(char *pszFile);
	BOOL SaveT(char *pszFile);

	friend class clsVector;
};

class clsMetric {
public:
	static void Add(double a[3], double b[3], double x[3]) {
		x[0] = a[0] + b[0]; x[1] = a[1] + b[1]; x[2] = a[2] + b[2]; }
	static void Sub(double a[3], double b[3], double x[3]) {
		x[0] = a[0] - b[0]; x[1] = a[1] - b[1]; x[2] = a[2] - b[2]; }
	static void Copy(double x[3], const double x1[3]) {
		x[0] = x1[0]; x[1] = x1[1]; x[2] = x1[2]; }
	static double Distance(double x1[3], double x2[3], double x[3] = NULL);
	static double Distance2(double x1[2], double x2[2], double x[2] = NULL);
	static void AttitudeToTransformMatrix(double att[3], double Mfg[3][3], double Mgf[3][3]);
	static void X(double Mgf[3][3], double xf[3], double xg[3]);
	static void X(double Mab[3][3], double Mbc[3][3], double Mac[3][3]);

	static double Norm(double x[3]) { return ::sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]); }
};

template <int N>				//N - capacity
class clsPackage {
public:
	int size;
	char content[N];

public:
	clsPackage() { size = 0; }

public:
	void clear() { size = 0; }
	int GetSize() { return size; }

	void push(char chData) { content[size++] = chData;}
	void push(unsigned char chData) { (unsigned char &)content[size++] = chData; }
	void push(short nData) { (short &)content[size] = nData; size += sizeof(short); }
	void push(unsigned short nData) { (unsigned short &)content[size] = nData; size += sizeof(unsigned short); }
	void push(int nData) { (int &)content[size] = nData; size += sizeof(int); }
	void push(unsigned int nData) { (unsigned int &)content[size] = nData; size += sizeof(unsigned int); }
	void push(float fData) { (float &)content[size] = fData; size += sizeof(float); }
	void push(double dData) { (double &)content[size] = dData; size += sizeof(double); }
	void push(void *pData) { (void *&)content[size] = pData; size += sizeof(void *); }
	void push(void *pBuffer, int nBuffer) { ::memcpy(content+size, pBuffer, nBuffer); size += nBuffer; }

	void pop(char *pData) { *pData = content[--size]; }
	void pop(unsigned char *pData) { *pData = (unsigned char &)content[--size]; }
	void pop(short *pData) { *pData = (short &)content[size-=sizeof(short)]; }
	void pop(unsigned short *pData) { *pData = (unsigned short &)content[size-=sizeof(unsigned short)]; }
	void pop(int *pData) { *pData = (int &)content[size-=sizeof(int)]; }
	void pop(unsigned int *pData) { *pData = (unsigned int &)content[size-=sizeof(unsigned int)]; }
	void pop(float *pData) { *pData = (float &)content[size-=sizeof(float)]; }
	void pop(double *pData) { *pData = (double &)content[size-=sizeof(double)]; }
	void pop(void **pData) { *pData = (void *&)content[size-=sizeof(void *)]; }
	void pop(void *pBuffer, int nBuffer) { ::memcpy(pBuffer, &content[size-=nBuffer], nBuffer); }

	void pop(int nPop) { size -= nPop; }				//just skip top nPop bytes

	void peep(char *pData) { *pData = content[size-1]; }
	void peep(unsigned char *pData) { *pData = (unsigned char &)content[size-1]; }
	void peep(short *pData) { *pData = (short &)content[size-sizeof(short)]; }
	void peep(unsigned short *pData) { *pData = (unsigned short &)content[size-sizeof(unsigned short)]; }
	void peep(int *pData) { *pData = (int &)content[size-sizeof(int)]; }
	void peep(unsigned int *pData) { *pData = (unsigned int &)content[size-sizeof(unsigned int)]; }
	void peep(float *pData) { *pData = (float &)content[size-sizeof(float)]; }
	void peep(double *pData) { *pData = (double &)content[size-sizeof(double)]; }
	void peep(void **pData) { *pData = (void *&)content[size-sizeof(void *)]; }
	void peep(void *pBuffer, int nBuffer) { ::memcpy(pBuffer, &content[size-nBuffer], nBuffer); }

	unsigned short check() {
		unsigned short check = 0;
		for (int i=0; i<=size-1; i++) check += (unsigned char)content[i];
		return check;
	}

	clsPackage<N> &operator=(clsPackage<N> &package) {
		size = package.size;
		::memcpy(content, package.content, size);
		return *this;
	}
};

#endif
