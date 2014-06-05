//#include <math.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "matrix.h"

clsVector::clsVector()
{
	m = 0; pV = NULL; m_bAutoDelete = FALSE;
}

clsVector::clsVector(clsVector &v)
{
	m = 0; pV = NULL; m_bAutoDelete = FALSE;

	Reset(v);
}

clsVector::clsVector(int i, double *p, BOOL bAssign)
{
	m = 0; pV = NULL; m_bAutoDelete = FALSE;

	Reset(i, p, bAssign);
}

clsVector::~clsVector()
{
	Reset();
}

void clsVector::Reset()
{
	if (pV != NULL) {
		if (m_bAutoDelete) delete[] pV;
		pV = NULL;
	}

	m = 0; m_bAutoDelete = FALSE;
}

void clsVector::Reset(int i, double *p, BOOL bAssign)
{
	assert(i > 0 && p != NULL);

	if (i <= 0) return;

	if (pV != NULL) {
		if (m_bAutoDelete) delete[] pV;
	}

	m = i;
	if (bAssign) pV = p;				//p!=NULL
	else {
		pV = new double[m];

		if (p == NULL) ::memset(pV, 0, m*sizeof(double));
		else ::memcpy(pV, p, m*sizeof(double));
	}

	m_bAutoDelete = !bAssign;
}

void clsVector::Reset(clsVector &V)
{
	Reset(V.m, V.pV);
}

clsVector & clsVector::operator = (clsVector &V)
{
	int i;
	if (!GetP()) { m = V.m; pV = new double[m]; }
	int iN = m < V.m ? m : V.m;

	for (i=0; i<=iN-1; i++) pV[i] = V.pV[i];
	return *this;
}

clsVector & clsVector::operator = (double *V)
{
	int i;

	for (i=0; i<=m-1; i++) pV[i] = V[i];
	return *this;
}

clsVector & clsVector::operator = (double x)
{
	int i;

	if (!GetP()) return *this;
	for (i=0; i<=m-1; i++) pV[i] = x;
	return *this;
}

clsVector & clsVector::operator += (clsVector &V)
{
	assert(m == V.m);

	int i;

	int iN = m < V.m ? m : V.m;

	for (i=0; i<=iN-1; i++) pV[i] += V.pV[i];
	
	return *this;
}

clsVector & clsVector::operator += (double *p)
{
	int i;

	for (i=0; i<=m-1; i++) pV[i] += p[i];
	
	return *this;
}

clsVector & clsVector::operator -= (clsVector &V)
{
	assert(m == V.m);

	int i;

	int iN = m < V.m ? m : V.m;
	
	for (i=0; i<=iN-1; i++) pV[i] -= V.pV[i];
	
	return *this;
}

clsVector & clsVector::operator -= (double *p)
{
	for (int i=0; i<=m-1; i++) pV[i] -= p[i];
	
	return *this;
}

clsVector & clsVector::operator *= (double a)
{
	for (int i=0; i<=m-1; i++) pV[i] *= a;
	return *this;
}

clsVector & clsVector::operator/=(double a)
{
	for (int i=0; i<=m-1; i++) pV[i] /= a;
	return *this;
}

double clsVector::operator*(clsVector &V)
{
	int i;
	double a;

	a = 0;
	for (i=0; i<=m-1; i++) a += pV[i] * V.pV[i];
	return a;
}

double clsVector::e2()
{
	double error = 0;

	for (int i=0; i<=m-1; i++) error += pV[i]*pV[i];

	return ::sqrt(error);
}

double clsVector::e2s()
{
	double error = 0;

	for (int i=0; i<=m-1; i++) error += pV[i]*pV[i];

	return error;
}

double clsVector::ei()
{
	double e = 0;
	
	for (int i=0; i<=m-1; i++) {
		double ej = ::fabs(pV[i]);
		if (ej > e) e = ej;
	}

	return e;
}

void clsVector::X(clsVector &V1, clsVector &V2, clsMatrix &M)
{
	assert((M.m == V1.m) && (M.n == V2.m));

	int i, j;
	for (i=0; i<=M.m-1; i++) for (j=0; j<=M.n-1; j++) M[i][j] = V1[i]*V2[j];
}

void clsVector::X3(clsVector &V1, clsVector &V2, clsVector &V)
{
	assert((V.m == 3) && (V1.m == 3) && (V2.m == 3));

	V[0] = V1[1]*V2[2] - V1[2]*V2[1];
	V[1] = V1[2]*V2[0] - V1[0]*V2[2];
	V[2] = V1[0]*V2[1] - V1[1]*V2[0];
}

void clsVector::X3(clsVector &V1, clsMatrix &M2, clsMatrix &M)
{
	assert((V1.m == 3) && (M2.m == 3) && (M2.n == 3) && (M.m == 3) && (M.n == 3));
	for (int j=0; j<=M.n-1; j++) {
		M[0][j] = V1[1]*M2[2][j] - V1[2]*M2[1][j];
		M[1][j] = V1[2]*M2[0][j] - V1[0]*M2[2][j];
		M[2][j] = V1[0]*M2[1][j] - V1[1]*M2[0][j];
	}
}

void clsVector::X(clsVector &V1, clsVector &V2, clsVector &V)
{
	assert((V1.m==V2.m) && (V2.m==V.m));

	for (int i=0; i<=V.m-1; i++) V[i] = V1[i] * V2[i];
}

clsMatrix::clsMatrix()
{
	m = n = 0; pM = NULL; m_bAutoDelete = FALSE;
}

clsMatrix::clsMatrix(int i, int j, double *p, BOOL bAssign)
{
	m = n = 0; pM = NULL; m_bAutoDelete = FALSE;

	Reset(i, j, p, bAssign);
}

clsMatrix::~clsMatrix() { Reset(); }

void clsMatrix::Reset(int i, int j, double *p, BOOL bAssign)
{
//	assert(i>0 && j>0);
//	assert(!bAssign || p!= NULL);				//p must not be NULL, if bAssign

	m = i; n = j;
	if (pM != NULL && m_bAutoDelete) delete[] pM;

	if (bAssign) pM = p;
	else {
		int nSize = m*n*sizeof(double);
		pM = new double[m*n];
		
		if (p == NULL) ::memset(pM, 0, nSize);
		else ::memcpy(pM, p, nSize);
	}

	m_bAutoDelete = !bAssign;
}

void clsMatrix::Reset()
{
	m = n = 0;
	if (pM == NULL) return;

	if (m_bAutoDelete) delete[] pM;
	pM = NULL;
}

clsMatrix & clsMatrix::operator = (clsMatrix &M)
{
	//assert (M.m >= 1 && M.n >= 1)
	if (m != M.m || n != M.n) Reset(M.m, M.n);

	::memcpy(pM, M.pM, m*n*sizeof(double));

	return *this;
}

clsMatrix &clsMatrix::operator=(double *p)
{
	::memcpy(pM, p, m*n*sizeof(double));
	return *this;
}

clsMatrix & clsMatrix::operator = (double x)
{
	int i, j;

	for (i=0; i<=m-1; i++) for (j=0; j<=n-1; j++) pM[i*n+j] = x;
	return *this;
}

clsMatrix & clsMatrix::operator += (clsMatrix &M)
{
	assert(m==M.m && n==M.n);

	for (int i=0; i<=m-1; i++) for (int j=0; j<=n-1; j++) pM[i*n+j] += M.pM[i*M.n+j];

	return *this;
}

clsMatrix &clsMatrix::operator+=(double *p)
{
	for (int i=0; i<=m-1; i++) for (int j=0; j<=n-1; j++) pM[i*n+j] += p[i*n+j];
	return *this;
}

clsMatrix & clsMatrix::operator -= (clsMatrix &M)
{
	assert((m==M.m) && (n==M.n));

	for (int i=0; i<=m-1; i++ ) for (int j=0; j<=n-1; j++) pM[i*n+j] -= M.pM[i*M.n+j];

	return *this;
}

clsMatrix &clsMatrix::operator-=(double *p)
{
	for (int i=0; i<=m-1; i++ ) for (int j=0; j<=n-1; j++) pM[i*n+j] -= p[i*n+j];
	return *this;
}

clsMatrix & clsMatrix::operator*=(double a)
{
	int i, j;

	for (i=0; i<=m-1; i++) for (j=0; j<=n-1; j++) pM[i*n+j] *= a;
	return *this;
}

void clsMatrix::X(clsMatrix &mtrx1, clsMatrix &mtrx2, clsMatrix &mtrx)
{
	int m = mtrx.m;
	int n = mtrx.n;

	assert((m==mtrx1.m) && (n==mtrx2.n) && (mtrx1.n==mtrx2.m));
	
	for (int i=0; i<=m-1; i++) for (int j=0; j<=n-1; j++) {
		double d = 0;
		for (int k=0; k<=mtrx1.n-1; k++) {
			d += mtrx1[i][k] * mtrx2[k][j];
		}
		mtrx.pM[i*n+j] = d;
//		cout<<"mtrx.pM[i*n+j]"<<i*n+j<<" "<<d<<endl;
	}
}

void clsMatrix::X(clsVector &vctr1, clsMatrix &mtrx2, clsVector &vctr)
{
	assert(vctr1.m == mtrx2.m && mtrx2.n == vctr.m);

	int m = mtrx2.m;
	int n = mtrx2.n;

	for (int j=0; j<=n-1; j++) {
		double d = 0;
		for (int i=0; i<=m-1; i++) d += vctr1.pV[i]*mtrx2.pM[i*n+j];
		vctr.pV[j] = d;
	}
}

void clsMatrix::X(clsMatrix &mtrx1, clsVector &vctr2, clsVector &vctr)
{
	assert((mtrx1.n==vctr2.m) && (mtrx1.m==vctr.m));

	int m = mtrx1.m;
	int n = mtrx1.n;

	for (int i=0; i<=m-1; i++) {
		double d = 0;
		for (int j=0; j<=n-1; j++) d += mtrx1.pM[i*n+j]*vctr2.pV[j];
		vctr.pV[i] = d;
	}
}

double clsMatrix::e()
{
	double error = 0;

	for (int i=0; i<=m-1; i++) for (int j=0; j<=n-1; j++) error += pM[i*n+j]*pM[i*n+j];
	
	return ::sqrt(error);
}

double clsMatrix::e2()
{
	double error = 0;

	for (int i=0; i<=m-1; i++) for (int j=0; j<=n-1; j++) error += pM[i*n+j]*pM[i*n+j];
	
	return error/2;
}

double clsMatrix::ei()
{
	double e = 0;
	for (int i=0; i<=m-1; i++) for (int j=0; j<=n-1; j++) {
		double eij = ::fabs(pM[i*n+j]);
		if (eij > e) e = eij;
	}
	
	return e;
}

void clsMatrix::T(clsMatrix &mtrx1, clsMatrix &mtrx)
{
	assert((mtrx1.m == mtrx.n) && (mtrx1.n == mtrx.m));

	int m = mtrx.m; int n = mtrx.n;
	for (int i=0; i<=m-1; i++) for (int j=0; j<=n-1; j++) mtrx.pM[i*n+j] = mtrx1.pM[j*m+i];
}

void clsMatrix::S(clsMatrix &mtrx1, int m0, int m1, int n0, int n1, clsMatrix &mtrx, int m, int n)
{
	int ms = m1 - m0 + 1; int ns = n1 - n0 + 1;

	for (int i=0; i<=ms-1; i++) 
		for (int j=0; j<=ns-1; j++)
			mtrx[m+i][n+j] = mtrx1[m0+i][n0+j];
}

void clsMatrix::S(clsVector &vctr1, int m0, int m1, clsVector &vctr, int m)
{
	int ms = m1 - m0 + 1;

	for (int i=0; i<=ms-1; i++) vctr[m+i] = vctr1[m0+i];
}

void clsMatrix::R(clsMatrix &mtrx1, clsMatrix &mtrx)
{
	int i, j, j2, k;
	double r;

	int nResult;
	int m=mtrx1.m;
	int n=mtrx1.n;

	assert((m==mtrx1.n) && (mtrx.m==mtrx.n) && (m==mtrx.m));

	clsMatrix MB(mtrx1);

	mtrx = 0.0; for (i=0; i<=m-1; i++) mtrx[i][i] = 1.0;

	nResult = OK;
	for (k=0; k<=n-1; k++) {
		// Select The Colomun Of Largest Head
		j = k; r = fabs(MB.pM[k*n+k]);
		for (j2=k+1; j2<=n-1; j2++) if (fabs(MB.pM[k*n+j2])>r) { j=j2; r=fabs(MB.pM[k*n+j2]); }
		if (j!=k) {
			for (i=k; i<=m-1; i++) {
				r = MB.pM[i*n+k];
				MB.pM[i*n+k] = MB.pM[i*n+j];
				MB.pM[i*n+j] = r;
			}
			for (i=0; i<=m-1; i++) {
				r = mtrx.pM[i*n+k];
				mtrx.pM[i*n+k] = mtrx.pM[i*n+j];
				mtrx.pM[i*n+j] = r;
			}
		}

		for (j=k+1; j<=n-1; j++) {
			if (MB.pM[k*n+k]==0) { nResult = ERR_M_DEVIDEDBYZERO; break; }
			r = MB.pM[k*n+j]/MB.pM[k*n+k];
			MB.pM[k*n+j] = 0;
			for (i=k+1; i<=m-1; i++) MB.pM[i*n+j] -= r*MB.pM[i*n+k];
			for (i=0; i<=m-1; i++) mtrx.pM[i*n+j] -= r*mtrx.pM[i*n+k];
		}
		if (nResult == ERR_M_DEVIDEDBYZERO) break;

		r = MB.pM[k*n+k];
		if (fabs(r)<LIM_M_MIN) nResult = ERR_M_COFFLIM;
		MB.pM[k*n+k] = 1.0;
		for (i=k+1; i<=m-1; i++) MB.pM[i*n+k] /= r;
		for (i=0; i<=m-1; i++) mtrx.pM[i*n+k] /= r;
	}
	if (nResult == ERR_M_DEVIDEDBYZERO) throw;

	for (k=n-1; k>=0; k--) {
		for (j=k-1; j>=0; j--) {
			r = MB.pM[k*n+j];
			MB.pM[k*n+j] = 0;
			for (i=0; i<=m-1; i++) mtrx.pM[i*n+j] -= r*mtrx.pM[i*n+k];
		}
	}
}

clsMatrix &clsMatrix::operator*=(clsMatrix &mtrx)
{
	assert((m==mtrx.m) && (n==mtrx.n));

	return (*this)*=mtrx.pM;
}

clsMatrix &clsMatrix::operator*=(double *p)
{
	for (int i=0; i<=m-1; i++) for (int j=0; j<=n-1; j++) pM[i*n+j] *= p[i*n+j];
	return *this;
}

clsMatrix &clsMatrix::operator/=(double a)
{
	for (int i=0; i<=m-1; i++) for (int j=0; j<=n-1; j++) pM[i*n+j] /= a;
	return *this;
}

double clsMetric::Distance(double x1[3], double x2[3], double x[3])
{
	double de[3];
	clsMetric::Sub(x2, x1, de);

	if (x != NULL) {
		x[0] = de[0]; x[1] = de[1]; x[2] = de[2]; }

	double r = ::sqrt(de[0]*de[0]+de[1]*de[1]+de[2]*de[2]);

	return r;
}

double clsMetric::Distance2(double x1[2], double x2[2], double x[2])
{
	double de[2] = { x2[0] - x1[0], x2[1] - x1[1] };

	if (x != NULL) { x[0] = de[0]; x[1] = de[1]; }

	double r = ::sqrt(de[0]*de[0]+de[1]*de[1]);

	return r;
}

void clsMetric::AttitudeToTransformMatrix(double att[3], double Mfg[3][3], double Mgf[3][3])
{
	double sina = ::sin(att[0]); double sinb = ::sin(att[1]); double sinc = ::sin(att[2]);
	double cosa = ::cos(att[0]); double cosb = ::cos(att[1]); double cosc = ::cos(att[2]);

	if (Mfg != NULL) {
		Mfg[0][0] = cosb*cosc;					Mfg[0][1] = cosb*sinc;					Mfg[0][2] = -sinb;
		Mfg[1][0] = sina*sinb*cosc-cosa*sinc;	Mfg[1][1] = sina*sinb*sinc+cosa*cosc;	Mfg[1][2] = sina*cosb;
		Mfg[2][0] = cosa*sinb*cosc+sina*sinc;	Mfg[2][1] = cosa*sinb*sinc-sina*cosc;	Mfg[2][2] = cosa*cosb;
	}

	if (Mgf == NULL) return;

	if (Mfg != NULL) {
		Mgf[0][0] = Mfg[0][0]; Mgf[0][1] = Mfg[1][0]; Mgf[0][2] = Mfg[2][0];
		Mgf[1][0] = Mfg[0][1]; Mgf[1][1] = Mfg[1][1]; Mgf[1][2] = Mfg[2][1];
		Mgf[2][0] = Mfg[0][2]; Mgf[2][1] = Mfg[1][2]; Mgf[2][2] = Mfg[2][2];
	}
	else {
		Mgf[0][0] = cosb*cosc;	Mgf[0][1] = sina*sinb*cosc-cosa*sinc;	Mgf[0][2] = cosa*sinb*cosc+sina*sinc;
		Mgf[1][0] = cosb*sinc;	Mgf[1][1] = sina*sinb*sinc+cosa*cosc;	Mgf[1][2] = cosa*sinb*sinc-sina*cosc;
		Mgf[2][0] = -sinb;		Mgf[2][1] = sina*cosb;					Mgf[2][2] = cosa*cosb;
	}
}

void clsMetric::X(double Mgf[3][3], double xf[3], double xg[3])
{
	xg[0] = Mgf[0][0]*xf[0]+Mgf[0][1]*xf[1]+Mgf[0][2]*xf[2];
	xg[1] = Mgf[1][0]*xf[0]+Mgf[1][1]*xf[1]+Mgf[1][2]*xf[2];
	xg[2] = Mgf[2][0]*xf[0]+Mgf[2][1]*xf[1]+Mgf[2][2]*xf[2];
}

void clsMetric::X(double Mab[3][3], double Mbc[3][3], double Mac[3][3])
{
	for (int i=0; i<=2; i++) for (int j=0; j<=2; j++) {
		Mac[i][j] = 0;
		for (int k=0; k<=2; k++) Mac[i][j] += Mab[i][k]*Mbc[k][j];
	}
}

static char STR_SEPARATOR[] = " \t";
BOOL clsMatrix::LoadT(char * lpszFile)
{
	FILE *pfMatrix = ::fopen(lpszFile, "r");
	if (pfMatrix == NULL) return FALSE;

	char strLine[LEN_LINE_MATRIXIMPORT];
	::memset(strLine, 0, LEN_LINE_MATRIXIMPORT);
	int m2, n2;

	if (::fgets(strLine, LEN_LINE_MATRIXIMPORT-1, pfMatrix) == NULL) { ::fclose(pfMatrix); return FALSE; }

	char *pChar = strLine;
	pChar += ::strspn(pChar, STR_SEPARATOR);
	int j;
	for (j=0; ; j++) {
		if (*pChar == '\0') break;
		pChar += ::strcspn(pChar, STR_SEPARATOR);
		pChar += ::strspn(pChar, STR_SEPARATOR);
	}
	n2 = j;
	if (j<=0) { ::fclose(pfMatrix); return FALSE; }
	
	BOOL bImport = TRUE;
	int i;
	for (i=1; ; i++) {
		if (::fgets(strLine, LEN_LINE_MATRIXIMPORT-1, pfMatrix) == NULL) break;

		pChar = strLine;
		pChar += ::strspn(pChar, STR_SEPARATOR);
		for (j=0; ; j++) {
			if (*pChar == '\0') break;
			pChar += ::strcspn(pChar, STR_SEPARATOR);
			pChar += ::strspn(pChar, STR_SEPARATOR);
		}
		if (j!=n2) { bImport = FALSE; break; }
	}
	if (!bImport) { ::fclose(pfMatrix); return FALSE; }
	m2 = i;

	if ((m2 > MAX_LINES) || (n2 > MAX_COLUMNS) || (m2*n2 > MAX_SIZE_MATRIX)) { ::fclose(pfMatrix); return FALSE; }
	Reset(m2, n2);

	::rewind(pfMatrix);
	bImport = TRUE;
	for (i=0; i<=m-1; i++) {
		if (::fgets(strLine, LEN_LINE_MATRIXIMPORT-1, pfMatrix) == NULL) { bImport = FALSE; break; }
		
		pChar = strLine;
		pChar += ::strspn(pChar, STR_SEPARATOR);
		for (j=0; j<=n-1; j++) {
			if (::sscanf(pChar, "%lf", &(*this)[i][j]) < 1) { bImport = FALSE; break; }
			pChar += ::strcspn(pChar, STR_SEPARATOR);
			pChar += ::strspn(pChar, STR_SEPARATOR);
		}
		if (!bImport) break;
	}
	if (!bImport) Reset();

	::fclose(pfMatrix);
	return bImport;
}

BOOL clsMatrix::SaveT(char * lpszFile)
{
	if (!GetP()) return FALSE;

	FILE *pfMatrix = ::fopen(lpszFile, "w");
	if (pfMatrix == NULL) return FALSE;

	BOOL bExport = TRUE;
	char strLine[LEN_LINE_MATRIXIMPORT], str[LEN_LINE_MATRIXIMPORT];
	for (int i=0; i<=GetM()-1; i++) {
		for (int j=0; j<=GetN()-1; j++) {
			::sprintf(str, "%f", (*this)[i][j]);
			if (::strlen(strLine) + ::strlen(str) > LEN_LINE_MATRIXIMPORT - 1) { bExport = FALSE; break; }
			::strcat(strLine, str);
		}
		if (!bExport) break;
		if (::fputs(strLine, pfMatrix) == EOF) { bExport = FALSE; break; }
	}

	::fclose(pfMatrix);
	return bExport;
}
