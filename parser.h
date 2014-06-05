// Parser.h: interface for the clsParser class.
// The clsParser class defined here is to define a parser for a language
// the language is used to load variables from a text file
// in which variables are defined with name and value which can be a number or a matrix
// the syntax is similiar to assignment command in Matlab
// 
// the clsParser uses parsing method to implement the variable as a practice of parser construction
// the parser is intended to be used in other places like command interpreter
//
// Dong Miaobo, March 21, 2007 
//
//////////////////////////////////////////////////////////////////////

#include "matrix.h"

#include <string.h>

#if !defined(AFX_PARSER_H__77F17135_4ED7_41FB_AF4B_7C012463DB43__INCLUDED_)
#define AFX_PARSER_H__77F17135_4ED7_41FB_AF4B_7C012463DB43__INCLUDED_

#define MAXSTR_TOKEN	256

#ifndef BOOL
#define BOOL	int
#define TRUE	1
#define FALSE	0
#endif

#ifndef NULL
#define NULL	0
#endif

#define STATE_ACCEPTED		55

class clsToken;
class clsSyntaxNode;
class clsParser;

enum ITEM { TOKEN_NUMBER, TOKEN_COMA, TOKEN_SEMICOLON, TOKEN_LBRACKET, TOKEN_RBRACKET,
	TOKEN_IDENTIFICATION, TOKEN_EQUAL, TOKEN_EOF,
	ITEM_ROW, ITEM_MATRIX, ITEM_VALUE, ITEM_STATEMENT, ITEM_PROGRAM, ITEM_MAIN,
	TOKEN_CHAR };

enum FORM { FORM_PROGRAM_NULL, FORM_PROGRAM_PROGRAMSTATEMENT, FORM_VALUE_NUMBER, FORM_VALUE_MATRIX,
	FORM_MATRIX_ROW, FORM_MATRIX_MATRIXROW, FORM_ROW_NUMBER, FORM_ROW_ROWNUMBER, FORM_ROW_ROWCOMANUMBER };

class clsToken {
public:
	clsToken();

public:
	int type;
	char szToken[MAXSTR_TOKEN];

public:
	clsToken &operator=(clsToken &token);

	friend class clsParser;
};

#define MAX_CHILDNODE	8

//SN_TOKEN for basic node constructed by a token

class clsSyntaxNode {
public:
	clsSyntaxNode();
	~clsSyntaxNode();

public:
	int type;
	int form;				//the form indicate how the node is constructed
							//there may be serveral forms to construct a node of one type
							//construction of compound node is defined in clsParser::Collapse(...)

	clsToken token;

	clsSyntaxNode *pParent;
	clsSyntaxNode *pChild[MAX_CHILDNODE];				//pointers to child nodes, up to 8 nodes

public:
	void ConnectChild(int index, clsSyntaxNode *pCh) { pChild[index] = pCh; pCh->pParent = this; }
};

#define MAXSTR_VARIABLENAME		32

enum VARIABLETYPE { VT_DOUBLE, VT_MATRIX };

class clsVariable {
public:
	clsVariable() { name[0] = '\0'; }

public:
	char name[MAXSTR_VARIABLENAME];
	int type;
	double value;
	clsMatrix matrix;

public:
	void ReleaseMatrix() { matrix.Reset(); }
};

#define MAX_STACK				1024				//capacity of the stack in bytes
#define MAX_STACKCOLLAPSE		32				//capacity of the stack to store collapsed node

#define MAX_VARIABLE			512

enum PURPOSE { PURPOSE_SIZE, PURPOSE_VALUE };

class clsParser  
{
public:
	clsParser();
	~clsParser();

protected:
	char *m_pSource;				//string to store source code
	char *m_pCode;				//pointer to the current position

	clsPackage<MAX_STACK> m_stack;

	static int _transfer[18][13];

protected:
	clsVariable m_var[MAX_VARIABLE];
	int m_nVar;

public:
	BOOL Load(const char *pszFile);
	int GetToken(clsToken *pToken);
	clsSyntaxNode *Parse();

	void Parse2();				//use state flow method

	void Output(char *pszFile = NULL);				//NULL for stdout
	void ExecuteProgram(clsSyntaxNode *pProgram);

	void GetVariable(const char *pszName, clsMatrix &mtrx);
	void GetVariable(const char *pszName, double *pDouble);
	void GetVariable(const char *pszName, void *pBuffer, int nBuffer);
	void GetVariable(const char *pszName, clsVector &vctr);

	clsVariable *FindVariable(const char *pszName);

protected:
	void ExecuteStatement(clsSyntaxNode *pStatement);

	void EvaluateMatrix(clsSyntaxNode *pMatrix, clsMatrix &mtrx);

	int EvaluateRow(clsSyntaxNode *pRow, double *pBuffer, int nPurpose);

protected:
	int IfCollapse(int state, int nToken);
	clsSyntaxNode *Collapse(int nCollapse);

//	int Transfer(int state, int nToken);
	int Transfer(int state, int item) { return _transfer[state][item]; }

	int MapIndex(int nItem) { return nItem; }

	void ClearStack();

public:
	static BOOL IsSeperator(char c) { return ::strchr(" \t\r\n", c) != NULL && c != '\0'; }
	static BOOL IsDigit(char ch) { return '0' <= ch && ch <= '9'; }
	static BOOL IsLetter(char ch) {	return ('a' <= ch && ch <= 'z') || ('A' <= ch && ch <= 'Z'); }
};

#endif // !defined(AFX_PARSER_H__77F17135_4ED7_41FB_AF4B_7C012463DB43__INCLUDED_)
