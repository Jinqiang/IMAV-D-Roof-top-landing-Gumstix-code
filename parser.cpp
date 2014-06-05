// Parser.cpp: implementation of the clsParser class.
//
// The clsParser class defined here is to define a parser for a language
// the language is used to load variables from a text file
// in which variables are defined with name and value which can be a number or a matrix
// the syntax is similiar to assignment commands in Matlab
// 
// the clsParser uses parsing method to implement the variable loader as a practice of parser construction
// the parser is intended to be used in other places like command interpreter
//
// Dong Miaobo, March 21, 2007 
//
// for principle of this parser, refer to technical note "TN12 - Compiler and Interpreter.doc"
// 
//////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "matrix.h"
#include "parser.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

clsToken::clsToken()
{
	type = 0;
	szToken[0] = '\0';
}

clsToken &clsToken::operator=(clsToken &token)
{
	type = token.type;
	::strcpy(szToken, token.szToken);

	return *this;
}

clsParser::clsParser()
{
	m_pSource = NULL;
	m_pCode = NULL;

	m_nVar = 0;
}

clsParser::~clsParser()
{
	if (m_pSource != NULL) delete m_pSource;
}

BOOL clsParser::Load(const char *pszFile)
{
	FILE *pfLoad = ::fopen(pszFile, "rb");
	if (pfLoad == NULL) return FALSE;

	::fseek(pfLoad, 0, SEEK_END);
	int length = ::ftell(pfLoad);

	::rewind(pfLoad);
	if (m_pSource != NULL) delete m_pSource;

	m_pSource = new char[length+1];
	::fread(m_pSource, 1, length, pfLoad);
	m_pSource[length] = '\0';

	m_pCode = m_pSource;
	::fclose(pfLoad);

	return TRUE;
}

int clsParser::GetToken(clsToken *pToken)
{
	char *pCode0 = m_pCode;
	while (1) {
		if (IsSeperator(pCode0[0])) pCode0 ++;
		else if (pCode0[0] == '%') pCode0 = ::strchr(pCode0, '\n') + 1;				//skip comment up to next line
		else break;
	}

	if (pCode0[0] == '\0') {
		pToken->type = TOKEN_EOF; 
		pToken->szToken[0] = '\0';
		return TOKEN_EOF;
	}

	int state = 0;
	char *pCode = NULL;				//next code position
	clsToken token;

	for (char *pChar = pCode0; ; pChar ++) {
		if (pChar[0] == '%') pChar = ::strchr(pChar, '\n') + 1;				//skip comment up to next line
		switch (state) {				//state, -1 for end, -2 for end (the code pointer move to next), -5 for dead
		case 0:				//beginning
			if (pChar[0] == '=') {
				state = -2;
				token.type = TOKEN_EQUAL;
			}
			else if (pChar[0] == '[') {
				state = -2;
				token.type = TOKEN_LBRACKET;
			}
			else if (pChar[0] == ']') {
				state = -2;
				token.type = TOKEN_RBRACKET;
			}
			else if (pChar[0] == ',') {
				state = -2;
				token.type = TOKEN_COMA;
			}
			else if (pChar[0] == ';') {
				state = -2;
				token.type = TOKEN_SEMICOLON;
			}
			else if (IsLetter(pChar[0]) || pChar[0] == '_') {
				state = 1;				//identification
			}
			else if (pChar[0] == '-') {
				state = 2;
			}
			else if ('0' <= pChar[0] && pChar[0] <= '9') {
				state = 3;
			}
			else if (pChar[0] == '.') {
				state = 4;
			}
			else {
				state = -2;
				token.type = TOKEN_CHAR;
			}
			break;

		case 1:
			if (IsLetter(pChar[0]) || IsDigit(pChar[0]) || pChar[0] == '_') {
				state = 1;				//identification
			}
			else {
				state = -1;
				token.type = TOKEN_IDENTIFICATION;
			}
			break;

		case 2:
			if (IsDigit(pChar[0])) state = 3;
			else if (pChar[0] == '.') state = 4;
			else state = -5;
			break;
		case 3:
			if (IsDigit(pChar[0])) {}				//do nothing, keep state
			else if (pChar[0] == '.') state = 4;
			else if (pChar[0] == 'e' || pChar[0] == 'E') state = 6;
			else {				//exit
				state = -1;
				token.type = TOKEN_NUMBER;
			};
			break;
		case 4:
			if (IsDigit(pChar[0])) state = 5;
			else state = -5;
			break;
		case 5:
			if (IsDigit(pChar[0])) {}
			else if (pChar[0] == 'e' || pChar[0] == 'E') state = 6;
			else {
				state = -1;
				token.type = TOKEN_NUMBER;
			}
			break;
		case 6:
			if (pChar[0] == '-' || pChar[0] == '+') state = 7;
			else if (IsDigit(pChar[0])) state = 8;
			else state = -5;
			break;
		case 7:
			if (IsDigit(pChar[0])) state = 8;
			else state = -5;
			break;
		case 8:
			if (IsDigit(pChar[0])) {}
			else {
				state = -1;
				token.type = TOKEN_NUMBER;
			}
			break;
		}

		if (state == -1) {
			pCode = pChar;
			break;
		}
		
		if (state == -2) {
			pCode = pChar + 1;
			break;
		}

		if (state == -5) {
			token.type = TOKEN_CHAR;
			pCode = pCode0 + 1;
			break;
		}
	}

	::strncpy(token.szToken, pCode0, pCode-pCode0);
	token.szToken[pCode-pCode0] = '\0';

	*pToken = token;

	m_pCode = pCode;
	return token.type;
}

clsSyntaxNode *clsParser::Parse()
{
	clsToken token;

	int state;
	m_stack.push(state=0);				//push state 0

	clsSyntaxNode *pNode;

	int nCollapse;

	GetToken(&token);
	while (1) {
		if ((nCollapse = IfCollapse(state, token.type)) != 0) {
			pNode = Collapse(nCollapse);
			m_stack.peep(&state);

			state = Transfer(state, pNode->type);

			if (state == 0) { delete pNode;	break; }				//no transfer
		}
		else {
			state = Transfer(state, token.type);

			if (state == 0) break;				//no transfer, wrong grammar

			pNode = new clsSyntaxNode;
			pNode->type = token.type;
			pNode->token = token;

			while (GetToken(&token) == TOKEN_CHAR);				//skip unknown tokens
		}

		m_stack.push(pNode);
		m_stack.push(state);

		if (state == STATE_ACCEPTED) break;				//accepted
	}

	if (state != STATE_ACCEPTED) {
		ClearStack();
		return NULL;
	}

	// state == STATE_ACCEPTED				//true for accepted
	m_stack.pop(&state);
	m_stack.pop((void **)&pNode); delete pNode;				//EOF
	m_stack.pop(&state);
	m_stack.pop((void **)&pNode);
	m_stack.clear();

	return pNode;
}

void clsParser::ClearStack()
{
	int state;
	clsSyntaxNode *pNode;

	while (1) {
		m_stack.pop(&state);
		if (state == 0) break;				//bottom of the stack

		m_stack.pop((void **)&pNode);
		delete pNode;				//free all nodes
	}
}

int clsParser::IfCollapse(int state, int nToken)
{
	int nCollapse = -_transfer[state][nToken];
	if (nCollapse <= 0) return 0;
	else return nCollapse;
}

clsSyntaxNode *clsParser::Collapse(int nCollapse)
{
	int state;

	clsSyntaxNode *pNode = new clsSyntaxNode;
	clsSyntaxNode *pChild;

	switch (nCollapse) {
	case 2:
//		syntax: program -> null
		pNode->type = ITEM_PROGRAM;
		pNode->form = FORM_PROGRAM_NULL;
		break;
	case 3:
//		syntax: program -> program statement
		pNode->type = ITEM_PROGRAM;
		pNode->form = FORM_PROGRAM_PROGRAMSTATEMENT;
		m_stack.pop(&state); m_stack.pop((void **)&pChild); pNode->ConnectChild(1, pChild);
		m_stack.pop(&state); m_stack.pop((void **)&pChild); pNode->ConnectChild(0, pChild);
		break;
	case 4:
//		syntax: statement -> identification = value;
		pNode->type = ITEM_STATEMENT;
		pNode->form = 0;				//only one form
		m_stack.pop(&state); m_stack.pop((void **)&pChild); pNode->ConnectChild(3, pChild);
		m_stack.pop(&state); m_stack.pop((void **)&pChild); pNode->ConnectChild(2, pChild);
		m_stack.pop(&state); m_stack.pop((void **)&pChild); pNode->ConnectChild(1, pChild);
		m_stack.pop(&state); m_stack.pop((void **)&pChild); pNode->ConnectChild(0, pChild);
		break;
	case 5:
//		syntax: value -> number
		pNode->type = ITEM_VALUE;
		pNode->form = FORM_VALUE_NUMBER;
		m_stack.pop(&state); m_stack.pop((void **)&pChild); pNode->ConnectChild(0, pChild);
		break;
	case 6:
//		syntax: value -> [matrix]
		pNode->type = ITEM_VALUE;
		pNode->form = FORM_VALUE_MATRIX;
		m_stack.pop(&state); m_stack.pop((void **)&pChild); pNode->ConnectChild(2, pChild);
		m_stack.pop(&state); m_stack.pop((void **)&pChild); pNode->ConnectChild(1, pChild);
		m_stack.pop(&state); m_stack.pop((void **)&pChild); pNode->ConnectChild(0, pChild);
		break;
	case 7:
//		syntax: matrix -> row
		pNode->type = ITEM_MATRIX;
		pNode->form = FORM_MATRIX_ROW;
		m_stack.pop(&state); m_stack.pop((void **)&pChild); pNode->ConnectChild(0, pChild);
		break;
	case 8:
//		syntax: matrix -> matrix; row
		pNode->type = ITEM_MATRIX;
		pNode->form = FORM_MATRIX_MATRIXROW;
		m_stack.pop(&state); m_stack.pop((void **)&pChild); pNode->ConnectChild(2, pChild);
		m_stack.pop(&state); m_stack.pop((void **)&pChild); pNode->ConnectChild(1, pChild);
		m_stack.pop(&state); m_stack.pop((void **)&pChild); pNode->ConnectChild(0, pChild);
		break;
	case 9:
//		syntax: row -> number
		pNode->type = ITEM_ROW;
		pNode->form = FORM_ROW_NUMBER;
		m_stack.pop(&state); m_stack.pop((void **)&pChild); pNode->ConnectChild(0, pChild);
		break;
	case 10:
//		syntax: row -> row, number
		pNode->type = ITEM_ROW;
		pNode->form = FORM_ROW_ROWCOMANUMBER;
		m_stack.pop(&state); m_stack.pop((void **)&pChild); pNode->ConnectChild(2, pChild);
		m_stack.pop(&state); m_stack.pop((void **)&pChild); pNode->ConnectChild(1, pChild);
		m_stack.pop(&state); m_stack.pop((void **)&pChild); pNode->ConnectChild(0, pChild);
		break;
	case 11:
//		syntax: row -> row number
		pNode->type = ITEM_ROW;
		pNode->form = FORM_ROW_ROWNUMBER;
		m_stack.pop(&state); m_stack.pop((void **)&pChild); pNode->ConnectChild(1, pChild);
		m_stack.pop(&state); m_stack.pop((void **)&pChild); pNode->ConnectChild(0, pChild);
		break;
	}

	return pNode;
}

int clsParser::_transfer[18][13] = {
//	nu.		,	;	[	]	id.	=	$	ro.	ma.	va.	st.	pr.
	{ 0,	0,	0,	0,	0,	-2,	0,	-2,	0,	0,	0,	0,	1},				//0
	{ 0,	0,	0,	0,	0,	3,	0,	55,	0,	0,	0,	2,	0},				//1				//55 for accepted
	{ 0,	0,	0,	0,	0,	-3,	0,	-3,	0,	0,	0,	0,	1},				//2
	{ 0,	0,	0,	0,	0,	0,	4,	0,	0,	0,	0,	0,	0},				//3
	{ 6,	0,	0,	7,	0,	0,	0,	0,	0,	0,	5,	0,	0},				//4
	{ 0,	0,	8,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},				//5
	{ 0,	0,	-5,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},				//6
	{ 11,	0,	0,	0,	0,	0,	0,	0,	10,	9,	0,	0,	0},				//7
	{ 0,	0,	0,	0,	0,	-4,	0,	-4,	0,	0,	0,	0,	0},				//8
	{ 0,	0,	13,	0,	12,	0,	0,	0,	0,	0,	0,	0,	0},				//9
	{ 15,	14,	-7,	0,	-7,	0,	0,	0,	0,	0,	0,	0,	0},				//10
	{ -9,	-9,	-9,	0,	-9,	0,	0,	0,	0,	0,	0,	0,	0},				//11
	{ 0,	0,	-6,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},				//12
	{ 11,	0,	0,	0,	0,	0,	0,	0,	16,	0,	0,	0,	0},				//13
	{ 17,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},				//14
	{ -11,	-11,-11,0,	-11,0,	0,	0,	0,	0,	0,	0,	0},				//15
	{ 15,	14,	-8,	0,	-8,	0,	0,	0,	0,	0,	0,	0,	0},				//16
	{ -10,	-10,-10,0,	-10,0,	0,	0,	0,	0,	0,	0,	0}				//17
};

clsSyntaxNode::clsSyntaxNode()
{
	pParent = NULL;
	for (int i=0; i<=MAX_CHILDNODE-1; i++) pChild[i] = NULL;
}

clsSyntaxNode::~clsSyntaxNode()
{
	for (int i=0; i<=MAX_CHILDNODE-1; i++) {
		if (pChild[i] == NULL) break;
		delete pChild[i];				//free child nodes
	}
}

void clsParser::Output(char *pszFile)
{
//this function is called after the the program is parsed
	FILE *pfOut = NULL;
	if (pszFile != NULL && pszFile[0] != '\0') pfOut = ::fopen(pszFile, "w");
	if (pfOut == NULL) pfOut = stdout;

	for (int i=0; i<=m_nVar-1; i++) {
		clsVariable &var = m_var[i];

		if (var.type == VT_DOUBLE) {
			::fprintf(pfOut, "\n%s = %.3g;\n", var.name, var.value);
		}
		else {
			clsMatrix &mtrx = var.matrix;
			char szOutput[512];
			::sprintf(szOutput, "\n%s = [", var.name);
			
			char *pChar = szOutput+::strlen(szOutput);
			for (int i=0; i<=mtrx.GetM()-1; i++) {
				for (int j=0; j<=mtrx.GetN()-1; j++) {
					::sprintf(pChar, "%.3g", mtrx[i][j]); pChar = ::strchr(pChar, '\0');
					if (j!=mtrx.GetN()-1) { *pChar++ = ','; }
				}
				if (i!=mtrx.GetM()-1) { *pChar++ = ';'; }
			}

			pChar[0] = ']'; pChar[1] = '\0';

			::fprintf(pfOut, "%s;\n", szOutput);
		}
	}

	::fprintf(pfOut, "\nTotal %d variable(s).\n", m_nVar);

	if (pfOut != stdout) ::fclose(pfOut);
}

void clsParser::ExecuteProgram(clsSyntaxNode *pProgram)
{
	int nFulfil = 0;

	clsSyntaxNode *pNode = pProgram;
	while (pNode != pProgram || nFulfil != 2) {
		if (pNode->type == ITEM_PROGRAM && pNode->form == FORM_PROGRAM_PROGRAMSTATEMENT) {
			if (nFulfil == 0) pNode = pNode->pChild[0];
			else if (nFulfil == 1) pNode = pNode->pChild[1];
			else {
				pNode = pNode->pParent;
				nFulfil = 1;
			}
		}
		else if (pNode->type == ITEM_PROGRAM && pNode->form == FORM_PROGRAM_NULL) {
			pNode = pNode->pParent;
			nFulfil = 1;
		}
		else if (pNode->type == ITEM_STATEMENT) {
			ExecuteStatement(pNode);
			pNode = pNode->pParent;
			nFulfil = 2;
		}
		else {
			::printf("Unrecognized pattern!\n");
			break;
		}
	}
}

void clsParser::ExecuteStatement(clsSyntaxNode *pStatement)
{
	clsVariable &var = m_var[m_nVar++];

	clsSyntaxNode *pIdentification = pStatement->pChild[0];
	::strcpy(var.name, pIdentification->token.szToken);

	clsSyntaxNode *pValue = pStatement->pChild[2];

	if (pValue->form == FORM_VALUE_NUMBER) {
		var.type = VT_DOUBLE;
		clsSyntaxNode *pNumber = pValue->pChild[0];
		var.value = ::atof(pNumber->token.szToken);
	}
	else {				//pValue->form == FORM_VALUE_MATRIX
		var.type = VT_MATRIX;
		clsSyntaxNode *pMatrix = pValue->pChild[1];
		EvaluateMatrix(pMatrix, var.matrix);
	}
}

void clsParser::EvaluateMatrix(clsSyntaxNode *pMatrix, clsMatrix &mtrx)
{
	clsSyntaxNode *pNode = pMatrix;

	int nFulfil = 0;

	int purpose = 1;				//check size;
	int row = 0;
	int column = 0;

EXPLORE:
	while (pNode != pMatrix->pParent) {
		if (pNode->type == ITEM_MATRIX && pNode->form == FORM_MATRIX_ROW) {
			if (nFulfil == 0) pNode = pNode->pChild[0];
			else {
				pNode = pNode->pParent;				//nFulfil == 2, returned from row node, fulfiled
				nFulfil = 1;
			}
		}
		else if (pNode->type == ITEM_MATRIX && pNode->form == FORM_MATRIX_MATRIXROW) {
			if (nFulfil == 0) pNode = pNode->pChild[0];
			else if (nFulfil == 1) pNode = pNode->pChild[2];
			else {				//nFulfil == 2, fulfiled
				pNode = pNode->pParent;
				nFulfil = 1;
			}
		}
		else if (pNode->type == ITEM_ROW) {
			if (purpose == 1) {
				int length = EvaluateRow(pNode, NULL, PURPOSE_SIZE);
				if (length > column) column = length;
				row ++;
			}
			else {				//purpose == 2
				EvaluateRow(pNode, mtrx[row++], PURPOSE_VALUE);
			}
			pNode = pNode->pParent;
			nFulfil = 2;
		}
	}

	if (purpose == 1) {
		mtrx.Reset(row, column);
		row = column = 0;
		
		pNode = pMatrix;
		nFulfil = 0;
		purpose = 2;
		goto EXPLORE;				//reexplore
	}
}

int clsParser::EvaluateRow(clsSyntaxNode *pRow, double *pBuffer, int nPurpose)
{
	clsSyntaxNode *pNode = pRow;
	int nFulfil = 0;

	int col = 0;
	int iBuffer = 0;

	while (pNode != pRow->pParent) {
		if (pNode->type == ITEM_ROW && pNode->form == FORM_ROW_NUMBER) {
			if (nFulfil == 0) pNode = pNode->pChild[0];
			else {
				pNode = pNode->pParent;
				nFulfil = 1;
			}
		}
		else if (pNode->type == ITEM_ROW && pNode->form == FORM_ROW_ROWNUMBER) {
			if (nFulfil == 0) pNode = pNode->pChild[0];
			else if (nFulfil == 1) pNode = pNode->pChild[1];
			else {
				pNode = pNode->pParent;
				nFulfil = 1;
			}
		}
		else if (pNode->type == ITEM_ROW && pNode->form == FORM_ROW_ROWCOMANUMBER) {
			if (nFulfil == 0) pNode = pNode->pChild[0];
			else if (nFulfil == 1) pNode = pNode->pChild[2];
			else {
				pNode = pNode->pParent;
				nFulfil = 1;
			}
		}
		else if (pNode->type == TOKEN_NUMBER) {
			col ++;
			if (nPurpose == PURPOSE_VALUE) pBuffer[iBuffer++] = ::atof(pNode->token.szToken);

			pNode = pNode->pParent;
			nFulfil = 2;
		}
		else {
			::printf("Unexpected pattern!\n");
			break;
		}
	}
	return col;
}

void clsParser::Parse2()
{
	enum { BEGIN, IDENTIFICATION, EQUAL, ASSIGNED, MATRIX, ROW, END };
	int state = BEGIN;

	clsToken token;

	double item[1024];				//maximum dimensions (m*n)
	int nItem = 0;

	int row = 0, col = 0, n = 0;

	while (state != END && state != -1) {				//-1 for error
		GetToken(&token);
		switch (state) {
		case BEGIN:
			if (token.type == TOKEN_IDENTIFICATION) {
				state = IDENTIFICATION;
				::strcpy(m_var[m_nVar].name, token.szToken);
			}
			else if (token.type == TOKEN_EOF) state = END;
			else {}				//skip all unexpected tokens in the beginning
			break;
		case IDENTIFICATION:
			if (token.type == TOKEN_EQUAL) state = EQUAL;
			else {
				state = -1;
			}
			break;
		case EQUAL:
			if (token.type == TOKEN_NUMBER) {
				state = ASSIGNED;
				m_var[m_nVar].type = VT_DOUBLE;
				m_var[m_nVar++].value = ::atof(token.szToken);
			}
			else if (token.type == TOKEN_LBRACKET) {
				state = MATRIX;
				nItem = 0;
				row = col = n = 0;
			}
			else state = -1;
			break;
		case ASSIGNED:
			if (token.type == TOKEN_SEMICOLON) state = BEGIN;
			else state = -1;
			break;
		case MATRIX:
			if (token.type == TOKEN_NUMBER) {
				item[nItem++] = ::atof(token.szToken);
				n ++;
			}
			else if (token.type == TOKEN_SEMICOLON || token.type == TOKEN_RBRACKET) {
				if (col != 0 && col != n) {	state = -1;	break; }
				if (col == 0) col = n;
				row ++; n = 0;				//reset item count for each row
				if (token.type == TOKEN_RBRACKET) {
					if (row == 0 || col == 0) {state = -1; break; }
					m_var[m_nVar].type = VT_MATRIX;
					m_var[m_nVar].matrix.Reset(row, col);
					m_var[m_nVar++].matrix = item;
				}
			}
			else if (token.type == TOKEN_COMA) {}				//skip coma
			else state = -1;
			break;
		}

		if (state == -1) state = BEGIN;				//resume to BEGIN state if any error occurs
	}
}

clsVariable *clsParser::FindVariable(const char *pszName)
{
	clsVariable *pFind = NULL;
	for (int i=0; i<=m_nVar-1; i++) {
		if (::stricmp(pszName, m_var[i].name) == 0) {
			pFind = &m_var[i];
			break;
		}
	}
	return pFind;
}

void clsParser::GetVariable(const char *pszName, clsMatrix &mtrx)
{
	clsVariable *pVar = FindVariable(pszName);
	if (pVar == NULL) {
		printf("[parser] variable not found - %s\n", pszName);
		return;
	}
	if (pVar->type != VT_MATRIX || !pVar->matrix.EqualSize(mtrx)) {
		printf("[parser] variable unmatched - %s\n", pszName);
		printf("[parser] type %d %d\n", pVar->type, VT_MATRIX);
		printf("[parser] source %d %d, destination %d %d\n",
			pVar->matrix.GetM(), pVar->matrix.GetN(), mtrx.GetM(), mtrx.GetN());
		return;
	}
	mtrx = pVar->matrix;
}

void clsParser::GetVariable(const char *pszName, void *pBuffer, int nBuffer)
{
	clsVariable *pVar = FindVariable(pszName);
	if (pVar == NULL) {
		printf("[parser] variable unfound - %s\n", pszName);
		return;
	}
	int nSource = pVar->matrix.GetM()*pVar->matrix.GetN()*sizeof(double);
	if (pVar->type != VT_MATRIX || nSource != nBuffer) {
		printf("[parser] variable unmatched - %s\n", pszName);
		printf("[parser] type %d %d\n", pVar->type, VT_MATRIX);
		printf("[parser] source %d bytes, destination %d bytes\n", nSource, nBuffer);
		return;
	}
	::memcpy(pBuffer, pVar->matrix.GetP(), nBuffer);
}

void clsParser::GetVariable(const char *pszName, double *pDouble)
{
	clsVariable *pVar = FindVariable(pszName);
	if (pVar == NULL) {
		printf("[parser] variable unfound - %s\n", pszName);
		return;
	}
	if (pVar->type != VT_DOUBLE) {
		printf("[parser] variable unmatched - %s\n", pszName);
		printf("[parser] type %d %d\n", pVar->type, VT_DOUBLE);
		return;
	}
	*pDouble = pVar->value;
}

void clsParser::GetVariable(const char *pszName, clsVector &vctr)
{
	clsVariable *pVar = FindVariable(pszName);
	if (pVar == NULL) {
		printf("[parser] variable unfound - %s\n", pszName);
		return;
	}
	int nSource = pVar->matrix.GetM()*pVar->matrix.GetN();
	if (pVar->type != VT_MATRIX || nSource != vctr.GetM()) {
		printf("[parser] variable unmatched - %s\n", pszName);
		printf("[parser] type %d %d\n", pVar->type, VT_MATRIX);
		printf("[parser] source %d doubles, destination %d doubles\n", nSource, vctr.GetM());
		return;
	}
	vctr = pVar->matrix.GetP();
}
