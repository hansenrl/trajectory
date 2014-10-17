// ParamDlg.cpp : implementation file
//

#include "stdafx.h"
#include "TraOutlier.h"
#include "ParamDlg.h"


// CParamDlg dialog

IMPLEMENT_DYNAMIC(CParamDlg, CDialog)

CParamDlg::CParamDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CParamDlg::IDD, pParent)
	, m_paramDistance(0)
	, m_paramFraction(0)
{

}

CParamDlg::~CParamDlg()
{
}

void CParamDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_PARAMD, m_paramDistance);
	DDX_Text(pDX, IDC_PARAMP, m_paramFraction);
}


BEGIN_MESSAGE_MAP(CParamDlg, CDialog)
END_MESSAGE_MAP()


// CParamDlg message handlers
