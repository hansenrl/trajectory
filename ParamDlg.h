#pragma once


// CParamDlg dialog

class CParamDlg : public CDialog
{
	DECLARE_DYNAMIC(CParamDlg)

public:
	CParamDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~CParamDlg();

// Dialog Data
	enum { IDD = IDD_PARAMDLG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	float m_paramDistance;
public:
	float m_paramFraction;
};
