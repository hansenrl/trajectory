// TraOutlierView.h : interface of the CTraOutlierView class
//


#pragma once


class CTraOutlierView : public CView
{
protected: // create from serialization only
	CTraOutlierView();
	DECLARE_DYNCREATE(CTraOutlierView)

// Attributes
public:
	CTraOutlierDoc* GetDocument() const;

// Operations
public:

// Overrides
public:
	virtual void OnDraw(CDC* pDC);  // overridden to draw this view
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:
	virtual BOOL OnPreparePrinting(CPrintInfo* pInfo);
	virtual void OnBeginPrinting(CDC* pDC, CPrintInfo* pInfo);
	virtual void OnEndPrinting(CDC* pDC, CPrintInfo* pInfo);

// Implementation
public:
	virtual ~CTraOutlierView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generated message map functions
protected:
	DECLARE_MESSAGE_MAP()
};

#ifndef _DEBUG  // debug version in TraOutlierView.cpp
inline CTraOutlierDoc* CTraOutlierView::GetDocument() const
   { return reinterpret_cast<CTraOutlierDoc*>(m_pDocument); }
#endif

