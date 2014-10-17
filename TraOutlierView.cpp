// TraOutlierView.cpp : implementation of the CTraOutlierView class
//

#include "stdafx.h"
#include "TraOutlier.h"

#include "TraOutlierDoc.h"
#include "TraOutlierView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CTraOutlierView

IMPLEMENT_DYNCREATE(CTraOutlierView, CView)

BEGIN_MESSAGE_MAP(CTraOutlierView, CView)
	// Standard printing commands
	ON_COMMAND(ID_FILE_PRINT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_DIRECT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_PREVIEW, &CView::OnFilePrintPreview)
END_MESSAGE_MAP()

// CTraOutlierView construction/destruction

CTraOutlierView::CTraOutlierView()
{
	// TODO: add construction code here

}

CTraOutlierView::~CTraOutlierView()
{
}

BOOL CTraOutlierView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

	return CView::PreCreateWindow(cs);
}

// CTraOutlierView drawing

void CTraOutlierView::OnDraw(CDC* pDC)
{
	CTraOutlierDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	// Display the numbers of trajectories and outliers
	// START ...
    CFont font;
    font.CreateFont(14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, TEXT("Arial"));
    CFont *oldfont = pDC->SelectObject(&font);
	
	CString message1("# of Trajectories = ");
	char buffer1[30]; sprintf_s(buffer1, "%d (%d)", pDoc->m_nTrajectories, pDoc->m_nLineSegments);
	CString number1(buffer1);
	pDC->TextOut(0, 0, message1 + number1);
	CString message2("# of Outliers = ");
	char buffer2[30]; sprintf_s(buffer2, "%d (%d)", pDoc->m_nOutliers, pDoc->m_nOutlyingPartitions);
	CString number2(buffer2);
	pDC->TextOut(0, 15, message2 + number2);

    pDC->SelectObject(oldfont);
	// ... END

	// Draw the original trjectories
	// START ...
	CTypedPtrList<CObList,CTrajectory*>& trajectoryList = pDoc->m_trajectoryList;
	POSITION pos1 = trajectoryList.GetHeadPosition();
	while (pos1 != NULL)
	{
		CTrajectory* pTrajectory = trajectoryList.GetNext(pos1);
		pTrajectory->DrawTrajectory(pDC);
#ifdef __SHOW_TRAJECTORY_PARTITION__
		pTrajectory->DrawSimplifiedTrajectory(pDC);
#endif	// __SHOW_TRAJECTORY_PARTITION__
	}
	// ... END

#ifdef __VISUALIZE_DEBUG_INFO__
	char buffer3[100]; sprintf_s(buffer3, "%.4f  %d  %d", pDoc->m_lineSegmentDensity, pDoc->m_nCloseLineSegments, pDoc->m_nCloseTrajectories);
	CString number3(buffer3);
	pDC->TextOut(400, 0, number3);

	CPen penLine1, penLine2;
	if (!penLine1.CreatePen(PS_SOLID, 2, RGB(255,0,0)))	return;
	if (!penLine2.CreatePen(PS_SOLID, 2, RGB(0,0,255)))	return;
	CPen* pOldPen = pDC->SelectObject(&penLine1);

	pDC->SelectObject(&penLine1);
	pDC->MoveTo(pDoc->m_currStartPoint);
	pDC->LineTo(pDoc->m_currEndPoint);

	pDC->SelectObject(&penLine2);
	for (int i = 0; i < pDoc->m_nCloseLineSegments; i++)
	{
		pDC->MoveTo(pDoc->m_startPointArray[i]);
		pDC->LineTo(pDoc->m_endPointArray[i]);
	}

	pDC->SelectObject(pOldPen);
#endif	// __VISUALIZE_DEBUG_INFO__

	// Draw the outlying trajectory partitions
	// START ...
	CTypedPtrList<CObList,COutlier*>& outlierList = pDoc->m_outlierList;
	POSITION pos2 = outlierList.GetHeadPosition();
	while (pos2 != NULL)
	{
		COutlier* pOutlier = outlierList.GetNext(pos2);
		pOutlier->DrawOutlier(pDC);
	}
	// ... END
}


// CTraOutlierView printing

BOOL CTraOutlierView::OnPreparePrinting(CPrintInfo* pInfo)
{
	// default preparation
	return DoPreparePrinting(pInfo);
}

void CTraOutlierView::OnBeginPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: add extra initialization before printing
}

void CTraOutlierView::OnEndPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: add cleanup after printing
}


// CTraOutlierView diagnostics

#ifdef _DEBUG
void CTraOutlierView::AssertValid() const
{
	CView::AssertValid();
}

void CTraOutlierView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CTraOutlierDoc* CTraOutlierView::GetDocument() const // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CTraOutlierDoc)));
	return (CTraOutlierDoc*)m_pDocument;
}
#endif //_DEBUG


// CTraOutlierView message handlers
