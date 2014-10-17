// TraOutlierDoc.h : interface of the CTraOutlierDoc class
//


#pragma once


#include "Trajectory.h"
#include "Outlier.h"

class CTraOutlierDoc : public CDocument
{
protected: // create from serialization only
	CTraOutlierDoc();
	DECLARE_DYNCREATE(CTraOutlierDoc)

// Attributes
public:
	CString m_inputFilePath;
	int m_nDimensions;
	int m_nTrajectories;
	int m_nOutliers;
	int m_nOutlyingPartitions;
	int m_maxNPoints;
	CTypedPtrList<CObList,CTrajectory*> m_trajectoryList;
	CTypedPtrList<CObList,COutlier*> m_outlierList;
	float m_paramFraction;
	float m_paramDistance;
	int m_nLineSegments;
	int m_nTrajectoryPartitions;
// DEBUG ...
#ifdef __VISUALIZE_DEBUG_INFO__
	float m_lineSegmentDensity;
	int m_nCloseTrajectories;
	int m_nCloseLineSegments;
	CPoint m_currStartPoint;
	CPoint m_currEndPoint;
	CArray<CPoint,CPoint> m_startPointArray;
	CArray<CPoint,CPoint> m_endPointArray;
#endif
// ... DEBUG

// Operations
public:

// Overrides
public:
	virtual BOOL OnNewDocument();
	virtual void Serialize(CArchive& ar);

// Implementation
public:
	virtual ~CTraOutlierDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:
	void InitDocument();

// Generated message map functions
protected:
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnOutlierDetect();
public:
	afx_msg void OnExperimentAllexperiments();
public:
	afx_msg void OnExperimentExperiment1();
public:
	afx_msg void OnExperimentExperiment2();
public:
	afx_msg void OnExperimentExperiment3();
public:
	afx_msg void OnExperimentExperiment4();
public:
	virtual BOOL OnOpenDocument(LPCTSTR lpszPathName);
public:
	afx_msg void OnOutlierSetparameter();
};