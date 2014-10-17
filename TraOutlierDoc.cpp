// TraOutlierDoc.cpp : implementation of the CTraOutlierDoc class
//

#include "stdafx.h"
#include "TraOutlier.h"

#include "TraOutlierDoc.h"

#include <fstream>
using namespace std;

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CTraOutlierDoc

IMPLEMENT_DYNCREATE(CTraOutlierDoc, CDocument)

BEGIN_MESSAGE_MAP(CTraOutlierDoc, CDocument)
	ON_COMMAND(ID_OUTLIER_DETECT, &CTraOutlierDoc::OnOutlierDetect)
	ON_COMMAND(ID_EXPERIMENT_ALLEXPERIMENTS, &CTraOutlierDoc::OnExperimentAllexperiments)
	ON_COMMAND(ID_EXPERIMENT_EXPERIMENT1, &CTraOutlierDoc::OnExperimentExperiment1)
	ON_COMMAND(ID_EXPERIMENT_EXPERIMENT2, &CTraOutlierDoc::OnExperimentExperiment2)
	ON_COMMAND(ID_EXPERIMENT_EXPERIMENT3, &CTraOutlierDoc::OnExperimentExperiment3)
	ON_COMMAND(ID_EXPERIMENT_EXPERIMENT4, &CTraOutlierDoc::OnExperimentExperiment4)
	ON_COMMAND(ID_OUTLIER_SETPARAMETER, &CTraOutlierDoc::OnOutlierSetparameter)
END_MESSAGE_MAP()


// CTraOutlierDoc construction/destruction

CTraOutlierDoc::CTraOutlierDoc()
{
	// TODO: add one-time construction code here
	m_nTrajectories = 0;
	m_nOutliers = 0;
	m_nOutlyingPartitions = 0;
	m_nLineSegments = 0;
	m_nTrajectoryPartitions = 0;

	m_paramFraction = g_FRACTION_PARAMETER;
	m_paramDistance = g_DISTANCE_PARAMETER;

#ifdef __VISUALIZE_DEBUG_INFO__
	m_lineSegmentDensity = 0.0;
	m_nCloseTrajectories = 0;
	m_nCloseLineSegments = 0;
#endif
}

CTraOutlierDoc::~CTraOutlierDoc()
{
	m_nTrajectories = 0;
	m_nOutliers = 0;
	m_nOutlyingPartitions = 0;
	m_nLineSegments = 0;
	m_nTrajectoryPartitions = 0;
}

BOOL CTraOutlierDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	// TODO: add reinitialization code here
	// (SDI documents will reuse this document)

	return TRUE;
}




// CTraOutlierDoc serialization

void CTraOutlierDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO: add storing code here
	}
	else
	{
		// TODO: add loading code here
	}
}


// CTraOutlierDoc diagnostics

#ifdef _DEBUG
void CTraOutlierDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CTraOutlierDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG


// CTraOutlierDoc commands

BOOL CTraOutlierDoc::OnOpenDocument(LPCTSTR lpszPathName)
{
	int nDimensions = 2;		// default dimension = 2
	int nTrajectories = 0;
	int nTotalPoints = 0;
	int trajectoryId;
	int nPoints;
	float value;
	char buffer[1024000];
	char tmp_buf1[1024], tmp_buf2[1024];
	int offset = 0;

	m_inputFilePath = lpszPathName;
    ifstream istr(lpszPathName);

    if(!istr)
    {
        AfxMessageBox(TEXT("Unable to open input file"));
        return FALSE;
    }

	istr.getline(buffer, sizeof(buffer));	// the number of dimensions
	sscanf_s(buffer, "%d", &nDimensions);
	m_nDimensions = nDimensions;
	istr.getline(buffer, sizeof(buffer));	// the number of trajectories
	sscanf_s(buffer, "%d", &nTrajectories);
	m_nTrajectories = nTrajectories;

	m_maxNPoints = INT_MIN;					// initialize for comparison

	// the trajectory Id, the number of points, the coordinate of a point ...
	for (int i = 0; i < nTrajectories; i++)
	{
		offset = 0;

		istr.getline(buffer, sizeof(buffer));	// each trajectory
		sscanf_s(buffer, "%d %d", &trajectoryId, &nPoints);
		sscanf_s(buffer, "%s %s", tmp_buf1, sizeof(tmp_buf1), tmp_buf2, sizeof(tmp_buf2));
		offset += (int)strlen(tmp_buf1) + (int)strlen(tmp_buf2) + 2;

		if (nPoints > m_maxNPoints) m_maxNPoints = nPoints;
		m_nLineSegments += (nPoints - 1);
		nTotalPoints += nPoints;

		CTrajectory* pTrajectoryItem = new CTrajectory(trajectoryId, nDimensions); 
		m_trajectoryList.AddTail(pTrajectoryItem);

		for (int j = 0; j < nPoints; j++)
		{
			CMDPoint point(nDimensions);	// initialize the CMDPoint class for each point
			
			for (int k = 0; k < nDimensions; k++)
			{
				sscanf_s(buffer + offset, "%f", &value);
				sscanf_s(buffer + offset, "%s", tmp_buf1, sizeof(tmp_buf1));
				offset += (int)strlen(tmp_buf1) + 1;

				point.SetCoordinate(k, value);
			}

			pTrajectoryItem->AddPointToArray(point);
		}
	}

	istr.close();

	SetModifiedFlag();  // Mark the document as having been modified,
						// for purposes of confirming File Close.
	UpdateAllViews(NULL);

	return TRUE;
}

void CTraOutlierDoc::InitDocument()
{
}


#include "OutlierDetector.h"
#include <time.h>

void CTraOutlierDoc::OnOutlierDetect()
{
	time_t start, end;
	double diff;

	COutlierDetector detector(this);

	if (m_nTrajectories == 0)
    {
        AfxMessageBox(TEXT("Load a trajectory data set first"));
        return;
    }

	time(&start);	// begin the timer

	detector.ResetOutlierDetector();	// initialize the outlier detector

	detector.PartitionTrajectory();

	detector.DetectOutlier();

	time(&end);  diff = difftime(end, start);	// finish the timer

	SetModifiedFlag();  // Mark the document as having been modified,
						// for purposes of confirming File Close.
	UpdateAllViews(NULL);

	FILE *fp;
	fopen_s(&fp, RESULT_FILE, "a");
	CT2CA temp1(m_inputFilePath);
	std::string temp2(temp1);
#ifdef __COMBO_I__
	fprintf(fp, "COMBO I: %s %.1f => %.1f (sec)\n", temp2.c_str(), g_DISTANCE_PARAMETER, diff);
#else
	fprintf(fp, "COMBO II: %s %.1f => %.1f (sec)\n", temp2.c_str(), g_DISTANCE_PARAMETER, diff);
#endif
	fclose(fp);

	return;
}

void CTraOutlierDoc::OnExperimentAllexperiments()
{
	// TODO: Add your command handler code here
}

void CTraOutlierDoc::OnExperimentExperiment1()
{
	// TODO: Add your command handler code here
}

void CTraOutlierDoc::OnExperimentExperiment2()
{
	// TODO: Add your command handler code here
}

void CTraOutlierDoc::OnExperimentExperiment3()
{
	// TODO: Add your command handler code here
}

void CTraOutlierDoc::OnExperimentExperiment4()
{
	// TODO: Add your command handler code here
}

#include "ParamDlg.h"

void CTraOutlierDoc::OnOutlierSetparameter()
{
	CParamDlg dlg;

	dlg.m_paramDistance = m_paramDistance;
	dlg.m_paramFraction = m_paramFraction;

	if(dlg.DoModal() == IDOK)
	{
		m_paramDistance = dlg.m_paramDistance;
		m_paramFraction = dlg.m_paramFraction;

		OnOutlierDetect();
	}
}
