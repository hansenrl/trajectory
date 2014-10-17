#pragma once

#include "TraOutlierDoc.h"
#include "Trajectory.h"
#include "Outlier.h"

#include <set>
#include <map>
using namespace std;


// CDistanceOutlier

typedef pair<int,int> IntIntPair;
typedef CArray<IntIntPair,IntIntPair> IntIntPairArray;
typedef CArray<float,float> FloatArray;
typedef CArray<FloatArray,FloatArray> FloatMatrix;
typedef CArray<CArray<IntIntPair,IntIntPair>,CArray<IntIntPair,IntIntPair>> IntIntPairMatrix;

#define DISTANCE_MATRIX(_x,_y) (m_distanceIndex->GetAt((_x)))[(_y)]
#define LENGTH_ARRAY(_x) (m_lengthArray->GetAt((_x)))

class CDistanceOutlier
{
public:
	CDistanceOutlier(int nTrajectories, int nLineSegments, IntIntPairArray* idArray, FloatMatrix* distanceIndex);
	virtual ~CDistanceOutlier();
public:
	BOOL DetectOutlyingLineSegment(CArray<bool,bool>* result);
	void SetFractionParameter(float fraction) { m_fractionParam = fraction; }
	void SetDistanceParameter(float distance) { m_distanceParam = distance; }
	void SetLengthArray(FloatArray* lengthArray) { m_lengthArray = lengthArray; }
	void SetCloseTrajectoryPartition(map<IntIntPair,int>* indexMap, IntIntPairMatrix* partitionMatrix) 
	     { m_closePartitionArrayIndexMap = indexMap; m_closePartitionArray = partitionMatrix; }
	void SetFinePartitionLength(map<IntIntPair,float>* lengthMap) { m_finePartitionLengthMap = lengthMap; }
	void SetCoarsePartitionId(map<IntIntPair,int>* idMap) { m_coarsePartitionIdMap = idMap; }
	void SetDensityFilePath(CString filePath) { m_densityFilePath = filePath; }
private:
	int  GetNumOfNearTrajectories(int index);
	bool IsTrajectoryNear(int lineSegmentId, int trajectoryId);
	void ConstructRangeOfTrajectory();
	void MeasureDensityOfLineSegment();
	float MeasureDistanceFromPointToPoint(CMDPoint* point1, CMDPoint* point2);
#if defined(__VISUALIZE_DEBUG_INFO__)
	void GetLineSegmentPoint(int lineSegmentId, CPoint* start, CPoint* end);
	void GetLineSegmentPoint(int trajectoryId, int index, CPoint* start, CPoint *end);
#endif
public:
	CTraOutlierDoc*  m_document;
private:
	int              m_nTrajectories;
	int              m_nLineSegments;
	IntIntPairArray* m_idArray;
	FloatMatrix*     m_distanceIndex;
	FloatArray*      m_lengthArray;
	float            m_fractionParam;
	float            m_distanceParam;
	int              m_nNearTrajectories;
	IntIntPairArray  m_trajectoryRange;
	FloatArray       m_densityArray;
	CString          m_densityFilePath;
private:
	map<IntIntPair,int>*	m_closePartitionArrayIndexMap;
	IntIntPairMatrix*		m_closePartitionArray;
	map<IntIntPair,float>*	m_finePartitionLengthMap;
	map<IntIntPair,int>*	m_coarsePartitionIdMap;
};