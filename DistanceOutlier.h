#pragma once

//#include "TraOutlierDoc.h"
#include "trajData.h"
#include "Trajectory.h"
#include "Outlier.h"

#include <set>
#include <map>
#include <vector>
#include <string>
using namespace std;


// CDistanceOutlier

typedef pair<int,int> IntIntPair;
typedef vector<IntIntPair> IntIntPairArray;
typedef vector<float> FloatArray;
typedef vector<FloatArray> FloatMatrix;
typedef vector<vector<IntIntPair>> IntIntPairMatrix;

#define DISTANCE_MATRIX(_x,_y) ((*m_distanceIndex)[_x])[(_y)]
#define LENGTH_ARRAY(_x) ((*m_lengthArray)[_x])

class CDistanceOutlier
{
public:
	CDistanceOutlier(int nTrajectories, int nLineSegments, IntIntPairArray* idArray, FloatMatrix* distanceIndex);
	virtual ~CDistanceOutlier();
public:
	bool DetectOutlyingLineSegment(vector<bool>* result);
	void SetFractionParameter(float fraction) { m_fractionParam = fraction; }
	void SetDistanceParameter(float distance) { m_distanceParam = distance; }
	void SetLengthArray(FloatArray* lengthArray) { m_lengthArray = lengthArray; }
	void SetCloseTrajectoryPartition(map<IntIntPair,int>* indexMap, IntIntPairMatrix* partitionMatrix) 
	     { m_closePartitionArrayIndexMap = indexMap; m_closePartitionArray = partitionMatrix; }
	void SetFinePartitionLength(map<IntIntPair,float>* lengthMap) { m_finePartitionLengthMap = lengthMap; }
	void SetCoarsePartitionId(map<IntIntPair,int>* idMap) { m_coarsePartitionIdMap = idMap; }
	void SetDensityFilePath(string filePath) { m_densityFilePath = filePath; }
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
	TrajData*  m_data;
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
	string          m_densityFilePath;
private:
	map<IntIntPair,int>*	m_closePartitionArrayIndexMap;
	IntIntPairMatrix*		m_closePartitionArray;
	map<IntIntPair,float>*	m_finePartitionLengthMap;
	map<IntIntPair,int>*	m_coarsePartitionIdMap;
};
