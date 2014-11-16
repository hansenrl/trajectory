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

/**
 * \brief Handles partition outlier computation
 *
 * Used by OutlierDetector to handle the actual distance comparisions to find outlying partitions. The comparisions and finding of
 * partition outliers are done here, while a lot of the management and higher-level functionality (finding and cataloging outlying
 * trajectories, pruning optimization, etc.) are handled in COutlierDetector.
 */
class CDistanceOutlier
{
public:
	CDistanceOutlier(int nTrajectories, int nLineSegments, IntIntPairArray* idArray, FloatMatrix* distanceIndex);
	virtual ~CDistanceOutlier();
public:
	bool DetectOutlyingLineSegment(vector<bool>* result);
	void SetFractionParameter(float fraction) { m_fractionParam = fraction; }
		///< Setter for the fraction of trajectories that must be far to be called an outlier (in the paper, "p")
	void SetDistanceParameter(float distance) { m_distanceParam = distance; }
		///< Setter for the distance parameter D
	void SetLengthArray(FloatArray* lengthArray) { m_lengthArray = lengthArray; }
		///< Setter for the array of partition lengths
	void SetCloseTrajectoryPartition(map<IntIntPair,int>* indexMap, IntIntPairMatrix* partitionMatrix) 
	     { m_closePartitionArrayIndexMap = indexMap; m_closePartitionArray = partitionMatrix; }
		///< Setter for mappings required for partition pruning optimization
	void SetFinePartitionLength(map<IntIntPair,float>* lengthMap) { m_finePartitionLengthMap = lengthMap; }
		///< Setter for lengths of fine level partitions
	void SetCoarsePartitionId(map<IntIntPair,int>* idMap) { m_coarsePartitionIdMap = idMap; }
		///< Setter for mapping of  coarse partition IDs
	void SetDensityFilePath(string filePath) { m_densityFilePath = filePath; }
		///< Set file path for precomputed density file
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
	FloatArray*      m_lengthArray; // array of partition lengths
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
