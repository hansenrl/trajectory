#pragma once

#include "trajData.h"
#include "Trajectory.h"
#include "Outlier.h"
#include "Param.h"

#include <set>
#include <map>
#include <vector>
using namespace std;


// COutlierDetector

typedef pair<int,int> IntIntPair;
typedef pair<CMDPoint*,CMDPoint*> PointPointPair;
typedef vector<float> FloatArray;

class COutlierDetector
{
public:
	COutlierDetector();
	COutlierDetector(TrajData* data);
	virtual ~COutlierDetector();
public:
	TrajData* m_data;
private:
	int m_nTotalLineSegments;
	vector<IntIntPair> m_idArray;
	vector<PointPointPair> m_lineSegmentArray;
	vector<FloatArray> m_distanceIndex;
	FloatArray m_lengthArray;
	vector<PartitionInfo*> m_partitionInfoArray;
	// programming trick: avoid frequent execution of the new and delete operations
	CMDPoint m_vector1, m_vector2;
	CMDPoint m_projectionPoint;
	float m_coefficient, m_coefficient1, m_coefficient2;
#if defined(__PARTITION_PRUNING_OPTIMIZATION__)
	// added for partition pruning optimization
	PartitionInfo m_partitionInfo;
	float m_distance1, m_distance2, m_theta, m_cosTheta;
	float m_lowerBoundPerp, m_lowerBoundPara, m_lowerBoundAngle;
	float m_upperBoundPerp, m_upperBoundPara, m_upperBoundAngle;
	map<IntIntPair,int> m_closePartitionArrayIndexMap;
	vector<vector<IntIntPair> > m_closePartitionArray;
	map<IntIntPair,float> m_finePartitionLengthMap;
	map<IntIntPair,int> m_coarsePartitionIdMap;
#endif	/* #if defined(__PARTITION_PRUNING_OPTIMIZATION__) */
public:
	bool ResetOutlierDetector();
	bool PartitionTrajectory();
	bool DetectOutlier();
private:
	bool FindOptimalPartition(CTrajectory* pTrajectory);
	bool StoreTrajectoryPartitionIntoIndex();
	bool CheckOutlyingProportion(CTrajectory* pTrajectory, float minProportion);
	bool GenerateAndSetupOutlier(CTrajectory* pTrajectory, int outlierId);
private:
	int ComputeModelCost(CTrajectory* pTrajectory, int startPIndex, int endPIndex);
	int ComputeEncodingCost(CTrajectory* pTrajectory, int startPIndex, int endPIndex);
	float MeasurePerpendicularDistance(CMDPoint* s1, CMDPoint* e1, CMDPoint* s2, CMDPoint* e2);
	float MeasureDistanceFromPointToLineSegment(CMDPoint* s, CMDPoint* e, CMDPoint* p);
	float MeasureDistanceFromPointToPoint(CMDPoint* point1, CMDPoint* point2);
	float ComputeVectorLength(CMDPoint* vector);
	float ComputeInnerProduct(CMDPoint* vector1, CMDPoint* vector2);
	float MeasureAngleDisntance(CMDPoint* s1, CMDPoint* e1, CMDPoint* s2, CMDPoint* e2);
	float ComputeDistanceBetweenTwoLineSegments(CMDPoint* startPoint1, CMDPoint* endPoint1, CMDPoint* startPoint2, CMDPoint* endPoint2);
	void ComputeDistanceBetweenTwoLineSegments(CMDPoint* startPoint1, CMDPoint* endPoint1, CMDPoint* startPoint2, CMDPoint* endPoint2, float* distanceVector);
	void SubComputeDistanceBetweenTwoLineSegments(CMDPoint* startPoint1, CMDPoint* endPoint1, CMDPoint* startPoint2, CMDPoint* endPoint2, float& perpendicularDistance, float& parallelDistance, float& angleDistance);
};
