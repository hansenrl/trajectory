#pragma once

#include "TraOutlierDoc.h"
#include "Trajectory.h"
#include "Outlier.h"

#include <set>
#include <map>
using namespace std;


// COutlierDetector

typedef pair<int,int> IntIntPair;
typedef pair<CMDPoint*,CMDPoint*> PointPointPair;
typedef CArray<float,float> FloatArray;

class COutlierDetector
{
public:
	COutlierDetector();
	COutlierDetector(CTraOutlierDoc* document);
	virtual ~COutlierDetector();
public:
	CTraOutlierDoc* m_document;
private:
	int m_nTotalLineSegments;
	CArray<IntIntPair,IntIntPair> m_idArray;
	CArray<PointPointPair,PointPointPair> m_lineSegmentArray;
	CArray<FloatArray,FloatArray> m_distanceIndex;
	FloatArray m_lengthArray;
	CArray<PartitionInfo*,PartitionInfo*> m_partitionInfoArray;
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
	CArray<CArray<IntIntPair,IntIntPair>,CArray<IntIntPair,IntIntPair>> m_closePartitionArray;
	map<IntIntPair,float> m_finePartitionLengthMap;
	map<IntIntPair,int> m_coarsePartitionIdMap;
#endif	/* #if defined(__PARTITION_PRUNING_OPTIMIZATION__) */
public:
	BOOL ResetOutlierDetector();
	BOOL PartitionTrajectory();
	BOOL DetectOutlier();
private:
	BOOL FindOptimalPartition(CTrajectory* pTrajectory);
	BOOL StoreTrajectoryPartitionIntoIndex();
	BOOL CheckOutlyingProportion(CTrajectory* pTrajectory, float minProportion);
	BOOL GenerateAndSetupOutlier(CTrajectory* pTrajectory, int outlierId);
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