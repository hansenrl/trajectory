// Measure.cpp : implementation file
//

#include "TraOutlier.h"
#include "OutlierDetector.h"
#include <math.h>

// COutlierDetector

#define LOG2(_x)	(float)(log((float)(_x)) / log((float)2))

int COutlierDetector::ComputeModelCost(CTrajectory* pTrajectory, int startPIndex, int endPIndex)
{
	CMDPoint* partitionStart = &(pTrajectory->m_pointArray[startPIndex]);
	CMDPoint* partitionEnd = &(pTrajectory->m_pointArray[endPIndex]);

	float distance = MeasureDistanceFromPointToPoint(partitionStart, partitionEnd);
	if (distance < 1.0) distance = 1.0;		// to take logarithm

	return (int)ceil(LOG2(distance));
}

int COutlierDetector::ComputeEncodingCost(CTrajectory* pTrajectory, int startPIndex, int endPIndex)
{
	CMDPoint* partitionStart;
	CMDPoint* partitionEnd;
	CMDPoint* lineSegmentStart;
	CMDPoint* lineSegmentEnd;
	float perpendicularDistance;
	float angleDistance;
	int encodingCost = 0;

	partitionStart = &(pTrajectory->m_pointArray[startPIndex]);
	partitionEnd = &(pTrajectory->m_pointArray[endPIndex]);

	for (int i = startPIndex; i < endPIndex; i++)
	{
		lineSegmentStart = &(pTrajectory->m_pointArray[i]);
		lineSegmentEnd = &(pTrajectory->m_pointArray[i + 1]);

		perpendicularDistance = MeasurePerpendicularDistance(partitionStart, partitionEnd, lineSegmentStart, lineSegmentEnd);
		angleDistance = MeasureAngleDisntance(partitionStart, partitionEnd, lineSegmentStart, lineSegmentEnd);

		if (perpendicularDistance < 1.0) perpendicularDistance = 1.0;	// to take logarithm
		if (angleDistance < 1.0) angleDistance = 1.0;					// to take logarithm
		encodingCost += ((int)ceil(LOG2(perpendicularDistance)) + (int)ceil(LOG2(angleDistance)));
	}

	return encodingCost;
}

float COutlierDetector::MeasurePerpendicularDistance(CMDPoint* s1, CMDPoint* e1, CMDPoint* s2, CMDPoint* e2)
{
	// we assume that the first line segment is longer than the second one
	float distance1;	// the distance from a start point to the trajectory partition 
	float distance2;	// the distance from an end point to the trajectory partition

	distance1 = MeasureDistanceFromPointToLineSegment(s1, e1, s2);
	distance2 = MeasureDistanceFromPointToLineSegment(s1, e1, e2);

	// if the first line segment is exactly the same as the second one, the perpendicular distance should be zero
	if (distance1 == 0.0 && distance2 == 0.0) return 0.0;

#if defined(__PARTITION_PRUNING_OPTIMIZATION__)
	if (m_partitionInfo.maxPerpDist < distance1) m_partitionInfo.maxPerpDist = distance1;
	if (m_partitionInfo.maxPerpDist < distance2) m_partitionInfo.maxPerpDist = distance2;
#endif

	// return (d1^2 + d2^2) / (d1 + d2) as the perpendicular distance
	return ((pow(distance1, 2) + pow(distance2, 2)) / (distance1 + distance2));
}

float COutlierDetector::MeasureDistanceFromPointToLineSegment(CMDPoint* s, CMDPoint* e, CMDPoint* p)
{
	int nDimensions = p->GetNDimensions();

	// NOTE: the variables m_vector1 and m_vector2 are declared as member variables

	// construct two vectors as follows
	// 1. the vector connecting the start point of the trajectory partition and a given point
	// 2. the vector representing the trajectory partition
	for (int i = 0; i < nDimensions; i++)
	{
		m_vector1.SetCoordinate(i, p->GetCoordinate(i) - s->GetCoordinate(i));
		m_vector2.SetCoordinate(i, e->GetCoordinate(i) - s->GetCoordinate(i));
	}

	// a coefficient (0 <= b <= 1)
	// NOTE: the variable m_coefficient is declared as a member variable
	m_coefficient = ComputeInnerProduct(&m_vector1, &m_vector2) / ComputeInnerProduct(&m_vector2, &m_vector2);

	// the projection on the trajectory partition from a given point
	// NOTE: the variable m_projectionPoint is declared as a member variable
	for (int i = 0; i < nDimensions; i++)
		m_projectionPoint.SetCoordinate(i, s->GetCoordinate(i) + m_coefficient * m_vector2.GetCoordinate(i));

	// return the distance between the projection point and the given point
	return MeasureDistanceFromPointToPoint(p, &m_projectionPoint);
}

float COutlierDetector::MeasureDistanceFromPointToPoint(CMDPoint* point1, CMDPoint* point2)
{
	int nDimensions = point1->GetNDimensions();
	float squareSum = 0.0;
	
	for (int i = 0; i < nDimensions; i++)
		squareSum += pow((point2->GetCoordinate(i) - point1->GetCoordinate(i)), 2);

	return sqrt(squareSum);
}

float COutlierDetector::ComputeVectorLength(CMDPoint* vector)
{
	int nDimensions = vector->GetNDimensions();
	float squareSum = 0.0;

	for (int i = 0; i < nDimensions; i++)
		squareSum += pow(vector->GetCoordinate(i), 2);

	return sqrt(squareSum);
}

float COutlierDetector::ComputeInnerProduct(CMDPoint* vector1, CMDPoint* vector2)
{
	int nDimensions = vector1->GetNDimensions();
	float innerProduct = 0.0;

	for (int i = 0; i < nDimensions; i++)
		innerProduct += (vector1->GetCoordinate(i) * vector2->GetCoordinate(i));

	return innerProduct;
}

float COutlierDetector::MeasureAngleDisntance(CMDPoint* s1, CMDPoint* e1, CMDPoint* s2, CMDPoint* e2)
{
	int nDimensions = s1->GetNDimensions();
	
	// NOTE: the variables m_vector1 and m_vector2 are declared as member variables

	// construct two vectors representing the trajectory partition and a line segment, respectively
	for (int i = 0; i < nDimensions; i++)
	{
		m_vector1.SetCoordinate(i, e1->GetCoordinate(i) - s1->GetCoordinate(i));
		m_vector2.SetCoordinate(i, e2->GetCoordinate(i) - s2->GetCoordinate(i));
	}

	// we assume that the first line segment is longer than the second one
	// i.e., vectorLength1 >= vectorLength2
	float vectorLength1 = ComputeVectorLength(&m_vector1);
	float vectorLength2 = ComputeVectorLength(&m_vector2);

	// if one of two vectors is a point, the angle distance becomes zero
	if (vectorLength1 == 0.0 || vectorLength2 == 0.0) return 0.0;

#if defined(__PARTITION_PRUNING_OPTIMIZATION__)
	if (m_partitionInfo.minLength > vectorLength2) m_partitionInfo.minLength = vectorLength2;
	if (m_partitionInfo.maxLength < vectorLength2) m_partitionInfo.maxLength = vectorLength2;
#endif

	// compute the inner product of the two vectors
	float innerProduct = ComputeInnerProduct(&m_vector1, &m_vector2);

	// compute the angle between two vectors by using the inner product
	float cosTheta = innerProduct / (vectorLength1 * vectorLength2);
	// compensate the computation error (e.g., 1.00001)
	// cos(theta) should be in the range [-1.0, 1.0]
	// START ...
	if (cosTheta > 1.0) cosTheta = 1.0; 
	if (cosTheta < -1.0) cosTheta = -1.0;
	// ... END
	float sinTheta = sqrt(1 - pow(cosTheta, 2));
	// if 90 <= theta <= 270, the angle distance becomes the length of the line segment
	// if (cosTheta < -1.0) sinTheta = 1.0;

#if defined(__PARTITION_PRUNING_OPTIMIZATION__)
	if (m_partitionInfo.maxTheta < asin(sinTheta)) m_partitionInfo.maxTheta = asin(sinTheta);
	m_cosTheta = fabs(cosTheta);
	m_theta    = asin(sinTheta);
#endif

	return (vectorLength2 * sinTheta);
}


float COutlierDetector::ComputeDistanceBetweenTwoLineSegments(CMDPoint* startPoint1, CMDPoint* endPoint1, CMDPoint* startPoint2, CMDPoint* endPoint2)
{
	float perpendicularDistance, parallelDistance, angleDistance;

	SubComputeDistanceBetweenTwoLineSegments(startPoint1, endPoint1, startPoint2, endPoint2, perpendicularDistance, parallelDistance, angleDistance);

	// all the weights are equally set to 1 in default
	return WEIGHTED_DISTANCE(perpendicularDistance, parallelDistance, angleDistance);
}


void COutlierDetector::ComputeDistanceBetweenTwoLineSegments(CMDPoint* startPoint1, CMDPoint* endPoint1, CMDPoint* startPoint2, CMDPoint* endPoint2, float* distanceVector)
{
	float perpendicularDistance, parallelDistance, angleDistance;

	SubComputeDistanceBetweenTwoLineSegments(startPoint1, endPoint1, startPoint2, endPoint2, perpendicularDistance, parallelDistance, angleDistance);

	distanceVector[0] = perpendicularDistance;
	distanceVector[1] = parallelDistance;
	distanceVector[2] = angleDistance;
}


void COutlierDetector::SubComputeDistanceBetweenTwoLineSegments(CMDPoint* startPoint1, CMDPoint* endPoint1, CMDPoint* startPoint2, CMDPoint* endPoint2, float& perpendicularDistance, float& parallelDistance, float& angleDistance)
{
	float perDistance1, perDistance2;
	float parDistance1, parDistance2;
	float length1, length2;

	// the length of the first line segment
	length1 = MeasureDistanceFromPointToPoint(startPoint1, endPoint1);
	// the length of the second line segment
	length2 = MeasureDistanceFromPointToPoint(startPoint2, endPoint2);

	// 05MAY2007: ignore a sort of detection errors---when a starting point coincides with an ending point
	if (length1 == 0 || length2 == 0)
	{
		perpendicularDistance = parallelDistance = angleDistance = 0.0;
		return;
	}

	// compute the perpendicular distance and the parallel distance
	// START ...
	if (length1 > length2)
	{
		perDistance1 = MeasureDistanceFromPointToLineSegment(startPoint1, endPoint1, startPoint2);
		if (m_coefficient < 0.5) parDistance1 = MeasureDistanceFromPointToPoint(startPoint1, &m_projectionPoint);
		else parDistance1 = MeasureDistanceFromPointToPoint(endPoint1, &m_projectionPoint);
		m_coefficient1 = m_coefficient;		// for optimization

		perDistance2 = MeasureDistanceFromPointToLineSegment(startPoint1, endPoint1, endPoint2);
		if (m_coefficient < 0.5) parDistance2 = MeasureDistanceFromPointToPoint(startPoint1, &m_projectionPoint);
		else parDistance2 = MeasureDistanceFromPointToPoint(endPoint1, &m_projectionPoint);
		m_coefficient2 = m_coefficient;		// for optimization
	}
	else
	{
		perDistance1 = MeasureDistanceFromPointToLineSegment(startPoint2, endPoint2, startPoint1);
		if (m_coefficient < 0.5) parDistance1 = MeasureDistanceFromPointToPoint(startPoint2, &m_projectionPoint);
		else parDistance1 = MeasureDistanceFromPointToPoint(endPoint2, &m_projectionPoint);
		m_coefficient1 = m_coefficient;		// for optimization

		perDistance2 = MeasureDistanceFromPointToLineSegment(startPoint2, endPoint2, endPoint1);
		if (m_coefficient < 0.5) parDistance2 = MeasureDistanceFromPointToPoint(startPoint2, &m_projectionPoint);
		else parDistance2 = MeasureDistanceFromPointToPoint(endPoint2, &m_projectionPoint);
		m_coefficient2 = m_coefficient;		// for optimization
	}

#if defined(__PARTITION_PRUNING_OPTIMIZATION__)
	m_distance1 = perDistance1;
	m_distance2 = perDistance2;
#endif

	// compute the perpendicular distance; take (d1^2 + d2^2) / (d1 + d2)
	if (!(perDistance1 == 0.0 && perDistance2 == 0.0)) 
		perpendicularDistance = ((pow(perDistance1, 2) + pow(perDistance2, 2)) / (perDistance1 + perDistance2));
	else
		perpendicularDistance = 0.0;

	// compute the parallel distance; take the minimum
	parallelDistance = (parDistance1 < parDistance2) ? parDistance1 : parDistance2;
	// ... END

	// compute the angle distance
	// START ...
	// MeasureAngleDisntance() assumes that the first line segment is longer than the second one
	if (length1 > length2)
		angleDistance = MeasureAngleDisntance(startPoint1, endPoint1, startPoint2, endPoint2);
	else
		angleDistance = MeasureAngleDisntance(startPoint2, endPoint2, startPoint1, endPoint1);
	// ... END

	return;
}
