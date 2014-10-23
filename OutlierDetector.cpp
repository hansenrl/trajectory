// OutlierDetector.cpp : implementation file
//

#include "trajData.h"
#include "OutlierDetector.h"
#include "DistanceOutlier.h"
#include "Param.h"
#include <climits>
#include <vector>
#include <math.h>
#include <algorithm>


// COutlierDetector

COutlierDetector::COutlierDetector()
{
}

COutlierDetector::COutlierDetector(TrajData* data)
{
	m_data = data;
}

COutlierDetector::~COutlierDetector()
{
}


// COutlierDetector member functions

bool COutlierDetector::ResetOutlierDetector()
{
	vector<CTrajectory*>& trajectoryList = m_data->m_trajectoryList;
	//list<CTrajectory*>::iterator pos = trajectoryList.begin();
	auto pos = trajectoryList.begin();
	while (pos != trajectoryList.end())
	{
		CTrajectory* pTrajectory = *pos;

		pTrajectory->m_nPartitionPoints = 0;
		pTrajectory->m_partitionPointArray.clear();
		pTrajectory->m_partitionIndexArray.clear();
		pTrajectory->m_partitionInfoArray.clear();
		pTrajectory->m_nOutlyingPartitions = 0;
		pTrajectory->m_outlyingPartitionArray.clear();
		pTrajectory->m_totalPartitionLength = 0.0;
		pTrajectory->m_outlyingPartitionLength = 0.0;
		pTrajectory->m_containOutlier = false;
		pos++;
	}
	m_data->m_nOutlyingPartitions = 0;

	while (!m_data->m_outlierList.empty())
	{
		delete m_data->m_outlierList.front();
		m_data->m_outlierList.pop_front();
	}
	m_data->m_nOutliers = 0;

	return true;
}

bool COutlierDetector::PartitionTrajectory()
{
#ifdef __USE_CONVENTIONAL_PARTITONING__
	vector<CTrajectory*>& trajectoryList = m_data->m_trajectoryList;
	auto pos = trajectoryList.begin();
	while (pos != trajectoryList.end())
	{
		CTrajectory* pTrajectory = *pos;
		if (!FindOptimalPartition(pTrajectory))
			return false;
		pos++;
	}
#endif

	if (!StoreTrajectoryPartitionIntoIndex())
		return false;

	return true;
}

bool COutlierDetector::FindOptimalPartition(CTrajectory* pTrajectory)
{
	int nPoints = pTrajectory->m_nPoints;
	int startIndex = 0, length = 0;
	int fullPartitionMDLCost, partialPartitionMDLCost;

	// add the start point of a trajectory
	pTrajectory->AddPartitionPointToArray(pTrajectory->m_pointArray[0], 0);

	for (;;)
	{
		fullPartitionMDLCost = partialPartitionMDLCost = 0;

#if defined(__PARTITION_PRUNING_OPTIMIZATION__)
		// reset some statistics per each trajectory
		m_partitionInfo.maxPerpDist = 0.0;
		m_partitionInfo.minLength   = (float)INT_MAX;
		m_partitionInfo.maxLength   = 0.0;
		m_partitionInfo.maxTheta    = 0.0;
#endif

		for (length = 1; startIndex + length < nPoints; length++)
		{
			// compute the total length of a trajectory
			fullPartitionMDLCost += ComputeModelCost(pTrajectory, startIndex + length - 1, startIndex + length);

			// compute the sum of (1) the length of a trajectory partition and
			//                    (2) the perpendicular and angle distances
			partialPartitionMDLCost = ComputeModelCost(pTrajectory, startIndex, startIndex + length) +
			                          ComputeEncodingCost(pTrajectory, startIndex, startIndex + length);

			if (fullPartitionMDLCost + MDL_COST_ADVANTAGE < partialPartitionMDLCost)
			{
				pTrajectory->AddPartitionPointToArray(pTrajectory->m_pointArray[startIndex + length - 1], startIndex + length - 1);

#if defined(__PARTITION_PRUNING_OPTIMIZATION__)
				// maintain some statistics for each trajectory
				PartitionInfo aPartitionInfo = m_partitionInfo;
				pTrajectory->StorePartitionInfo(aPartitionInfo);
#endif

				startIndex = startIndex + length - 1;
				length = 0;

				break;
			}
		}

		// if we reach at the end of a trajectory
		if (startIndex + length >= nPoints)
			break;
	}

	// add the end point of a trajectory
	pTrajectory->AddPartitionPointToArray(pTrajectory->m_pointArray[nPoints - 1], nPoints - 1);

#if defined(__PARTITION_PRUNING_OPTIMIZATION__)
	// maintain some statistics for each trajectory; this is the last partition
	PartitionInfo aPartitionInfo = m_partitionInfo;
	pTrajectory->StorePartitionInfo(aPartitionInfo);
#endif

	return true;
}

bool COutlierDetector::StoreTrajectoryPartitionIntoIndex()
{
	int nDimensions = m_data->m_nDimensions;
	CMDPoint* startPoint;
	CMDPoint* endPoint;
	float length, totalLengthPerTrajectory;

	m_nTotalLineSegments = 0;

	vector<CTrajectory*>& trajectoryList = m_data->m_trajectoryList;
	auto pos = trajectoryList.begin();
	while (pos != trajectoryList.end())
	{
		CTrajectory* pTrajectory = *pos;
		totalLengthPerTrajectory = 0.0;				// reset for each trajectory

#if defined(__USE_CONVENTIONAL_PARTITONING__)
		for (int i = 0; i < pTrajectory->m_nPartitionPoints - 1; i++)
		{
			startPoint = &(pTrajectory->m_partitionPointArray[i]);
			endPoint = &(pTrajectory->m_partitionPointArray[i + 1]);
#elif defined(__USE_NO_PARTITIONING__)
		for (int i = 0; i < pTrajectory->m_nPoints - 1; i++)
		{
			startPoint = &(pTrajectory->m_pointArray[i]);
			endPoint = &(pTrajectory->m_pointArray[i + 1]);
#endif	// __USE_CONVENTIONAL_PARTITONING__

			// if a line segment is too short, ignore it
			if ((length = MeasureDistanceFromPointToPoint(startPoint, endPoint)) < MIN_LINESEGMENT_LENGTH)
				continue;

			m_nTotalLineSegments++;				// increase the total number of line segments
			totalLengthPerTrajectory += length;	// accumulate the length of a trajectory partition per trajectory

			// keep the pair of a trajectory id and an order in the trajectory
			m_idArray.push_back(IntIntPair(pTrajectory->GetId(), i));
			// temporarily keep the coordinate of a line segment for efficient distance computation
			// m_lineSegmentArray will be released soon
			m_lineSegmentArray.push_back(PointPointPair(startPoint, endPoint));
			// keep the length of a line segment; this is used for determining the close trajectory
			m_lengthArray.push_back(length);
#if defined(__PARTITION_PRUNING_OPTIMIZATION__) 
			m_partitionInfoArray.push_back(&(pTrajectory->m_partitionInfoArray[i]));
#endif
		}

		pTrajectory->SetLength(totalLengthPerTrajectory);
		pos++;
	}

	m_data->m_nTrajectoryPartitions = m_nTotalLineSegments;

	// pre-calculate the distance between two line segments
	// START ...
	m_distanceIndex.resize(m_nTotalLineSegments);
	for (int i = 0; i < m_nTotalLineSegments; i++) m_distanceIndex[i].resize(m_nTotalLineSegments);

	// hold the perpendicular, parallel, angle distances
	float distance[3];
	// used for measuring the error of pruning
	int nTotalPartitionPairs = 0, nPrunedPartitionPairs = 0, nFalsePrunedPairs = 0, nOptimalPrunedPairs = 0;
	// for debugging
	// float minRealDistance[3], maxRealDistance[3];

	for (int i = 0; i < m_nTotalLineSegments; i++)
	{
		for (int j = i + 1; j < m_nTotalLineSegments; j++)
		{
			ComputeDistanceBetweenTwoLineSegments(m_lineSegmentArray[i].first, m_lineSegmentArray[i].second, 
			                                      m_lineSegmentArray[j].first, m_lineSegmentArray[j].second, distance);

			// NOTE: 
			// 1) our distance measure is symmetric
			// 2) three weights are determined by users (or applications)
			m_distanceIndex[i][j] = m_distanceIndex[j][i] = WEIGHTED_DISTANCE(distance[0], distance[1], distance[2]);

#if defined(__PARTITION_PRUNING_OPTIMIZATION__)

			#define PI (float)3.141593
			#define MIN(_x,_y) ((_x)<(_y) ? (_x):(_y))
			#define MAX(_x,_y) ((_x)>(_y) ? (_x):(_y))

			// for debugging
			// PartitionInfo forDebug1 = *(m_partitionInfoArray[i]), forDebug2 = *(m_partitionInfoArray[j]);
			nTotalPartitionPairs++;

			// compute the lower and upper bounds for the perpendicular distance
			// START ...
			float minPerpDistance, maxPerpDistance;

			minPerpDistance = MIN(m_distance1, m_distance2) - m_partitionInfoArray[i]->maxPerpDist - m_partitionInfoArray[j]->maxPerpDist;
			maxPerpDistance = MAX(m_distance1, m_distance2) + m_partitionInfoArray[i]->maxPerpDist + m_partitionInfoArray[j]->maxPerpDist;
			if (minPerpDistance < 0.0) minPerpDistance = 0.0;

			m_lowerBoundPerp = minPerpDistance;
			m_upperBoundPerp = maxPerpDistance;
			// ... END

			// compute the lower and upper bounds for the parallel distance
			// START ...
			float length1, length2;

			length1 = MeasureDistanceFromPointToPoint(m_lineSegmentArray[i].first, m_lineSegmentArray[i].second);
			length2 = MeasureDistanceFromPointToPoint(m_lineSegmentArray[j].first, m_lineSegmentArray[j].second);
	
			if (fabs(m_coefficient1) < 1 && fabs(m_coefficient2) < 1)  {		// enclose
				m_lowerBoundPara  = (float)0.0;
				m_upperBoundPara  = MAX(length1, length2);
			}
			else if (fabs(m_coefficient1) < 1 || fabs(m_coefficient2) < 1)  {	// intersect
				m_lowerBoundPara  = (float)0.0;
				m_upperBoundPara  = length1 + length2 - distance[1];
			}
			else  {																// distinct
				m_lowerBoundPara  = distance[1];
				m_upperBoundPara  = length1 + length2 + distance[1];
			}
			// ... END

			// compute the lower and upper bounds for the angle distance
			// START ...
			float possibleMinTheta, possibleMaxTheta;
			float minMinLength, minMaxLength;

			possibleMinTheta = m_theta - m_partitionInfoArray[i]->maxTheta - m_partitionInfoArray[j]->maxTheta;
			possibleMaxTheta = m_theta + m_partitionInfoArray[i]->maxTheta + m_partitionInfoArray[j]->maxTheta;
			if (possibleMinTheta < 0.0) possibleMinTheta = 0.0;
			if (possibleMaxTheta > PI / 2) possibleMaxTheta = PI / 2;
			minMinLength = MIN(m_partitionInfoArray[i]->minLength, m_partitionInfoArray[j]->minLength);
			minMaxLength = MIN(m_partitionInfoArray[i]->maxLength, m_partitionInfoArray[j]->maxLength);

			m_lowerBoundAngle = minMinLength * sin(possibleMinTheta);
			m_upperBoundAngle = minMaxLength * sin(possibleMaxTheta);
			// ... END

			float estimatedLowerBound = WEIGHTED_DISTANCE(m_lowerBoundPerp, m_lowerBoundPara, m_lowerBoundAngle);
			float estimatedUpperBound = WEIGHTED_DISTANCE(m_upperBoundPerp, m_upperBoundPara, m_upperBoundAngle);

			if (estimatedLowerBound > m_data->m_paramDistance || estimatedUpperBound < m_data->m_paramDistance)
			{				
				nPrunedPartitionPairs++;
			}
			else
			{
				int trajectoryId1 = m_idArray[i].first, index1 = m_idArray[i].second;
				int trajectoryId2 = m_idArray[j].first, index2 = m_idArray[j].second;
				if (trajectoryId1 == trajectoryId2) continue;
m_data->m_trajectoryList;
				auto comp1 = [trajectoryId1](CTrajectory* traj) {return traj->m_trajectoryId == trajectoryId1;};
				auto comp2 = [trajectoryId2](CTrajectory* traj) {return traj->m_trajectoryId == trajectoryId2;};
				//CTrajectory* pTrajectory1 = m_data->m_trajectoryList.GetAt(m_data->m_trajectoryList.FindIndex(trajectoryId1));
				//CTrajectory* pTrajectory2 = m_data->m_trajectoryList.GetAt(m_data->m_trajectoryList.FindIndex(trajectoryId2));
				auto p1 = find_if(m_data->m_trajectoryList.begin(), m_data->m_trajectoryList.end(),comp1);
				auto p2 = find_if(m_data->m_trajectoryList.begin(), m_data->m_trajectoryList.end(),comp2);
				CTrajectory* pTrajectory1 = *p1;
				CTrajectory* pTrajectory2 = *p2;

				CMDPoint *startPoint1, *endPoint1, *startPoint2, *endPoint2;
				float length1, length2;
				float fineDistance[3];

				for (int k = pTrajectory1->m_partitionIndexArray[index1]; k < pTrajectory1->m_partitionIndexArray[index1 + 1]; k++)
				{
					startPoint1 = &(pTrajectory1->m_pointArray[k]);
					endPoint1   = &(pTrajectory1->m_pointArray[k + 1]);
					length1     = MeasureDistanceFromPointToPoint(startPoint1, endPoint1);
					if (length1 < MIN_LINESEGMENT_LENGTH || length1 > MAX_LINESEGMENT_LENGTH) continue;

					for (int l = pTrajectory2->m_partitionIndexArray[index2]; l < pTrajectory2->m_partitionIndexArray[index2 + 1]; l++)
					{
						startPoint2 = &(pTrajectory2->m_pointArray[l]);
						endPoint2   = &(pTrajectory2->m_pointArray[l + 1]);
						length2     = MeasureDistanceFromPointToPoint(startPoint2, endPoint2);
						if (length2 < MIN_LINESEGMENT_LENGTH || length2 > MAX_LINESEGMENT_LENGTH) continue;

						ComputeDistanceBetweenTwoLineSegments(startPoint1, endPoint1, startPoint2, endPoint2, fineDistance);
						if (WEIGHTED_DISTANCE(fineDistance[0], fineDistance[1], fineDistance[2]) <= m_data->m_paramDistance)
						{
							if (m_closePartitionArrayIndexMap.count(IntIntPair(trajectoryId1,k)) != 0)  {
								int index1 = m_closePartitionArrayIndexMap[IntIntPair(trajectoryId1,k)];
								m_closePartitionArray[index1].push_back(IntIntPair(trajectoryId2,l));
							}
							else  {
								int size1 = (int)m_closePartitionArrayIndexMap.size();
								m_closePartitionArrayIndexMap[IntIntPair(trajectoryId1,k)] = size1;
								m_closePartitionArray.resize(size1 + 1);
								m_closePartitionArray[size1].push_back(IntIntPair(trajectoryId2,l));
							}
							if (m_closePartitionArrayIndexMap.count(IntIntPair(trajectoryId2,l)) != 0)  {
								int index2 = m_closePartitionArrayIndexMap[IntIntPair(trajectoryId2,l)];
								m_closePartitionArray[index2].push_back(IntIntPair(trajectoryId1,k));
							}
							else  {
								int size2 = (int)m_closePartitionArrayIndexMap.size();
								m_closePartitionArrayIndexMap[IntIntPair(trajectoryId2,l)] = size2;
								m_closePartitionArray.resize(size2 + 1);
								m_closePartitionArray[size2].push_back(IntIntPair(trajectoryId1,k));
							}
							m_finePartitionLengthMap[IntIntPair(trajectoryId1,k)] = length1;
							m_finePartitionLengthMap[IntIntPair(trajectoryId2,l)] = length2;
#ifndef __PRECOMPUTE_DENSITY__
							m_coarsePartitionIdMap[IntIntPair(trajectoryId1,k)]   = i;
							m_coarsePartitionIdMap[IntIntPair(trajectoryId2,l)]   = j;
#endif
						}
					}
				}
			}

			// All line segments in this pair should be considered
			// START ...
			if (estimatedUpperBound < m_data->m_paramDistance)
			{
				int trajectoryId1 = m_idArray[i].first, index1 = m_idArray[i].second;
				int trajectoryId2 = m_idArray[j].first, index2 = m_idArray[j].second;
				if (trajectoryId1 == trajectoryId2) continue;

				auto comp1 = [trajectoryId1](CTrajectory* traj) {return traj->m_trajectoryId == trajectoryId1;};
				auto comp2 = [trajectoryId2](CTrajectory* traj) {return traj->m_trajectoryId == trajectoryId2;};
				//CTrajectory* pTrajectory1 = m_data->m_trajectoryList.GetAt(m_data->m_trajectoryList.FindIndex(trajectoryId1));
				//CTrajectory* pTrajectory2 = m_data->m_trajectoryList.GetAt(m_data->m_trajectoryList.FindIndex(trajectoryId2));
				auto p1 = find_if(m_data->m_trajectoryList.begin(), m_data->m_trajectoryList.end(),comp1);
				auto p2 = find_if(m_data->m_trajectoryList.begin(), m_data->m_trajectoryList.end(),comp2);
				CTrajectory* pTrajectory1 = *p1;
				CTrajectory* pTrajectory2 = *p2;

				CMDPoint *startPoint1, *endPoint1, *startPoint2, *endPoint2;
				float length1, length2;

				for (int k = pTrajectory1->m_partitionIndexArray[index1]; k < pTrajectory1->m_partitionIndexArray[index1 + 1]; k++)
				{
					startPoint1 = &(pTrajectory1->m_pointArray[k]);
					endPoint1   = &(pTrajectory1->m_pointArray[k + 1]);
					length1     = MeasureDistanceFromPointToPoint(startPoint1, endPoint1);
					if (length1 < MIN_LINESEGMENT_LENGTH || length1 > MAX_LINESEGMENT_LENGTH) continue;

					for (int l = pTrajectory2->m_partitionIndexArray[index2]; l < pTrajectory2->m_partitionIndexArray[index2 + 1]; l++)
					{
						startPoint2 = &(pTrajectory2->m_pointArray[l]);
						endPoint2   = &(pTrajectory2->m_pointArray[l + 1]);
						length2     = MeasureDistanceFromPointToPoint(startPoint2, endPoint2);
						if (length2 < MIN_LINESEGMENT_LENGTH || length2 > MAX_LINESEGMENT_LENGTH) continue;

						if (m_closePartitionArrayIndexMap.count(IntIntPair(trajectoryId1,k)) != 0)  {
							int index1 = m_closePartitionArrayIndexMap[IntIntPair(trajectoryId1,k)];
							m_closePartitionArray[index1].push_back(IntIntPair(trajectoryId2,l));
						}
						else  {
							int size1 = (int)m_closePartitionArrayIndexMap.size();
							m_closePartitionArrayIndexMap[IntIntPair(trajectoryId1,k)] = size1;
							m_closePartitionArray.resize(size1 + 1);
							m_closePartitionArray[size1].push_back(IntIntPair(trajectoryId2,l));
						}
						if (m_closePartitionArrayIndexMap.count(IntIntPair(trajectoryId2,l)) != 0)  {
							int index2 = m_closePartitionArrayIndexMap[IntIntPair(trajectoryId2,l)];
							m_closePartitionArray[index2].push_back(IntIntPair(trajectoryId1,k));
						}
						else  {
							int size2 = (int)m_closePartitionArrayIndexMap.size();
							m_closePartitionArrayIndexMap[IntIntPair(trajectoryId2,l)] = size2;
							m_closePartitionArray.resize(size2 + 1);
							m_closePartitionArray[size2].push_back(IntIntPair(trajectoryId1,k));
						}
						m_finePartitionLengthMap[IntIntPair(trajectoryId1,k)] = length1;
						m_finePartitionLengthMap[IntIntPair(trajectoryId2,l)] = length2;
#ifndef __PRECOMPUTE_DENSITY__
						m_coarsePartitionIdMap[IntIntPair(trajectoryId1,k)]   = i;
						m_coarsePartitionIdMap[IntIntPair(trajectoryId2,l)]   = j;
#endif
					}
				}
			}
			// ... END

#undef __VERIFY_TIGHTNESS__	// enable this block to measure the tightness of the bounds
#ifdef __VERIFY_TIGHTNESS__
			// verify the tightness of the lower and upper bounds
			// START ...
			int trajectoryId1 = m_idArray[i].first, index1 = m_idArray[i].second;
			int trajectoryId2 = m_idArray[j].first, index2 = m_idArray[j].second;

			CTrajectory* pTrajectory1 = m_data->m_trajectoryList.GetAt(m_data->m_trajectoryList.FindIndex(trajectoryId1));
			CTrajectory* pTrajectory2 = m_data->m_trajectoryList.GetAt(m_data->m_trajectoryList.FindIndex(trajectoryId2));

			CMDPoint *startPoint1, *endPoint1, *startPoint2, *endPoint2;
			float realLowerBoundPerp  = (float)INT_MAX, realUpperBoundPerp  = (float)0.0;
			float realLowerBoundPara  = (float)INT_MAX, realUpperBoundPara  = (float)0.0;
			float realLowerBoundAngle = (float)INT_MAX, realUpperBoundAngle = (float)0.0;
			float realLowerBound = (float)INT_MAX, realUpperBound = (float)0.0;
			float realDistance[3], realDistanceSum;

			for (int k = pTrajectory1->m_partitionIndexArray[index1]; k < pTrajectory1->m_partitionIndexArray[index1 + 1]; k++)
			{
				startPoint1 = &(pTrajectory1->m_pointArray[k]);
				endPoint1   = &(pTrajectory1->m_pointArray[k + 1]);

				for (int l = pTrajectory2->m_partitionIndexArray[index2]; l < pTrajectory2->m_partitionIndexArray[index2 + 1]; l++)
				{
					startPoint2 = &(pTrajectory2->m_pointArray[l]);
					endPoint2   = &(pTrajectory2->m_pointArray[l + 1]);

					ComputeDistanceBetweenTwoLineSegments(startPoint1, endPoint1, startPoint2, endPoint2, realDistance);
					realDistanceSum = WEIGHTED_DISTANCE(realDistance[0], realDistance[1], realDistance[2]);

					if (realLowerBoundPerp > realDistance[0]) realLowerBoundPerp = realDistance[0];
					if (realUpperBoundPerp < realDistance[0]) realUpperBoundPerp = realDistance[0];
					if (realLowerBoundPara > realDistance[1]) realLowerBoundPara = realDistance[1];
					if (realUpperBoundPara < realDistance[1]) realUpperBoundPara = realDistance[1];
					if (realLowerBoundAngle > realDistance[2]) realLowerBoundAngle = realDistance[2];
					if (realUpperBoundAngle < realDistance[2]) realUpperBoundAngle = realDistance[2];
					if (realLowerBound > realDistanceSum)  { 
						realLowerBound = realDistanceSum; 
						// memcpy(minRealDistance, realDistance, sizeof(float) * 3); 
					}
					if (realUpperBound < realDistanceSum)  { 
						realUpperBound = realDistanceSum; 
						// memcpy(maxRealDistance, realDistance, sizeof(float) * 3); 
					}
				}
			}

			// count the number of maximal prunings
			if (realLowerBound > m_data->m_paramDistance || realUpperBound < m_data->m_paramDistance) nOptimalPrunedPairs++; 

			// count the number of false dismissals
			if ((estimatedLowerBound > m_data->m_paramDistance && realLowerBound <= m_data->m_paramDistance) || 
			    (estimatedUpperBound < m_data->m_paramDistance && realUpperBound >= m_data->m_paramDistance))
				nFalsePrunedPairs++;
			// ... END
#endif
#endif	// #if defined(__PARTITION_PRUNING_OPTIMIZATION__)
		}	// for (int j = i + 1; j < m_nTotalLineSegments; j++)
	}	// for (int i = 0; i < m_nTotalLineSegments; i++)
	// ... END	

#ifdef __VERIFY_TIGHTNESS__
	FILE *fp;
	fopen_s(&fp, RESULT_FILE, "a");
	CT2CA temp1(m_data->m_inputFilePath);
	std::string temp2(temp1);
#ifdef __COMBO_I__
	fprintf(fp, "COMBO I: %s %.1f => %d, %d, %d, %d\n", temp2.c_str(), g_DISTANCE_PARAMETER, nTotalPartitionPairs, nPrunedPartitionPairs, nFalsePrunedPairs, nOptimalPrunedPairs);
#else
	fprintf(fp, "COMBO II: %s %.1f => %d, %d, %d, %d\n", temp2.c_str(), g_DISTANCE_PARAMETER, nTotalPartitionPairs, nPrunedPartitionPairs, nFalsePrunedPairs, nOptimalPrunedPairs);
#endif
	fclose(fp);
#endif	// #ifdef __VERIFY_TIGHTNESS__

	// deallocate the copy of all line segments
	m_lineSegmentArray.clear();
	//m_lineSegmentArray.FreeExtra();

	return true;
}

bool COutlierDetector::DetectOutlier()
{
	CDistanceOutlier distanceOutlier(m_data->m_nTrajectories, m_nTotalLineSegments, &m_idArray, &m_distanceIndex);
	distanceOutlier.m_data = m_data;	// pass the pointer of the document

	// setup two parameters: p (i.e., fraction) and D (i.e., distance)
	distanceOutlier.SetFractionParameter(m_data->m_paramFraction);
	distanceOutlier.SetDistanceParameter(m_data->m_paramDistance);

	// setup the array of trajectory partitions' length
	distanceOutlier.SetLengthArray(&m_lengthArray);

#ifdef __PRECOMPUTE_DENSITY__
	distanceOutlier.SetDensityFilePath((m_data->m_inputFilePath + ".density"));
#endif

	vector<bool> outlierFlagArray;
#ifndef __PARTITION_PRUNING_OPTIMIZATION__
	outlierFlagArray.SetSize(m_nTotalLineSegments);
#else
	outlierFlagArray.resize((int)m_closePartitionArrayIndexMap.size());

	// setup the information for pruning optimization
	distanceOutlier.SetCloseTrajectoryPartition(&m_closePartitionArrayIndexMap, &m_closePartitionArray);
	distanceOutlier.SetFinePartitionLength(&m_finePartitionLengthMap);
	distanceOutlier.SetCoarsePartitionId(&m_coarsePartitionIdMap);
#endif

	// identify outlying trajectory partitions
	if (!distanceOutlier.DetectOutlyingLineSegment(&outlierFlagArray))
		return false;

	int currOutlierId = 0;
#ifndef __PARTITION_PRUNING_OPTIMIZATION__
	int currLineSegmentId = 0;

	CTypedPtrList<CObList,CTrajectory*>& trajectoryList = m_data->m_trajectoryList;
	POSITION pos = trajectoryList.GetHeadPosition();
	while (pos != NULL)
	{
		CTrajectory* pTrajectory = trajectoryList.GetNext(pos);
		int currTrajectoryId = pTrajectory->GetId();		

		// identify which trajectory partitions are outlying
		if (currLineSegmentId < m_nTotalLineSegments)
		{
			// check whether a specific line segment belongs to the current trajectory
			while (m_idArray[currLineSegmentId].first == currTrajectoryId)
			{
				if (outlierFlagArray[currLineSegmentId])
				{
					pTrajectory->AddOutlyingPartition(m_idArray[currLineSegmentId].second);
					m_data->m_nOutlyingPartitions++;
				}
				if (++currLineSegmentId >= m_nTotalLineSegments) break;
			}
		}

		// check if the proportion of outlying trajectory partitions is significant
		if (CheckOutlyingProportion(pTrajectory, g_MINIMUM_OUTLYING_PROPORTION))
		{
			pTrajectory->m_containOutlier = true;
			GenerateAndSetupOutlier(pTrajectory, currOutlierId);
			currOutlierId++;		// increase the number of outliers
		}
	}
#else	// #ifndef __PARTITION_PRUNING_OPTIMIZATION__
	map<IntIntPair,int>::const_iterator iter;
	for (iter = m_closePartitionArrayIndexMap.begin(); iter != m_closePartitionArrayIndexMap.end(); iter++)
	{
		if (outlierFlagArray[iter->second])
		{
			auto comp = [&](CTrajectory* traj) { return traj->m_trajectoryId == iter->first.first; };
			CTrajectory* pTrajectory = *(find_if(m_data->m_trajectoryList.begin(), m_data->m_trajectoryList.end(), comp));
			//CTrajectory* pTrajectory = m_data->m_trajectoryList.GetAt(m_data->m_trajectoryList.FindIndex(iter->first.first));
			pTrajectory->AddOutlyingPartition(iter->first.second);
			m_data->m_nOutlyingPartitions++;
		}
	}

	vector<CTrajectory*>& trajectoryList = m_data->m_trajectoryList;
	auto pos = trajectoryList.begin();
	while (pos != trajectoryList.end())
	{
		CTrajectory* pTrajectory = *pos;

		// check if the proportion of outlying trajectory partitions is significant
		if (CheckOutlyingProportion(pTrajectory, g_MINIMUM_OUTLYING_PROPORTION))
		{
			pTrajectory->m_containOutlier = true;
			GenerateAndSetupOutlier(pTrajectory, currOutlierId);
			currOutlierId++;		// increase the number of outliers
		}
		pos++;
	}
#endif	// #ifndef __PARTITION_PRUNING_OPTIMIZATION__

	return true;
}

bool COutlierDetector::CheckOutlyingProportion(CTrajectory* pTrajectory, float minProportion)
{
	CMDPoint* startPoint;
	CMDPoint* endPoint;
	float totalOutlyingLength = 0.0;

	// measure the ratio of the outlying length to the entire length
	for (int i = 0; i < pTrajectory->m_nOutlyingPartitions; i++)
	{
		int index = pTrajectory->m_outlyingPartitionArray[i];

#if defined(__USE_CONVENTIONAL_PARTITONING__) && !defined(__PARTITION_PRUNING_OPTIMIZATION__)
		startPoint = &(pTrajectory->m_partitionPointArray[index]);
		endPoint = &(pTrajectory->m_partitionPointArray[index + 1]);
#elif defined(__USE_NO_PARTITIONING__) || defined(__PARTITION_PRUNING_OPTIMIZATION__)
		startPoint = &(pTrajectory->m_pointArray[index]);
		endPoint = &(pTrajectory->m_pointArray[index + 1]);
#endif

		totalOutlyingLength += MeasureDistanceFromPointToPoint(startPoint, endPoint);
	}

	// keep the length of outlying partitions for a future use
	pTrajectory->SetOutlyingLength(totalOutlyingLength);

	// check if the outlying proportion is larger than a given threshold
	if ((totalOutlyingLength / pTrajectory->GetLength()) >= minProportion)
		return true;
	else
		return false;
}

bool COutlierDetector::GenerateAndSetupOutlier(CTrajectory* pTrajectory, int outlierId)
{
	COutlier* pOutlierItem = new COutlier(outlierId, pTrajectory->GetId(), m_data->m_nDimensions);

	// copy the information about an outlier to a newly created structure
	pOutlierItem->SetupInfo(pTrajectory);
	// append the newly created structure to the list
	m_data->m_outlierList.push_back(pOutlierItem);
	// increase the number of outlier trajectories
	m_data->m_nOutliers++;

	return true;
}


