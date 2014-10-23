// DistanceOutlier.cpp : implementation file
//

#include "TraOutlier.h"
#include "DistanceOutlier.h"
#include <fstream>
#include <math.h>


// CDistanceOutlier

CDistanceOutlier::CDistanceOutlier(int nTrajectories, int nLineSegments, IntIntPairArray* idArray, FloatMatrix* distanceIndex)
{
	m_nTrajectories = nTrajectories;
	m_nLineSegments = nLineSegments;
	m_idArray       = idArray;
	m_distanceIndex = distanceIndex;
}

CDistanceOutlier::~CDistanceOutlier()
{
}

bool CDistanceOutlier::DetectOutlyingLineSegment(vector<bool>* result)
{
	m_nNearTrajectories = (int)ceil((double)m_nTrajectories * (1.0 - (double)m_fractionParam));

#ifdef __INCORPORATE_DENSITY__
	MeasureDensityOfLineSegment();
#endif

#ifndef __PARTITION_PRUNING_OPTIMIZATION__
	ConstructRangeOfTrajectory();

	for (int i = 0; i < m_nLineSegments; i++)
	{
#ifdef __VISUALIZE_DEBUG_INFO__
		GetLineSegmentPoint(i, &m_document->m_currStartPoint, &m_document->m_currEndPoint);
		m_document->m_lineSegmentDensity = m_densityArray[i];
		m_document->m_nCloseLineSegments = 0;
		m_document->m_startPointArray.RemoveAll();
		m_document->m_endPointArray.RemoveAll();
#endif

#ifdef __INCORPORATE_DENSITY__
		if ((int)ceil(GetNumOfNearTrajectories(i) * m_densityArray[i]) <= m_nNearTrajectories)
#else
		if (GetNumOfNearTrajectories(i) <= m_nNearTrajectories)
#endif
			result->SetAt(i, true);
		else
			result->SetAt(i, false);

#ifdef __VISUALIZE_DEBUG_INFO__
		m_document->UpdateAllViews(NULL);
		AfxMessageBox(TEXT("Check the outlierness"));
#endif	
	}
#else	/* #ifndef __PARTITION_PRUNING_OPTIMIZATION__ */
	int nCloseTrajectories;
	map<int,float> closeTrajectoryMap;

	map<IntIntPair,int>::const_iterator iter;
	for (iter = m_closePartitionArrayIndexMap->begin(); iter != m_closePartitionArrayIndexMap->end(); iter++)
	{
		float length         = (*m_finePartitionLengthMap)[iter->first];
		int currIndex        = iter->second;
		int nClosePartitions = (int)(*m_closePartitionArray)[currIndex].size();
		IntIntPair closePartition;

		nCloseTrajectories = 0;
		closeTrajectoryMap.clear();

		for (int i = 0; i < nClosePartitions; i++)
		{
			closePartition = (*m_closePartitionArray)[currIndex][i];

			if (closeTrajectoryMap.find(closePartition.first) != closeTrajectoryMap.end())
				closeTrajectoryMap[closePartition.first] += (*m_finePartitionLengthMap)[closePartition];
			else
				closeTrajectoryMap[closePartition.first]  = (*m_finePartitionLengthMap)[closePartition];
		}

		map<int,float>::const_iterator iter_tra;
		for (iter_tra = closeTrajectoryMap.begin(); iter_tra != closeTrajectoryMap.end(); iter_tra++)
			// NOTE: be careful for this design
			if (iter_tra->second >= length) nCloseTrajectories++;
			// nCloseTrajectories++;

#ifdef __INCORPORATE_DENSITY__
		float density = m_densityArray[(*m_coarsePartitionIdMap)[iter->first]];
		if ((int)ceil(nCloseTrajectories * density) <= m_nNearTrajectories)
#else
		if (nCloseTrajectories <= m_nNearTrajectories)
#endif
			(*result)[currIndex] = true;
		else
			(*result)[currIndex] = false;

#ifdef __VISUALIZE_DEBUG_INFO__
		GetLineSegmentPoint(iter->first.first, iter->first.second, &m_document->m_currStartPoint, &m_document->m_currEndPoint);
		m_document->m_lineSegmentDensity = m_densityArray[(*m_coarsePartitionIdMap)[iter->first]];
		m_document->m_nCloseTrajectories = nCloseTrajectories;
		m_document->m_nCloseLineSegments = nClosePartitions;
		m_document->m_startPointArray.RemoveAll();
		m_document->m_endPointArray.RemoveAll();
		for (int i = 0; i < nClosePartitions; i++)
		{
			CPoint startPoint, endPoint;
			IntIntPair partition = (m_closePartitionArray->GetAt(currIndex)).GetAt(i);
			GetLineSegmentPoint(partition.first, partition.second, &startPoint, &endPoint);
			m_document->m_startPointArray.Add(startPoint);
			m_document->m_endPointArray.Add(endPoint);
		}
		m_document->UpdateAllViews(NULL);
		AfxMessageBox(TEXT("Check the outlierness"));
#endif
	}
#endif	/* #ifndef __PARTITION_PRUNING_OPTIMIZATION__ */

	return true;
}

int CDistanceOutlier::GetNumOfNearTrajectories(int index)
{
	int currTrajectoryId = (*m_idArray)[index].first;
	int nNearTrajectories = 0;

	for (int trajectoryId = 0; trajectoryId < currTrajectoryId; trajectoryId++)
		if (IsTrajectoryNear(index, trajectoryId))
			nNearTrajectories++;
	
	for (int trajectoryId = currTrajectoryId + 1; trajectoryId < m_nTrajectories; trajectoryId++)
		if (IsTrajectoryNear(index, trajectoryId))
			nNearTrajectories++;

#ifdef __VISUALIZE_DEBUG_INFO__
	m_document->m_nCloseTrajectories = nNearTrajectories;
#endif

	return nNearTrajectories;
}

bool CDistanceOutlier::IsTrajectoryNear(int lineSegmentId, int trajectoryId)
{
	int startLineSegmentId = m_trajectoryRange[trajectoryId].first;
	int endLineSegmentId =   m_trajectoryRange[trajectoryId].second;
	float totalCloseDistance = 0.0;

	for (int i = startLineSegmentId; i <= endLineSegmentId; i++)
	{
		if (DISTANCE_MATRIX(lineSegmentId, i) <= m_distanceParam)
		{
			totalCloseDistance += LENGTH_ARRAY(i);
#ifdef __VISUALIZE_DEBUG_INFO__
			CPoint startPoint, endPoint;
			GetLineSegmentPoint(i, &startPoint, &endPoint);
			m_document->m_nCloseLineSegments++;
			m_document->m_startPointArray.Add(startPoint);
			m_document->m_endPointArray.Add(endPoint);
#endif
		}
	}

	// NOTE: be careful for this design
	if (totalCloseDistance >= LENGTH_ARRAY(lineSegmentId)) return true;
	// if (totalCloseDistance > 0.0) return true;

	// if no trajectory partition is close to a specific line segment
	return false;
}

void CDistanceOutlier::ConstructRangeOfTrajectory()
{
	m_trajectoryRange.resize(m_nTrajectories);

	for (int i = 0; i < m_nTrajectories; i++)
		m_trajectoryRange[i].first = m_trajectoryRange[i].second = 0;

	for (int i = 0; i < m_nLineSegments; i++)
	{
		int trajectoryId = (*m_idArray)[i].first;
		// the first appearance of a specific trajectory
		if (m_trajectoryRange[trajectoryId].first == 0)
			m_trajectoryRange[trajectoryId].first = m_trajectoryRange[trajectoryId].second = i;
		// subsequent appearances of the trajectory
		else
			m_trajectoryRange[trajectoryId].second = i;
	}
}

#ifdef __INCORPORATE_DENSITY__
void CDistanceOutlier::MeasureDensityOfLineSegment()
{
#ifdef __PRECOMPUTE_DENSITY__
	ifstream istr(m_densityFilePath);
	if (istr)
	{
		int nLineSegments, trajectoryId, offset;
		istr >> nLineSegments;
		m_densityArray.resize(nLineSegments);
		for (int i = 0; i < nLineSegments; i++)
		{
			istr >> trajectoryId >> offset >> m_densityArray[i];
#ifdef __PARTITION_PRUNING_OPTIMIZATION__
			(*m_coarsePartitionIdMap)[IntIntPair(trajectoryId,offset)] = i;
#endif
		}
		istr.close();
		return;
	}
#endif

	// calculate the standard deviation of the distances between line segments
	// START ...
	double avgDistance = 0.0;
	double avgSquaredError = 0.0;
	double stDev;

	for (int i = 0; i < m_nLineSegments; i++)
		for (int j = i + 1; j < m_nLineSegments; j++)
			avgDistance += (2 * DISTANCE_MATRIX(i, j) / (float)(m_nLineSegments * (m_nLineSegments - 1)));

	for (int i = 0; i < m_nLineSegments; i++)
		for (int j = i + 1; j < m_nLineSegments; j++)
			avgSquaredError += (2 * pow((DISTANCE_MATRIX(i, j) - avgDistance), 2) / (float)(m_nLineSegments * (m_nLineSegments - 1)));

	stDev = sqrt(avgSquaredError);
	// ... END

	// calculate the density of each line segment
	// START ...
	set<int> trajectoryIds;
	int lineSegmentCount;
	int totalDensity = 0;
	float avgDensity;

	for (int i = 0; i < m_nLineSegments; i++)
	{
		trajectoryIds.clear();
		lineSegmentCount = 0;

#ifdef __NEVER_DEFINE__
		// the density is defined as the number of distinct trajectories within a given radius
		// here, the radius is the standard deviation
		for (int j = 0; j < i; j++)
			if (DISTANCE_MATRIX(i, j) <= stDev) trajectoryIds.insert(m_idArray->GetAt(j).first);
		for (int j = i + 1; j < m_nLineSegments; j++)
			if (DISTANCE_MATRIX(i, j) <= stDev) trajectoryIds.insert(m_idArray->GetAt(j).first);

		totalDensity += (int)trajectoryIds.size();
		m_densityArray.Add((float)trajectoryIds.size());
#endif

		// if the density is computed using the number of line segments, enable this block
		for (int j = 0; j < i; j++)
			if (DISTANCE_MATRIX(i, j) <= stDev) lineSegmentCount += 1;
		for (int j = i + 1; j < m_nLineSegments; j++)
			if (DISTANCE_MATRIX(i, j) <= stDev) lineSegmentCount += 1;

		totalDensity += lineSegmentCount;
		m_densityArray.push_back((float)lineSegmentCount);
	}

	avgDensity = (float)totalDensity / (float)m_nLineSegments;

	// keep the ratio of the average density to the density of a specific line segment
	for (int i = 0; i < m_nLineSegments; i++)
		m_densityArray[i] = avgDensity / m_densityArray[i];
	// ... END

#if 0	// enable this block to create a density file
	ofstream ofs(m_densityFilePath);
	if (ofs)
	{
		ofs << m_nLineSegments << endl;

		int currLineSegmentId = 0;
		CMDPoint *startPoint, *endPoint;
		CTypedPtrList<CObList,CTrajectory*>& trajectoryList = m_document->m_trajectoryList;
		POSITION pos = trajectoryList.GetHeadPosition();
		while (pos != NULL)
		{
			CTrajectory* pTrajectory = trajectoryList.GetNext(pos);

			for (int i = 0; i < pTrajectory->m_nPoints - 1; i++)
			{
				startPoint = &(pTrajectory->m_pointArray[i]);
				endPoint = &(pTrajectory->m_pointArray[i + 1]);

				if (MeasureDistanceFromPointToPoint(startPoint, endPoint) < MIN_LINESEGMENT_LENGTH)
					continue;

				ofs << pTrajectory->GetId() << ' ' << i << ' ' << m_densityArray[currLineSegmentId] << endl;
				currLineSegmentId++;
			}
		}

		ASSERT(m_nLineSegments == currLineSegmentId);
		ofs.close();
	}
#endif
}
#endif

// this function is borrowed from the COutlierDetector class
float CDistanceOutlier::MeasureDistanceFromPointToPoint(CMDPoint* point1, CMDPoint* point2)
{
	int nDimensions = point1->GetNDimensions();
	float squareSum = 0.0;
	
	for (int i = 0; i < nDimensions; i++)
		squareSum += pow((point2->GetCoordinate(i) - point1->GetCoordinate(i)), 2);

	return sqrt(squareSum);
}

#ifdef __VISUALIZE_DEBUG_INFO__
void CDistanceOutlier::GetLineSegmentPoint(int lineSegmentId, CPoint* start, CPoint *end)
{
	int trajectoryId = (m_idArray->GetAt(lineSegmentId)).first;
	int index        = (m_idArray->GetAt(lineSegmentId)).second;

	CTrajectory* pTrajectory = m_document->m_trajectoryList.GetAt(m_document->m_trajectoryList.FindIndex(trajectoryId));

	CMDPoint *startPoint, *endPoint;
#if defined(__USE_CONVENTIONAL_PARTITONING__) && !defined(__PARTITION_PRUNING_OPTIMIZATION__)
	startPoint = &(pTrajectory->m_partitionPointArray[index]);
	endPoint = &(pTrajectory->m_partitionPointArray[index + 1]);
#elif defined(__USE_NO_PARTITIONING__) || defined(__PARTITION_PRUNING_OPTIMIZATION__)
	startPoint = &(pTrajectory->m_pointArray[index]);
	endPoint = &(pTrajectory->m_pointArray[index + 1]);
#endif

	start->SetPoint((int)startPoint->GetCoordinate(0), (int)startPoint->GetCoordinate(1));
	end->SetPoint((int)endPoint->GetCoordinate(0), (int)endPoint->GetCoordinate(1));
}

void CDistanceOutlier::GetLineSegmentPoint(int trajectoryId, int index, CPoint* start, CPoint *end)
{
	CTrajectory* pTrajectory = m_document->m_trajectoryList.GetAt(m_document->m_trajectoryList.FindIndex(trajectoryId));

	CMDPoint *startPoint, *endPoint;
#if defined(__USE_CONVENTIONAL_PARTITONING__) && !defined(__PARTITION_PRUNING_OPTIMIZATION__)
	startPoint = &(pTrajectory->m_partitionPointArray[index]);
	endPoint = &(pTrajectory->m_partitionPointArray[index + 1]);
#elif defined(__USE_NO_PARTITIONING__) || defined(__PARTITION_PRUNING_OPTIMIZATION__)
	startPoint = &(pTrajectory->m_pointArray[index]);
	endPoint = &(pTrajectory->m_pointArray[index + 1]);
#endif

	start->SetPoint((int)startPoint->GetCoordinate(0), (int)startPoint->GetCoordinate(1));
	end->SetPoint((int)endPoint->GetCoordinate(0), (int)endPoint->GetCoordinate(1));
}
#endif
