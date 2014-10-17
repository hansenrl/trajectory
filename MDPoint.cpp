// MDPoint.cpp : implementation file
//

//#include "stdafx.h"
#include "MDPoint.h"


// CMDPoint

CMDPoint::CMDPoint()
{
	m_nDimensions = 2;
	m_coordinate = new float [m_nDimensions];
	m_coordinate[0] = m_coordinate[1] = 0.0;
}

CMDPoint::CMDPoint(int nDimensions)
{
	m_nDimensions = nDimensions;
	m_coordinate = new float [m_nDimensions];
	for (int i = 0; i < m_nDimensions; i++) m_coordinate[i] = 0.0;
}

CMDPoint::~CMDPoint()
{
}


// CMDPoint member functions
