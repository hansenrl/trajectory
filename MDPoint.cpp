// MDPoint.cpp : implementation file
//

//#include "stdafx.h"
#include "MDPoint.h"


// CMDPoint

/**
 * \brief Default constructor
 *
 * Constructs a point with dimension 2 and both dimension values equal to 0.0.
 */
CMDPoint::CMDPoint()
{
	m_nDimensions = 2;
	m_coordinate = new float [m_nDimensions];
	m_coordinate[0] = m_coordinate[1] = 0.0;
}

/**
 * \brief Constructor for a point with custom dimension
 *
 * @param [in] nDimensions the number of dimensions the point exists in
 */
CMDPoint::CMDPoint(int nDimensions)
{
	m_nDimensions = nDimensions;
	m_coordinate = new float [m_nDimensions];
	for (int i = 0; i < m_nDimensions; i++) m_coordinate[i] = 0.0;
}

CMDPoint::~CMDPoint()
{
}
