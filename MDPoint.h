#pragma once

// CMDPoint command target

/**
 * \brief A simple point class
 *
 * Has support for a variable number of dimensions.
 */
class CMDPoint
{
public:
	CMDPoint();
	CMDPoint(int nDimensions);
	virtual ~CMDPoint();
private:
	int m_nDimensions;		// the number of dimensions of a point
	float* m_coordinate;	// the coordinate of a point
public:
	const int GetNDimensions() const { return m_nDimensions; }
		///< Getter for the number of dimensions
	const float GetCoordinate(int nth) const { return m_coordinate[nth]; }
		///< Getter for the value of the nth dimension of the point
	void SetCoordinate(int nth, float value) { m_coordinate[nth] = value; }
		///< Setter for the value of the nth dimension of the point
};
