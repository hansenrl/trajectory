#pragma once

// CMDPoint command target

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
	const float GetCoordinate(int nth) const { return m_coordinate[nth]; }
	void SetCoordinate(int nth, float value) { m_coordinate[nth] = value; }
};