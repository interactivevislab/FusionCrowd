#pragma once

#include "../Math/vector.h"

class Obstacle
{
public:

	enum NearTypeEnum
	{
		FIRST,
		MIDDLE,
		LAST
	};

	Obstacle();
	~Obstacle();

	inline  FusionCrowd::Math::Vector2 normal() const {
		return FusionCrowd::Math::Vector2(_unitDir.y(), -_unitDir.x());
	}

	inline const FusionCrowd::Math::Vector2 & getP0() const { return _point; }

	inline const FusionCrowd::Math::Vector2 midPt() const {
		return _point + (0.5f * _length) * _unitDir;
	}

	FusionCrowd::Math::Vector2 getP1() const;

	const Obstacle * next() const { return _nextObstacle; }

	NearTypeEnum distanceSqToPoint(const FusionCrowd::Math::Vector2 & pt, FusionCrowd::Math::Vector2 & nearPt,
		float & distSq) const;

	float circleIntersection(const FusionCrowd::Math::Vector2 & dir, const FusionCrowd::Math::Vector2 & start,
		float radius) const;

	inline float length() const { return _length; }

	bool pointOnObstacle(const FusionCrowd::Math::Vector2 & pt) const;

	inline bool pointOutside(const FusionCrowd::Math::Vector2 & point) const {
		return _doubleSided || (leftOf(_point, getP1(), point) < 0.f);
	}

	inline bool p0Convex(bool agtOnRight) const {
		return agtOnRight ? _isConvex : _doubleSided && !_isConvex;
	}

	inline bool p1Convex(bool agtOnRight) const {
		return _nextObstacle == 0x0 ?
			true :
			(agtOnRight ?
				_nextObstacle->_isConvex :
				_doubleSided && _nextObstacle->_isConvex
				);
	}

	inline void setClosedState(bool closed) { _doubleSided = !closed; }

	bool _doubleSided;
	bool _isConvex;
	Obstacle* _nextObstacle;
	FusionCrowd::Math::Vector2 _point;
	Obstacle* _prevObstacle;
	FusionCrowd::Math::Vector2 _unitDir;
	float _length;
	size_t _id;
	size_t _class;
};

inline float distSqPointLineSegment(const FusionCrowd::Math::Vector2& a,
	const FusionCrowd::Math::Vector2& b,
	const FusionCrowd::Math::Vector2& c)
{
	const float r = ((c - a) * (b - a)) / absSq(b - a);

	if (r < 0.0f) {
		return absSq(c - a);
	}
	else if (r > 1.0f) {
		return absSq(c - b);
	}
	else {
		return absSq(c - (a + r * (b - a)));
	}
}
