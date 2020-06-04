#pragma once

#include "Math/Util.h"
#include "Math/BoundingBox.h"

namespace FusionCrowd
{
	// public API here is a mess
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

		inline size_t getId() const { return _id; }
		BoundingBox GetBB() const;

		inline DirectX::SimpleMath::Vector2 normal() const
		{
			return DirectX::SimpleMath::Vector2(_unitDir.y, -_unitDir.x);
		}

		inline const DirectX::SimpleMath::Vector2& getP0() const { return _point; }

		inline const DirectX::SimpleMath::Vector2 midPt() const
		{
			return _point + (0.5f * _length) * _unitDir;
		}

		DirectX::SimpleMath::Vector2 getP1() const;

		const Obstacle* next() const { return _nextObstacle; }

		NearTypeEnum distanceSqToPoint(const DirectX::SimpleMath::Vector2& pt, DirectX::SimpleMath::Vector2& nearPt,
		                               float& distSq) const;

		float circleIntersection(const DirectX::SimpleMath::Vector2& dir, const DirectX::SimpleMath::Vector2& start,
		                         float radius) const;

		inline float length() const { return _length; }

		bool pointOnObstacle(const DirectX::SimpleMath::Vector2& pt) const;

		inline bool pointOutside(const DirectX::SimpleMath::Vector2& point) const
		{
			return _doubleSided || (FusionCrowd::Math::leftOf(_point, getP1(), point) < 0.f);
		}

		inline bool p0Convex(bool agtOnRight) const
		{
			return agtOnRight ? _isConvex : _doubleSided && !_isConvex;
		}

		inline bool p1Convex(bool agtOnRight) const
		{
			return _nextObstacle == 0x0
				       ? true
				       : (agtOnRight ? _nextObstacle->_isConvex : _doubleSided && _nextObstacle->_isConvex
				       );
		}

		inline void setClosedState(bool closed) { _doubleSided = !closed; }

		float distSqPoint(const DirectX::SimpleMath::Vector2& pt) const;

		bool _doubleSided;
		bool _isConvex;
		Obstacle* _nextObstacle;
		DirectX::SimpleMath::Vector2 _point;
		Obstacle* _prevObstacle;
		DirectX::SimpleMath::Vector2 _unitDir;
		float _length;
		size_t _id;
		size_t _class;
	};
}

inline float distSqPointLineSegment(const DirectX::SimpleMath::Vector2& a,
                                                     const DirectX::SimpleMath::Vector2& b,
                                                     const DirectX::SimpleMath::Vector2& c)
{
	const float r = (c - a).Dot(b - a) / (b - a).LengthSquared();

	if (r < 0.0f)
	{
		return (c - a).LengthSquared();
	}
	else if (r > 1.0f)
	{
		return (c - b).LengthSquared();
	}
	else
	{
		return (c - (a + r * (b - a))).LengthSquared();
	}
}
