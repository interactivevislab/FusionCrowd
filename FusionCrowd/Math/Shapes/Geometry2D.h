#pragma once

#include "Math/Util.h"
#include "Export/Math/Shapes.h"

namespace FusionCrowd
{
	namespace Agents
	{
		class PrefVelocity;
	}

	namespace Math
	{
		class Geometry2D
		{
		public:
			virtual bool containsPoint(const DirectX::SimpleMath::Vector2 & pt) const = 0;
			virtual float squaredDistance(const DirectX::SimpleMath::Vector2 & pt) const = 0;
			virtual void setDirections(const DirectX::SimpleMath::Vector2 & q, float r, Agents::PrefVelocity & directions) const = 0;
			virtual DirectX::SimpleMath::Vector2 getTargetPoint(const DirectX::SimpleMath::Vector2 & q, float r) const = 0;
			virtual DirectX::SimpleMath::Vector2 getCentroid() const = 0;

			virtual float BoundingRadius() const = 0;
			virtual Geometry2D* Clone() const = 0;

			virtual ~Geometry2D();
		};
	}
}
