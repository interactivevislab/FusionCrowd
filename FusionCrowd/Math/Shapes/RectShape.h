#pragma once

#include "Math/Shapes/Geometry2D.h"
#include "Math/Util.h"

#include "Export/Math/Shapes.h"

namespace FusionCrowd
{
	namespace Math
	{
		class RectShape : public Geometry2D
		{
		public:
			RectShape(const Rect & rect);

			virtual bool containsPoint(const DirectX::SimpleMath::Vector2 & pt) const override;
			virtual float squaredDistance(const DirectX::SimpleMath::Vector2 & pt) const override;
			virtual void setDirections(const DirectX::SimpleMath::Vector2 & q, float r,
				Agents::PrefVelocity & directions) const override;
			virtual DirectX::SimpleMath::Vector2 getTargetPoint(const DirectX::SimpleMath::Vector2 & q, float r) const override;
			virtual DirectX::SimpleMath::Vector2 getCentroid() const override;

			float BoundingRadius() const override;

			RectShape* Clone() const override;

		private:
			bool InsideXStripe(const DirectX::SimpleMath::Vector2 pt) const;
			bool InsideYStripe(const DirectX::SimpleMath::Vector2 pt) const;

			DirectX::SimpleMath::Vector2 _topLeft;
			DirectX::SimpleMath::Vector2 _bottomRight;
		};
	}
}
