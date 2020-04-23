#pragma once

#include "Math/Shapes/Geometry2D.h"
#include "Math/Util.h"

#include "Export/Math/Shapes.h"

namespace FusionCrowd
{
	namespace Math
	{
		class ConeShape : public Geometry2D
		{
		public:
			ConeShape(const Cone & cone);
			ConeShape(DirectX::SimpleMath::Vector2 point, float range, float angle_rad);

			virtual bool containsPoint(const DirectX::SimpleMath::Vector2 & pt) const override;
			virtual float squaredDistance(const DirectX::SimpleMath::Vector2 & pt) const;
			virtual void setDirections(const DirectX::SimpleMath::Vector2 & q, float r, Agents::PrefVelocity & directions) const override;
			virtual DirectX::SimpleMath::Vector2 getTargetPoint(const DirectX::SimpleMath::Vector2 & q, float r) const override;
			virtual DirectX::SimpleMath::Vector2 getCentroid() const override;

			float BoundingRadius() const override;

			ConeShape* Clone() const override;
		private:
			float _range;
			float _angle;
			DirectX::SimpleMath::Vector2 _point;
		};
	}
}
