#pragma once

#include "Math/Shapes/Geometry2D.h"
#include "Math/Util.h"

#include "Export/Math/Shapes.h"

namespace FusionCrowd
{
	namespace Math
	{
		class PointShape : public Geometry2D
		{
		public:
			PointShape();
			PointShape(const DirectX::SimpleMath::Vector2 & pos);
			PointShape(const Point & point);

			PointShape(const PointShape & shape);
			PointShape(const PointShape & shape, const DirectX::SimpleMath::Vector2 & offset);
			PointShape operator+(const DirectX::SimpleMath::Vector2 & pt);

			void setPosition(const DirectX::SimpleMath::Vector2 & pos);
			const DirectX::SimpleMath::Vector2 & getPosition() const;

			virtual bool containsPoint(const DirectX::SimpleMath::Vector2 & pt) const override;
			virtual float squaredDistance(const DirectX::SimpleMath::Vector2 & pt) const override;
			virtual void setDirections(const DirectX::SimpleMath::Vector2 & q, float r,
				Agents::PrefVelocity & directions) const override;
			virtual DirectX::SimpleMath::Vector2 getTargetPoint(const DirectX::SimpleMath::Vector2 & q, float r) const override;
			virtual DirectX::SimpleMath::Vector2 getCentroid() const override;

			float BoundingRadius() const override;

			PointShape* Clone() const override;
		protected:
			DirectX::SimpleMath::Vector2 _position;
		};
	}
}
