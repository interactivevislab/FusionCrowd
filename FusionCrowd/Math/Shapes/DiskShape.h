#pragma once

#include "Math/Util.h"
#include "Math/Shapes/Geometry2D.h"

#include "Export/Math/Shapes.h"

namespace FusionCrowd
{
	namespace Math
	{
		class DiskShape: public Geometry2D
		{
		public:
			DiskShape(DirectX::SimpleMath::Vector2 center, float size);
			DiskShape(const Disk & disk);

			bool containsPoint(const DirectX::SimpleMath::Vector2& pt) const override;
			float squaredDistance(const DirectX::SimpleMath::Vector2& pt) const override;
			void setDirections(const DirectX::SimpleMath::Vector2& q, float r, Agents::PrefVelocity& directions) const override;
			DirectX::SimpleMath::Vector2 getTargetPoint(const DirectX::SimpleMath::Vector2& q, float r) const override;
			DirectX::SimpleMath::Vector2 getCentroid() const override;

			float BoundingRadius() const override;

			DiskShape* Clone() const override;
		private:
			const DirectX::SimpleMath::Vector2 _center;
			const float _R;
		};
	}
}
