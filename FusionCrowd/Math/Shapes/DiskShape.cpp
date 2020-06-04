#include "DiskShape.h"

#include "TacticComponent/PrefVelocity.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	namespace Math
	{
		DiskShape::DiskShape(DirectX::SimpleMath::Vector2 center, float R)
			: _center(center), _R(R)
		{ }

		DiskShape::DiskShape(const Disk & disk)
			: _center(Vector2(disk.x, disk.y)), _R(disk.r)
		{ }

		bool DiskShape::containsPoint(const DirectX::SimpleMath::Vector2& pt) const
		{
			return (pt - _center).LengthSquared() <= _R * _R;
		}

		float DiskShape::squaredDistance(const DirectX::SimpleMath::Vector2& pt) const
		{
			float dist = (pt - _center).Length() - _R;
			return dist * dist;
		}

		void DiskShape::setDirections(const DirectX::SimpleMath::Vector2& q, float r, Agents::PrefVelocity& directions) const
		{
			Vector2 disp = _center - q;
			const float distSq = disp.LengthSquared();
			Vector2 dir;
			if (distSq > 1e-8) {
				// Distant enough that I can normalize the direction.
				dir = disp / sqrtf(distSq);
			}
			else {
				dir = Vector2(0.f, 0.f);
			}
			directions.setSingle(dir);
			directions.setTarget(_center);
		}

		Vector2 DiskShape::getTargetPoint(const Vector2& q, float r) const
		{
			return _center;
		}

		Vector2 DiskShape::getCentroid() const
		{
			return _center;
		}

		float DiskShape::BoundingRadius() const
		{
			return _R;
		}

		DiskShape* DiskShape::Clone() const
		{
			return new DiskShape(*this);
		}
	}
}