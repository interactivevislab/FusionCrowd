#include "ConeShape.h"
#include <iostream>

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	namespace Math
	{
		ConeShape::ConeShape(const Cone& cone)
			: _point(Vector2(cone.x, cone.y)), _range(cone.range), _angle(cone.angleRad)
		{ }

		ConeShape::ConeShape(DirectX::SimpleMath::Vector2 point, float range, float angle_rad)
			: _point(point), _range(range), _angle(angle_rad)
		{ }

		bool ConeShape::containsPoint(const Vector2 & pt) const {
			if (Vector2::DistanceSquared(pt, _point) > _range * _range) return false;
			float pt_cos = pt.x / pt.Length();
			float min_cos = cos(_angle / 2);
			return pt_cos >= min_cos;
		}

		float ConeShape::squaredDistance(const Vector2 & pt) const {
			throw std::runtime_error("Not implemented");
		}

		void ConeShape::setDirections(const Vector2 & q, float r, Agents::PrefVelocity & directions) const {
			throw std::runtime_error("Not implemented");
		}

		Vector2 ConeShape::getTargetPoint(const Vector2 & q, float r) const {
			throw std::runtime_error("Not implemented");
		}

		Vector2 ConeShape::getCentroid() const {
			throw std::runtime_error("Not implemented");
		}

		float ConeShape::BoundingRadius() const
		{
			return _range;
		}

		ConeShape* ConeShape::Clone() const
		{
			return new ConeShape(*this);
		}
	}
}
