#include "PointShape.h"

#include "TacticComponent/PrefVelocity.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	namespace Math
	{
		PointShape::PointShape()
			: _position(0.f, 0.f)
		{ }

		PointShape::PointShape(const Point & point)
			: _position(point.x, point.y)
		{ }

		PointShape::PointShape(const PointShape & shape)
			: _position(shape._position)
		{ }

		PointShape::PointShape(const PointShape & shape, const Vector2 & offset)
			: _position(shape._position + offset)
		{ }

		PointShape::PointShape(const DirectX::SimpleMath::Vector2& pos)
			: Geometry2D(), _position(pos)
		{ }

		PointShape PointShape::operator+(const Vector2 & pt)
		{
			return PointShape(*this, pt);
		}

		const DirectX::SimpleMath::Vector2& PointShape::getPosition() const
		{
			 return _position;
		}

		bool PointShape::containsPoint(const Vector2 & pt) const {
			float distSq = (pt - _position).LengthSquared();
			return distSq < 1e-6f;
		}

		float PointShape::squaredDistance(const Vector2 & pt) const {
			return (pt - _position).LengthSquared();
		}

		void PointShape::setDirections(const Vector2 & q, float r, Agents::PrefVelocity & directions) const
		{
			Vector2 disp = _position - q;
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
			directions.setTarget(_position);
		}

		Vector2 PointShape::getTargetPoint(const Vector2 & q, float r) const {
			return _position;
		}

		Vector2 PointShape::getCentroid() const {
			return _position;
		}

		float PointShape::BoundingRadius() const
		{
			return 1e-5;
		}

		PointShape* PointShape::Clone() const
		{
			return new PointShape(*this);
		}

		void PointShape::setPosition(const DirectX::SimpleMath::Vector2& pos)
		{
			_position = pos;
		}
	}
}
