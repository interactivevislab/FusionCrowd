#include "Geometry2D.h"
#include "../Path/PrefVelocity.h"

namespace FusionCrowd
{
	namespace Math
	{
		/////////////////////////////////////////////////////////////////////
		//                   Implementation of PointShape
		/////////////////////////////////////////////////////////////////////

		PointShape::PointShape(const PointShape & shape)
		{
			_position = shape._position;
		}

		/////////////////////////////////////////////////////////////////////

		PointShape::PointShape(const PointShape & shape, const Vector2 & offset) {
			_position = shape._position + offset;
		}

		/////////////////////////////////////////////////////////////////////

		PointShape PointShape::operator+(const Vector2 & pt) {
			return PointShape(*this, pt);
		}

		/////////////////////////////////////////////////////////////////////

		bool PointShape::containsPoint(const Vector2 & pt) const {
			float distSq = absSq(pt - _position);
			return distSq < 1e-6f;
		}

		/////////////////////////////////////////////////////////////////////

		bool PointShape::containsPoint(const Vector2 & pt, const Vector2 & pos) const {
			float distSq = absSq(pt - pos);
			return distSq < 1e-6f;
		}

		/////////////////////////////////////////////////////////////////////

		float PointShape::squaredDistance(const Vector2 & pt) const {
			return absSq(pt - _position);
		}

		/////////////////////////////////////////////////////////////////////

		void PointShape::setDirections(const Vector2 & q, float r,
			Agents::PrefVelocity & directions) const {
			Vector2 disp = _position - q;
			const float distSq = absSq(disp);
			Vector2 dir;
			if (distSq > 1e-8) {
				// Distant enough that I can normalize the direction.
				dir.set(disp / sqrtf(distSq));
			}
			else {
				dir.set(0.f, 0.f);
			}
			directions.setSingle(dir);
			directions.setTarget(_position);
		}

		/////////////////////////////////////////////////////////////////////

		Vector2 PointShape::getTargetPoint(const Vector2 & q, float r) const {
			return _position;
		}

		/////////////////////////////////////////////////////////////////////

		Vector2 PointShape::getCentroid() const {
			return _position;
		}
	}
}