#pragma once
#include "vector.h"
#include "../Config.h"
#include "../Path/PrefVelocity.h"

namespace FusionCrowd
{
	namespace Math
	{
		class FUSION_CROWD_API Geometry2D
		{
		public:
			Geometry2D() {}
			virtual ~Geometry2D() {}
			virtual bool containsPoint(const Vector2 & pt) const = 0;
			virtual bool containsPoint(const Vector2 & pt, const Vector2 & pos) const = 0;
			virtual float squaredDistance(const Vector2 & pt) const = 0;
			virtual void setDirections(const Vector2 & q, float r,
				Agents::PrefVelocity & directions) const = 0;
			virtual Vector2 getTargetPoint(const Vector2 & q, float r) const = 0;
			virtual Vector2 getCentroid() const = 0;
		};

		class FUSION_CROWD_API PointShape : public Geometry2D
		{
		public:
			PointShape() : _position(0.f, 0.f) {}
			PointShape(const Vector2 & pos) : Geometry2D(), _position(pos) {}
			PointShape(const PointShape & shape);
			PointShape(const PointShape & shape, const Vector2 & offset);
			PointShape operator+(const Vector2 & pt);
			void setPosition(const Vector2 & pos) { _position.set(pos); }
			const Vector2 & getPosition() const { return _position; }
			virtual bool containsPoint(const Vector2 & pt) const;
			virtual bool containsPoint(const Vector2 & pt, const Vector2 & pos) const;
			virtual float squaredDistance(const Vector2 & pt) const;
			virtual void setDirections(const Vector2 & q, float r,
				Agents::PrefVelocity & directions) const;
			virtual Vector2 getTargetPoint(const Vector2 & q, float r) const;
			virtual Vector2 getCentroid() const;
		protected:
			Vector2 _position;
		};
	}
}

