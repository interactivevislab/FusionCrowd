#pragma once

#include "../Config.h"
#include "../Path/PrefVelocity.h"
#include "../MathUtil.h"

namespace FusionCrowd
{
	namespace Math
	{
		class FUSION_CROWD_API Geometry2D
		{
		public:
			Geometry2D() {}
			virtual ~Geometry2D() {}
			virtual bool containsPoint(const DirectX::SimpleMath::Vector2 & pt) const = 0;
			virtual bool containsPoint(const DirectX::SimpleMath::Vector2 & pt, const DirectX::SimpleMath::Vector2 & pos) const = 0;
			virtual float squaredDistance(const DirectX::SimpleMath::Vector2 & pt) const = 0;
			virtual void setDirections(const DirectX::SimpleMath::Vector2 & q, float r,
				Agents::PrefVelocity & directions) const = 0;
			virtual DirectX::SimpleMath::Vector2 getTargetPoint(const DirectX::SimpleMath::Vector2 & q, float r) const = 0;
			virtual DirectX::SimpleMath::Vector2 getCentroid() const = 0;
		};

		class FUSION_CROWD_API PointShape : public Geometry2D
		{
		public:
			PointShape() : _position(0.f, 0.f) {}
			PointShape(const DirectX::SimpleMath::Vector2 & pos) : Geometry2D(), _position(pos) {}
			PointShape(const PointShape & shape);
			PointShape(const PointShape & shape, const DirectX::SimpleMath::Vector2 & offset);
			PointShape operator+(const DirectX::SimpleMath::Vector2 & pt);
			void setPosition(const DirectX::SimpleMath::Vector2 & pos) { _position = pos; }
			const DirectX::SimpleMath::Vector2 & getPosition() const { return _position; }
			virtual bool containsPoint(const DirectX::SimpleMath::Vector2 & pt) const;
			virtual bool containsPoint(const DirectX::SimpleMath::Vector2 & pt, const DirectX::SimpleMath::Vector2 & pos) const;
			virtual float squaredDistance(const DirectX::SimpleMath::Vector2 & pt) const;
			virtual void setDirections(const DirectX::SimpleMath::Vector2 & q, float r,
				Agents::PrefVelocity & directions) const;
			virtual DirectX::SimpleMath::Vector2 getTargetPoint(const DirectX::SimpleMath::Vector2 & q, float r) const;
			virtual DirectX::SimpleMath::Vector2 getCentroid() const;
		protected:
			DirectX::SimpleMath::Vector2 _position;
		};
	}
}

