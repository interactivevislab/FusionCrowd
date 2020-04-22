#pragma once

#include "Math/Util.h"
#include "Export/Math/Shapes.h"

namespace FusionCrowd
{
	namespace Agents
	{
		class PrefVelocity;
	}

	namespace Math
	{
		class Geometry2D
		{
		public:
			virtual bool containsPoint(const DirectX::SimpleMath::Vector2 & pt) const = 0;
			virtual bool containsPoint(const DirectX::SimpleMath::Vector2 & pt, const DirectX::SimpleMath::Vector2 & pos, float yaw) const = 0;
			virtual float squaredDistance(const DirectX::SimpleMath::Vector2 & pt) const = 0;
			virtual void setDirections(const DirectX::SimpleMath::Vector2 & q, float r, Agents::PrefVelocity & directions) const = 0;
			virtual DirectX::SimpleMath::Vector2 getTargetPoint(const DirectX::SimpleMath::Vector2 & q, float r) const = 0;
			virtual DirectX::SimpleMath::Vector2 getCentroid() const = 0;

			virtual float BoundingRadius() const = 0;
			virtual Geometry2D* Clone() const = 0;

			virtual ~Geometry2D();
		};

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
			virtual bool containsPoint(const DirectX::SimpleMath::Vector2 & pt) const;
			virtual bool containsPoint(const DirectX::SimpleMath::Vector2 & pt, const DirectX::SimpleMath::Vector2 & pos, float yaw) const;
			virtual float squaredDistance(const DirectX::SimpleMath::Vector2 & pt) const;
			virtual void setDirections(const DirectX::SimpleMath::Vector2 & q, float r,
				Agents::PrefVelocity & directions) const;
			virtual DirectX::SimpleMath::Vector2 getTargetPoint(const DirectX::SimpleMath::Vector2 & q, float r) const;
			virtual DirectX::SimpleMath::Vector2 getCentroid() const;

			float BoundingRadius() const override;

			PointShape* Clone() const override;
		protected:
			DirectX::SimpleMath::Vector2 _position;
		};

		class DiskShape: public Geometry2D
		{
		public:
			DiskShape(DirectX::SimpleMath::Vector2 center, float size);
			DiskShape(const Disk & disk);

			bool containsPoint(const DirectX::SimpleMath::Vector2& pt) const override;
			bool containsPoint(const DirectX::SimpleMath::Vector2& pt, const DirectX::SimpleMath::Vector2& pos, float yaw) const override;
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

		class ConeShape : public Geometry2D
		{
		public:
			ConeShape(DirectX::SimpleMath::Vector2 point, float range, float angle_rad) : _point(point), _range(range), _angle(angle_rad){}

			virtual bool containsPoint(const DirectX::SimpleMath::Vector2 & pt) const override;
			virtual bool containsPoint(const DirectX::SimpleMath::Vector2 & pt, const DirectX::SimpleMath::Vector2 & pos, float yaw) const override;
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

