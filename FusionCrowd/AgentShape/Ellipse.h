#pragma once
#include "../MathUtil.h"
#include "../NavComponents/Obstacle.h"

namespace FusionCrowd
{
	namespace AgentShape
	{
		class Ellipse
		{
		public:
			Ellipse();
			Ellipse(const DirectX::SimpleMath::Vector2& center);
			Ellipse(const DirectX::SimpleMath::Vector2& center, const DirectX::SimpleMath::Vector2& axes);
			Ellipse(const DirectX::SimpleMath::Vector2& center, const DirectX::SimpleMath::Vector2& axes, float angle);

			inline float ellipseCenterDistance(const Ellipse& other) const
			{
				return (other._center - _center).Length();
			}

			inline DirectX::SimpleMath::Vector2 ellipseCenterDisplace(const Ellipse& other) const {
				return _center - other._center;
			}

			float DistanceOfClosestApproach(const Ellipse& other) const;
			float ApproxDistanceOfClosestApproach(const Ellipse& other) const;
			DirectX::SimpleMath::Vector2 ClosestPoint(const DirectX::SimpleMath::Vector2& pt) const;
			float MinimumDistance(const Obstacle* line, DirectX::SimpleMath::Vector2& dir) const;
			float DistanceOfClosestApproach(const Obstacle* line) const;
			float ApproximateMinimumDistance(const DirectX::SimpleMath::Vector2& pt) const;
			float RadiusInPointDirection(const DirectX::SimpleMath::Vector2& pt) const;
			float RadiusInDirection(const DirectX::SimpleMath::Vector2& dir) const;
			DirectX::SimpleMath::Vector2 ToEllipseSpace(const DirectX::SimpleMath::Vector2& pt) const;
			DirectX::SimpleMath::Vector2 FromEllipseSpace(const DirectX::SimpleMath::Vector2& pt) const;

			inline void SetOrientation(float angle)
			{
				_cosPhi = cos(angle);
				_sinPhi = sin(angle);
			}

			inline void SetOrientation(const DirectX::SimpleMath::Vector2& dir)
			{
				_cosPhi = dir.x;
				_sinPhi = dir.y;
			}

			inline DirectX::SimpleMath::Vector2 GetOrientation() const
			{
				return DirectX::SimpleMath::Vector2(_cosPhi, _sinPhi);
			}

			inline void SetCenter(const DirectX::SimpleMath::Vector2& pos)
			{
				_center = pos;
			}

			inline const DirectX::SimpleMath::Vector2& getCenter() const
			{
				return _center;
			}

			inline void SetAxes(const DirectX::SimpleMath::Vector2& axes) {
				_majorAxis = axes.x;
				_minorAxis = axes.y;
			}

			inline void SetAxes(float major, float minor)
			{
				_majorAxis = major;
				_minorAxis = minor;
			}

			inline void SetMajorAxis(float length)
			{
				_majorAxis = length;
			}

			inline float GetMajor() const
			{
				return _majorAxis;
			}

			inline void SetMinorAxis(float length)
			{
				_minorAxis = length;
			}

			inline float GetMinor() const
			{
				return _minorAxis;
			}

			inline float GetSmallerAxis() const
			{
				return _minorAxis < _majorAxis ? _minorAxis : _majorAxis;
			}

			inline float GetLargerAxis() const
			{
				return _minorAxis < _majorAxis ? _majorAxis : _minorAxis;
			}

		protected:
			DirectX::SimpleMath::Vector2 _center;
			float _cosPhi;
			float _sinPhi;
			float _majorAxis;
			float _minorAxis;
		};
	}
}

