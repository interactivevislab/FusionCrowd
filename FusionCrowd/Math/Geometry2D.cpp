#include "Geometry2D.h"
#include "TacticComponent/Path/PrefVelocity.h"
#include <iostream>
#include "Math/Util.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	namespace Math
	{
		PointShape::PointShape(const PointShape & shape)
		{
			_position = shape._position;
		}

		PointShape::PointShape(const PointShape & shape, const Vector2 & offset) {
			_position = shape._position + offset;
		}

		PointShape PointShape::operator+(const Vector2 & pt) {
			return PointShape(*this, pt);
		}

		bool PointShape::containsPoint(const Vector2 & pt) const {
			float distSq = (pt - _position).LengthSquared();
			return distSq < 1e-6f;
		}

		bool PointShape::containsPoint(const Vector2 & pt, const Vector2 & pos, float yaw) const {
			float distSq = (pt - pos).LengthSquared();
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

		/////////////////////////////////////////////////////////////////////

		DiskShape::DiskShape(DirectX::SimpleMath::Vector2 center, float R) : _center(center), _R(R) {}

		bool DiskShape::containsPoint(const DirectX::SimpleMath::Vector2& pt) const
		{
			return (pt - _center).LengthSquared() <= _R * _R;
		}

		bool DiskShape::containsPoint(const DirectX::SimpleMath::Vector2& pt, const DirectX::SimpleMath::Vector2& pos, float yaw) const
		{
			return (pt - (_center+pos)).LengthSquared() <= _R * _R;
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

		/////////////////////////////////////////////////////////////////////
		//                   Implementation of ConeShape
		/////////////////////////////////////////////////////////////////////
		bool ConeShape::containsPoint(const Vector2 & pt) const {
			if (Vector2::DistanceSquared(pt, _point) > _range * _range) return false;
			float pt_cos = pt.x / pt.Length();
			float min_cos = cos(_angle / 2);
			return pt_cos >= min_cos;
		}

		bool ConeShape::containsPoint(const Vector2 & pt, const Vector2 & pos, float yaw) const {
			Vector2 local_pt = pt - pos;
			local_pt = Math::rotate(local_pt, -yaw);
			return containsPoint(local_pt);
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