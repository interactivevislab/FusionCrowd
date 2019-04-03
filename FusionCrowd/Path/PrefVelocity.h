#pragma once

#include "../Math/vector.h"
#include "../Config.h"

namespace Agents
{
	class FUSION_CROWD_API PrefVelocity
	{
	public:
		PrefVelocity();
		PrefVelocity(const FusionCrowd::Math::Vector2 & dir, float speed, const FusionCrowd::Math::Vector2 & target);
		PrefVelocity(const FusionCrowd::Math::Vector2 & left, const FusionCrowd::Math::Vector2 & right,
			const FusionCrowd::Math::Vector2 & pref, float speed, const FusionCrowd::Math::Vector2 & target);
		PrefVelocity(const PrefVelocity & vel);
		PrefVelocity & operator=(const PrefVelocity & vel);
		inline FusionCrowd::Math::Vector2 getLeft() const { return _left; }
		inline FusionCrowd::Math::Vector2 getRight() const { return _right; }
		inline FusionCrowd::Math::Vector2 getPreferred() const { return _preferred; }
		inline FusionCrowd::Math::Vector2 getPreferredVel() const { return _preferred * _speed; }
		inline float getSpeed() const { return _speed; }
		inline void setSpeed(float speed) { _speed = speed; }
		inline void setSingle(const FusionCrowd::Math::Vector2 & dir)
		{
			_left = _preferred = _right = dir;
		}
		inline FusionCrowd::Math::Vector2 getTarget() const { return _target; }
		inline void setTarget(const FusionCrowd::Math::Vector2 & target) { _target = target; }
		void setSpan(const FusionCrowd::Math::Vector2 & left, const FusionCrowd::Math::Vector2 & right,
			const FusionCrowd::Math::Vector2 & preferred);
		inline bool hasArea() const { return _left * _right < 1.f; }

	protected:
		FusionCrowd::Math::Vector2 _left;
		FusionCrowd::Math::Vector2 _right;
		float _speed;
		FusionCrowd::Math::Vector2 _preferred;
		FusionCrowd::Math::Vector2 _target;
	};
}
