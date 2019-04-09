#pragma once

#include "../Config.h"
#include "../MathUtil.h"

namespace Agents
{
	class FUSION_CROWD_API PrefVelocity
	{
	public:
		PrefVelocity();
		PrefVelocity(const DirectX::SimpleMath::Vector2 & dir, float speed, const DirectX::SimpleMath::Vector2 & target);
		PrefVelocity(const DirectX::SimpleMath::Vector2 & left, const DirectX::SimpleMath::Vector2 & right,
			const DirectX::SimpleMath::Vector2 & pref, float speed, const DirectX::SimpleMath::Vector2 & target);
		PrefVelocity(const PrefVelocity & vel);
		PrefVelocity & operator=(const PrefVelocity & vel);
		inline DirectX::SimpleMath::Vector2 getLeft() const { return _left; }
		inline DirectX::SimpleMath::Vector2 getRight() const { return _right; }
		inline DirectX::SimpleMath::Vector2 getPreferred() const { return _preferred; }
		inline DirectX::SimpleMath::Vector2 getPreferredVel() const { return _preferred * _speed; }
		inline float getSpeed() const { return _speed; }
		inline void setSpeed(float speed) { _speed = speed; }
		inline void setSingle(const DirectX::SimpleMath::Vector2 & dir)
		{
			_left = _preferred = _right = dir;
		}
		inline DirectX::SimpleMath::Vector2 getTarget() const { return _target; }
		inline void setTarget(const DirectX::SimpleMath::Vector2 & target) { _target = target; }
		void setSpan(const DirectX::SimpleMath::Vector2 & left, const DirectX::SimpleMath::Vector2 & right,
			const DirectX::SimpleMath::Vector2 & preferred);
		inline bool hasArea() const { return _left.Dot(_right) < 1.f; }

	protected:
		DirectX::SimpleMath::Vector2 _left;
		DirectX::SimpleMath::Vector2 _right;
		float _speed;
		DirectX::SimpleMath::Vector2 _preferred;
		DirectX::SimpleMath::Vector2 _target;
	};
}
