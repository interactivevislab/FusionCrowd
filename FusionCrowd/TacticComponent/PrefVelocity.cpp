#include "PrefVelocity.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	namespace Agents
	{
		PrefVelocity::PrefVelocity() : _left(1.f, 0.f), _right(1.f, 0.f), _speed(1.f),
		                               _preferred(1.f, 0.f), _target(0.f, 0.f)
		{
		}

		PrefVelocity::PrefVelocity(const Vector2& dir, float speed, const Vector2& target) :
			_left(dir), _right(dir), _speed(speed), _preferred(dir), _target(target)
		{
		}

		PrefVelocity::PrefVelocity(const Vector2& left, const Vector2& right,
		                           const Vector2& pref, float speed, const Vector2& target) :
			_left(left), _right(right), _speed(speed),
			_preferred(pref), _target(target)
		{
		}

		/*
		PrefVelocity::PrefVelocity(const PrefVelocity& vel) : _left(vel._left),
		                                                      _right(vel._right),
		                                                      _speed(vel._speed),
		                                                      _preferred(vel._preferred),
		                                                      _target(vel._target)
		{
		}

		PrefVelocity& PrefVelocity::operator=(const PrefVelocity& vel)
		{
			// NOTE:
			//	This doesn't include the typical if ( this != &vel ) test.
			//	The reason for this is, it is considered highly unlikely that the user will
			//	ever do a = a (even indirectly), and, as such, it is deemed better to simply
			//	perform the copying cost instead of incurring the test for EVERY
			//	other event.
			_left = vel._left;
			_right = vel._right;
			_speed = vel._speed;
			_preferred = vel._preferred;
			_target = vel._target;
			return (*this);
		}
		*/

		void PrefVelocity::setSpan(const Vector2& left, const Vector2& right,
		                           const Vector2& preferred)
		{
			_left = left;
			_right = right;
			_preferred = preferred;

			const float SPAN_EPS = -1.e-4f;
		}
	}
}
