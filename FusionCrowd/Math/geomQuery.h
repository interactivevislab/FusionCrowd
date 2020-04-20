#pragma once

#include "Math/Util.h"

namespace FusionCrowd
{
	namespace Math
	{
		/*!
			*	@brief		Computes the time to collision between a ray and a circle
			*
			*				This is a special-case test.  It assumes the ray originates
			*				from the origin of the world.
			*
			*	@param		dir			A vector in R2 describing the direction (from the origin)
			*							of the ray.  (Does not need to be normalized)
			*	@param		center		A vector in R2 describing the position of the circle center.
			*	@param		radius		A float.  The radius of the circle.
			*	@returns	The expected "time" to collision ("infinity" if there is no collision).
			*/
		float rayCircleTTC(const DirectX::SimpleMath::Vector2 & dir, const DirectX::SimpleMath::Vector2 & center, float radius);

		/*!
			*	@brief		Perform spherical linear interpolation between two vectors
			*
			*	@param		t			The blend parameter.  T lies in the interval [0, 1]
			*	@param		p0			The first vector to interpolate (assumes ||p0|| = 1.0.
			*	@param		p1			The first vector to interpolate (assumes ||p1|| = 1.0.
			*	@param		sinTheta	The sine of the angle between the two vectors.
			*	@returns	The interpolated vector.
			*/
		DirectX::SimpleMath::Vector2 slerp(float t, const DirectX::SimpleMath::Vector2 & p0, const DirectX::SimpleMath::Vector2 & p1, float sinTheta);
	}	// namespace Math
}