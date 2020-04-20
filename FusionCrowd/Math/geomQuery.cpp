#include "geomQuery.h"
#include "Math/consts.h"
#include "Math/Util.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	namespace Math
	{

		////////////////////////////////////////////////////////////////

		// determines the time to collision of a ray from the origin with a circle (center, radius)
		//	Returns an "infinity" style number if no collision
		float rayCircleTTC(const Vector2 & dir, const Vector2 & center, float radius) {
			float a = dir.LengthSquared();
			float b = -2 * dir.Dot(center);
			float c = center.LengthSquared() - (radius * radius);
			float discr = b * b - 4 * a * c;
			if (discr < 0.f) {
				return INFTY;
			}
			const float sqrtDiscr = sqrtf(discr);
			float t0 = (-b - sqrtDiscr) / (2.f * a);
			float t1 = (-b + sqrtDiscr) / (2.f * a);
			// If the points of collision have different signs, it means I'm already colliding
			if ((t0 < 0.f && t1 > 0.f) || (t1 < 0.f && t0 > 0.f)) return 0.f;
			if (t0 < t1 && t0 > 0.f)
				return t0;
			else if (t1 > 0.f)
				return t1;
			else
				return INFTY;
		}

		////////////////////////////////////////////////////////////////

		// Computes the spherical linear interpolation between two vectors
		//	the result is (conceptually) (1-t)*p0 + t*p1
		//	sinTheta is the sine of the angle between p1 and p1
		Vector2 slerp(float t, const Vector2 & p0, const Vector2 & p1, float sinTheta) {
			float theta = asin(sinTheta);
			float t0 = sin((1 - t) * theta) / sinTheta;
			float t1 = sin(t * theta) / sinTheta;
			return p0 * t0 + p1 * t1;
		}
	}	// namespace Math
}