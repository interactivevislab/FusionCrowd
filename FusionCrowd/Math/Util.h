#pragma once

#define NOMINMAX
#include <d3d11.h>
#include "SimpleMath.h"
#undef NOMINMAX

namespace FusionCrowd
{
	namespace Math
	{
		inline float det(const DirectX::SimpleMath::Vector2 & v1,
	             const DirectX::SimpleMath::Vector2 & v2)
		{
			return v1.x * v2.y - v1.y * v2.x;
		}

		inline float leftOf(const DirectX::SimpleMath::Vector2 & a,
							const DirectX::SimpleMath::Vector2 & b,
							const DirectX::SimpleMath::Vector2 & c)
		{
			return det(a - c, b - a);
		}

		inline DirectX::SimpleMath::Vector2 rotate(DirectX::SimpleMath::Vector2 vec, float rad)
		{
			return DirectX::SimpleMath::Vector2(
				vec.x * cosf(rad) - vec.y * sinf(rad),
				vec.x * sinf(rad) + vec.y * cosf(rad)
			);
		}

		inline int sgn(float val)
		{
			return (0 < val) - (val < 0);
		}

		inline float clamp(float val, float min, float max)
		{
			if(val < min) return min;
			if(val > max) return max;

			return val;
		}

		inline float distanceToSegment(DirectX::SimpleMath::Vector2 s1, DirectX::SimpleMath::Vector2 s2, DirectX::SimpleMath::Vector2 p)
		{
			  const float l2 = (s2 - s1).LengthSquared();

			  if (l2 == 0.0)
				  return DirectX::SimpleMath::Vector2::Distance(p, s2);

			  const float t = clamp((p - s1).Dot(s2 - s1) / l2, 0, 1);
			  const DirectX::SimpleMath::Vector2 projection = s1 + t * (s2 - s1);  // Projection falls on the segment
			  return DirectX::SimpleMath::Vector2::Distance(p, projection);
		}
	}
}