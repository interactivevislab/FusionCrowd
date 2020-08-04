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

		inline float clamp(const float& val, const float& min, const float& max)
		{
			if(val < min) return min;
			if(val > max) return max;

			return val;
		}

		inline void projectOnSegment(const DirectX::SimpleMath::Vector2& s1, const DirectX::SimpleMath::Vector2& s2,
			const DirectX::SimpleMath::Vector2& p, DirectX::SimpleMath::Vector2& outProjection)
		{
			if (s1 == s2)
			{
				outProjection = s1;
				return;
			}

			outProjection = s2 - s1;
			outProjection *= clamp((p - s1).Dot(outProjection) / outProjection.LengthSquared(), 0.0f, 1.0f);
			outProjection += s1;
		}

		inline float distanceToSegment(DirectX::SimpleMath::Vector2 s1, DirectX::SimpleMath::Vector2 s2, DirectX::SimpleMath::Vector2 p)
		{
			DirectX::SimpleMath::Vector2 projection;
			projectOnSegment(s1, s2, p, projection);

			return DirectX::SimpleMath::Vector2::Distance(p, projection);
		}
	}
}