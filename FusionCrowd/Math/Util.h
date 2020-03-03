#pragma once

#define NOMINMAX
#include <d3d11.h>
#include "SimpleMath.h"
#undef NOMINMAX

namespace FusionCrowd
{
	namespace MathUtil
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

		const float INFTY  = 3.402823E+38f;

		const float PI     = 3.14159265f;
		const float TWOPI  = 6.28318530718f;
		const float HALFPI = 1.57079632679f;

		const float DEG_TO_RAD = 0.0174533f;
		const float RAD_TO_DEG = 57.2958f;

		const float EPS = 0.00001f;
	}
}