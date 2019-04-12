#pragma once

#include "Config.h"
#include "Math/MathUtil.h"

namespace FusionCrowd
{
	namespace Math
	{
		class FUSION_CROWD_API Line
		{
		public:
			Line();
			Line(const DirectX::SimpleMath::Vector2 & p, const DirectX::SimpleMath::Vector2 & d);
			~Line();
			DirectX::SimpleMath::Vector2 NearestPt(const DirectX::SimpleMath::Vector2 & p) const;

			DirectX::SimpleMath::Vector2 _point;
			DirectX::SimpleMath::Vector2 _direction;
		};
	}
}

