#pragma once

#include "Math/Util.h"

namespace FusionCrowd
{
	namespace Math
	{
		class Line
		{
		public:
			Line();
			Line(const DirectX::SimpleMath::Vector2 & p, const DirectX::SimpleMath::Vector2 & d);
			DirectX::SimpleMath::Vector2 NearestPt(const DirectX::SimpleMath::Vector2 & p) const;

			DirectX::SimpleMath::Vector2 _point;
			DirectX::SimpleMath::Vector2 _direction;
		};
	}
}

