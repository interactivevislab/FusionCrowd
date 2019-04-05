#pragma once

#include "../Config.h"
#include "vector.h"

namespace FusionCrowd
{

	namespace Math
	{
		class FUSION_CROWD_API Line
		{
		public:
			Line();
			Line(const Vector2 & p, const Vector2 & d);
			~Line();
			Vector2 NearestPt(const Vector2 & p) const;

			Vector2 _point;
			Vector2 _direction;
		};
	}
}

