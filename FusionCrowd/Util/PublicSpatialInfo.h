#pragma once

#include "Config.h"

namespace FusionCrowd
{
	struct FUSION_CROWD_API PublicSpatialInfo
	{
		size_t id;
		float posX, posY;
		float velX, velY;
		float orientX, orientY;
		float radius;
	};
}
