#pragma once

#include "Export/Config.h"

namespace FusionCrowd
{
	namespace Export
	{
		struct FUSION_CROWD_API NavGraphNode
		{
			size_t id;
			float x, y;
		};

		struct FUSION_CROWD_API NavGraphEdge
		{
			size_t nodeFrom;
			size_t nodeTo;
			float weight;
			float width;
		};
	}
}