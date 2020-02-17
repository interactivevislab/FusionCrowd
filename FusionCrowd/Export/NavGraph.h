#pragma once

#include "Export/Config.h"

namespace FusionCrowd
{
	namespace Export
	{
		struct NavGraphNode
		{
			size_t id;
			float x, y;
		};

		struct NavGraphEdge
		{
			size_t nodeFrom;
			size_t nodeTo;
			float weight;
			float width;
		};
	}
}