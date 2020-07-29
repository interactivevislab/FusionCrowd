#pragma once

#include "Math/Util.h"

namespace FusionCrowd
{
	// FORWARD DECLARATION
	class PortalPath;

	class PathRandomization
	{
	public:
		PathRandomization();
		~PathRandomization();
		void RandomizePath(FusionCrowd::PortalPath* path);
		void RandomizePath(FusionCrowd::PortalPath* path, size_t start, size_t end);
		void RandomizePath(FusionCrowd::PortalPath* path, float customEdgePosition, float customDistribution, float agentRadius);
		void RandomizePath(FusionCrowd::PortalPath* path, size_t start, size_t end, float customEdgePosition, float customDistribution, float agentRadius);
	};
}
