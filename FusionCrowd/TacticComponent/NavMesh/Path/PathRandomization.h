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
	};
}
