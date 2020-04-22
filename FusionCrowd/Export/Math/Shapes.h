#pragma once

#include "Export/Config.h"

namespace FusionCrowd
{
	struct Disk
	{
		float x, y;
		float r;
	};

	struct Point
	{
		float x, y;
	};

	struct Rect
	{
		float xl, yb;
		float xr, yt;

		Rect(float x1, float y1, float x2, float y2)
			: xl((x1 < x2)? x1 : x2), xr((x1 < x2) ? x2 : x1),
			yb((y1 < y2)? y1 : y2), yt((y1 < y2) ? y2 : y1)
		{ }
	};
}
