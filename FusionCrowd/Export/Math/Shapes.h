#pragma once

#include "Export/Config.h"

namespace FusionCrowd
{
	struct Shape { };

	struct Disk : public Shape
	{
		float x, y;
		float r;
		Disk(float x, float y, float r)
			: x(x), y(y), r(r)
		{ }
	};

	struct Point : public Shape
	{
		float x, y;
		Point()
			: x(0), y(0)
		{ }
		Point(float x, float y)
			: x(x), y(y)
		{ }
	};

	struct Rect : public Shape
	{
		float xl, yb;
		float xr, yt;

		Rect(float x1, float y1, float x2, float y2)
			: xl((x1 < x2)? x1 : x2), xr((x1 < x2) ? x2 : x1),
			yb((y1 < y2)? y1 : y2), yt((y1 < y2) ? y2 : y1)
		{ }
	};

	struct Cone : public Shape
	{
		float x, y;
		float range;
		float angleRad;

		Cone(float x, float y, float range, float angleRad)
			: x(x), y(y), range(range), angleRad(angleRad)
		{ }
	};
}
