#pragma once

namespace FusionCrowd
{
	struct BoundingBox
	{
		float xmin;
		float ymin;
		float xmax;
		float ymax;

		BoundingBox() : xmin(0), ymin(0), xmax(0), ymax(0)
		{
		}

		BoundingBox(float xmin, float ymin, float xmax, float ymax)
			: xmin(xmin), ymin(ymin), xmax(xmax), ymax(ymax)
		{
		}

		inline bool Contains(float x, float y) const
		{
			return x >= xmin && x <= xmax && y >= ymin && y <= ymax;
		}

		inline bool Overlaps(const BoundingBox & bb) const
		{
			return bb.xmin <= xmax && bb.xmax >= xmin && bb.ymin <= ymax && bb.ymax >= ymin;
		}

		inline BoundingBox Union(const BoundingBox & bb)
		{
			return BoundingBox {
				xmin < bb.xmin ? xmin : bb.xmin,
				ymin < bb.ymin ? ymin : bb.ymin,
				xmax > bb.xmax ? xmax : bb.xmax,
				ymax > bb.ymax ? ymax : bb.ymax
			};
		}
	};
}
