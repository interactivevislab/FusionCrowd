#include "RectShape.h"

#include "Math/consts.h"
#include "TacticComponent/PrefVelocity.h"

#include <algorithm>

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	namespace Math
	{
		RectShape::RectShape(const Rect& rect)
			: _topLeft(rect.xl, rect.yt), _bottomRight(rect.xr, rect.yb)
		{ }

		bool RectShape::InsideXStripe(const DirectX::SimpleMath::Vector2 pt) const
		{
			return _topLeft.x <= pt.x && pt.x <= _bottomRight.x;
		}

		bool RectShape::InsideYStripe(const DirectX::SimpleMath::Vector2 pt) const
		{
			return _topLeft.y >= pt.y && pt.y >= _bottomRight.y;
		}

		bool RectShape::containsPoint(const DirectX::SimpleMath::Vector2& pt) const
		{
			return InsideXStripe(pt) && InsideYStripe(pt);
		}

		float RectShape::squaredDistance(const DirectX::SimpleMath::Vector2& pt) const
		{
			if(containsPoint(pt))
				return 0;

			if(InsideXStripe(pt) && !InsideYStripe(pt))
			{
				float d = (pt.y < _bottomRight.y) ? _bottomRight.y - pt.y : pt.y - _topLeft.y;
				return d*d;
			}

			if(!InsideXStripe(pt) && InsideYStripe(pt))
			{
				float d = (pt.x < _topLeft.x) ? _topLeft.x - pt.x : pt.x - _bottomRight.x;
				return d*d;
			}

			// Some of the corners
			float d1 = Vector2::Distance(pt, _topLeft);
			float d2 = Vector2::Distance(pt, _bottomRight);

			float d3 = Vector2::Distance(pt, Vector2(_bottomRight.x, _topLeft.y)); // topRight
			float d4 = Vector2::Distance(pt, Vector2(_topLeft.x, _bottomRight.y)); // bottomLeft

			float d = std::min<float>({d1, d2, d3, d4});
			return d*d;
		}

		void RectShape::setDirections(const DirectX::SimpleMath::Vector2& q, float r,
			Agents::PrefVelocity& directions) const
		{
			Vector2 target = getTargetPoint(q, r);
			Vector2 disp = target - q;
			const float distSq = disp.LengthSquared();
			Vector2 dir;
			if (distSq > 1e-8) {
				// Distant enough that I can normalize the direction.
				dir = disp / sqrtf(distSq);
			}
			else {
				dir = Vector2(0.f, 0.f);
			}
			directions.setSingle(dir);
			directions.setTarget(target);
		}

		DirectX::SimpleMath::Vector2 RectShape::getTargetPoint(const DirectX::SimpleMath::Vector2& pt, float r) const
		{
			if(squaredDistance(pt) <= r * r)
				return pt;

			if(InsideXStripe(pt) && !InsideYStripe(pt))
			{
				float y = (pt.y < _bottomRight.y) ? _bottomRight.y - r : _topLeft.y + r;
				return Vector2(pt.x, y);
			}

			if(!InsideXStripe(pt) && InsideYStripe(pt))
			{
				float x = (pt.x < _topLeft.x) ? _topLeft.x - r : _bottomRight.x + r;
				return Vector2(x, pt.y);
			}

			Vector2 corner[] = {
				_bottomRight,
				Vector2(_bottomRight.x, _topLeft.y),
				Vector2(_topLeft.x, _bottomRight.y)
			};

			float minD = Vector2::Distance(pt, _topLeft);
			Vector2 minC = _topLeft;
			for(Vector2 & c : corner)
			{
				float d = Vector2::Distance(pt, c);

				if(d < minD)
				{
					minD = d;
					minC = c;
				}
			}

			Vector2 dir; (pt - minC).Normalize(dir);

			return minC + dir * r;
		}

		DirectX::SimpleMath::Vector2 RectShape::getCentroid() const
		{
			return (_topLeft + _bottomRight) * 0.5f;
		}

		float RectShape::BoundingRadius() const
		{
			return Vector2::Distance(_topLeft, _bottomRight) * 0.5f;
		}

		RectShape* RectShape::Clone() const
		{
			return new RectShape(*this);
		}
	}
}
