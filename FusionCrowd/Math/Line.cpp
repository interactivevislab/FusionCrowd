#include "Line.h"
#include "../MathUtil.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{

	namespace Math
	{
		Line::Line() : _point(), _direction()
		{
		}

		Line::Line(const Vector2 & p, const Vector2 & d) : _point(p), _direction(d)
		{
		}

		Vector2 Line::NearestPt(const Vector2 & p) const {
			float t = _direction.Dot(p - _point);
			return _point + t * _direction;
		}

		Line::~Line()
		{
		}
	}
}
