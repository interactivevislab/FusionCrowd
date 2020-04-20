#include "Obstacle.h"

#include "Math/geomQuery.h"
#include "Math/consts.h"
#include "Math/Util.h"

using namespace DirectX::SimpleMath;
using namespace FusionCrowd;

Obstacle::Obstacle()
	: _doubleSided(true), _isConvex(false), _nextObstacle(0x0), _point(), _prevObstacle(0x0), _unitDir(), _id(0), _class(0x1)
{
}

Obstacle::~Obstacle()
{
}

BoundingBox Obstacle::GetBB() const
{
	auto p0 = getP0();
	auto p1 = getP1();

	return BoundingBox(p0.x, p0.y, p0.x, p0.y).Union(BoundingBox(p1.x, p1.y, p1.x, p1.y));
}

Vector2 Obstacle::getP1() const
{
	return _point + _unitDir * _length;
}

Obstacle::NearTypeEnum Obstacle::distanceSqToPoint(const Vector2 & pt, Vector2 & nearPt,
	float & distSq) const
{
	Vector2 P1 = getP1();
	Vector2 ba(P1 - _point);
	Vector2 ca(pt - _point);
	float r = ca.Dot(ba) / ba.LengthSquared();

	if (r < 0) { // point a is closest to c
		nearPt = _point;
		distSq = ca.LengthSquared();
		return FIRST;
	}
	else if (r > 1) { // point b is closest to c
		nearPt = P1;
		distSq = (nearPt - pt).LengthSquared();
		return LAST;
	}
	else { // some point in between a and b is closest to c
		nearPt = _point + ba * r;
		distSq = (nearPt - pt).LengthSquared();
		return MIDDLE;
	}
}

float Obstacle::circleIntersection(const Vector2 & dir, const Vector2 & start, float radius) const {
	const float radSqd = radius * radius;
	const float SPEED = dir.Length();
	Vector2 forward(dir / SPEED);
	// Find the end points relative to the start position
	Vector2 a = getP0() - start;
	Vector2 b = getP1() - start;

	// rotate the segment so that the direction is aligned with the x-axis
	//	TODO: Where is this exploited???
	float x = a.x * forward.x + a.y * forward.y;
	float y = a.y * forward.x - a.x * forward.y;
	a = Vector2(x, y);
	x = b.x * forward.x + b.y * forward.y;
	y = b.y * forward.x - b.x * forward.y;
	b = Vector2(x, y);

	// compute the implicit equation of the obstacle line
	Vector2 disp = b - a;
	float dist = disp.Length();
	Vector2 D = disp / dist;
	Vector2 N(D.y, -D.x);
	float C = - N.Dot(a);		// Ax + By + C = 0 --> implicit equation
	// Test for collision
	if (C < 0.f) {
		// the agent lies on the "wrong" side of the obstacle and can't see it.
		return Math::INFTY;
	}
	else if (C < radius) {	// the circle overlaps the line on the visible side
		float t = D.Dot(-a);	// projection of origin on the line
		if (t >= -radius && t <= dist + radius) {
			// The projection of the circle center lies within the projection of
			//	the minkowski sum on the line (i.e. extends past the points by
			//	a distance equal to the radius).
			if ((t >= 0 && t <= dist) ||
				(t < 0 && a.LengthSquared() < radSqd) ||
				(t > dist && b.LengthSquared() < radSqd)) {
				return 0.f;
			}
		}
	}

	// Not currently colliding -- now compute potential collision in the future
	// M points to the side of the line on which the origin (aka agent) lies
	//	This creates the leading edge of the minkowski sum (defined by (a2, b2)).
	Vector2 M(C < 0.f ? -N : N);
	Vector2 a2(a + M * radius);
	Vector2 b2(b + M * radius);
	// I use this to do quick and dirty floating-point SIGN tests
	//	This may not be particularly portable
	union {
		float f;
		unsigned int u;
	} w1, w2;
	w1.f = a2.y;
	w2.f = b2.y;
	if ((w1.u ^ w2.u) & 0x80000000) {
		// signs of the y-values are different; the segment crosses the line
		float t = -a2.y / D.y;
		float x = a2.x + D.x * t;
		if (x > 0) {
			// The time it takes to travel distance x
			return x / SPEED;
		}
	}
	else {
		// both end points are on the same side of the line
		// Note: Both of these are possible if the obstacle is near parallel
		//	to the forward direction
		float minT = Math::INFTY;
		float aDist2 = a.y * a.y;
		if (aDist2 < radSqd) {
			// collision with a
			// dx < radius
			float dx = sqrtf(radSqd - aDist2);
			float x = a.x - dx;	// collision point candidate
			// This is a bit tricky - I don't have to consider a.x + dx
			//		1) the direction is in the positive x-axis direction, so I know
			//			the earliest collision must have a lesser x-value.
			//		2) It's POSSIBLE for x to have a negative value, but if that's
			//			true, then a.x + dx must ALSO be negative, otherwise
			//			the point is inside the circle and it would be detected
			//			as a collision.  So, it's enough to just test one value
			if (x > 0.f) {
				float t = x / (dist * D.x);
				if (t < minT) {
					minT = t;
				}
			}
		}
		float bDist2 = b.y * b.y;
		if (bDist2 < radSqd) {
			// collision with a
			// dx < radius
			float dx = sqrtf(radSqd - bDist2);
			float x = b.x - dx;	// collision point candidate
			if (x > 0.f) {
				float t = x / dir.x;
				if (t < minT) {
					minT = t;
				}
			}
		}
		return minT;
	}
	return Math::INFTY;
}

/////////////////////////////////////////////////////////////////////////////

bool Obstacle::pointOnObstacle(const Vector2 &pt) const {
	Vector2 disp = pt - _point;
	float t = disp.Dot(_unitDir);
	// The point projects onto the line beyond the extents of the segment
	if (t > _length || t < 0.f) return false;
	float dispSq = disp.LengthSquared();
	// the point doesn't lie on the line, because its displacement to the originating
	//	point is not the same as t^2.
	if (fabs(t * t - dispSq) > 0.001f) return false;
	return true;
}

float Obstacle::distSqPoint(const DirectX::SimpleMath::Vector2& pt) const
{
	const float r = (pt - _point).Dot(_unitDir) / _length;

	if (r < 0.0f)
	{
		return (pt - _point).LengthSquared();
	}
	else if (r > 1.0f)
	{
		return (pt - getP1()).LengthSquared();
	}
	else
	{
		return (pt - (_point + r * _length * _unitDir)).LengthSquared();
	}
}

