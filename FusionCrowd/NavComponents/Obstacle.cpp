#include "Obstacle.h"

#include "../Math/consts.h"
#include "../Math/geomQuery.h"

Obstacle::Obstacle() : _doubleSided(false), _isConvex(false), _nextObstacle(0x0),
_point(), _prevObstacle(0x0), _unitDir(), _id(0),
_class(0x1)
{
}

Obstacle::~Obstacle()
{
}

FusionCrowd::Math::Vector2 Obstacle::getP1() const
{
	if (_nextObstacle != 0x0) {
		return _nextObstacle->_point;
	}
	else {
		return _point + _unitDir * _length;
	}
}

Obstacle::NearTypeEnum Obstacle::distanceSqToPoint(const FusionCrowd::Math::Vector2 & pt, FusionCrowd::Math::Vector2 & nearPt,
	float & distSq) const
{
	FusionCrowd::Math::Vector2 P1 = getP1();
	FusionCrowd::Math::Vector2 ba(P1 - _point);
	FusionCrowd::Math::Vector2 ca(pt - _point);
	float r = (ca * ba) / absSq(ba);

	if (r < 0) { // point a is closest to c
		nearPt.set(_point);
		distSq = absSq(ca);
		return FIRST;
	}
	else if (r > 1) { // point b is closest to c
		nearPt.set(P1);
		distSq = absSq(nearPt - pt);
		return LAST;
	}
	else { // some point in between a and b is closest to c
		nearPt.set(_point + ba * r);
		distSq = absSq(nearPt - pt);
		return MIDDLE;
	}
}
float Obstacle::circleIntersection(const FusionCrowd::Math::Vector2 & dir, const FusionCrowd::Math::Vector2 & start,
	float radius) const {
	const float radSqd = radius * radius;
	const float SPEED = abs(dir);
	FusionCrowd::Math::Vector2 forward(dir / SPEED);
	// Find the end points relative to the start position
	FusionCrowd::Math::Vector2 a = getP0() - start;
	FusionCrowd::Math::Vector2 b = getP1() - start;

	// rotate the segment so that the direction is aligned with the x-axis
	//	TODO: Where is this exploited???
	float x = a.x() * forward.x() + a.y() * forward.y();
	float y = a.y() * forward.x() - a.x() * forward.y();
	a.set(x, y);
	x = b.x() * forward.x() + b.y() * forward.y();
	y = b.y() * forward.x() - b.x() * forward.y();
	b.set(x, y);

	// compute the implicit equation of the obstacle line
	FusionCrowd::Math::Vector2 disp = b - a;
	float dist = abs(disp);
	FusionCrowd::Math::Vector2 D = disp / dist;
	FusionCrowd::Math::Vector2 N(D.y(), -D.x());
	float C = -(N * a);		// Ax + By + C = 0 --> implicit equation
	// Test for collision
	if (C < 0.f) {
		// the agent lies on the "wrong" side of the obstacle and can't see it.
		return FusionCrowd::INFTY;
	}
	else if (C < radius) {	// the circle overlaps the line on the visible side
		float t = D * (-a);	// projection of origin on the line
		if (t >= -radius && t <= dist + radius) {
			// The projection of the circle center lies within the projection of
			//	the minkowski sum on the line (i.e. extends past the points by
			//	a distance equal to the radius).
			if ((t >= 0 && t <= dist) ||
				(t < 0 && absSq(a) < radSqd) ||
				(t > dist && absSq(b) < radSqd)) {
				return 0.f;
			}
		}
	}

	// Not currently colliding -- now compute potential collision in the future
	// M points to the side of the line on which the origin (aka agent) lies
	//	This creates the leading edge of the minkowski sum (defined by (a2, b2)).
	FusionCrowd::Math::Vector2 M(C < 0.f ? -N : N);
	FusionCrowd::Math::Vector2 a2(a + M * radius);
	FusionCrowd::Math::Vector2 b2(b + M * radius);
	// I use this to do quick and dirty floating-point SIGN tests
	//	This may not be particularly portable
	union {
		float f;
		unsigned int u;
	} w1, w2;
	w1.f = a2.y();
	w2.f = b2.y();
	if ((w1.u ^ w2.u) & 0x80000000) {
		// signs of the y-values are different; the segment crosses the line
		float t = -a2.y() / D.y();
		float x = a2.x() + D.x() * t;
		if (x > 0) {
			// The time it takes to travel distance x
			return x / SPEED;
		}
	}
	else {
		// both end points are on the same side of the line
		// Note: Both of these are possible if the obstacle is near parallel
		//	to the forward direction
		float minT = FusionCrowd::INFTY;
		float aDist2 = a.y() * a.y();
		if (aDist2 < radSqd) {
			// collision with a
			// dx < radius
			float dx = sqrtf(radSqd - aDist2);
			float x = a.x() - dx;	// collision point candidate
			// This is a bit tricky - I don't have to consider a.x() + dx
			//		1) the direction is in the positive x-axis direction, so I know
			//			the earliest collision must have a lesser x-value.
			//		2) It's POSSIBLE for x to have a negative value, but if that's
			//			true, then a.x() + dx must ALSO be negative, otherwise
			//			the point is inside the circle and it would be detected
			//			as a collision.  So, it's enough to just test one value
			if (x > 0.f) {
				float t = x / (dist * D.x());
				if (t < minT) {
					minT = t;
				}
			}
		}
		float bDist2 = b.y() * b.y();
		if (bDist2 < radSqd) {
			// collision with a
			// dx < radius
			float dx = sqrtf(radSqd - bDist2);
			float x = b.x() - dx;	// collision point candidate
			if (x > 0.f) {
				float t = x / dir.x();
				if (t < minT) {
					minT = t;
				}
			}
		}
		return minT;
	}
	return FusionCrowd::INFTY;
}

/////////////////////////////////////////////////////////////////////////////

bool Obstacle::pointOnObstacle(const FusionCrowd::Math::Vector2 &pt) const {
	FusionCrowd::Math::Vector2 disp = pt - _point;
	float t = disp * _unitDir;
	// The point projects onto the line beyond the extents of the segment
	if (t > _length || t < 0.f) return false;
	float dispSq = absSq(disp);
	// the point doesn't lie on the line, because its displacement to the originating
	//	point is not the same as t^2.
	if (fabs(t * t - dispSq) > 0.001f) return false;
	return true;
}

