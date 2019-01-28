

#include "NavMeshEdge.h"
#include "NavMeshNode.h"

const float MIN_EDGE_WIDTH = 0.00001f;

NavMeshEdge::NavMeshEdge() : _point(0.f, 0.f), _dir(0.f, 0.f), _width(0.f), _distance(0.f),
_node0(0x0), _node1(0x0) {
}

NavMeshEdge::~NavMeshEdge()
{
}

NavMeshNode * NavMeshEdge::getOtherByID(unsigned int id) const
{
	if (_node0->_id == id) {
		return _node1;
	}
	else {
		return _node0;
	}
}

NavMeshNode * NavMeshEdge::getOtherByPtr(const NavMeshNode * node)
{
	if (_node0 == node) {
		return _node1;
	}
	else {
		return _node0;
	}
}

const NavMeshNode * NavMeshEdge::getOtherByPtr(const NavMeshNode * node) const
{
	if (_node0 == node) {
		return _node1;
	}
	else {
		return _node0;
	}
}

float NavMeshEdge::getSqDist(const FusionCrowd::Math::Vector2 & pt) const
{
	FusionCrowd::Math::Vector2 disp(pt - _point);
	float t = disp * _dir;
	if (t <= 0.f) {
		return absSq(disp);
	}
	else if (t >= _width) {
		return absSq(pt - (_point + _dir * _width));
	}
	else {
		float dist = det(_dir, pt - _point);
		return dist * dist;
	}
}

float NavMeshEdge::getSqDist(const FusionCrowd::Math::Vector2 & pt, FusionCrowd::Math::Vector2 & nearPt) const
{
	FusionCrowd::Math::Vector2 disp(pt - _point);
	float t = disp * _dir;
	if (t <= 0.f) {
		nearPt.set(_point);
		return absSq(disp);
	}
	else if (t >= _width) {
		nearPt.set(_point + _dir * _width);
		return absSq(pt - nearPt);
	}
	else {
		nearPt.set(_point + _dir * t);
		return absSq(pt - nearPt);
	}
}

float NavMeshEdge::getNodeDistance(float minWidth)
{
	float RR = minWidth;
	if (RR > _width) {
		return -1.f;
	}
	else {
		return _distance;
	}
}

bool NavMeshEdge::loadFromAscii(std::ifstream & f, FusionCrowd::Math::Vector2 * vertices) {
	size_t v0, v1, n0, n1;
	if (!(f >> v0 >> v1 >> n0 >> n1)) {
		return false;
	}
	else {
		_point.set(vertices[v0]);
		FusionCrowd::Math::Vector2 disp = vertices[v1] - vertices[v0];
		_width = abs(disp);
		if (_width <= MIN_EDGE_WIDTH) {
			return false;
		}
		_dir.set(disp / _width);
		// Stash indices as pointers
		_node0 = (NavMeshNode *)n0;
		_node1 = (NavMeshNode *)n1;
	}
	return true;
}

bool NavMeshEdge::pointClear(const FusionCrowd::Math::Vector2 & pos, float radius, float param) const {
	FusionCrowd::Math::Vector2 goal(_point + param * _dir);
	FusionCrowd::Math::Vector2 dir(norm(goal - pos));
	float dist = fabs(det(dir, _point - pos));
	if (dist < radius) return false;
	FusionCrowd::Math::Vector2 p1 = _point + _width * _dir;
	dist = fabs(det(dir, p1 - pos));
	if (dist < radius) return false;
	return true;
}

FusionCrowd::Math::Vector2 NavMeshEdge::targetPoint(const FusionCrowd::Math::Vector2 & pos, float radius) const {
	assert(_width > 2.f * radius &&
		"Agent's radius bigger than the portal width -- can't pass through");
	// Be smart about this
	//	If the position projects onto the portal, then simply find the closest point to the
	//		"effective" portal
	//  If the position lies to the left of p0
	//		We simply rotate the direction from end point to test position by the appropriate
	//			angle.  We do this as follows:
	//		d = || pos - p0 ||
	//		dHat = || pos - p0 || / d
	//		l = sqrtf( d^2 - radius^2 )
	//		cosTheta = r / d
	//		sinTheta = l / d
	//		gx = R * ( cosTheta * dHat.x + sinTheta * dHat.y ) + p0.x
	//		gy = R * ( cosTheta * dHat.y - sinTheta * dHat.x ) + p0.y
	//	If the position lies to the right of p0, it does the same computation, but with
	//		the opposite rotation

	// Does pos project onto the portal
	FusionCrowd::Math::Vector2 p1 = _point + _dir * _width;
	FusionCrowd::Math::Vector2 disp = pos - _point;
	float dp = disp * _dir;
	float mag = _width - radius;
	if (dp < radius || dp > mag) {
		if (dp > mag) {
			disp = pos - p1;
		}
		float d2 = absSq(disp);
		float R2 = radius * radius;
		if (R2 > d2) {
			// Currently overlapping the end point
			if (dp < radius) {
				return pos + _dir;
			}
			else {
				return pos - _dir;
			}
		}
		float d = sqrtf(d2);
		float l = sqrtf(d2 - R2);
		float cTheta = radius / d;
		float sTheta = l / d;
		float x, y;
		if (dp < radius) {
			x = cTheta * disp.x() + sTheta * disp.y();
			y = cTheta * disp.y() - sTheta * disp.x();
		}
		else {
			x = cTheta * disp.x() - sTheta * disp.y();
			y = cTheta * disp.y() + sTheta * disp.x();
		}
		FusionCrowd::Math::Vector2 goal(x, y);
		goal *= radius / d;
		if (dp < radius) {
			return goal + _point;
		}
		else {
			return goal + p1;
		}
	}
	else {
		return _point + _dir * dp;
	}
}

FusionCrowd::Math::Vector2	NavMeshEdge::getClearDirection(const FusionCrowd::Math::Vector2 & pos, float radius,
	const FusionCrowd::Math::Vector2 & dir) const {
	assert(_width > 2.f * radius &&
		"Agent's radius bigger than the portal width -- can't pass through");
	/*
		The algorithm seeks to find the direction, closest to the given direction (dir) but
		doesn't lead to collisions with the portal endpoints (and, therefore, the adjacent
		obstacles).

		It does so in the following manner.

					  _
		P0            /|                     P1
		o------------------------------------o
		 \          /
		  \        /
		   \      /
			\    / dir
			 \  /
			  \/
			  /
			 o
			pos

		ASSUMPTIONS:
			1) the provided direction intersects the portal between p0 and p1
			2) pos, P0 and P1 are not co-linear.
			3) The input direction is NOT normalized.

		First, it determines if moving the given direction will bring it closer to the end
			points than radius distance (shown w.r.t. P0).
			This is a two-part test.  Given the line passing through pos in the direction of
				dir, it determines the distance of each portal endpoint to this line.  If the
				distance > agent radius, then it's fine.
			HOWEVER, just because the distance is less than the radius does NOT mean it's
				necessarily bad.  For example, if the agent is moving AWAY from the point, then
				the point can approach the line from behind.
				- this is detected by determining if the projection lies on the opposite side
					of the point, pos, than the movement.
			If the distance > radius, or the projection is "behind" the agent, the path is
				clear and the preferred direction is returned.

		If it isn't clear, then a clear direction must be computed.  A direction is computed
		from pos to a point Q on a circle positioned at each end point (P0 and P1).  The
		tangent point is defined such that the line connecting Q and pos is tangent to the
		circle AND intersects the portal between P0 and P1 (generally, there are two lines
		passing through pos and tangent to the circle, but one must, by definition point
		outside of the portal).  It is computed by rotatining the vector TO the end point such
		that it becomes tangent. The direction to P0 is rotated to the right and the direction
		to P1 is rotated to the left.

		The two directions define a cone which spans the passable region of directions..  If
		the cone is well formed (i.e. the right limit is on the right side of the left limit)
		then a valid direction exists. In that case, the preferred direction must lie outside
		of the cone (if it lies inside, then the original test would have already passed).  So,
		if the preferred direction lies to the left of the cone, we return the left limit.  If
		it lies to the right of the cone, we return the right limit.

		If the cone is not well-formed, it is because the agent's position does not project
		onto the portal and it is coming at the portal at an oblique angle.  As such, there is
		no direction which will take it straight through the portal.  Instead, we'll take the
		direction that will have it clear the nearest end point.
	 */

	 // Test to see if goal direction is valid
	const float dirMag2 = absSq(dir);
	const float threshold = radius * radius * dirMag2;
	FusionCrowd::Math::Vector2 p0Delta = _point - pos;
	float dist = det(dir, p0Delta);
	if (dist * dist >= threshold || p0Delta * dir < 0.f) {
		FusionCrowd::Math::Vector2 p1Delta = _point + _width * _dir - pos;
		dist = det(dir, p1Delta);
		if (dist * dist >= threshold || p1Delta * dir < 0.f) {
			return norm(dir);
		}
	}

	// Directions towards portal's PHYSICAL endpoints
	FusionCrowd::Math::Vector2 d0 = _point - pos;
	FusionCrowd::Math::Vector2 p1 = _point + _dir * _width;
	FusionCrowd::Math::Vector2 d1 = p1 - pos;
	FusionCrowd::Math::Vector2 portalDir(_dir);
	if (det(d1, d0) < 0.f) {
		// make sure that d0 is on the left and d1 is on the right
		FusionCrowd::Math::Vector2 tmp(d0);
		d0.set(d1);
		d1.set(tmp);
		portalDir.negate();
	}

	// compute left extent of passable cone
	float d = abs(d0);
	FusionCrowd::Math::Vector2 dHat0 = d0 / d;
	if (d <= radius) {
		// assume I can only overlap one edge at a time.
		FusionCrowd::Math::Vector2 N(dHat0.y(), -dHat0.x());
		if (N * dir >= 0.f) {
			return N;
		}
		else {
			return -N;
		}
	}
	float l = sqrtf(d * d - radius * radius);
	float sinTheta0 = radius / d;
	float cosTheta0 = l / d;
	float dx = cosTheta0 * dHat0.x() + sinTheta0 * dHat0.y();
	float dy = cosTheta0 * dHat0.y() - sinTheta0 * dHat0.x();
	FusionCrowd::Math::Vector2 leftLimit(dx, dy);

	// Compute right extent of passable cone
	d = abs(d1);
	FusionCrowd::Math::Vector2 dHat1(d1 / d);
	if (d <= radius) {
		FusionCrowd::Math::Vector2 N(dHat1.y(), -dHat1.x());
		if (N * dir >= 0.f) {
			return N;
		}
		else {
			return -N;
		}
	}
	l = sqrtf(d * d - radius * radius);
	float cosTheta1 = l / d;
	float sinTheta1 = radius / d;
	dx = cosTheta1 * dHat1.x() - sinTheta1 * dHat1.y();
	dy = cosTheta1 * dHat1.y() + sinTheta1 * dHat1.x();

	FusionCrowd::Math::Vector2 rightLimit(dx, dy);
	if (det(rightLimit, leftLimit) < 0) {
		// The cone spans no valid directions.
		// No direct path exists - simply clear the nearer endpoint
		if (leftLimit * portalDir >= 0.f) {
			// the left end point is the near goal
			return leftLimit;
		}
		else {
			// the right end point is the near goal
		}
		return FusionCrowd::Math::Vector2(dx, dy);
	}
	else {
		if (det(leftLimit, dir) > 0) {
			// The preferred direction lies left of the left extent the cone
			return leftLimit;
		}
		else {
			// The preferred direction lies right of the right extent the cone
			return rightLimit;
		}
	}
}

//void NavMeshEdge::setClearDirections(const FusionCrowd::Math::Vector2 & pos, float radius, const FusionCrowd::Math::Vector2 & dir,
//	Agents::PrefVelocity & pVel) const {
//	/* See getClearDirections for how this works. */
//
//	// Directions towards portal's PHYSICAL endpoints
//	FusionCrowd::Math::Vector2 d0 = _point - pos;
//	FusionCrowd::Math::Vector2 p1 = _point + _dir * _width;
//	FusionCrowd::Math::Vector2 d1 = p1 - pos;
//	FusionCrowd::Math::Vector2 portalDir(_dir);
//	if (det(d1, d0) < 0.f) {
//		// make sure that d0 is on the left and d1 is on the right
//		FusionCrowd::Math::Vector2 tmp(d0);
//		d0.set(d1);
//		d1.set(tmp);
//		portalDir.negate();
//	}
//
//	const float R2 = radius * radius;
//	// compute left extent of passable cone
//	float d2 = absSq(d0);
//	if (d2 < R2) {
//		// Already colliding with the left portal edge
//		//	If the portal is wide enough to allow the agent through, I can't collide with
//		//	both.  So, this is the only possible collision.
//		// Simply move parallel with the portal (into the opening) to get out of collision
//		pVel.setSingle(portalDir);
//		return;
//	}
//	// This represents refactored algebra.
//	float l = sqrtf(d2 - R2);
//	float sinTheta0 = radius;
//	float cosTheta0 = l;
//	float dx = cosTheta0 * d0._x + sinTheta0 * d0._y;
//	float dy = cosTheta0 * d0._y - sinTheta0 * d0._x;
//	FusionCrowd::Math::Vector2 leftLimit(dx / d2, dy / d2);
//
//	// Compute right extent of passable cone
//	d2 = absSq(d1);
//	if (d2 <= R2) {
//		pVel.setSingle(-portalDir);
//		return;
//	}
//	l = sqrtf(d2 - R2);
//	float cosTheta1 = l;
//	float sinTheta1 = radius;
//	dx = cosTheta1 * d1._x - sinTheta1 * d1._y;
//	dy = cosTheta1 * d1._y + sinTheta1 * d1._x;
//	FusionCrowd::Math::Vector2 rightLimit(dx / d2, dy / d2);
//
//	if (det(rightLimit, leftLimit) < 0) {
//		// The cone spans no valid directions.
//		// No direct path exists - simply clear the nearer endpoint
//		if (leftLimit * portalDir >= 0.f) {
//			// the left end point is the near goal
//			pVel.setSingle(leftLimit);
//		}
//		else {
//			// the right end point is the near goal
//			pVel.setSingle(rightLimit);
//		}
//	}
//	else {
//		FusionCrowd::Math::Vector2 prefDir;
//
//		if (det(leftLimit, dir) > 0) {
//			// The preferred direction lies left of the left extent the cone
//			prefDir.set(leftLimit);
//		}
//		else if (det(dir, rightLimit) > 0) {
//			// The preferred direction lies right of the right extent the cone
//			prefDir.set(rightLimit);
//		}
//		else {
//			prefDir.set(norm(dir));
//		}
//		pVel.setSpan(leftLimit, rightLimit, prefDir);
//	}
//}

bool NavMeshEdge::pointOnLeft(unsigned int id) const
{
	return id == _node0->getID();
}

//////////////////////////////////////////////////////////////////////////////////////

bool NavMeshEdge::pointOnLeft(const NavMeshNode * node) const
{
	return node == _node0;
}