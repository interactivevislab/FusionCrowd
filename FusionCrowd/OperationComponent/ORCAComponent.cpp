#include "ORCAComponent.h"

#include "Math/consts.h"

#include <algorithm>
#include <cassert>
#include <limits>
#include <vector>
#include <set>
#include <thread>

using namespace  DirectX::SimpleMath;

namespace FusionCrowd
{
namespace ORCA
{
	ORCAComponent::ORCAComponent(std::shared_ptr<NavSystem> navSystem) : ORCAComponent(navSystem, 2.5f, 0.15f)
	{}

	ORCAComponent::ORCAComponent(std::shared_ptr<NavSystem> navSystem, float timeHorizon, float timeHorizonObst)
		: _navSystem(navSystem), _timeHorizon(timeHorizon), _timeHorizonObst(timeHorizonObst), _pool(1) //std::thread::hardware_concurrency() - 1)
	{}

	void ORCAComponent::AddAgent(size_t id)
	{
		_agents.insert(id);
		_navSystem->GetSpatialInfo(id).inertiaEnabled = false;
		_navSystem->GetSpatialInfo(id).maxAngVel = 60.0f;
	}

	bool ORCAComponent::DeleteAgent(size_t id)
	{
		_agents.erase(id);

		return true;
	}

	struct Task
	{
		AgentSpatialInfo & agent;
		std::future<Vector2> newSpeed;
	};

	void ORCAComponent::Update(float timeStep)
	{
		std::vector<Task> tasks;
		for(size_t agentId : _agents)
		{
			auto & info = _navSystem->GetSpatialInfo(agentId);

			EnvironmentArgs args {
				_timeHorizon,
				_timeHorizonObst,
				timeStep,
				info,
				_navSystem->GetClosestObstacles(agentId),
				_navSystem->GetNeighbours(agentId)
			};

			tasks.push_back({info, _pool.push([=](int threadId) { return ComputeNewVelocity(args); })});
		}

		for(auto & t : tasks)
		{
			t.agent.velNew = t.newSpeed.get();
		}
	}

	Vector2 ORCAComponent::ComputeNewVelocity(EnvironmentArgs args)
	{
		std::vector<Math::Line> orcaLines;
		Vector2 result;
 		const size_t numObstLines = ComputeORCALines(orcaLines, args);

		Vector2 velPref(args.info.prefVelocity.getPreferredVel());

		size_t lineFail = LinearProgram2(orcaLines, args.info.maxSpeed, velPref, false, result);

		if (lineFail < orcaLines.size()) {
			LinearProgram3(orcaLines, numObstLines, lineFail, args.info.maxSpeed, result);
		}

		return result;
	}

	size_t ORCAComponent::ComputeORCALines(std::vector<Math::Line>& _orcaLines, EnvironmentArgs & args)
	{
		const float invTimeHorizonObst = 1.0f / args.timeHorizonObst;

		/* Create obstacle ORCA lines. */

		for (Obstacle & obst : args.obstacles) {
			const Vector2 P0 = obst.getP0();
			const Vector2 P1 = obst.getP1();
			const bool agtOnRight = FusionCrowd::Math::leftOf(P0, P1, args.info.GetPos()) < 0.f;
			ObstacleLine(_orcaLines, obst, invTimeHorizonObst, !agtOnRight && obst._doubleSided, args.info);
		}

		const size_t numObstLines = _orcaLines.size();

		const float invTimeHorizon = 1.0f / args.timeHorizon;

		/* Create agent ORCA lines. */
		for (auto & other : args.neighbors)
		{
			const Vector2 relativePosition = other.pos - args.info.GetPos();
			const Vector2 relativeVelocity = args.info.GetVel() - other.vel;

			const float distSq = relativePosition.LengthSquared();
			const float combinedRadius = args.info.radius + other.radius;
			const float combinedRadiusSq = combinedRadius * combinedRadius;

			FusionCrowd::Math::Line line;
			Vector2 u;

			if (distSq > combinedRadiusSq) {
				// No collision.
				const Vector2 w = relativeVelocity - invTimeHorizon * relativePosition;
				// Vector from cutoff center to relative velocity.
				const float wLengthSq = w.LengthSquared();

				const float dotProduct1 = w.Dot(relativePosition);

				if (dotProduct1 < 0.0f && dotProduct1 * dotProduct1 > combinedRadiusSq * wLengthSq) {
					// Project on cut-off circle.
					const float wLength = std::sqrt(wLengthSq);
					const Vector2 unitW = w / wLength;

					line._direction = Vector2(unitW.y, -unitW.x);
					u = (combinedRadius * invTimeHorizon - wLength) * unitW;
				}
				else {
					// Project on legs.
					const float leg = std::sqrt(distSq - combinedRadiusSq);

					if (Math::det(relativePosition, w) > 0.0f) {
						// Project on left leg.
						line._direction = Vector2(
							relativePosition.x * leg - relativePosition.y * combinedRadius,
							relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
					}
					else {
						// Project on right leg.
						line._direction = -Vector2(
							relativePosition.x * leg + relativePosition.y * combinedRadius,
							-relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
					}

					const float dotProduct2 = relativeVelocity.Dot(line._direction);

					u = dotProduct2 * line._direction - relativeVelocity;
				}

				line._point = args.info.GetVel() + 0.5f * u;
			}
			else {
				// Collision. Project on cut-off circle of time timeStep.
				const float invTimeStep = 1.0f / args.timeStep;

				// Vector from cutoff center to relative velocity.
				const Vector2 w = relativeVelocity - invTimeStep * relativePosition;

				const float wLength = w.Length();
				const Vector2 unitW = w / wLength;

				line._direction = Vector2(unitW.y, -unitW.x);
				u = (combinedRadius * invTimeStep - wLength) * unitW;
				float coopWeight = 0.5f;
				line._point = args.info.GetVel() + coopWeight * u;
			}

			_orcaLines.push_back(line);
		}

		return numObstLines;
	}

	void ORCAComponent::ObstacleLine(std::vector<FusionCrowd::Math::Line>& _orcaLines, Obstacle & obst, const float invTau, bool flip, const AgentSpatialInfo& agentInfo)
	{
		const float LENGTH = obst.length();
		const Vector2 P0 = flip ? obst.getP1() : obst.getP0();
		const Vector2 P1 = flip ? obst.getP0() : obst.getP1();
		const Vector2 obstDir = flip ? -obst._unitDir : obst._unitDir;
		const bool p0Convex = flip ? obst.p1Convex(true) : obst.p0Convex(true);
		const bool p1Convex = flip ? obst.p0Convex(true) : obst.p1Convex(true);

		const Obstacle* const leftNeighbor  = flip ? obst._nextObstacle : obst._prevObstacle;
		const Obstacle* const rightNeighbor = flip ? obst._prevObstacle : obst._nextObstacle;

		const Vector2 relativePosition1 = P0 - agentInfo.GetPos();
		const Vector2 relativePosition2 = P1 - agentInfo.GetPos();

		bool alreadyCovered = false;

		for (size_t j = 0; j < _orcaLines.size(); ++j)
		{
			auto & line = _orcaLines[j];
			auto d1 = Math::det(invTau * relativePosition1 - line._point, line._direction) - invTau * agentInfo.radius;
			auto d2 = Math::det(invTau * relativePosition2 - line._point, line._direction) - invTau * agentInfo.radius;
			if (d1 >= -Math::EPS && d2 >= -Math::EPS)
			{
				alreadyCovered = true;
				break;
			}
		}

		if (alreadyCovered) {
			return;
		}

		const float distSq1 = relativePosition1.LengthSquared();
		const float distSq2 = relativePosition2.LengthSquared();

		const float radiusSq = agentInfo.radius * agentInfo.radius;

		const float s = -(relativePosition1.Dot(obstDir));
		const float distSqLine = (relativePosition1 + s * obstDir).LengthSquared();

		FusionCrowd::Math::Line line;

		if (s < 0 && distSq1 <= radiusSq) {
			/* Collision with left vertex. Ignore if non-convex. */
			if (p0Convex) {
				line._point = Vector2(0.f, 0.f);
				Vector2(-relativePosition1.y, relativePosition1.x).Normalize(line._direction);
				_orcaLines.push_back(line);
			}
			return;
		}
		else if (s > LENGTH && distSq2 <= radiusSq) {
			/* Collision with right vertex. Ignore if non-convex
			* or if it will be taken care of by neighoring obstace */
			if ((obst._nextObstacle == 0x0) || (p1Convex && Math::det(relativePosition2, obst._nextObstacle->_unitDir) >= 0))
			{
				line._point = Vector2(0.f, 0.f);
				Vector2(-relativePosition2.y, relativePosition2.x).Normalize(line._direction);

				_orcaLines.push_back(line);
			}
			return;
		}
		else if (s >= 0 && s <= LENGTH && distSqLine <= radiusSq) {
			/* Collision with obstacle segment. */
			line._point = Vector2(0.f, 0.f);
			line._direction = -obstDir;
			_orcaLines.push_back(line);
			return;
		}

		Vector2 leftLegDirection, rightLegDirection;

		bool prevIsCurrent = false;
		bool nextIsCurrent = false;
		if (s < 0 && distSqLine <= radiusSq) {
			 // Obstacle viewed obliquely so that left vertex defines velocity obstacle.
			if (!p0Convex) {
				// Ignore obstacle.
				return;
			}

			nextIsCurrent = true;

			const float leg1 = std::sqrt(distSq1 - radiusSq);
			leftLegDirection = Vector2(relativePosition1.x * leg1 - relativePosition1.y * agentInfo.radius,
									   relativePosition1.x * agentInfo.radius + relativePosition1.y * leg1) / distSq1;
			rightLegDirection = Vector2(relativePosition1.x * leg1 + relativePosition1.y * agentInfo.radius,
									   -relativePosition1.x * agentInfo.radius + relativePosition1.y * leg1) / distSq1;
		}
		else if (s > LENGTH && distSqLine <= radiusSq) {
			// Obstacle viewed obliquely so that right vertex defines velocity obstacle.
			if (!p1Convex) {
				// Ignore obstacle.
				return;
			}

			prevIsCurrent = true;

			const float leg2 = std::sqrt(distSq2 - radiusSq);
			leftLegDirection = Vector2(relativePosition2.x * leg2 - relativePosition2.y * agentInfo.radius,
									   relativePosition2.x * agentInfo.radius + relativePosition2.y * leg2) / distSq2;
			rightLegDirection = Vector2(relativePosition2.x * leg2 + relativePosition2.y * agentInfo.radius,
									   -relativePosition2.x * agentInfo.radius + relativePosition2.y * leg2) / distSq2;
		}
		else
		{
			// Usual situation.
			if (p0Convex) {
				const float leg1 = std::sqrt(distSq1 - radiusSq);
				leftLegDirection = Vector2(relativePosition1.x * leg1 - relativePosition1.y * agentInfo.radius,
					    				   relativePosition1.x * agentInfo.radius + relativePosition1.y * leg1) / distSq1;
			}
			else {
				// Left vertex non-convex; left leg extends cut-off line.
				leftLegDirection = -obstDir;
			}

			if (p1Convex) {
				const float leg2 = std::sqrt(distSq2 - radiusSq);
				rightLegDirection = Vector2(relativePosition2.x * leg2 + relativePosition2.y * agentInfo.radius,
					                       -relativePosition2.x * agentInfo.radius + relativePosition2.y * leg2) / distSq2;
			}
			else {
				// Right vertex non-convex; right leg extends cut-off line.
				rightLegDirection = obstDir;
			}
		}

		/*
		* Legs can never point into neighboring edge when convex vertex,
		* take cutoff-line of neighboring edge instead. If velocity projected on
		* "foreign" leg, no constraint is added.
		*/
		bool isLeftLegForeign = false;
		bool isRightLegForeign = false;

		if (!prevIsCurrent) {
			if (leftNeighbor != 0x0) {
				if (p0Convex && Math::det(leftLegDirection, -leftNeighbor->_unitDir) >= 0.0f) {
					/* Left leg points into obstacle. */
					leftLegDirection = -leftNeighbor->_unitDir;
					isLeftLegForeign = true;
				}
			}
		}

		if (!nextIsCurrent) {
			if (rightNeighbor != 0x0) {
				if (p1Convex && Math::det(rightLegDirection, rightNeighbor->_unitDir) <= 0.0f) {
					/* Right leg points into obstacle. */
					rightLegDirection = rightNeighbor->_unitDir;
					isRightLegForeign = true;
				}
			}
		}

		/* Compute cut-off centers. */
		const Vector2 leftCutoff = invTau * (prevIsCurrent ? relativePosition2 : relativePosition1);
		const Vector2 rightCutoff = nextIsCurrent ? leftCutoff : (invTau * relativePosition2);
		const Vector2 cutoffVec = rightCutoff - leftCutoff;
		const bool obstaclesSame = nextIsCurrent || prevIsCurrent;


		/* Project current velocity on velocity obstacle. */
		/* Check if current velocity is projected on cutoff circles. */
		const float t = obstaclesSame ? 0.5f : (agentInfo.GetVel() - leftCutoff).Dot(cutoffVec / cutoffVec.LengthSquared());
		const float tLeft = (agentInfo.GetVel() - leftCutoff).Dot(leftLegDirection);
		const float tRight = (agentInfo.GetVel() - rightCutoff).Dot(rightLegDirection);

		if ((t < 0.0f && tLeft < 0.0f) || (obstaclesSame && tLeft < 0.0f && tRight < 0.0f)) {
			/* Project on left cut-off circle. */
			Vector2 unitW;
			(agentInfo.GetVel() - leftCutoff).Normalize(unitW);
			line._direction = Vector2(unitW.y, -unitW.x);
			line._point = leftCutoff + agentInfo.radius * invTau * unitW;
			_orcaLines.push_back(line);
			return;
		}
		else if (t > 1.0f && tRight < 0.0f) {
			/* Project on right cut-off circle. */
			Vector2 unitW;
			(agentInfo.GetVel() - rightCutoff).Normalize(unitW);
			line._direction = Vector2(unitW.y, -unitW.x);
			line._point = rightCutoff + agentInfo.radius * invTau * unitW;
			_orcaLines.push_back(line);
			return;
		}

		/*
		* Project on left leg, right leg, or cut-off line, whichever is closest
		* to velocity.
		*/
		const float INF = std::numeric_limits<float>::infinity();

		const float distSqCutoff = ((t < 0.0f || t > 1.0f || obstaclesSame) ? INF : (agentInfo.GetVel() - (leftCutoff + t * cutoffVec)).LengthSquared());
		const float distSqLeft  = (( tLeft < 0.0f) ? INF : (agentInfo.GetVel() - (leftCutoff + tLeft * leftLegDirection)).LengthSquared());
		const float distSqRight = ((tRight < 0.0f) ? INF : (agentInfo.GetVel() - (rightCutoff + tRight * rightLegDirection)).LengthSquared());

		if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
			/* Project on cut-off line. */
			line._direction = -obstDir;
			line._point = leftCutoff + agentInfo.radius * invTau * Vector2(-line._direction.y, line._direction.x);
			_orcaLines.push_back(line);
		}
		else if (distSqLeft <= distSqRight) {
			/* Project on left leg. */
			if (!isLeftLegForeign) {
				line._direction = leftLegDirection;
				line._point = leftCutoff + agentInfo.radius * invTau * Vector2(-line._direction.y, line._direction.x);
				_orcaLines.push_back(line);
			}
		}
		else {
			/* Project on right leg. */
			if (!isRightLegForeign) {
				line._direction = -rightLegDirection;
				line._point = rightCutoff + agentInfo.radius * invTau * Vector2(-line._direction.y, line._direction.x);
				_orcaLines.push_back(line);
			}
		}
	}

	bool ORCAComponent::LinearProgram1(const std::vector<FusionCrowd::Math::Line>& lines, size_t lineNo,
		float radius, const DirectX::SimpleMath::Vector2 & optVelocity,
		bool directionOpt, DirectX::SimpleMath::Vector2& result)
	{
		const float dotProduct = lines[lineNo]._point.Dot(lines[lineNo]._direction);
		const float discriminant = dotProduct * dotProduct + radius * radius - lines[lineNo]._point.LengthSquared();

		if (discriminant < 0.0f) {
			/* Max speed circle fully invalidates line lineNo. */
			return false;
		}

		const float sqrtDiscriminant = std::sqrt(discriminant);
		float tLeft = -dotProduct - sqrtDiscriminant;
		float tRight = -dotProduct + sqrtDiscriminant;

		for (size_t i = 0; i < lineNo; ++i) {
			const float denominator = Math::det(lines[lineNo]._direction, lines[i]._direction);
			const float numerator = Math::det(lines[i]._direction, lines[lineNo]._point - lines[i]._point);

			if (std::fabs(denominator) <= Math::EPS) {
				/* Lines lineNo and i are (almost) parallel. */
				if (numerator < 0.0f) {
					return false;
				}
				else {
					continue;
				}
			}

			const float t = numerator / denominator;

			if (denominator >= 0.0f) {
				/* Line i bounds line lineNo on the right. */
				tRight = std::min(tRight, t);
			}
			else {
				/* Line i bounds line lineNo on the left. */
				tLeft = std::max(tLeft, t);
			}

			if (tLeft > tRight) {
				return false;
			}
		}

		if (directionOpt) {
			/* Optimize direction. */
			if (optVelocity.Dot(lines[lineNo]._direction) > 0.0f) {
				/* Take right extreme. */
				result = lines[lineNo]._point + tRight * lines[lineNo]._direction;
			}
			else {
				/* Take left extreme. */
				result = lines[lineNo]._point + tLeft * lines[lineNo]._direction;
			}
		}
		else {
			/* Optimize closest point. */
			const float t = lines[lineNo]._direction.Dot(optVelocity - lines[lineNo]._point);

			if (t < tLeft) {
				result = lines[lineNo]._point + tLeft * lines[lineNo]._direction;
			}
			else if (t > tRight) {
				result = lines[lineNo]._point + tRight * lines[lineNo]._direction;
			}
			else {
				result = lines[lineNo]._point + t * lines[lineNo]._direction;
			}
		}

		return true;
	}

	size_t ORCAComponent::LinearProgram2(const std::vector<FusionCrowd::Math::Line>& lines, float radius, const Vector2& optVelocity, bool directionOpt, Vector2& result)
	{
		if (directionOpt) {
			/*
			* Optimize direction. Note that the optimization velocity is of unit
			* length in this case.
			*/
			result = optVelocity * radius;
		}
		else if (optVelocity.LengthSquared() > radius * radius) {
			/* Optimize closest point and outside circle. */
			optVelocity.Normalize(result);
			result *= radius;
		}
		else {
			/* Optimize closest point and inside circle. */
			result = optVelocity;
		}

		for (size_t i = 0; i < lines.size(); ++i) {
			if (Math::det(lines[i]._direction, lines[i]._point - result) > 0.0f) {
				/* Result does not satisfy constraint i. Compute new optimal result. */
				const Vector2 tempResult = result;
				if (!LinearProgram1(lines, i, radius, optVelocity, directionOpt, result)) {
					result = tempResult;
					return i;
				}
			}
		}

		return lines.size();
	}

	void ORCAComponent::LinearProgram3(const std::vector<FusionCrowd::Math::Line>& lines, size_t numObstLines,
		size_t beginLine, float radius, Vector2& result)
	{
		float distance = 0.0f;

		for (size_t i = beginLine; i < lines.size(); ++i) {
			if (Math::det(lines[i]._direction, lines[i]._point - result) > distance) {
				/* Result does not satisfy constraint of line i. */
				std::vector<Math::Line> projLines(lines.begin(), lines.begin() + numObstLines);

				for (size_t j = numObstLines; j < i; ++j) {
					Math::Line line;

					float determinant = Math::det(lines[i]._direction, lines[j]._direction);

					if (std::fabs(determinant) <= Math::EPS) {
						/* Math::Line i and line j are parallel. */
						if (lines[i]._direction.Dot(lines[j]._direction) > 0.0f) {
							/* Line i and line j point in the same direction. */
							continue;
						}
						else {
							/* Line i and line j point in opposite direction. */
							line._point = 0.5f * (lines[i]._point + lines[j]._point);
						}
					}
					else {
						line._point = lines[i]._point + (Math::det(lines[j]._direction,
							lines[i]._point - lines[j]._point) / determinant) *
							lines[i]._direction;
					}

					(lines[j]._direction - lines[i]._direction).Normalize(line._direction);
					projLines.push_back(line);
				}

				const Vector2 tempResult = result;
				if (LinearProgram2(projLines, radius,
								   Vector2(-lines[i]._direction.y, lines[i]._direction.x), true,
								   result) < projLines.size()) {
					/* This should in principle not happen.  The result is by definition
					* already in the feasible region of this linear program. If it fails,
					* it is due to small floating point error, and the current result is
					* kept.
					*/
					result = tempResult;
				}

				distance = Math::det(lines[i]._direction, lines[i]._point - result);
			}
		}
	}
}
}
