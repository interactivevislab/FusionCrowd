#include "ORCAComponent.h"
#include "../NavComponents/Obstacle.h"

#include <algorithm>
#include <cassert>
#include <limits>

namespace FusionCrowd
{
	namespace ORCA
	{
		ORCAComponent::ORCAComponent() : _timeHorizon(2.5f), _timeHorizonObst(0.15f)
		{

		}

		ORCAComponent::ORCAComponent(float timeHorizon, float timeHorizonObst) : _timeHorizon(timeHorizon), _timeHorizonObst(timeHorizonObst)
		{
		}

		void ORCAComponent::ComputeNewVelocity(FusionCrowd::Agent* agent)
		{
			const size_t numObstLines = ComputeORCALines(agent);

			FusionCrowd::Math::Vector2 velPref(agent->_velPref.getPreferredVel());

			size_t lineFail = LinearProgram2(_orcaLines, agent->_maxSpeed, velPref, false, agent->_velNew);

			if (lineFail < _orcaLines.size()) {
				LinearProgram3(_orcaLines, numObstLines, lineFail, agent->_maxSpeed, agent->_velNew);
			}
		}

		bool  ORCAComponent::LinearProgram1(const std::vector<FusionCrowd::Math::Line>& lines, size_t lineNo,
			float radius, const FusionCrowd::Math::Vector2 & optVelocity,
			bool directionOpt, FusionCrowd::Math::Vector2& result)
		{
			const float dotProduct = lines[lineNo]._point * lines[lineNo]._direction;
			const float discriminant = FusionCrowd::Math::sqr(dotProduct) + FusionCrowd::Math::sqr(radius) - absSq(lines[lineNo]._point);

			if (discriminant < 0.0f) {
				/* Max speed circle fully invalidates line lineNo. */
				return false;
			}

			const float sqrtDiscriminant = std::sqrt(discriminant);
			float tLeft = -dotProduct - sqrtDiscriminant;
			float tRight = -dotProduct + sqrtDiscriminant;

			for (size_t i = 0; i < lineNo; ++i) {
				const float denominator = det(lines[lineNo]._direction, lines[i]._direction);
				const float numerator = det(lines[i]._direction,
					lines[lineNo]._point - lines[i]._point);

				if (std::fabs(denominator) <= FusionCrowd::EPS) {
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
				if (optVelocity * lines[lineNo]._direction > 0.0f) {
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
				const float t = lines[lineNo]._direction * (optVelocity - lines[lineNo]._point);

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

		size_t ORCAComponent::LinearProgram2(const std::vector<FusionCrowd::Math::Line>& lines, float radius,
			const FusionCrowd::Math::Vector2& optVelocity, bool directionOpt,
			FusionCrowd::Math::Vector2& result)
		{
			if (directionOpt) {
				/*
				* Optimize direction. Note that the optimization velocity is of unit
				* length in this case.
				*/
				result = optVelocity * radius;
			}
			else if (absSq(optVelocity) > FusionCrowd::Math::sqr(radius)) {
				/* Optimize closest point and outside circle. */
				result = norm(optVelocity) * radius;
			}
			else {
				/* Optimize closest point and inside circle. */
				result = optVelocity;
			}

			for (size_t i = 0; i < lines.size(); ++i) {
				if (det(lines[i]._direction, lines[i]._point - result) > 0.0f) {
					/* Result does not satisfy constraint i. Compute new optimal result. */
					const FusionCrowd::Math::Vector2 tempResult = result;
					if (!LinearProgram1(lines, i, radius, optVelocity, directionOpt, result)) {
						result = tempResult;
						return i;
					}
				}
			}

			return lines.size();
		}

		void ORCAComponent::LinearProgram3(const std::vector<FusionCrowd::Math::Line>& lines, size_t numObstLines,
			size_t beginLine, float radius, FusionCrowd::Math::Vector2& result)
		{
			float distance = 0.0f;

			for (size_t i = beginLine; i < lines.size(); ++i) {
				if (det(lines[i]._direction, lines[i]._point - result) > distance) {
					/* Result does not satisfy constraint of line i. */
					std::vector<FusionCrowd::Math::Line> projLines(lines.begin(),
						lines.begin() + numObstLines);

					for (size_t j = numObstLines; j < i; ++j) {
						FusionCrowd::Math::Line line;

						float determinant = det(lines[i]._direction, lines[j]._direction);

						if (std::fabs(determinant) <= FusionCrowd::EPS) {
							/* Math::Line i and line j are parallel. */
							if (lines[i]._direction * lines[j]._direction > 0.0f) {
								/* Line i and line j point in the same direction. */
								continue;
							}
							else {
								/* Line i and line j point in opposite direction. */
								line._point = 0.5f * (lines[i]._point + lines[j]._point);
							}
						}
						else {
							line._point = lines[i]._point + (det(lines[j]._direction,
								lines[i]._point - lines[j]._point) / determinant) *
								lines[i]._direction;
						}

						line._direction = norm(lines[j]._direction - lines[i]._direction);
						projLines.push_back(line);
					}

					const FusionCrowd::Math::Vector2 tempResult = result;
					if (LinearProgram2(projLines, radius, FusionCrowd::Math::Vector2(-lines[i]._direction.y(),
						lines[i]._direction.x()), true, result) < projLines.size()) {
						/* This should in principle not happen.  The result is by definition
						* already in the feasible region of this linear program. If it fails,
						* it is due to small floating point error, and the current result is
						* kept.
						*/
						result = tempResult;
					}

					distance = det(lines[i]._direction, lines[i]._point - result);
				}
			}
		}

		void ORCAComponent::ObstacleLine(size_t obstNbrID, const float invTau, bool flip, FusionCrowd::Agent* agent)
		{
			const Obstacle* obst = agent->_nearObstacles[obstNbrID].obstacle;
			const float LENGTH = obst->length();
			const FusionCrowd::Math::Vector2 P0 = flip ? obst->getP1() : obst->getP0();
			const FusionCrowd::Math::Vector2 P1 = flip ? obst->getP0() : obst->getP1();
			const FusionCrowd::Math::Vector2 obstDir = flip ? -obst->_unitDir : obst->_unitDir;
			const bool p0Convex = flip ? obst->p1Convex(true) : obst->p0Convex(true);
			const bool p1Convex = flip ? obst->p0Convex(true) : obst->p1Convex(true);
			const Obstacle* const leftNeighbor =
				flip ? obst->_nextObstacle : obst->_prevObstacle;
			const Obstacle* const rightNeighbor =
				flip ? obst->_prevObstacle : obst->_nextObstacle;

			const FusionCrowd::Math::Vector2 relativePosition1 = P0 - agent->_pos;
			const FusionCrowd::Math::Vector2 relativePosition2 = P1 - agent->_pos;

			bool alreadyCovered = false;

			for (size_t j = 0; j < _orcaLines.size(); ++j) {
				if (det(invTau * relativePosition1 - _orcaLines[j]._point,
					_orcaLines[j]._direction) - invTau * agent->_radius >=
					-FusionCrowd::EPS && det(invTau * relativePosition2 - _orcaLines[j]._point,
						_orcaLines[j]._direction) - invTau * agent->_radius >= -FusionCrowd::EPS) {
					alreadyCovered = true;
					break;
				}
			}

			if (alreadyCovered) {
				return;
			}

			const float distSq1 = absSq(relativePosition1);
			const float distSq2 = absSq(relativePosition2);

			const float radiusSq = FusionCrowd::Math::sqr(agent->_radius);

			const float s = -(relativePosition1 * obstDir);
			const float distSqLine = absSq(relativePosition1 + s * obstDir);

			FusionCrowd::Math::Line line;

			if (s < 0 && distSq1 <= radiusSq) {
				/* Collision with left vertex. Ignore if non-convex. */
				if (p0Convex) {
					line._point = FusionCrowd::Math::Vector2(0.f, 0.f);
					line._direction = FusionCrowd::Math::norm(FusionCrowd::Math::Vector2(-relativePosition1.y(), relativePosition1.x()));
					_orcaLines.push_back(line);
				}
				return;
			}
			else if (s > LENGTH && distSq2 <= radiusSq) {
				/* Collision with right vertex. Ignore if non-convex
				* or if it will be taken care of by neighoring obstace */
				if ((obst->_nextObstacle == 0x0) || (p1Convex && det(relativePosition2,
					obst->_nextObstacle->_unitDir) >= 0)) {
					line._point = FusionCrowd::Math::Vector2(0.f, 0.f);
					line._direction = FusionCrowd::Math::norm(FusionCrowd::Math::Vector2(-relativePosition2.y(), relativePosition2.x()));
					_orcaLines.push_back(line);
				}
				return;
			}
			else if (s >= 0 && s < LENGTH && distSqLine <= radiusSq) {
				/* Collision with obstacle segment. */
				line._point = FusionCrowd::Math::Vector2(0.f, 0.f);
				line._direction = -obstDir;
				_orcaLines.push_back(line);
				return;
			}

			FusionCrowd::Math::Vector2 leftLegDirection, rightLegDirection;

			bool prevIsCurrent = false;
			bool nextIsCurrent = false;
			if (s < 0 && distSqLine <= radiusSq) {
				/*
				 * Obstacle viewed obliquely so that left vertex defines velocity obstacle.
				 */
				if (!p0Convex) {
					/* Ignore obstacle. */
					return;
				}

				nextIsCurrent = true;

				const float leg1 = std::sqrt(distSq1 - radiusSq);
				leftLegDirection = FusionCrowd::Math::Vector2(relativePosition1.x() * leg1 -
					relativePosition1.y() * agent->_radius,
					relativePosition1.x() * agent->_radius +
					relativePosition1.y() * leg1) / distSq1;
				rightLegDirection = FusionCrowd::Math::Vector2(relativePosition1.x() * leg1 +
					relativePosition1.y() * agent->_radius,
					-relativePosition1.x() * agent->_radius +
					relativePosition1.y() * leg1) / distSq1;
			}
			else if (s > LENGTH && distSqLine <= radiusSq) {
				/*
				* Obstacle viewed obliquely so that right vertex defines velocity obstacle.
				*/
				if (!p1Convex) {
					/* Ignore obstacle. */
					return;
				}

				prevIsCurrent = true;

				const float leg2 = std::sqrt(distSq2 - radiusSq);
				leftLegDirection = FusionCrowd::Math::Vector2(relativePosition2.x() * leg2 -
					relativePosition2.y() * agent->_radius,
					relativePosition2.x() * agent->_radius +
					relativePosition2.y() * leg2) / distSq2;
				rightLegDirection = FusionCrowd::Math::Vector2(relativePosition2.x() * leg2 +
					relativePosition2.y() * agent->_radius,
					-relativePosition2.x() * agent->_radius +
					relativePosition2.y() * leg2) / distSq2;
			}
			else {
				/* Usual situation. */
				if (p0Convex) {
					const float leg1 = std::sqrt(distSq1 - radiusSq);
					leftLegDirection = FusionCrowd::Math::Vector2(relativePosition1.x() * leg1 -
						relativePosition1.y() * agent->_radius,
						relativePosition1.x() * agent->_radius +
						relativePosition1.y() * leg1) / distSq1;
				}
				else {
					/* Left vertex non-convex; left leg extends cut-off line. */
					leftLegDirection = -obstDir;
				}

				if (p1Convex) {
					const float leg2 = std::sqrt(distSq2 - radiusSq);
					rightLegDirection = FusionCrowd::Math::Vector2(relativePosition2.x() * leg2 +
						relativePosition2.y() * agent->_radius,
						-relativePosition2.x() * agent->_radius +
						relativePosition2.y() * leg2) / distSq2;
				}
				else {
					/* Right vertex non-convex; right leg extends cut-off line. */
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
					if (p0Convex && det(leftLegDirection, -leftNeighbor->_unitDir) >= 0.0f) {
						/* Left leg points into obstacle. */
						leftLegDirection = -leftNeighbor->_unitDir;
						isLeftLegForeign = true;
					}
				}
			}

			if (!nextIsCurrent) {
				if (rightNeighbor != 0x0) {
					if (p1Convex && det(rightLegDirection, rightNeighbor->_unitDir) <= 0.0f) {
						/* Right leg points into obstacle. */
						rightLegDirection = rightNeighbor->_unitDir;
						isRightLegForeign = true;
					}
				}
			}

			/* Compute cut-off centers. */
			const FusionCrowd::Math::Vector2 leftCutoff = invTau *
				(prevIsCurrent ? relativePosition2 : relativePosition1);
			const FusionCrowd::Math::Vector2 rightCutoff = nextIsCurrent ? leftCutoff : (invTau * relativePosition2);
			const FusionCrowd::Math::Vector2 cutoffVec = rightCutoff - leftCutoff;
			const bool obstaclesSame = nextIsCurrent || prevIsCurrent;


			/* Project current velocity on velocity obstacle. */
			/* Check if current velocity is projected on cutoff circles. */
			const float t = obstaclesSame ?
				0.5f : ((agent->_vel - leftCutoff) * (cutoffVec / absSq(cutoffVec)));
			const float tLeft = ((agent->_vel - leftCutoff) * leftLegDirection);
			const float tRight = ((agent->_vel - rightCutoff) * rightLegDirection);

			if ((t < 0.0f && tLeft < 0.0f) || (obstaclesSame && tLeft < 0.0f && tRight < 0.0f)) {
				/* Project on left cut-off circle. */
				const FusionCrowd::Math::Vector2 unitW = norm(agent->_vel - leftCutoff);
				line._direction = FusionCrowd::Math::Vector2(unitW.y(), -unitW.x());
				line._point = leftCutoff + agent->_radius * invTau * unitW;
				_orcaLines.push_back(line);
				return;
			}
			else if (t > 1.0f && tRight < 0.0f) {
				/* Project on right cut-off circle. */
				const FusionCrowd::Math::Vector2 unitW = norm(agent->_vel - rightCutoff);
				line._direction = FusionCrowd::Math::Vector2(unitW.y(), -unitW.x());
				line._point = rightCutoff + agent->_radius * invTau * unitW;
				_orcaLines.push_back(line);
				return;
			}

			/*
			* Project on left leg, right leg, or cut-off line, whichever is closest
			* to velocity.
			*/
			const float distSqCutoff = ((t < 0.0f || t > 1.0f || obstaclesSame) ?
				std::numeric_limits<float>::infinity() :
				absSq(agent->_vel - (leftCutoff + t * cutoffVec)));
			const float distSqLeft = ((tLeft < 0.0f) ?
				std::numeric_limits<float>::infinity() :
				absSq(agent->_vel - (leftCutoff + tLeft * leftLegDirection)));
			const float distSqRight = ((tRight < 0.0f) ?
				std::numeric_limits<float>::infinity() :
				absSq(agent->_vel - (rightCutoff + tRight * rightLegDirection)));

			if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
				/* Project on cut-off line. */
				line._direction = -obstDir;
				line._point = leftCutoff + agent->_radius * invTau * FusionCrowd::Math::Vector2(-line._direction.y(),
					line._direction.x());
				_orcaLines.push_back(line);
			}
			else if (distSqLeft <= distSqRight) {
				/* Project on left leg. */
				if (!isLeftLegForeign) {
					line._direction = leftLegDirection;
					line._point = leftCutoff + agent->_radius * invTau * FusionCrowd::Math::Vector2(-line._direction.y(),
						line._direction.x());
					_orcaLines.push_back(line);
				}
			}
			else {
				/* Project on right leg. */
				if (!isRightLegForeign) {
					line._direction = -rightLegDirection;
					line._point = rightCutoff + agent->_radius * invTau * FusionCrowd::Math::Vector2(-line._direction.y(),
						line._direction.x());
					_orcaLines.push_back(line);
				}
			}
		}

		size_t ORCAComponent::ComputeORCALines(FusionCrowd::Agent* agent)
		{
			_orcaLines.clear();

			const float invTimeHorizonObst = 1.0f / _timeHorizonObst;

			/* Create obstacle ORCA lines. */
			for (size_t i = 0; i < agent->_nearObstacles.size(); ++i) {

				const Obstacle* obst = agent->_nearObstacles[i].obstacle;
				const FusionCrowd::Math::Vector2 P0 = obst->getP0();
				const FusionCrowd::Math::Vector2 P1 = obst->getP1();
				const bool agtOnRight = FusionCrowd::Math::leftOf(P0, P1, agent->_pos) < 0.f;
				ObstacleLine(i, invTimeHorizonObst, !agtOnRight && obst->_doubleSided, agent);
			}

			const size_t numObstLines = _orcaLines.size();

			const float invTimeHorizon = 1.0f / _timeHorizon;

			/* Create agent ORCA lines. */
			for (size_t i = 0; i < agent->_nearAgents.size(); ++i) {
				const Agent* const other = static_cast<const Agent *>(agent->_nearAgents[i].agent);

				const FusionCrowd::Math::Vector2 relativePosition = other->_pos - agent->_pos;
				const FusionCrowd::Math::Vector2 relativeVelocity = agent->_vel - other->_vel;

				const float distSq = absSq(relativePosition);
				const float combinedRadius = agent->_radius + other->_radius;
				const float combinedRadiusSq = FusionCrowd::Math::sqr(combinedRadius);

				FusionCrowd::Math::Line line;
				FusionCrowd::Math::Vector2 u;

				if (distSq > combinedRadiusSq) {
					/* No collision. */
					const FusionCrowd::Math::Vector2 w = relativeVelocity - invTimeHorizon * relativePosition;
					/* Vector from cutoff center to relative velocity. */
					const float wLengthSq = absSq(w);

					const float dotProduct1 = w * relativePosition;

					if (dotProduct1 < 0.0f && FusionCrowd::Math::sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
						/* Project on cut-off circle. */
						const float wLength = std::sqrt(wLengthSq);
						const FusionCrowd::Math::Vector2 unitW = w / wLength;

						line._direction = FusionCrowd::Math::Vector2(unitW.y(), -unitW.x());
						u = (combinedRadius * invTimeHorizon - wLength) * unitW;
					}
					else {
						/* Project on legs. */
						const float leg = std::sqrt(distSq - combinedRadiusSq);

						if (det(relativePosition, w) > 0.0f) {
							/* Project on left leg. */
							line._direction = FusionCrowd::Math::Vector2(relativePosition.x() * leg -
								relativePosition.y() * combinedRadius,
								relativePosition.x() * combinedRadius +
								relativePosition.y() * leg) / distSq;
						}
						else {
							/* Project on right leg. */
							line._direction = -FusionCrowd::Math::Vector2(relativePosition.x() * leg +
								relativePosition.y() * combinedRadius,
								-relativePosition.x() * combinedRadius +
								relativePosition.y() * leg) / distSq;
						}

						const float dotProduct2 = relativeVelocity * line._direction;

						u = dotProduct2 * line._direction - relativeVelocity;
					}

					line._point = agent->_vel + 0.5f * u;
				}
				else {
					/* Collision. Project on cut-off circle of time timeStep. */
					const float invTimeStep = 1.0f / 0.1f;//Simulator::TIME_STEP;

					/* Vector from cutoff center to relative velocity. */
					const FusionCrowd::Math::Vector2 w = relativeVelocity - invTimeStep * relativePosition;

					const float wLength = abs(w);
					const FusionCrowd::Math::Vector2 unitW = w / wLength;

					line._direction = FusionCrowd::Math::Vector2(unitW.y(), -unitW.x());
					u = (combinedRadius * invTimeStep - wLength) * unitW;
					float coopWeight = 0.5f;
					line._point = agent->_vel + coopWeight * u;
				}

				_orcaLines.push_back(line);
			}
			return numObstLines;
		}

		void ORCAComponent::Update(FusionCrowd::Agent* agent, float timeStep)
		{
			ComputeNewVelocity(agent);

			agent->UpdateOrient(timeStep);
			agent->PostUpdate();
		}


		ORCAComponent::~ORCAComponent()
		{
		}
	}
}
