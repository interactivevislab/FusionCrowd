#include "PedVOComponent.h"

#include "Navigation/AgentSpatialInfo.h"
#include "Navigation/NavSystem.h"
#include "Navigation/Obstacle.h"

#include "Math/consts.h"
#include "Math/geomQuery.h"
#include "Math/Util.h"

#include <algorithm>
#include <list>
#include <iostream>

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	namespace PedVO
	{
		PedVOComponent::PedVOComponent(std::shared_ptr<NavSystem> navSystem) :
			_cosObstTurn(1.0f), _sinObstTurn(0.0f), _navSystem(navSystem)
		{
		}

		PedVOComponent::PedVOComponent(std::shared_ptr<NavSystem> navSystem, float cosObstTurn, float sinObstTurn) :
			_cosObstTurn(cosObstTurn), _sinObstTurn(sinObstTurn), _navSystem(navSystem)
		{
		}

		PedVOComponent::~PedVOComponent()
		{
		}

		void PedVOComponent::AddAgent(size_t agentId)
		{
			AddAgent(agentId, 3.0f, 0.1f, 2.0, true, 1.57f, 0.9f);
		}

		void PedVOComponent::AddAgent(size_t agentId, float timeHorizon, float timeHorizonObst, float turningBias, bool denseAware, float factor, float buffer)
		{
			_agents[agentId] = AgentParamentrs(timeHorizon, timeHorizonObst, turningBias, denseAware, factor, buffer);
			_navSystem->GetSpatialInfo(agentId).inertiaEnabled = false;
		}

		bool PedVOComponent::DeleteAgent(size_t agentId)
		{
			return _agents.erase(agentId) > 0;
		}

		void PedVOComponent::Update(float timeStep)
		{
			for(auto & p : _agents)
			{
				size_t id = p.first;
				AgentParamentrs & params = p.second;
				AgentSpatialInfo & spatialInfo = _navSystem->GetSpatialInfo(id);

				ComputeNewVelocity(params, spatialInfo, timeStep);
			}
		}

		void PedVOComponent::ComputeNewVelocity(AgentParamentrs & agentParams, AgentSpatialInfo & agentInfo, float timeStep)
		{
			AdaptPreferredVelocity(agentParams, agentInfo);

			Vector2 optVel;
			Vector2 prefDir;
			float prefSpeed;

			const size_t numObstLines = ComputeORCALinesTurning(agentParams, agentInfo, optVel, prefDir, timeStep, prefSpeed);

			size_t lineFail = LinearProgram2(_orcaLines, agentInfo.maxSpeed, optVel, false, agentParams._turningBias, agentInfo.velNew);

			if (lineFail < _orcaLines.size()) {
				LinearProgram3(_orcaLines, numObstLines, lineFail, agentInfo.maxSpeed, agentParams._turningBias, agentInfo.velNew);
			}
			if (agentParams._turningBias != 1.f && prefSpeed > Math::EPS) {
				// Transform _velNew from affine space to real space
				// Undo the scale
				Vector2 vel(agentInfo.velNew.x, agentInfo.velNew.y * agentParams._turningBias);
				// Rotate it back
				// Flip the y-value so I perform rotation in the other direction
				//	I'm multiplying v * R, where R is the matrix:
				//
				//   R  = [ prefDir'  n' ]  (i.e. concatenation of two column vectors
				//
				//	I rotated INTO affine space using prefDir and n.  To reverse the rotation
				//	I had to flip the signs of the upper right and lower left corners.  That's
				//	What the negation of prefDir.y does.
				prefDir = Vector2(prefDir.x, -prefDir.y);
				Vector2 n(-prefDir.y, prefDir.x);
				float vx = vel.Dot(prefDir);
				float vy = vel.Dot(n);
				agentInfo.velNew = Vector2(vx, vy);
			}
		}

		void PedVOComponent::AdaptPreferredVelocity(AgentParamentrs & agentParams, AgentSpatialInfo & agentInfo)
		{
			if (agentParams._denseAware) {
				float prefSpeed = agentInfo.prefVelocity.getSpeed();
				Vector2 prefDir(agentInfo.prefVelocity.getPreferred());
				// start assuming there is infinite space
				float availSpace = 1e6f;

				// Not the speed-dependent stride length, but rather the mid-point of the
				float strideLen = 1.f;
				// elliptical personal space.
				Vector2 critPt = agentInfo.GetPos() + strideLen * prefDir;
				float density = 0.f;
				// For now, assume some constants
				const float area = 1.5f;
				const float areaSq2Inv = 1.f / (2 * area * area);
				const float sqrt2Pi = sqrtf(Math::TWOPI);
				const float norm = 1.f / (area * sqrt2Pi);

				// AGENTS
				for (auto & other : _navSystem->GetNeighbours(agentInfo.id))
				{
					Vector2 critDisp = other.pos - critPt;
					// dot project gets projection, in the preferred direction
					Vector2 yComp = critDisp.Dot(prefDir) * prefDir;
					// penalize displacement perpindicular to the preferred direction
					Vector2 xComp = (critDisp - yComp) * 2.5f;
					critDisp = xComp + yComp;
					float distSq = critDisp.LengthSquared();
					density += norm * expf(-distSq * areaSq2Inv);
				}
				//// OBSTACLES
				const float OBST_AREA = 0.75f;
				const float OBST_AREA_SQ_INV = 1.f / (2 * OBST_AREA * OBST_AREA);
				const float OBST_SCALE = norm;  // * 6.25f;	// what is the "density" of an obstacle?
				for (auto const obst : _navSystem->GetClosestObstacles(agentInfo.id)) {
					Vector2 nearPt;
					float distSq;  // set by distanceSqToPoint
					if (obst.distanceSqToPoint(critPt, nearPt, distSq) == Obstacle::LAST)
						continue;

					if ( prefDir.Dot(nearPt - agentInfo.GetPos()) < 0.f) continue;
					density += OBST_SCALE * expf(-distSq * OBST_AREA_SQ_INV);
				}

				const float AGENT_WIDTH = 0.48f;
				if (density < 0.001f) {
					availSpace = 100.f;
				}
				else {
					availSpace = AGENT_WIDTH / density;
				}

				// Compute the maximum speed I could take for the available space
				float maxSpeed = agentParams._speedConst * availSpace * availSpace;
				if (maxSpeed < prefSpeed) agentInfo.prefVelocity.setSpeed(maxSpeed);
			}
		}

		size_t PedVOComponent::ComputeORCALinesTurning(
			AgentParamentrs & agentParams, AgentSpatialInfo & agentInfo,
			DirectX::SimpleMath::Vector2& optVel, DirectX::SimpleMath::Vector2& prefDir, float timeStep, float& prefSpeed)
		{
			_orcaLines.clear();

			const float invTimeHorizonObst = 1.0f / agentParams._timeHorizonObst;

			for (Obstacle & obst : _navSystem->GetClosestObstacles(agentInfo.id))
			{
				const Vector2 P0 = obst.getP0();
				const Vector2 P1 = obst.getP1();
				const bool agtOnRight = Math::leftOf(P0, P1, agentInfo.GetPos()) < 0.f;

				ObstacleLine(agentParams, agentInfo, obst, invTimeHorizonObst, !agtOnRight && obst._doubleSided);
			}

			const size_t numObstLines = _orcaLines.size();

			const float invTimeHorizon = 1.0f / agentParams._timeHorizon;
			/* Create agent ORCA lines. */
			for (auto & other : _navSystem->GetNeighbours(agentInfo.id))
			{
				const Vector2 relativePosition = other.pos - agentInfo.GetPos();

				// TODO: priorities
				//float rightOfWay = fabs(agentInfo.priority - other->_priority);
				//if (rightOfWay > 1.f)
				//	rightOfWay = 1.f;
				//
				float rightOfWay = rand();

				// Right of way-dependent calculations
				Vector2 myVel = agentInfo.GetVel();
				Vector2 hisVel = other.vel;
				// this is my fraction of effort
				float weight = 0.5f;
				float const MAX_DEV = 0.1f;
				float const MAX_DEV_SQD = MAX_DEV * MAX_DEV;

				// TODO: we don't have priorities yet, so lets flip a coin.
				bool myAdvantage = rightOfWay > 0.5f;
				//if (agent->_priority < other->_priority) {
				if (!myAdvantage) {
					// his advantage
					weight += 0.5f * rightOfWay;
					hisVel = other.prefVel * rightOfWay + (1.f - rightOfWay) * other.vel;
					if ((hisVel - other.vel).LengthSquared() > MAX_DEV_SQD) {
						hisVel = other.prefVel - other.vel;
						hisVel.Normalize();

						hisVel = hisVel * MAX_DEV + other.vel;
					}
				}
				else
				{
					// my advantage
					weight -= 0.5f * rightOfWay;
					myVel = agentInfo.prefVelocity.getPreferredVel() * rightOfWay + (1.f - rightOfWay) * agentInfo.GetVel();
					if ((myVel - agentInfo.GetVel()).LengthSquared() > MAX_DEV_SQD) {
						(agentInfo.prefVelocity.getPreferredVel() - agentInfo.GetVel()).Normalize(myVel);
						myVel = myVel * MAX_DEV + agentInfo.GetVel();
					}
				}

				const Vector2 relativeVelocity = myVel - hisVel;

				const float distSq = relativePosition.LengthSquared();
				const float combinedRadius = agentInfo.radius + other.radius;
				const float combinedRadiusSq = pow(combinedRadius, 2);

				Math::Line line;
				Vector2 u;

				if (distSq > combinedRadiusSq) {
					/* No collision. */
					const Vector2 w = relativeVelocity - invTimeHorizon * relativePosition;
					/* Vector from cutoff center to relative velocity. */
					const float wLengthSq = w.LengthSquared();

					const float dotProduct1 = w.Dot(relativePosition);

					if (dotProduct1 < 0.0f && pow(dotProduct1, 2) > combinedRadiusSq * wLengthSq) {
						/* Project on cut-off circle. */
						const float wLength = std::sqrt(wLengthSq);
						const Vector2 unitW = w / wLength;

						line._direction = Vector2(unitW.y, -unitW.x);
						u = (combinedRadius * invTimeHorizon - wLength) * unitW;
					}
					else {
						/* Project on legs. */
						const float leg = std::sqrt(distSq - combinedRadiusSq);

						if (Math::det(relativePosition, w) > 0.0f) {
							/* Project on left leg. */
							line._direction =
								Vector2(relativePosition.x * leg - relativePosition.y * combinedRadius,
									relativePosition.x * combinedRadius + relativePosition.y * leg) /
								distSq;
						}
						else {
							/* Project on right leg. */
							line._direction =
								-Vector2(relativePosition.x * leg + relativePosition.y * combinedRadius,
									-relativePosition.x * combinedRadius + relativePosition.y * leg) /
								distSq;
						}

						const float dotProduct2 = relativeVelocity.Dot(line._direction);

						u = dotProduct2 * line._direction - relativeVelocity;
					}
					if (u.Dot(relativePosition) > 0) weight = 0.5f;

					line._point = myVel + weight * u;
				}
				else {
					/* Collision. Project on cut-off circle of time timeStep. */
					const float invTimeStep = 1.0f / timeStep;

					/* Vector from cutoff center to relative velocity. */
					const Vector2 w = relativeVelocity - invTimeStep * relativePosition;

					const float wLength = w.Length();
					const Vector2 unitW = w / wLength;

					line._direction = Vector2(unitW.y, -unitW.x);
					u = (combinedRadius * invTimeStep - wLength) * unitW;
					line._point = myVel + weight * u;
				}

				_orcaLines.push_back(line);
			}

			// Transform the lines
			if (agentParams._turningBias != 1.f) {
				prefSpeed = agentInfo.prefVelocity.getSpeed();
				optVel = Vector2(prefSpeed, 0.f);
				// Transformation is dependent on prefSpeed being non-zero
				if (prefSpeed > Math::EPS) {
					prefDir = Vector2(agentInfo.prefVelocity.getPreferred());
					Vector2 n(-prefDir.y, prefDir.x);
					// rotate and scale all of the lines
					float turnInv = 1.f / agentParams._turningBias;
					for (size_t i = 0; i < _orcaLines.size(); ++i) {
						// Make sure I'm not perpendicular
						// turning threshhold is bigger than zero degrees
						if (_cosObstTurn < 1.f &&
							// only tilt if I'm seeking to magnify differences
							agentParams._turningBias > 1.f && i >= numObstLines &&  // don't perturb obstacles
							Math::det(_orcaLines[i]._direction,
								// preferred velocity is not feasible w.r.t. this obstacle
								_orcaLines[i]._point - agentInfo.prefVelocity.getPreferredVel()) > 0.0f &&
							// This is a trick: det with the line direction, is dot product with normal
							// angle between pref vel and line norm is less than 1/2 degree either way
							Math::det(-_orcaLines[i]._direction, prefDir) > _cosObstTurn) {
							// Compute the intersection with the circle of maximum velocity
							float dotProduct = _orcaLines[i]._point.Dot(_orcaLines[i]._direction);
							float discriminant = pow(dotProduct, 2) + pow(agentInfo.maxSpeed, 2) - _orcaLines[i]._point.LengthSquared();
							if (discriminant >= 0.f) {
								// Intersects the circle of maximum speed
								// I already know from the previous test that the preferred velocity
								// lies on the infeasible side of the obstacle so, if there's no
								// intersection, the whole circle must be infeasible. don't bother
								// perturbing.
								const float sqrtDiscriminant = std::sqrt(discriminant);
								if (agentInfo.GetVel().Dot(_orcaLines[i]._direction) > 0.f) {
									float t = -dotProduct + sqrtDiscriminant;
									// new line point
									Vector2 p = _orcaLines[i]._point + t * _orcaLines[i]._direction;
									// clockwise rotation
									const Vector2 rx(_cosObstTurn, _sinObstTurn);
									const Vector2 ry(-_sinObstTurn, _cosObstTurn);
									float dx = Math::det(prefDir, rx);
									float dy = Math::det(prefDir, ry);
									_orcaLines[i]._direction = Vector2(dx, dy);
									_orcaLines[i]._point = p;
								}
								else {
									float t = -dotProduct - sqrtDiscriminant;
									// new line point
									Vector2 p = _orcaLines[i]._point + t * _orcaLines[i]._direction;
									// counter-clockwise rotation
									const Vector2 rx(_cosObstTurn, -_sinObstTurn);
									const Vector2 ry(_sinObstTurn, _cosObstTurn);
									float dx = Math::det(prefDir, rx);
									float dy = Math::det(prefDir, ry);
									_orcaLines[i]._direction = Vector2(dx, dy);
									_orcaLines[i]._point =p;
								}
							}
						}

						// rotate
						float px = _orcaLines[i]._point.Dot(prefDir);
						float py = _orcaLines[i]._point.Dot(n);
						float dx = _orcaLines[i]._direction.Dot(prefDir);
						float dy = _orcaLines[i]._direction.Dot(n);
						// scale
						py *= turnInv;
						dy *= turnInv;
						// set
						_orcaLines[i]._point = Vector2(px, py);
						(Vector2(dx, dy)).Normalize(_orcaLines[i]._direction);
					}
				}
			}
			else {
				optVel = (agentInfo.prefVelocity.getPreferredVel());
			}

			return numObstLines;
		}

		void PedVOComponent::ObstacleLine(AgentParamentrs & agentParams, AgentSpatialInfo & agentInfo, Obstacle & obst, const float invTau, bool flip)
		{
			const float LENGTH = obst.length();
			const Vector2 P0 = flip ? obst.getP1() : obst.getP0();
			const Vector2 P1 = flip ? obst.getP0() : obst.getP1();
			const Vector2 obstDir = flip ? -obst._unitDir : obst._unitDir;
			const bool p0Convex = flip ? obst.p1Convex(true) : obst.p0Convex(true);
			const bool p1Convex = flip ? obst.p0Convex(true) : obst.p1Convex(true);
			const Obstacle* const leftNeighbor = flip ? obst._nextObstacle : obst._prevObstacle;
			const Obstacle* const rightNeighbor = flip ? obst._prevObstacle : obst._nextObstacle;

			const Vector2 relativePosition1 = P0 - agentInfo.GetPos();
			const Vector2 relativePosition2 = P1 - agentInfo.GetPos();

			/*
			 * Check if velocity obstacle of obstacle is already taken care of by
			 * previously constructed obstacle ORCA lines.
			 */
			bool alreadyCovered = false;

			for (size_t j = 0; j < _orcaLines.size(); ++j) {
				if (Math::det(invTau * relativePosition1 - _orcaLines[j]._point, _orcaLines[j]._direction) -
					invTau * agentInfo.radius >=
					-Math::EPS &&
					Math::det(invTau * relativePosition2 - _orcaLines[j]._point, _orcaLines[j]._direction) -
					invTau * agentInfo.radius >=
					-Math::EPS) {
					alreadyCovered = true;
					break;
				}
			}

			if (alreadyCovered) {
				return;
			}

			/* Not yet covered. Check for collisions. */

			const float distSq1 = relativePosition1.LengthSquared();
			const float distSq2 = relativePosition2.LengthSquared();

			const float radiusSq = pow(agentInfo.radius, 2);

			const float s = -(relativePosition1.Dot(obstDir));
			const float distSqLine = (relativePosition1 + s * obstDir).LengthSquared();

			Math::Line line;

			if (s < 0 && distSq1 <= radiusSq) {
				/* Collision with left vertex. Ignore if non-convex. */
				if (p0Convex) {
					line._point = Vector2(0.f, 0.f);
					(Vector2(-relativePosition1.y, relativePosition1.x)).Normalize(line._direction);
					_orcaLines.push_back(line);
				}
				return;
			}
			else if (s > LENGTH && distSq2 <= radiusSq) {
				/* Collision with right vertex. Ignore if non-convex
				 * or if it will be taken care of by neighoring obstace */
				if ((obst._nextObstacle == 0x0) ||
					(p1Convex && Math::det(relativePosition2, obst._nextObstacle->_unitDir) >= 0)) {
					line._point = Vector2(0.f, 0.f);
					(Vector2(-relativePosition2.y, relativePosition2.x)).Normalize(line._direction);
					_orcaLines.push_back(line);
				}
				return;
			}
			else if (s >= 0 && s < LENGTH && distSqLine <= radiusSq) {
				/* Collision with obstacle segment. */
				line._point = Vector2(0.f, 0.f);
				line._direction = -obstDir;
				_orcaLines.push_back(line);
				return;
			}

			/*
			 * No collision.
			 * Compute legs. When obliquely viewed, both legs can come from a single
			 * vertex. Legs extend cut-off line when nonconvex vertex.
			 */

			Vector2 leftLegDirection, rightLegDirection;

			/*!
			 *	These booleans short-cut the later code in which we make sure a leg direction does not
			 *	cut into a "neighboring" obstacle.
			 *
			 *	In the case where the agent is "obliquely viewing" the obstacle near the left or right
			 *	edge, we end up testing one of the legs against obstacle 1 itself.  However, by
			 *	definition, we know that both legs lie outside of the obstacle.
			 */
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
				leftLegDirection = Vector2(relativePosition1.x * leg1 - relativePosition1.y * agentInfo.radius,
					relativePosition1.x * agentInfo.radius + relativePosition1.y * leg1) /
					distSq1;
				rightLegDirection = Vector2(relativePosition1.x * leg1 + relativePosition1.y * agentInfo.radius,
					-relativePosition1.x * agentInfo.radius + relativePosition1.y * leg1) /
					distSq1;
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
				leftLegDirection = Vector2(relativePosition2.x * leg2 - relativePosition2.y * agentInfo.radius,
					relativePosition2.x * agentInfo.radius + relativePosition2.y * leg2) /
					distSq2;
				rightLegDirection = Vector2(relativePosition2.x * leg2 + relativePosition2.y * agentInfo.radius,
					-relativePosition2.x * agentInfo.radius + relativePosition2.y * leg2) /
					distSq2;
			}
			else {
				/* Usual situation. */
				if (p0Convex) {
					const float leg1 = std::sqrt(distSq1 - radiusSq);
					leftLegDirection = Vector2(relativePosition1.x * leg1 - relativePosition1.y * agentInfo.radius,
						relativePosition1.x * agentInfo.radius + relativePosition1.y * leg1) /
						distSq1;
				}
				else {
					/* Left vertex non-convex; left leg extends cut-off line. */
					leftLegDirection = -obstDir;
				}

				if (p1Convex) {
					const float leg2 = std::sqrt(distSq2 - radiusSq);
					rightLegDirection = Vector2(relativePosition2.x * leg2 + relativePosition2.y * agentInfo.radius,
						-relativePosition2.x * agentInfo.radius + relativePosition2.y * leg2) /
						distSq2;
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
			const float t = obstaclesSame ? 0.5f : ((agentInfo.GetVel() - leftCutoff).Dot(cutoffVec / cutoffVec.LengthSquared()));
			const float tLeft  = (agentInfo.GetVel() - leftCutoff).Dot(leftLegDirection);
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
			const float distSqCutoff =
				((t < 0.0f || t > 1.0f || obstaclesSame) ? std::numeric_limits<float>::infinity()
					:(agentInfo.GetVel() - (leftCutoff + t * cutoffVec)).LengthSquared());
			const float distSqLeft = ((tLeft < 0.0f) ? std::numeric_limits<float>::infinity()
				: (agentInfo.GetVel() - (leftCutoff + tLeft * leftLegDirection)).LengthSquared());
			const float distSqRight =
				((tRight < 0.0f) ? std::numeric_limits<float>::infinity()
					: (agentInfo.GetVel() - (rightCutoff + tRight * rightLegDirection)).LengthSquared());

			if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
				/* Project on cut-off line. */
				line._direction = -obstDir;
				line._point =
					leftCutoff + agentInfo.radius * invTau * Vector2(-line._direction.y, line._direction.x);
				_orcaLines.push_back(line);
			}
			else if (distSqLeft <= distSqRight) {
				/* Project on left leg. */
				if (!isLeftLegForeign) {
					line._direction = leftLegDirection;
					line._point =
						leftCutoff + agentInfo.radius * invTau * Vector2(-line._direction.y, line._direction.x);
					_orcaLines.push_back(line);
				}
			}
			else {
				/* Project on right leg. */
				if (!isRightLegForeign) {
					line._direction = -rightLegDirection;
					line._point =
						rightCutoff + agentInfo.radius * invTau * Vector2(-line._direction.y, line._direction.x);
					_orcaLines.push_back(line);
				}
			}
		}

		bool PedVOComponent::LinearProgram1(const std::vector<Math::Line>& lines, size_t lineNo, float radius,
			const DirectX::SimpleMath::Vector2& optVelocity, bool directionOpt, float turnBias,
			DirectX::SimpleMath::Vector2& result)
		{
			// Despite turn the dot product is the same
  //	This is because the point got scaled by <1, 1/turn> and the dir got scaled by
  //	<1, turn>.  So, pt.x * dir.x + pt.y * dir.y = pt.x * dir.x + pt.y /turn * dir.y * turn
  //	So, they are mathematically equivalent.
			float dotProduct = lines[lineNo]._point.Dot(lines[lineNo]._direction);
			float discriminant;
			if (turnBias != 1.f) {
				// test against transformed lines
				Vector2 pt(lines[lineNo]._point.x, lines[lineNo]._point.y * turnBias);
				discriminant = pow(dotProduct, 2) + pow(radius, 2) - pt.LengthSquared();

				if (discriminant < 0.0f) {
					/* Max speed circle fully invalidates line lineNo. */
					return false;
				}
				discriminant = pow(dotProduct, 2) + pow(radius, 2) - lines[lineNo]._point.LengthSquared();
			}
			else {
				discriminant = pow(dotProduct, 2) + pow(radius, 2) - lines[lineNo]._point.LengthSquared();

				if (discriminant < 0.0f) {
					/* Max speed circle fully invalidates line lineNo. */
					return false;
				}
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

		size_t PedVOComponent::LinearProgram2(const std::vector<Math::Line>& lines, float radius,
			const DirectX::SimpleMath::Vector2& optVelocity, bool directionOpt, float turnBias,
			DirectX::SimpleMath::Vector2& result)
		{
			if (directionOpt) {
				/*
				 * Optimize direction. Note that the optimization velocity is of unit
				 * length in this case.
				 */
				result = optVelocity * radius;
			}
			else if (optVelocity.LengthSquared() > pow(radius, 2)) {
				/* Optimize closest point and outside circle. */
				optVelocity.Normalize(result);
				result = result * radius;

			}
			else {
				/* Optimize closest point and inside circle. */
				result = optVelocity;
			}

			for (size_t i = 0; i < lines.size(); ++i) {
				if (Math::det(lines[i]._direction, lines[i]._point - result) > 0.0f) {
					/* Result does not satisfy constraint i. Compute new optimal result. */
					const Vector2 tempResult = result;
					if (!LinearProgram1(lines, i, radius, optVelocity, directionOpt, turnBias, result)) {
						result = tempResult;
						return i;
					}
				}
			}
			return lines.size();
		}

		void  PedVOComponent::LinearProgram3(const std::vector<Math::Line>& lines, size_t numObstLines,
			size_t beginLine, float radius, float turnBias, DirectX::SimpleMath::Vector2& result)
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
							/* Line i and line j are parallel. */
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
							line._point =
								lines[i]._point +
								(Math::det(lines[j]._direction, lines[i]._point - lines[j]._point) / determinant) *
								lines[i]._direction;
						}

						(lines[j]._direction - lines[i]._direction).Normalize(line._direction);
						projLines.push_back(line);
					}

					const Vector2 tempResult = result;
					if (LinearProgram2(projLines, radius,
						Vector2(-lines[i]._direction.y, lines[i]._direction.x), true, turnBias,
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
