#include "BicycleComponent.h"

#include "Math/consts.h"
#include "Math/geomQuery.h"
#include "Math/Util.h"

#include "Navigation/AgentSpatialInfo.h"
#include "Navigation/Obstacle.h"

#include <algorithm>
#include <list>
#include <iostream>
#include <cmath>


using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	namespace Bicycle
	{
		int step = 0;
		BicycleComponent::BicycleComponent(std::shared_ptr<NavSystem> navSystem) : _navSystem(navSystem)
		{
		}

		void BicycleComponent::Update(float timeStep)
		{
			for (auto p : _agents)
			{
				auto id = p.first;
				AgentSpatialInfo & agent = _navSystem->GetSpatialInfo(id);
				ComputeNewVelocity(agent, timeStep);
			}
		}

		Vector2 rotateVector(Vector2 vector, float angle)
		{
			Vector2 rotatedVector;
			rotatedVector.x = vector.x * cos(angle) - vector.y * sin(angle);
			rotatedVector.y = vector.x * sin(angle) + vector.y * cos(angle);
			return rotatedVector;
		}

		void dump(AgentSpatialInfo & agent)
		{
			std::cout << step<<"\n";
			std::cout << "orient" << agent.GetOrient().x << ',' << agent.GetOrient().y<<"\n";
			std::cout << "prefSpeed" << agent.prefSpeed<<"\n";
			std::cout << "prefVel" << agent.prefVelocity.getPreferredVel().x << ',' << agent.prefVelocity.getPreferredVel().y << "\n";
			std::cout << "Vel" << agent.GetVel().x << ',' << agent.GetVel().y << "\n";
			std::cout << "NewVel" << agent.velNew.x << ',' << agent.velNew.y << "\n";
		}

		float BicycleComponent::CalcTargetSteeringRadius(const AgentParamentrs & agent, const AgentSpatialInfo & spatialInfo, Vector2 targetPoint)
		{
			Vector2 wheelPos = spatialInfo.GetPos() + (agent._length / 2.0f) * spatialInfo.GetOrient();
			Vector2 delta = targetPoint - wheelPos;

			Vector2 middleLinePoint = wheelPos + delta * .5f;
			Vector2 middleLineDir(delta.y, -delta.x); middleLineDir.Normalize();

			Vector2 backWheelLinePoint = spatialInfo.GetPos() - (agent._length / 2.0f) * spatialInfo.GetOrient();
			Vector2 backWheelLineDir(spatialInfo.GetOrient().y, -spatialInfo.GetOrient().x);

			// https://math.stackexchange.com/questions/406864/intersection-of-two-lines-in-vector-form
			float u1 = middleLineDir.x;
			float v1 = middleLineDir.y;
			float u2 = backWheelLineDir.x;
			float v2 = backWheelLineDir.y;

			float dx = backWheelLinePoint.x - middleLinePoint.x;
			float dy = backWheelLinePoint.y - middleLinePoint.y;

			float det1 = (u1 * (-v2) - v1 * (-u2));

			if(det1 < 1e-6) // Target radius is very large
			{
				return _maxSteeringR * 2;
			}

			float a = 1.0f / det1 * (dx *(-v2) - dy * (-u2));

			Vector2 center = middleLinePoint + a * middleLineDir;
			return (wheelPos - center).Length();
		}

		float GetAdjustedPreferredSpeed(float prefSpeed, float distanceToTarget)
		{
			const float closeSpeed = 0.5f * prefSpeed;

			const float closeDist  =  3.0f;
			const float farDist    = 10.0f;

			if(distanceToTarget > farDist)
				return prefSpeed;

			if(distanceToTarget < closeDist)
				return closeSpeed;

			return (distanceToTarget - closeDist) / (farDist - closeDist) * (prefSpeed - closeSpeed) + closeSpeed;
		}

		void BicycleComponent::ComputeNewVelocity(AgentSpatialInfo & spatialInfo, float timeStep)
		{
			const float maxAcceleration = spatialInfo.maxAccel * timeStep;

			AgentParamentrs & agent = _agents[spatialInfo.id];

			//Update our length if it was changed for some reason
			agent._length = spatialInfo.radius * 2.0f;
			agent._theta  = atan2(spatialInfo.GetOrient().y, spatialInfo.GetOrient().x);

			float distanceToTarget = Vector2::Distance(spatialInfo.prefVelocity.getTarget(), spatialInfo.GetPos());
			Vector2 prefVel = spatialInfo.prefVelocity.getPreferredVel();
			Vector2 normalizedPrefVel; prefVel.Normalize(normalizedPrefVel);
			Vector2 orient = spatialInfo.GetOrient();

			if(prefVel.Length() < 1e-6f)
				return;

			Vector2 vel = spatialInfo.GetVel();
			// Angle between current direction and target direction
			float angleToTarget = (float) atan2(orient.x * normalizedPrefVel.y - orient.y * normalizedPrefVel.x, orient.x * normalizedPrefVel.x + orient.y * normalizedPrefVel.y);

			float currentSteeringR = 0;
			if(abs(agent._delta) < 0.0001f)
			{
				currentSteeringR = _maxSteeringR;
			} else
			{
				currentSteeringR = agent._length / cos(Math::PI / 2.0f - abs(agent._delta));
			}

			float targetSteeringR = CalcTargetSteeringRadius(agent, spatialInfo, spatialInfo.prefVelocity.getTarget());

			float speed = vel.Length();
			float adjustedPreferredSpeed = GetAdjustedPreferredSpeed(spatialInfo.prefVelocity.getSpeed(), distanceToTarget);
			float accelerationToPref = adjustedPreferredSpeed - speed;

			bool sameSide = Math::sgn(angleToTarget) == Math::sgn(agent._delta);
			bool noNeedToBrake = abs(currentSteeringR - targetSteeringR) < 0.1f;

			// Catching up to preferred velocity if we can
			float reqiredAcceleration = accelerationToPref;

			if(abs(reqiredAcceleration) > maxAcceleration)
			{
				reqiredAcceleration = Math::sgn(reqiredAcceleration) * maxAcceleration;
			}

			speed += reqiredAcceleration;

			if(speed < 0.05f)
			{
				speed = 0.05f;
			}
			/*
			for (auto const & obst : _navSystem->GetClosestObstacles(agent.id)) {

				Vector2 nearPt;
				float sqDist;
				float SAFE_DIST2 = 0.03;
				if (obst.distanceSqToPoint(agent.pos, nearPt, sqDist) == Obstacle::LAST) continue;
				if (SAFE_DIST2 > sqDist)
				{
					_agents[agent.id]._delta = 0.0f;
					_agents[agent.id]._theta = atan2(normalizedPrefVel.y, normalizedPrefVel.x);
				}
			}
			*/

			float deltaDelta = timeStep * 10.0f * 0.05f * (3.f + 1.f / (10.f * speed + 0.2f));


			float targetDelta = atan(angleToTarget * agent._length / speed);

			float steer = targetDelta - agent._delta;

			// Actually steer now
			if(abs(steer) < deltaDelta)
			{
				agent._delta += steer;
			} else
			{
				agent._delta += Math::sgn(steer) * deltaDelta;
			}

			// rotate virtual bike body
			agent._theta += speed * tan(agent._delta) / agent._length;

			spatialInfo.velNew = speed * Vector2(cos(agent._theta), sin(agent._theta));

			step++;
		}


		void BicycleComponent::AddAgent(size_t id, float mass)
		{
			_agents[id] = AgentParamentrs();
			_navSystem->GetSpatialInfo(id).inertiaEnabled = true;
		}

		void BicycleComponent::AddAgent(size_t id)
		{
			AddAgent(id, 80.0f);
		}

		bool BicycleComponent::DeleteAgent(size_t id)
		{
			_agents.erase(id);
			return true;
		}
	}
}