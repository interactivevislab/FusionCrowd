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
			std::cout << "orient" << agent.orient.x << ',' << agent.orient.y<<"\n";
			std::cout << "prefSpeed" << agent.prefSpeed<<"\n";
			std::cout << "prefVel" << agent.prefVelocity.getPreferredVel().x << ',' << agent.prefVelocity.getPreferredVel().y << "\n";
			std::cout << "Vel" << agent.vel.x << ',' << agent.vel.y << "\n";
			std::cout << "NewVel" << agent.velNew.x << ',' << agent.velNew.y << "\n";
		}

		float BicycleComponent::calcTargetSteeringRadius(const AgentParamentrs & agent, const AgentSpatialInfo & spatialInfo, Vector2 targetPoint)
		{
			Vector2 wheelPos = spatialInfo.pos + (agent._length / 2.0f) * spatialInfo.orient;
			Vector2 delta = targetPoint - wheelPos;

			Vector2 middleLinePoint = wheelPos + delta * .5f;
			Vector2 middleLineDir(delta.y, -delta.x); middleLineDir.Normalize();

			Vector2 backWheelLinePoint = spatialInfo.pos - (agent._length / 2.0f) * spatialInfo.orient;
			Vector2 backWheelLineDir(spatialInfo.orient.y, -spatialInfo.orient.x);

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

		void BicycleComponent::ComputeNewVelocity(AgentSpatialInfo & spatialInfo, float timeStep)
		{
			const float maxAcceleration = spatialInfo.maxAccel * timeStep;

			AgentParamentrs & agent = _agents[spatialInfo.id];

			//Update our length if it was changed for some reason
			agent._length = spatialInfo.radius * 2.0f;
			agent._theta  = atan2(spatialInfo.orient.y, spatialInfo.orient.x);

			float distanceToTarget = Vector2::Distance(spatialInfo.prefVelocity.getTarget(), spatialInfo.pos);
			Vector2 prefVel = spatialInfo.prefVelocity.getPreferredVel();
			Vector2 normalizedPrefVel; prefVel.Normalize(normalizedPrefVel);
			Vector2 orient = spatialInfo.orient;

			if(prefVel.Length() < 1e-6f)
				return;

			Vector2 vel = spatialInfo.vel;

			float currentSteeringR = 0;
			if(abs(agent._delta) < 0.001f)
			{
				currentSteeringR = abs(agent._length / cos(MathUtil::PI / 2.0f - 0.001f));
			} else
			{
				currentSteeringR = abs(agent._length / cos(MathUtil::PI / 2.0f - agent._delta));
			}

			float targetSteeringR = calcTargetSteeringRadius(agent, spatialInfo, spatialInfo.prefVelocity.getTarget());

			float speed = vel.Length();

			// Catching up to preferred velocity if we can
			if(targetSteeringR >= _maxSteeringR)
			{
				float reqiredAcceleration = (spatialInfo.prefVelocity.getSpeed() - speed) / timeStep;

				if(abs(reqiredAcceleration) < maxAcceleration)
				{
					speed += reqiredAcceleration;
				} else
				{
					speed += MathUtil::sgn(reqiredAcceleration) * maxAcceleration;
				}
			}

			if(currentSteeringR > targetSteeringR && targetSteeringR < _maxSteeringR)
			{
				speed -= maxAcceleration;
			} else
			{
				speed += maxAcceleration;
			}

			if(speed < 0.01f)
			{
				speed = 0.01f;
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

			float deltaDelta = timeStep * 10.0f * 0.05f * (3 + 1.f / (10 * speed + 0.2f));

			// Angle between current direction and target direction
			float angleToTarget = (float) atan2(orient.x * normalizedPrefVel.y - orient.y * normalizedPrefVel.x, orient.x * normalizedPrefVel.x + orient.y * normalizedPrefVel.y);

			float targetDelta = atan(angleToTarget * agent._length / speed);

			float steer = targetDelta - agent._delta;

			// Actually steer now
			if(abs(steer) < deltaDelta)
			{
				agent._delta += steer;
			} else
			{
				agent._delta += MathUtil::sgn(steer) * deltaDelta;
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