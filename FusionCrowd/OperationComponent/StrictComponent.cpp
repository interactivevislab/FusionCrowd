#include "StrictComponent.h"

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
	namespace StrictComp
	{
		StrictComponent::StrictComponent(std::shared_ptr<NavSystem> navSystem) : _navSystem(navSystem)
		{
		}

		void StrictComponent::Update(float timeStep)
		{
			for (auto p : _agents)
			{
				auto id = p.first;
				AgentSpatialInfo & agent = _navSystem->GetSpatialInfo(id);
				ComputeNewVelocity(agent, timeStep);
			}
		}

		void StrictComponent::AddAgent(size_t id)
		{
			_agents[id] = AgentParamentrs();
			_navSystem->GetSpatialInfo(id).inertiaEnabled = true;
		}

		bool StrictComponent::DeleteAgent(size_t id)
		{
			_agents.erase(id);
			return true;
		}

		void StrictComponent::ComputeNewVelocity(AgentSpatialInfo & spatialInfo, float timeStep)
		{
			auto neighbours = _navSystem->GetNeighbours(spatialInfo.id);
			const float maxAcceleration = spatialInfo.maxAccel * timeStep;

			AgentParamentrs & agent = _agents[spatialInfo.id];

			float distanceToTarget = Vector2::Distance(spatialInfo.prefVelocity.getTarget(), spatialInfo.GetPos());
			Vector2 prefVel = spatialInfo.prefVelocity.getPreferredVel();
			Vector2 normalizedPrefVel = spatialInfo.prefVelocity.getPreferredVel();
			normalizedPrefVel.Normalize();
			Vector2 orient = spatialInfo.GetOrient();

			if (prefVel.Length() < 1e-6f) return;

			Vector2 vel = spatialInfo.GetVel();
			// Angle between current direction and target direction
			float angleToTarget = (float)atan2(orient.x * normalizedPrefVel.y - orient.y * normalizedPrefVel.x, orient.x * normalizedPrefVel.x + orient.y * normalizedPrefVel.y);


			float speed = vel.Length();

			//speed = spatialInfo.prefSpeed;

			if (speed < 0.05f)
			{
				speed = 0.05f;
			}

			if (distanceToTarget > 5.0f && speed < spatialInfo.prefSpeed && neighbours.size() < 10) {
				speed += maxAcceleration;
				if (speed > spatialInfo.prefSpeed) speed = spatialInfo.prefSpeed;
			}
			if (distanceToTarget < 5.0f && speed > 0.3f) {
			}

			if (distanceToTarget < 1e-2f) {
				speed = 0.0f;
			}
			float inCrowdSpeed = spatialInfo.prefSpeed * 0.3f;
			if (neighbours.size() > 5 && speed > inCrowdSpeed) {
				speed -= maxAcceleration;
			}

			spatialInfo.velNew =Vector2(normalizedPrefVel.x * speed, normalizedPrefVel.y * speed);

		}
	}
}