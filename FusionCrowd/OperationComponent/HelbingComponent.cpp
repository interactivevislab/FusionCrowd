#include "HelbingComponent.h"

#include "Math/consts.h"
#include "Math/geomQuery.h"
#include "Math/Util.h"

#include "Navigation/Obstacle.h"

#include <algorithm>
#include <list>
#include <iostream>


using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	namespace Helbing
	{
		HelbingComponent::HelbingComponent(std::shared_ptr<NavSystem> navSystem)
			: _navSystem(navSystem), _agentScale(2000.f), _obstScale(2000.f), _reactionTime(0.5f), _bodyForse(1.2e5f), _friction(2.4e5f), _forceDistance(0.08f)
		{
		}

		HelbingComponent::HelbingComponent(std::shared_ptr<NavSystem> navSystem, float AGENT_SCALE, float OBST_SCALE, float REACTION_TIME, float BODY_FORCE, float FRICTION, float FORCE_DISTANCE):
			_navSystem(navSystem), _agentScale(AGENT_SCALE), _obstScale(OBST_SCALE), _reactionTime(REACTION_TIME), _bodyForse(BODY_FORCE), _friction(FRICTION), _forceDistance(FORCE_DISTANCE)
		{
		}

		HelbingComponent::~HelbingComponent()
		{
		}

		void HelbingComponent::Update(float timeStep)
		{
			for (auto p : _agents)
			{
				auto id = p.first;
				AgentSpatialInfo & agent = _navSystem->GetSpatialInfo(id);
				ComputeNewVelocity(agent, timeStep);
			}
		}

		void HelbingComponent::ComputeNewVelocity(AgentSpatialInfo & agent, float timeStep)
		{
			Vector2 force(DrivingForce(&agent));
			std::vector<NeighborInfo> nearAgents(_navSystem->GetNeighbours(agent.id));
			for (size_t i = 0; i < nearAgents.size(); ++i)
			{
				NeighborInfo other = nearAgents.at(i);

				force += AgentForce(&agent, &other);
			}

			for (auto obst : _navSystem->GetClosestObstacles(agent.id)) {
				force += ObstacleForce(&agent, &obst);
			}
			Vector2 acc = force / _agents[agent.id]._mass;
			agent.velNew = agent.GetVel() + acc * timeStep;
		}

		Vector2 HelbingComponent::AgentForce(AgentSpatialInfo* agent, NeighborInfo * other) const
		{
			/* compute right of way */
			//float rightOfWay = fabs(agent->_priority - other->_priority);
			//if (rightOfWay >= 1.f) {
			//	rightOfWay = 1.f;
			//}

			float rightOfWay = 0.0f;

			const float D = _forceDistance;
			Vector2 normal_ij = agent->GetPos() - other->pos;
			float distance_ij = normal_ij.Length();
			normal_ij.Normalize();

			float Radii_ij = agent->radius + other->radius;

			float AGENT_SCALE = _agentScale;
			float D_AGT = D;

			// Right of way-dependent calculations
			// Compute the direction perpinduclar to preferred velocity (on the side
			//		of the normal force.

			Vector2 avoidNorm(normal_ij);
			//if (rightOfWay) {
			//	Vector2 perpDir;
			//	if (agent->_priority < other->_priority) {
			//		// his advantage
			//		D_AGT += (rightOfWay * rightOfWay) * agent->radius * .5f;	// Note: there is no symmetric reduction on
			//										// the other side
			//		// modify normal direction
			//		//	The perpendicular direction should always be in the direction that gets the
			//		//	agent out of the way as easily as possible
			//		float prefSpeed = other->_velPref.getSpeed();
			//		if (prefSpeed < 0.0001f) {
			//			// he wants to be stationary, accelerate perpinduclarly to displacement
			//			perpDir = Vector2(-normal_ij.y, normal_ij.x);
			//			if (perpDir.Dot(agent->vel) < 0.f)
			//				perpDir *= -1;
			//		}
			//		else {
			//			// He's moving somewhere, accelerate perpindicularly to his preferred direction
			//			// of travel.
			//			const Vector2 prefDir(other->_velPref.getPreferred());
			//			perpDir = Vector2(-prefDir.y, prefDir.x);	// perpendicular to preferred velocity
			//			if (perpDir.Dot(normal_ij) < 0.f)
			//				perpDir *= -1;
			//		}
			//		// spherical linear interpolation
			//		float sinTheta = Math::det(perpDir, normal_ij);
			//		if (sinTheta < 0.f) {
			//			sinTheta = -sinTheta;
			//		}
			//		if (sinTheta > 1.f) {
			//			sinTheta = 1.f;	// clean up numerical error arising from determinant
			//		}
			//		avoidNorm = Math::slerp(rightOfWay, normal_ij, perpDir, sinTheta);
			//	}
			//}
			float mag = (AGENT_SCALE * expf((Radii_ij - distance_ij) / D_AGT));
			const float MAX_FORCE = 1e15f;
			if (mag >= MAX_FORCE) {
				mag = MAX_FORCE;
			}
			Vector2 force(avoidNorm * mag);

			if (distance_ij < Radii_ij) {
				Vector2 f_pushing(0.f, 0.f);
				Vector2 f_friction(0.f, 0.f);
				// pushing
				Vector2 tangent_ij(normal_ij.y, -normal_ij.x);

				f_pushing = normal_ij * (_bodyForse * (Radii_ij - distance_ij));
				f_friction = tangent_ij * (_friction * (Radii_ij - distance_ij)) * fabs((other->vel - agent->GetVel()).Dot(tangent_ij));// / distance_ij;
				force += f_pushing + f_friction;
			}
			return force;
		}

		Vector2 HelbingComponent::ObstacleForce(AgentSpatialInfo* agent, Obstacle * obst) const
		{
			return Vector2::Zero;
		}

		Vector2 HelbingComponent::DrivingForce(AgentSpatialInfo* agent)
		{
			auto & agentInfo = _navSystem->GetSpatialInfo(agent->id);
			return (agentInfo.prefVelocity.getPreferredVel() - agent->GetVel()) * (_agents[agent->id]._mass / _reactionTime);
		}

		void HelbingComponent::AddAgent(size_t id, float mass)
		{
			_agents[id] = AgentParamentrs(mass);
			_navSystem->GetSpatialInfo(id).inertiaEnabled = false;
		}

		void HelbingComponent::AddAgent(size_t id)
		{
			AddAgent(id, 80.0f);
		}

		bool HelbingComponent::DeleteAgent(size_t id)
		{
			_agents.erase(id);
			return true;
		}
	}
}