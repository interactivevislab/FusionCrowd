//#include "HelbingComponent.h"
//#include "Helbing/HelbingParameters.h"
//#include "Math/geomQuery.h"
//
//using namespace DirectX::SimpleMath;
//
//namespace FusionCrowd
//{
//	namespace Helbing
//	{
//		HelbingComponent::HelbingComponent() : _mass(80)
//		{
//		}
//
//
//		HelbingComponent::~HelbingComponent()
//		{
//		}
//
//		void HelbingComponent::ComputeNewVelocity(FusionCrowd::Agent* agent)
//		{
//			Vector2 force(DrivingForce(agent));
//
//			for (size_t i = 0; i < agent->_nearAgents.size(); ++i) {
//				const Agent * otherBase = agent->_nearAgents[i].agent;
//				const Agent * const other = static_cast<const Agent *>(otherBase);
//
//				force += AgentForce(agent, other);
//			}
//
//			for (size_t obs = 0; obs < agent->_nearObstacles.size(); ++obs) {
//				const Obstacle * obst = agent->_nearObstacles[obs].obstacle;
//				force += ObstacleForce(agent, obst);
//			}
//
//			Vector2 acc = force / _mass;
//			Vector2 r1Test = acc * 0.1f;
//			agent->_velNew = agent->_vel + acc * 0.1f;
//		}
//
//		Vector2 HelbingComponent::AgentForce(FusionCrowd::Agent* agent, const FusionCrowd::Agent * other) const
//		{
//			/* compute right of way */
//			float rightOfWay = fabs(agent->_priority - other->_priority);
//			if (rightOfWay >= 1.f) {
//				rightOfWay = 1.f;
//			}
//
//			const float D = HelbingParameters::FORCE_DISTANCE;
//			Vector2 normal_ij = agent->_pos - other->_pos;
//			float distance_ij = normal_ij.Length();
//			normal_ij /= distance_ij;
//			float Radii_ij = agent->_radius + other->_radius;
//
//			float AGENT_SCALE = HelbingParameters::AGENT_SCALE;
//			float D_AGT = D;
//
//			// Right of way-dependent calculations
//			// Compute the direction perpinduclar to preferred velocity (on the side
//			//		of the normal force.
//
//			Vector2 avoidNorm(normal_ij);
//			if (rightOfWay) {
//				Vector2 perpDir;
//				if (agent->_priority < other->_priority) {
//					// his advantage
//					D_AGT += (rightOfWay * rightOfWay) * agent->_radius * .5f;	// Note: there is no symmetric reduction on
//													// the other side
//					// modify normal direction
//					//	The perpendicular direction should always be in the direction that gets the
//					//	agent out of the way as easily as possible
//					float prefSpeed = other->_velPref.getSpeed();
//					if (prefSpeed < 0.0001f) {
//						// he wants to be stationary, accelerate perpinduclarly to displacement
//						perpDir = Vector2(-normal_ij.y, normal_ij.x);
//						if (perpDir.Dot(agent->_vel) < 0.f)
//							perpDir *= -1;
//					}
//					else {
//						// He's moving somewhere, accelerate perpindicularly to his preferred direction
//						// of travel.
//						const Vector2 prefDir(other->_velPref.getPreferred());
//						perpDir = Vector2(-prefDir.y, prefDir.x);	// perpendicular to preferred velocity
//						if (perpDir.Dot(normal_ij) < 0.f)
//							perpDir *= -1;
//					}
//					// spherical linear interpolation
//					float sinTheta = MathUtil::det(perpDir, normal_ij);
//					if (sinTheta < 0.f) {
//						sinTheta = -sinTheta;
//					}
//					if (sinTheta > 1.f) {
//						sinTheta = 1.f;	// clean up numerical error arising from determinant
//					}
//					avoidNorm = Math::slerp(rightOfWay, normal_ij, perpDir, sinTheta);
//				}
//			}
//			float mag = (AGENT_SCALE * expf((Radii_ij - distance_ij) / D_AGT));
//			const float MAX_FORCE = 1e15f;
//			if (mag >= MAX_FORCE) {
//				mag = MAX_FORCE;
//			}
//			Vector2 force(avoidNorm * mag);
//
//			if (distance_ij < Radii_ij) {
//				Vector2 f_pushing(0.f, 0.f);
//				Vector2 f_friction(0.f, 0.f);
//				// pushing
//				Vector2 tangent_ij(normal_ij.y, -normal_ij.x);
//
//				f_pushing = normal_ij * (HelbingParameters::BODY_FORCE * (Radii_ij - distance_ij));
//				f_friction = tangent_ij * (HelbingParameters::FRICTION * (Radii_ij - distance_ij)) * fabs((other->_vel - agent->_vel).Dot(tangent_ij));// / distance_ij;
//				force += f_pushing + f_friction;
//			}
//			return force;
//		}
//
//		Vector2 HelbingComponent::ObstacleForce(FusionCrowd::Agent* agent, const Obstacle * obst) const
//		{
//			const float D = HelbingParameters::FORCE_DISTANCE;
//			const float OBST_MAG = HelbingParameters::OBST_SCALE;
//			Vector2 nearPt;	// set by distanceSqToPoint
//			float distSq;	// set by distanceSqToPoint
//			if (obst->distanceSqToPoint(agent->_pos, nearPt, distSq) ==
//				Obstacle::LAST) return Vector2(0.f, 0.f);
//			float dist = sqrtf(distSq);
//			Vector2 forceDir((agent->_pos - nearPt) / dist);
//
//			Vector2 force = forceDir * (OBST_MAG * exp((agent->_radius - dist) / D));
//
//			// pushing, friction
//			if (dist < agent->_radius) { // intersection has occurred
//				Vector2 f_pushing(0.f, 0.f);
//				Vector2 f_friction(0.f, 0.f);
//
//				Vector2 tangent_io(forceDir.y, -forceDir.x);
//
//				// make sure direction is opposite i's velocity
//				if (tangent_io.Dot(agent->_vel) < 0.f) {
//					tangent_io *= -1;
//				}
//
//				f_pushing = forceDir * (HelbingParameters::BODY_FORCE * (agent->_radius - dist));
//
//				// friction
//				f_friction = tangent_io * HelbingParameters::FRICTION * (agent->_radius - dist) * (agent->_vel * tangent_io);
//				force += f_pushing - f_friction;
//			}
//			return force;
//		}
//
//		Vector2 HelbingComponent::DrivingForce(FusionCrowd::Agent* agent) const
//		{
//			return (agent->_velPref.getPreferredVel() - agent->_vel) * (_mass / HelbingParameters::REACTION_TIME);
//		}
//
//		void HelbingComponent::Update(FusionCrowd::Agent* agent, float timeStep)
//		{
//			ComputeNewVelocity(agent);
//
//			float delV = (agent->_vel - agent->_velNew).Length();
//
//			if (delV > agent->_maxAccel * timeStep) {
//				float w = agent->_maxAccel * timeStep / delV;
//				agent->_vel = (1.f - w) *  agent->_vel + w * agent->_velNew;
//			}
//			else {
//				agent->_vel = agent->_velNew;
//			}
//			Vector2 t = agent->_vel * timeStep;
//
//			agent->_pos += agent->_vel * timeStep;
//
//			agent->UpdateOrient(timeStep);
//			agent->PostUpdate();
//		}
//	}
//}

//namespace FusionCrowd
//{
//	namespace Helbing
//	{
//		HelbingComponent::HelbingComponent(Simulator & simulator)
//		{
//
//		}
//	}
//}

#include "HelbingComponent.h"

#include "Math/consts.h"
#include "Math/geomQuery.h"
#include "Math/Util.h"

#include "Navigation/AgentSpatialInfo.h"

#include <algorithm>
#include <list>
#include <iostream>


using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	namespace Helbing
	{
		HelbingComponent::HelbingComponent(Simulator & simulator): _simulator(simulator), _navSystem(simulator.GetNavSystem()), _agentScale(2000.f), _obstScale(2000.f), _reactionTime(0.5f), _bodyForse(1.2e5f), _friction(2.4e5f), _forceDistance(0.08f)
		{
		}

		HelbingComponent::HelbingComponent(Simulator & simulator, float AGENT_SCALE, float OBST_SCALE, float REACTION_TIME, float BODY_FORCE, float FRICTION, float FORCE_DISTANCE):
			_simulator(simulator), _navSystem(simulator.GetNavSystem()), _agentScale(AGENT_SCALE), _obstScale(OBST_SCALE), _reactionTime(REACTION_TIME), _bodyForse(BODY_FORCE), _friction(FRICTION), _forceDistance(FORCE_DISTANCE)
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
				AgentSpatialInfo & agent = _simulator.GetNavSystem().GetSpatialInfo(id);
				ComputeNewVelocity(agent, timeStep);
			}
		}

		void HelbingComponent::ComputeNewVelocity(AgentSpatialInfo & agent, float timeStep)
		{
			Vector2 force(DrivingForce(&agent));
			std::vector<AgentSpatialInfo> nearAgents = _navSystem.GetNeighbours(agent.id);
			for (size_t i = 0; i < nearAgents.size(); ++i)
			{
				//const AgentSpatialInfo* otherBase = nearAgents[i];
				AgentSpatialInfo other = nearAgents[i];

				force += AgentForce(&agent, &other);
			}

			for (auto obst : _navSystem.GetClosestObstacles(agent.id)) {
				force += ObstacleForce(&agent, &obst);
			}
			Vector2 acc = force / _agents[agent.id]._mass;
			agent.velNew = agent.vel + acc * timeStep;
		}

		Vector2 HelbingComponent::AgentForce(AgentSpatialInfo* agent, AgentSpatialInfo * other) const
		{
			/* compute right of way */
			//float rightOfWay = fabs(agent->_priority - other->_priority);
			//if (rightOfWay >= 1.f) {
			//	rightOfWay = 1.f;
			//}

			float rightOfWay = 0.0f;

			const float D = _forceDistance;
			Vector2 normal_ij = agent->pos - other->pos;
			float distance_ij = normal_ij.Length();
			normal_ij /= distance_ij;
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
			//		float sinTheta = MathUtil::det(perpDir, normal_ij);
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
				f_friction = tangent_ij * (_friction * (Radii_ij - distance_ij)) * fabs((other->vel - agent->vel).Dot(tangent_ij));// / distance_ij;
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
			auto & agentInfo = _simulator.GetNavSystem().GetSpatialInfo(agent->id);
			return (agentInfo.prefVelocity.getPreferredVel() - agent->vel) * (_agents[agent->id]._mass / _reactionTime);
		}

		void HelbingComponent::AddAgent(size_t id, float mass)
		{
			_agents[id] = AgentParamentrs(mass);
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