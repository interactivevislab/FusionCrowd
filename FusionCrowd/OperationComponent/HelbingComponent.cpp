#include "HelbingComponent.h"
#include "Helbing/HelbingParameters.h"
#include "../Math/geomQuery.h"

namespace FusionCrowd
{
	namespace Helbing
	{
		HelbingComponent::HelbingComponent() : _mass(80)
		{
		}


		HelbingComponent::~HelbingComponent()
		{
		}

		void HelbingComponent::ComputeNewVelocity(FusionCrowd::Agent* agent)
		{
			Math::Vector2 force(DrivingForce(agent));

			for (size_t i = 0; i < agent->_nearAgents.size(); ++i) {
				const Agent * otherBase = agent->_nearAgents[i].agent;
				const Agent * const other = static_cast<const Agent *>(otherBase);

				force += AgentForce(agent, other);
			}

			for (size_t obs = 0; obs < agent->_nearObstacles.size(); ++obs) {
				const Obstacle * obst = agent->_nearObstacles[obs].obstacle;
				force += ObstacleForce(agent, obst);
			}

			Math::Vector2 acc = force / _mass;
			Math ::Vector2 r1Test = acc * 0.1f;
			agent->_velNew = agent->_vel + acc * 0.1f;
		}

		Math::Vector2 HelbingComponent::AgentForce(FusionCrowd::Agent* agent, const FusionCrowd::Agent * other) const
		{
			/* compute right of way */
			float rightOfWay = fabs(agent->_priority - other->_priority);
			if (rightOfWay >= 1.f) {
				rightOfWay = 1.f;
			}

			const float D = HelbingParameters::FORCE_DISTANCE;
			Math::Vector2 normal_ij = agent->_pos - other->_pos;
			float distance_ij = abs(normal_ij);
			normal_ij /= distance_ij;
			float Radii_ij = agent->_radius + other->_radius;

			float AGENT_SCALE = HelbingParameters::AGENT_SCALE;
			float D_AGT = D;

			// Right of way-dependent calculations
			// Compute the direction perpinduclar to preferred velocity (on the side
			//		of the normal force.

			Math::Vector2 avoidNorm(normal_ij);
			if (rightOfWay) {
				Math::Vector2 perpDir;
				if (agent->_priority < other->_priority) {
					// his advantage
					D_AGT += (rightOfWay * rightOfWay) * agent->_radius * .5f;	// Note: there is no symmetric reduction on
													// the other side
					// modify normal direction
					//	The perpendicular direction should always be in the direction that gets the
					//	agent out of the way as easily as possible
					float prefSpeed = other->_velPref.getSpeed();
					if (prefSpeed < 0.0001f) {
						// he wants to be stationary, accelerate perpinduclarly to displacement
						perpDir.set(-normal_ij.y(), normal_ij.x());
						if (perpDir * agent->_vel < 0.f) perpDir.negate();
					}
					else {
						// He's moving somewhere, accelerate perpindicularly to his preferred direction
						// of travel.
						const Math::Vector2 prefDir(other->_velPref.getPreferred());
						perpDir.set(-prefDir.y(), prefDir.x());	// perpendicular to preferred velocity
						if (perpDir * normal_ij < 0.f) perpDir.negate();
					}
					// spherical linear interpolation
					float sinTheta = det(perpDir, normal_ij);
					if (sinTheta < 0.f) {
						sinTheta = -sinTheta;
					}
					if (sinTheta > 1.f) {
						sinTheta = 1.f;	// clean up numerical error arising from determinant
					}
					avoidNorm.set(slerp(rightOfWay, normal_ij, perpDir, sinTheta));
				}
			}
			float mag = (AGENT_SCALE * expf((Radii_ij - distance_ij) / D_AGT));
			const float MAX_FORCE = 1e15f;
			if (mag >= MAX_FORCE) {
				mag = MAX_FORCE;
			}
			Math::Vector2 force(avoidNorm * mag);

			if (distance_ij < Radii_ij) {
				Math::Vector2 f_pushing(0.f, 0.f);
				Math::Vector2 f_friction(0.f, 0.f);
				// pushing
				Math::Vector2 tangent_ij(normal_ij.y(), -normal_ij.x());

				f_pushing = normal_ij * (HelbingParameters::BODY_FORCE * (Radii_ij - distance_ij));
				f_friction = tangent_ij * (HelbingParameters::FRICTION * (Radii_ij - distance_ij)) * fabs((other->_vel - agent->_vel) * tangent_ij);// / distance_ij;
				force += f_pushing + f_friction;
			}
			return force;
		}

		Math::Vector2 HelbingComponent::ObstacleForce(FusionCrowd::Agent* agent, const Obstacle * obst) const
		{
			const float D = HelbingParameters::FORCE_DISTANCE;
			const float OBST_MAG = HelbingParameters::OBST_SCALE;
			Math::Vector2 nearPt;	// set by distanceSqToPoint
			float distSq;	// set by distanceSqToPoint
			if (obst->distanceSqToPoint(agent->_pos, nearPt, distSq) ==
				Obstacle::LAST) return Math::Vector2(0.f, 0.f);
			float dist = sqrtf(distSq);
			Math::Vector2 forceDir((agent->_pos - nearPt) / dist);

			Math::Vector2 force = forceDir * (OBST_MAG * exp((agent->_radius - dist) / D));

			// pushing, friction
			if (dist < agent->_radius) { // intersection has occurred
				Math::Vector2 f_pushing(0.f, 0.f);
				Math::Vector2 f_friction(0.f, 0.f);

				Math::Vector2 tangent_io(forceDir.y(), -forceDir.x());

				// make sure direction is opposite i's velocity
				if ((tangent_io * agent->_vel) < 0.f) {
					tangent_io.negate();
				}

				f_pushing = forceDir * (HelbingParameters::BODY_FORCE * (agent->_radius - dist));

				// friction
				f_friction = tangent_io * HelbingParameters::FRICTION * (agent->_radius - dist) * (agent->_vel * tangent_io);
				force += f_pushing - f_friction;
			}
			return force;
		}

		Math::Vector2 HelbingComponent::DrivingForce(FusionCrowd::Agent* agent) const
		{
			return (agent->_velPref.getPreferredVel() - agent->_vel) * (_mass / HelbingParameters::REACTION_TIME);
		}

		void HelbingComponent::Update(FusionCrowd::Agent* agent, float timeStep)
		{
			ComputeNewVelocity(agent);

			float delV = abs(agent->_vel - agent->_velNew);

			if (delV > agent->_maxAccel * timeStep) {
				float w = agent->_maxAccel * timeStep / delV;
				agent->_vel = (1.f - w) *  agent->_vel + w * agent->_velNew;
			}
			else {
				agent->_vel = agent->_velNew;
			}
			Math::Vector2 t = agent->_vel * timeStep;

			agent->_pos += agent->_vel * timeStep;

			agent->UpdateOrient(timeStep);
			agent->PostUpdate();
		}
	}
}
