#include "ZanlungoComponent.h"
#include "../Math/vector.h"
#include "../Math/consts.h"
#include "../Math/geomQuery.h"

namespace FusionCrowd
{
	namespace Zanlungo
	{
		ZanlungoComponent::ZanlungoComponent() :
			_agentScale(2000), _obstScale(2000), _reactionTime(0.5f), _forceDistance(0.08f)
		{
		}


		ZanlungoComponent::ZanlungoComponent(float agentScale, float obstScale, float reactionTime, float forceDistance) :
			_agentScale(agentScale), _obstScale(obstScale), _reactionTime(reactionTime), _forceDistance(forceDistance)
		{
		}

		ZanlungoComponent::~ZanlungoComponent()
		{
		}

		void ZanlungoComponent::AddAgent(int idAgent, float mass)
		{
			_agents[idAgent] = AgentParamentrs(mass);
		}

		void ZanlungoComponent::DeleteAgent(int idAgent)
		{
			_agents.erase(idAgent);
		}

		void ZanlungoComponent::Update(FusionCrowd::Agent* agent, float timeStep)
		{
			_timeStep = timeStep;
			ComputeNewVelocity(agent);

			float delV = FusionCrowd::Math::abs(agent->_vel - agent->_velNew);

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

		void ZanlungoComponent::ComputeNewVelocity(FusionCrowd::Agent* agent)
		{
			float T_i;
			bool interacts = ComputeTTI(agent, T_i);

			const float TAU = _reactionTime;
			FusionCrowd::Math::Vector2 force((agent->_velPref.getPreferredVel() - agent->_vel) * _agents[agent->_id]._mass / TAU);

			if (interacts) {
				// if T_i never got set, there are no interactions to do
				const float SPEED = abs(agent->_vel);
				const float B = _forceDistance;
				// const float MAG = Simulator::AGENT_SCALE * SPEED / T_i;
				for (size_t j = 0; j < agent->_nearAgents.size(); ++j) {
					// 2. Use T_i to compute the direction
					const FusionCrowd::Agent* otherBase = agent->_nearAgents[j].agent;
					const FusionCrowd::Agent* const other = static_cast<const Agent*>(otherBase);
					force += AgentForce(agent, other, T_i);
				}
				// obstacles
				FusionCrowd::Math::Vector2 futurePos = agent->_pos + agent->_vel * T_i;
				const float OBST_MAG = _obstScale * SPEED / T_i;
				for (size_t obs = 0; obs < agent->_nearObstacles.size(); ++obs) {
					FusionCrowd::Math::Vector2 nearPt;  // set by call to distanceSqToPoint
					float d2;        // set by call to distanceSqToPoint
					if (agent->_nearObstacles[obs].obstacle->distanceSqToPoint(futurePos, nearPt, d2) == Obstacle::LAST)
						continue;
					FusionCrowd::Math::Vector2 D_ij = futurePos - nearPt;
					float dist = abs(D_ij);
					D_ij /= dist;
					dist -= agent->_radius;
					force += D_ij * (OBST_MAG * expf(-dist / B));
				}
			}

			FusionCrowd::Math::Vector2 acc = force / _agents[agent->_id]._mass;
			agent->_velNew = agent->_vel + acc * _timeStep;
		}

		bool ZanlungoComponent::ComputeTTI(FusionCrowd::Agent* agent, float& T_i) const
		{
			const float COS_FOV = -0.8f;  // cos( HALFPI );// cos( PI / 4.f ); //
			bool interacts = false;
			T_i = INFTY;
#define COLLIDE_PRIORITY
#ifdef COLLIDE_PRIORITY
			float t_collision = T_i;
#endif
			for (size_t j = 0; j < agent->_nearAgents.size(); ++j) {
				const FusionCrowd::Agent* otherBase = agent->_nearAgents[j].agent;
				const FusionCrowd::Agent* const other = static_cast<const Agent*>(otherBase);

				// Right of way-dependent calculations
				FusionCrowd::Math::Vector2 myVel = agent->_vel;
				FusionCrowd::Math::Vector2 hisVel = other->_vel;
				RightOfWayVel(agent, hisVel, other->_velPref.getPreferredVel(), other->_priority, myVel);

				const FusionCrowd::Math::Vector2 relVel = myVel - hisVel;
				FusionCrowd::Math::Vector2 relPos = agent->_pos - other->_pos;
				//	This define determines if additional prediction code is executed
				//		The original zanlungo model does not include performing exact collisions
				//		between disks.  It simply estimates the time to interaction based on
				//		projection of center on preferred velocity.
				//		This define includes a precise test of ray-circle intersection to test
				//		to see if the relative velocity intersects the minkowski sum of this
				//		agent with its neighbor.  It makes the respones far more robust.
#define PRECISE
#ifdef PRECISE
				float circRadius = agent->_radius + other->_radius;
				// first test to see if there's an actual collision imminent
				float contactT = rayCircleTTC(relVel, -relPos, circRadius);
				// std::cout << "\tColliding with " << other->_id << " at " << contactT << "\n";
#ifdef COLLIDE_PRIORITY
				if (contactT < t_collision) {
					// the ray intersects the circle -- actual collision is possible
					t_collision = contactT;
					interacts = true;
				}
				else if (t_collision == INFTY) {
#else
				if (contactT < T_i) {
					// the ray intersects the circle -- actual collision is possible
					T_i = contactT;
					interacts = true;
				}
				else {
#endif  // COLLIDE_PRIORITY
#endif  // PRECISE
					// no collision possible, see if getting close is possible
#ifdef ZANLUNGO_FOV
					float dist = abs(relPos);
					float relSpeed = abs(relVel);
					// This is the angle between relative position and ORIENTATION
					float cosTheta = -(relPos * _orient) / dist;
					if (cosTheta >= COS_FOV) {
						// This is the angle between relative position and relative velocity
						cosTheta = relPos * relVel / (-dist * relSpeed);
						float t_ij = cosTheta * dist / relSpeed;
						// std::cout << "\tApproaching " << other->_id << " at " << t_ij << "\n";
						if (t_ij < T_i) {
							T_i = t_ij;
							interacts = true;
						}
					}
#else
	// note: relPos points from other agent to this agent, so they need to point in
	// OPPOSITE directions for convergence
					float dp = -(relPos * relVel);
					if (dp > 0.f) {
						float t_ij = dp / absSq(relVel);
						if (t_ij < T_i) {
							T_i = t_ij;
							interacts = true;
						}
					}
#endif
#ifdef PRECISE
				}
#endif
			}
			// Compute time to interaction for obstacles
			for (size_t obs = 0; obs < agent->_nearObstacles.size(); ++obs) {
				// TODO: Interaction with obstacles is, currently, defined strictly
				//	by COLLISIONS.  Only if I'm going to collide with an obstacle is
				//	a force applied.  This may be too naive.
				//	I'll have to investigate this.
				float t = agent->_nearObstacles[obs].obstacle->circleIntersection(agent->_vel, agent->_pos, agent->_radius);
				if (t < T_i) {
					T_i = t;
					interacts = true;
				}
			}
#ifdef COLLIDE_PRIORITY
			if (t_collision < INFTY) T_i = t_collision;
#endif
			if (T_i < _timeStep) {
				T_i = _timeStep;
			}

			return interacts;
		}

		float ZanlungoComponent::RightOfWayVel(FusionCrowd::Agent* agent, FusionCrowd::Math::Vector2& otherVel, const FusionCrowd::Math::Vector2& otherPrefVel,
			float otherPriority, FusionCrowd::Math::Vector2& vel) const
		{
			float rightOfWay = agent->_priority - otherPriority;
			rightOfWay = (rightOfWay < -1.f) ? -1.f : (rightOfWay > 1.f) ? 1.f : rightOfWay;
			if (rightOfWay < 0.f) {
				float R2 = sqrtf(-rightOfWay);  // rightOfWay * rightOfWay; // -rightOfWay; //
				vel = agent->_vel;
				otherVel += R2 * (otherPrefVel - otherVel);
				return -R2;
			}
			else if (rightOfWay > 0.f) {
				float R2 = sqrtf(rightOfWay);  // rightOfWay * rightOfWay; // rightOfWay; //
				vel = agent->_vel + R2 * (agent->_velPref.getPreferredVel() - agent->_vel);
				return R2;
			}
			else {
				vel = agent->_vel;
				return 0.f;
			}
		}

		FusionCrowd::Math::Vector2 ZanlungoComponent::AgentForce(FusionCrowd::Agent* agent, const Agent* other, float T_i) const
		{
			float D = _forceDistance;
			// Right of way-dependent calculations
			FusionCrowd::Math::Vector2 myVel = agent->_vel;
			FusionCrowd::Math::Vector2 hisVel = other->_vel;
			float weight =
				1.f - RightOfWayVel(agent, hisVel, other->_velPref.getPreferredVel(), other->_priority, myVel);

			const FusionCrowd::Math::Vector2 relVel = myVel - hisVel;

			FusionCrowd::Math::Vector2 futPos = agent->_pos + myVel * T_i;
			FusionCrowd::Math::Vector2 otherFuturePos = other->_pos + hisVel * T_i;
			FusionCrowd::Math::Vector2 D_ij = futPos - otherFuturePos;

			// If the relative velocity is divergent do nothing
			if (D_ij * (agent->_vel - other->_vel) > 0.f) return FusionCrowd::Math::Vector2(0.f, 0.f);
			float dist = abs(D_ij);
			D_ij /= dist;
			if (weight > 1.f) {
				// Other agent has right of way
				float prefSpeed = other->_velPref.getSpeed();
				FusionCrowd::Math::Vector2 perpDir;
				bool interpolate = true;
				if (prefSpeed < 0.0001f) {
					// he wants to be stationary, accelerate perpinduclarly to displacement
					FusionCrowd::Math::Vector2 currRelPos = agent->_pos - other->_pos;
					perpDir.set(-currRelPos.y(), currRelPos.x());
					if (perpDir * agent->_vel < 0.f) perpDir.negate();
				}
				else {
					// He's moving somewhere, accelerate perpindicularly to his preferred direction
					// of travel.
					const FusionCrowd::Math::Vector2 prefDir(other->_velPref.getPreferred());
					if (prefDir * D_ij > 0.f) {
						// perpendicular to preferred velocity
						perpDir.set(-prefDir.y(), prefDir.x());
						if (perpDir * D_ij < 0.f) perpDir.negate();
					}
					else {
						interpolate = false;
					}
				}
				// spherical linear interpolation
				if (interpolate) {
					float sinTheta = det(perpDir, D_ij);
					if (sinTheta < 0.f) {
						sinTheta = -sinTheta;
					}
					if (sinTheta > 1.f) {
						sinTheta = 1.f;  // clean up numerical error arising from determinant
					}
					D_ij.set(slerp(weight - 1.f, D_ij, perpDir, sinTheta));
				}
			}
			dist -= (agent->_radius + other->_radius);
			float magnitude = weight * _agentScale * abs(agent->_vel - other->_vel) / T_i;
			const float MAX_FORCE = 1e15f;
			if (magnitude >= MAX_FORCE) {
				magnitude = MAX_FORCE;
			}
			// float magnitude = weight * Simulator::AGENT_SCALE * abs( myVel - hisVel ) / T_i;
			// 3. Compute the force
			return D_ij * (magnitude * expf(-dist / D));
		}
	}
}
