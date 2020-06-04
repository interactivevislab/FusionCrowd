#include "KaramouzasComponent.h"

#include "Math/consts.h"
#include "Math/geomQuery.h"
#include "Math/Util.h"

#include "Navigation/AgentSpatialInfo.h"
#include "Navigation/Obstacle.h"

#include <algorithm>
#include <list>
#include <iostream>
#include <map>

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	namespace Karamouzas
	{
#pragma region Impl
		class KaramouzasComponent::KaramouzasComponentImpl
		{
		public:
			KaramouzasComponentImpl(std::shared_ptr<NavSystem> navSystem):
				_navSystem(navSystem),
				_orientWeight(0.8f), _cosFOVAngle(cos(100.f * Math::DEG_TO_RAD)), _reactionTime(0.4f), _wallSteepness(2.f),
				_wallDistance(2.f), _collidingCount(5), _dMin(1.f), _dMid(8.f), _dMax(10.f), _agentForce(3.f)
			{
			}

			KaramouzasComponentImpl(std::shared_ptr<NavSystem> navSystem, float ORIENT_WEIGHT, float COS_FOV_ANGLE, float REACTION_TIME, float WALL_STEEPNESS, float WALL_DISTANCE, int COLLIDING_COUNT,
				float D_MIN, float D_MID, float D_MAX, float AGENT_FORCE)
				: _navSystem(navSystem)
			{
				_orientWeight = ORIENT_WEIGHT;
				_cosFOVAngle = COS_FOV_ANGLE;
				_reactionTime = REACTION_TIME;
				_wallSteepness = WALL_STEEPNESS;
				_wallDistance = WALL_DISTANCE;
				_collidingCount = COLLIDING_COUNT;
				_dMin = D_MIN;
				_dMid = D_MID;
				_dMax = D_MAX;
				_agentForce = AGENT_FORCE;
			}

			~KaramouzasComponentImpl() = default;

			void AddAgent(size_t id)
			{
				AddAgent(id, 1.0f, 3.0f);
			}

			void AddAgent(size_t id, float perSpace, float anticipation)
			{
				_agents[id] = AgentParamentrs(perSpace, anticipation);
				_navSystem->GetSpatialInfo(id).inertiaEnabled = true;
			}

			bool DeleteAgent(size_t id)
			{
				_agents.erase(id);
				return true;
			}

			void Update(float timeStep)
			{
				for(auto p : _agents)
				{
					auto id = p.first;
					AgentSpatialInfo & agent = _navSystem->GetSpatialInfo(id);
					ComputeNewVelocity(agent, timeStep);
				}
			}

		private:

			struct SpatialInfoWithCollisionTime
			{
				float tc;
				NeighborInfo info;
			};

			void ComputeNewVelocity(AgentSpatialInfo & agent, float timeStep)
			{
				const float EPSILON = 0.01f; // this eps from Ioannis
				const float FOV = _cosFOVAngle;

				Vector2 force((agent.prefVelocity.getPreferredVel() - agent.GetVel()) / _reactionTime);
				const float SAFE_DIST = _wallDistance + agent.radius;
				const float SAFE_DIST2 = SAFE_DIST * SAFE_DIST;

				for (auto const & obst : _navSystem->GetClosestObstacles(agent.id)) {
					// TODO: Interaction with obstacles is, currently, defined strictly
					//	by COLLISIONS.  Only if I'm going to collide with an obstacle is
					//	a force applied.  This may be too naive.
					//	I'll have to investigate this.
					Vector2 nearPt;		// set by distanceSqToPoint
					float sqDist;		// set by distanceSqToPoint
					if (obst.distanceSqToPoint(agent.GetPos(), nearPt, sqDist) == Obstacle::LAST) continue;
					if (SAFE_DIST2 > sqDist) {
						// A repulsive force is actually possible
						float dist = sqrtf(sqDist);
						float num = SAFE_DIST - dist;
						float distMradius = (dist - agent.radius) < EPSILON ? EPSILON : dist - agent.radius;
						float denom = powf(distMradius, _wallSteepness);
						Vector2 dir;
						(agent.GetPos() - nearPt).Normalize(dir);
						float mag = num / denom;
						force += dir * mag;
					}
				}

				Vector2 desVel = agent.GetVel() + force * timeStep;
				float desSpeed = desVel.Length();
				force = Vector2(0.f, 0.f);
				//#if 0
				//		// iteratively evaluate neighbors
				//#else
						// Weight all neighbors
				bool colliding = false;
				int collidingCount = _collidingCount;
				bool VERBOSE = false; // _id == 1;
				if (VERBOSE) std::cout << "Agent " << agent.id << "\n";
				float totalTime = 1.f;

				auto const & neighbours = _navSystem->GetNeighbours(agent.id);
				for (const auto & other : neighbours)
				{
					float circRadius = _agents[agent.id]._perSpace + other.radius;
					Vector2 relVel = desVel - other.vel;
					Vector2 relPos = other.pos - agent.GetPos();

					if (relPos.LengthSquared() < circRadius * circRadius) { ///collision!
						if (!colliding) {
							colliding = true;

							collidingSet.clear();
							collidingSet.reserve(neighbours.size());
						}
						//collidingSet.push({.0f, other});
						collidingSet.push_back({.0f, other});
						if (static_cast<int>(collidingSet.size()) > collidingCount) ++collidingCount;
						continue;
					}

					// TODO: evalute field of view
					//		If relPos is not within the field of view around preferred direction, continue
					Vector2 relDir;
					relPos.Normalize(relDir);
					if (relDir.Dot(agent.GetOrient()) < FOV) continue;
					float tc = Math::rayCircleTTC(relVel, relPos, circRadius);
					if (tc < _agents[agent.id]._anticipation && !colliding) {
						if (VERBOSE) std::cout << "\tAgent " << other.id << " t_c: " << tc << "\n";
						//totalTime += tc;
						// insert into colliding set (in order)

						/*
						auto itr = collidingSet.begin();
						while (itr != collidingSet.end() && tc > itr->tc) ++itr;
						collidingSet.insert(itr, {tc, other});
						*/
						collidingSet.push_back({tc, other});
					}
				}

				auto cmp = [](const SpatialInfoWithCollisionTime & left, const SpatialInfoWithCollisionTime & right) { return left.tc < right.tc; };
				std::sort(collidingSet.begin(), collidingSet.end(), cmp);

				int count = 0;
				for (auto itr = collidingSet.begin(); itr != collidingSet.end(); ++itr) {
					const auto& other = itr->info;
					float tc = itr->tc;
					// future positions
					Vector2 myPos = agent.GetPos() + desVel * tc;
					Vector2 hisPos = other.pos + other.vel * tc;
					Vector2 forceDir = myPos - hisPos;

					float futureDist = std::max(forceDir.Length(), EPSILON);
					forceDir /= futureDist;
					float collisionDist = futureDist - agent.radius - other.radius;
					float D = std::max(desSpeed * tc + (collisionDist < 0 ? 0 : collisionDist), EPSILON);

					// determine magnitude

					float mag = 0;
					if (D < _dMin) {
						mag = _agentForce * _dMin / D;
					}
					else if (D < _dMid) {
						mag = _agentForce;
					}
					else if (D < _dMax) {
						//D -= Simulator::D_MID;
						//mag =  D * Simulator::AGENT_FORCE / ( Simulator::D_MAX - Simulator::D_MID ) + Simulator::D_MID ;
						mag = _agentForce * (_dMax - D) / (_dMax - _dMid);
					}
					else {
						continue;	// magnitude is zero
					}
					float weight = pow(colliding ? 1.f : 0.8f, count++);
					//float weight = ( totalTime - tc  ) / totalTime; //1.f / ( tc * totalTime );
					if (VERBOSE) {
						std::cout << "\tAgent " << other.id;
						std::cout << " magnitude: " << mag;
						std::cout << " weight: " << weight;
						std::cout << " total force: " << (mag * weight);
						std::cout << " D: " << D << "\n";
					}
					force += forceDir * (mag * weight);
				}
				// Add some noise to avoid deadlocks and introduce variation
				//float angle = rand() * 2.0f * M_PI / RAND_MAX;
				float angle = rand() * 2.0f * 3.1415f / RAND_MAX;
				float dist = rand() * 0.001f / RAND_MAX;
				force += dist * Vector2(cos(angle), sin(angle));
				// do we need a drag force?

				 // Cap the force to maxAccel
				float forceL = force.Length();
				if (forceL > agent.maxAccel) {
					force /= forceL;
					force *= agent.maxAccel;
				}

				agent.velNew = desVel + force * timeStep;	// assumes unit mass
			}

			std::vector<SpatialInfoWithCollisionTime> collidingSet;
			std::shared_ptr<NavSystem> _navSystem;
			std::map<int, AgentParamentrs> _agents;
			float _orientWeight;
			float _cosFOVAngle;
			float _reactionTime;
			float _wallSteepness;
			float _wallDistance;
			int _collidingCount;
			float _dMin;
			float _dMid;
			float _dMax;
			float _agentForce;
		};

#pragma endregion

#pragma region Proxy methods
		KaramouzasComponent::KaramouzasComponent(std::shared_ptr<NavSystem> navSystem)
			: pimpl(spimpl::make_unique_impl<KaramouzasComponentImpl>(navSystem))
		{
		}

		KaramouzasComponent::KaramouzasComponent(std::shared_ptr<NavSystem> navSystem,
			float ORIENT_WEIGHT, float COS_FOV_ANGLE, float REACTION_TIME,
			float WALL_STEEPNESS, float WALL_DISTANCE, int COLLIDING_COUNT,
			float D_MIN, float D_MID, float D_MAX, float AGENT_FORCE)
			: pimpl(spimpl::make_unique_impl<KaramouzasComponentImpl>(navSystem, ORIENT_WEIGHT, COS_FOV_ANGLE, REACTION_TIME, WALL_STEEPNESS, WALL_DISTANCE, COLLIDING_COUNT, D_MIN, D_MID, D_MAX, AGENT_FORCE))
		{
		}

		void KaramouzasComponent::Update(float timeStep)
		{
			pimpl->Update(timeStep);
		}

		void KaramouzasComponent::AddAgent(size_t id, float perSpace, float anticipation)
		{
			pimpl->AddAgent(id, perSpace, anticipation);
		}

		void KaramouzasComponent::AddAgent(size_t id)
		{
			pimpl->AddAgent(id);
		}

		bool KaramouzasComponent::DeleteAgent(size_t id)
		{
			return pimpl->DeleteAgent(id);
		}
#pragma endregion
	}
}