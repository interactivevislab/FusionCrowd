#include "TransportOperationComponent.h"

#include "Navigation/NavSystem.h"
#include "TacticComponent/NavGraph/NavGraphComponent.h"
#include "Navigation/AgentSpatialInfo.h"
#include <iostream>

using namespace  DirectX::SimpleMath;

namespace FusionCrowd
{
	namespace Transport
	{
#pragma region Impl
		class TransportOperationComponent::TransportOperationComponentImpl
		{

		public:
			TransportOperationComponentImpl(std::shared_ptr<NavSystem> navSystem) :
				_navSystem(navSystem), _navGraph(navSystem->GetNavGraph())
			{
			}

			~TransportOperationComponentImpl() = default;

			void AddAgent(size_t id)
			{
				_agents.insert(id);
				_navSystem->GetSpatialInfo(id).inertiaEnabled = false;
			}

			bool DeleteAgent(size_t id)
			{
				_agents.erase(id);

				return true;
			}

			void Update(float timeStep)
			{
				for (size_t agentId : _agents)
				{
					ComputeNewVelocity(agentId, timeStep);
				}
			}

			void ComputeNewVelocity(size_t agentId, float timeStep)
			{
				float acceleration = 0.0f;
				int forwardAgtId = GetForwardAgent(agentId);
				AgentSpatialInfo & curAgentInfo = _navSystem->GetSpatialInfo(agentId);
				float speed = curAgentInfo.prefVelocity.getSpeed();
				float safeDist = curAgentInfo.radius * 6;
				float angleSpeed = 45;

				if (forwardAgtId > 0) //move acording forward-moving agent
				{
					AgentSpatialInfo & otherAgentInfo = _navSystem->GetSpatialInfo(forwardAgtId);

					float d = curAgentInfo.GetPos().Distance(curAgentInfo.GetPos(), otherAgentInfo.GetPos());
					float dv = speed - otherAgentInfo.prefVelocity.getSpeed();
					if (d > safeDist)
					{
						acceleration = dv * dv / (2*(d - safeDist));
						acceleration = dv >= -0.1 ? (-1.0 * acceleration) : 0.5f;
					}

					else
					{
						acceleration = -1 * curAgentInfo.maxAccel;
						curAgentInfo.prefSpeed = otherAgentInfo.prefVelocity.getSpeed();
					}			
				}

				else //increase speed if there no forward agent
				{
					acceleration = 0.5f;
				}


				Vector2 previousVel = curAgentInfo.GetVel();
				Vector2 newVel = curAgentInfo.prefVelocity.getPreferredVel();

				int avoidAgt = GetForwardAgentToAvoid(agentId);
				float avoidAngle = 0.5;

				if (avoidAgt > 0 && avoidAgt != forwardAgtId) //turn right to avoid agents (except lead agent)
				{
					AgentSpatialInfo & avoidAgentInfo = _navSystem->GetSpatialInfo(avoidAgt);

					if (newVel.Length() > 1e-5)
					{

						newVel.x = curAgentInfo.prefVelocity.getPreferredVel().x * cos(avoidAngle) -
							curAgentInfo.prefVelocity.getPreferredVel().y * sin(avoidAngle);
						newVel.y = curAgentInfo.prefVelocity.getPreferredVel().x * sin(avoidAngle) +
							curAgentInfo.prefVelocity.getPreferredVel().y * cos(avoidAngle);
					}

					acceleration = -0.3f;

				}

				
				float deltaAngle = acos(previousVel.Dot(newVel) / (newVel.Length() * previousVel.Length() + 1e-6))*180/3.14;
				if (abs(deltaAngle) > angleSpeed*timeStep)
				{
					newVel = previousVel.Lerp(previousVel, newVel, (angleSpeed*timeStep / abs(deltaAngle)));//smooth turn with angleSpeed
				}

				curAgentInfo.prefSpeed += acceleration * timeStep;

				curAgentInfo.prefSpeed = Clamp(curAgentInfo.prefSpeed, curAgentInfo.maxSpeed);

				curAgentInfo.velNew = newVel;

				
			}


			int GetForwardAgent(size_t curAgentId)
			{
				std::vector<size_t> nearestAgents = GetAllAgentsInRadius(curAgentId, 2.0f);
				AgentSpatialInfo & curAgentInfo = _navSystem->GetSpatialInfo(curAgentId);
				float minDist = INFINITY;
				int retID = -1;
				float dotProduct;
				for (auto agentID : nearestAgents)
				{
					AgentSpatialInfo & info = _navSystem->GetSpatialInfo(agentID);
					dotProduct = info.GetOrient().Dot(curAgentInfo.GetOrient());

					if (dotProduct > 0.8f)
					{
						DirectX::SimpleMath::Vector2 distV = info.GetPos() - curAgentInfo.GetPos();
						distV.Normalize();
						float d = curAgentInfo.GetOrient().Dot(distV);
						if (d > 0.3f)
						{
							distV = info.GetPos() - curAgentInfo.GetPos();

							if (distV.Length() < minDist)
							{
								minDist = distV.Length();
								retID = agentID;
							}
						}
					}
				}

				return retID;
			}

			int GetForwardAgentToAvoid(size_t curAgentId)
			{
				std::vector<size_t> nearestAgents = GetAllAgentsInRadius(curAgentId, 2.0f);
				AgentSpatialInfo & curAgentInfo = _navSystem->GetSpatialInfo(curAgentId);
				float minDist = INFINITY;
				int retID = -1;
				float l;
				for (auto agentID : nearestAgents)
				{
					AgentSpatialInfo & info = _navSystem->GetSpatialInfo(agentID);
					DirectX::SimpleMath::Vector2 distV = info.GetPos() - curAgentInfo.GetPos();
					distV.Normalize();
					float d = curAgentInfo.GetOrient().Dot(distV);
					if (d > 0.9f)
					{
						distV = info.GetPos() - curAgentInfo.GetPos();
						if (distV.Length() < minDist)
						{
							minDist = distV.Length();
							retID = agentID;
						}
					}
				}
				return retID;

			}

			std::vector<size_t> GetAllAgentsInRadius(size_t curAgentId, float radius)
			{
				std::vector<size_t> ret;
				AgentSpatialInfo & curAgentInfo = _navSystem->GetSpatialInfo(curAgentId);
				for (auto & agtId : _agents)
				{
					AgentSpatialInfo & info = _navSystem->GetSpatialInfo(agtId);
					float dist = info.GetPos().Distance(info.GetPos(), curAgentInfo.GetPos());

					if (dist < radius && curAgentId != agtId)
					{
						ret.push_back(agtId);
					}
				}

				return ret;
			}

			float Clamp(float curSpeed, float maxSpeed)
			{
				if (curSpeed > maxSpeed)
				{
					return maxSpeed;
				}

				else if (curSpeed < 1e-6)
				{
					return 1e-6;
				}

				else
				{
					return curSpeed;
				}
			}

		private:

			std::shared_ptr<NavSystem> _navSystem;
			std::shared_ptr<NavGraph> _navGraph;
			std::set<size_t> _agents;
		};
#pragma endregion

#pragma region proxy methods
		TransportOperationComponent::TransportOperationComponent(std::shared_ptr<NavSystem> navSystem)
			: pimpl(spimpl::make_unique_impl<TransportOperationComponentImpl>(navSystem))
		{
		}

		void TransportOperationComponent::AddAgent(size_t id)
		{
			pimpl->AddAgent(id);
		}

		bool TransportOperationComponent::DeleteAgent(size_t id)
		{
			return pimpl->DeleteAgent(id);
		}

		void TransportOperationComponent::Update(float timeStep)
		{
			pimpl->Update(timeStep);
		}
	}
#pragma endregion
}