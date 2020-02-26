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
		BicycleComponent::BicycleComponent(std::shared_ptr<NavSystem> navSystem) : _navSystem(navSystem), _agentScale(2000.f), _obstScale(2000.f), _reactionTime(0.5f), _bodyForse(1.2e5f), _friction(2.4e5f), _forceDistance(0.08f)
		{
		}

		BicycleComponent::BicycleComponent(std::shared_ptr<NavSystem> navSystem, float AGENT_SCALE, float OBST_SCALE, float REACTION_TIME, float BODY_FORCE, float FRICTION, float FORCE_DISTANCE) :
			_navSystem(navSystem), _agentScale(AGENT_SCALE), _obstScale(OBST_SCALE), _reactionTime(REACTION_TIME), _bodyForse(BODY_FORCE), _friction(FRICTION), _forceDistance(FORCE_DISTANCE)
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

		void dump(AgentSpatialInfo & agent)
		{
			std::cout << step<<"\n";
			std::cout << "orient" << agent.orient.x << ',' << agent.orient.y<<"\n";
			std::cout << "prefSpeed" << agent.prefSpeed<<"\n";
			std::cout << "prefVel" << agent.prefVelocity.getPreferredVel().x << ',' << agent.prefVelocity.getPreferredVel().y << "\n";
			std::cout << "Vel" << agent.vel.x << ',' << agent.vel.y << "\n";
			std::cout << "NewVel" << agent.velNew.x << ',' << agent.velNew.y << "\n";
		}

		void BicycleComponent::ComputeNewVelocity(AgentSpatialInfo & agent, float timeStep)
		{
			Vector2 normalizedPrefVel;
			Vector2 prefVel = agent.prefVelocity.getPreferredVel();
			prefVel.Normalize(normalizedPrefVel);

			if(prefVel.Length() < 1e-6f)
				return;

			float angle = 0;
			auto velLength = agent.vel.Length();
			auto newVel = agent.vel;
			auto & agentParams = _agents[agent.id];

			if (velLength != 0)
			{
				Vector2 orient(agent.orient.x, agent.orient.y);

				angle = atan2f(orient.x * normalizedPrefVel.y - orient.y * normalizedPrefVel.x, orient.x * normalizedPrefVel.x + orient.y * normalizedPrefVel.y);
				angle *= 180 / MathUtil::PI;

				// magic good-loking function, no physical rationale behind
				float deltaDelta = 0.05f * (3.f + 1.f / (3 * velLength + 0.2f));
				float acceleration = 1.0f;

				float prefTurningVelocity = 0.5f;

				float softBreakAngle = 10.0f;
				float hardBreakAngle = 3.0f;

				if(abs(angle) > softBreakAngle && velLength > 1.5f * prefTurningVelocity)
				{
					acceleration = (1.5f * prefTurningVelocity - velLength) / timeStep;
				}
				if(abs(angle) >= hardBreakAngle && abs(angle) <= softBreakAngle && velLength > prefTurningVelocity)
				{
					acceleration = (prefTurningVelocity - velLength) / timeStep;
				}
				if(abs(angle) < hardBreakAngle && velLength < prefVel.Length())
				{
					acceleration = (prefVel.Length() - velLength) / timeStep;
				}

				if(abs(acceleration) > agent.maxAccel)
				{
					acceleration = acceleration / abs(acceleration) * agent.maxAccel;
				}

				newVel *= (velLength + acceleration * timeStep) / velLength;

				if (angle > softBreakAngle && agentParams._delta < 0.5)
				{
					agentParams._delta += deltaDelta;
				}
				else if (angle < -softBreakAngle && agentParams._delta > -0.5)
				{
					agentParams._delta -= deltaDelta;
				}
				else
				{
					agentParams._delta = 0.0f;
					agentParams._theta = atan2(normalizedPrefVel.y, normalizedPrefVel.x);
				}

				for (auto const & obst : _navSystem->GetClosestObstacles(agent.id))
				{
					Vector2 nearPt;
					float sqDist;
					float SAFE_DIST2 = 0.03;
					if (obst.distanceSqToPoint(agent.pos, nearPt, sqDist) == Obstacle::LAST) continue;
					if (SAFE_DIST2 > sqDist)
					{
						agentParams._delta = 0.0f;
						agentParams._theta = atan2(normalizedPrefVel.y, normalizedPrefVel.x);
						//std::cout << "Collisonh step: " << step << "\n";
					}
				}

				agentParams._theta += newVel.Length() * tan(agentParams._delta) / agentParams._length;
				agentParams._orintX = cos(agentParams._theta);
				agentParams._orintY = sin(agentParams._theta);

				agent.velNew.x = newVel.Length() * cos(agentParams._theta);
				agent.velNew.y = newVel.Length() * sin(agentParams._theta);

				step++;
			}
			else
			{
				agent.velNew.x = agent.orient.x;
				agent.velNew.y = agent.orient.y;
			}
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