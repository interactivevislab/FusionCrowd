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

		BicycleComponent::~BicycleComponent()
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

		double ToDegrees(double angel)
		{
			return angel * 180 / 3.1415926;
		}
		double ToRadian(float angel)
		{
			return angel / 180 * 3.1415926;
		}
		float mathAngel(Vector2 a, Vector2 b)
		{


			float scalarSum = a.x*b.x + a.y + b.y;

			float absA = sqrt(pow(a.x, 2) + pow(a.y, 2));
			float absB = sqrt(pow(b.x, 2) + pow(b.y, 2));

			float cosAngel = scalarSum / (absA*absB);
			return ToDegrees(acos(cosAngel));

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

		float LengthVector(Vector2 vect)
		{
			float l= sqrt(pow(vect.x, 2) + pow(vect.y, 2));

			return l;
		}

		Vector2 normalizeVector(Vector2 vect)
		{
			float inverseLength = 1 / LengthVector(vect);
			Vector2 norm = Vector2(vect.x*inverseLength, vect.y*inverseLength);
			return norm;
		}

		void BicycleComponent::ComputeNewVelocity(AgentSpatialInfo & agent, float timeStep)
		{
			float mult = timeStep * 10.0f;
			float acceleration = 0.01f * mult;
			float deltaDelta = 0.05 * mult;// *mult;//0.05f * (1 + 1.f / (10 * LengthVector(agent.vel) + 0.2f));

			float Angle=0;
			Vector2 prefVel;
			Vector2 normalizePrefVel;
			Vector2 orint;
			prefVel = agent.prefVelocity.getPreferredVel();
			if(LengthVector(prefVel) < 1e-6f)
				return;

			Vector2 vel = agent.vel;
			if (LengthVector(vel) != 0)
			{

				normalizePrefVel = normalizeVector(prefVel);

				orint.x = agent.orient.x;
				orint.y = agent.orient.y;

				Angle = (float)atan2(orint.x * normalizePrefVel.y - orint.y * normalizePrefVel.x, orint.x * normalizePrefVel.x + orint.y * normalizePrefVel.y);
				Angle = Angle * 180 / PI;

				if (Angle > 5)
				{
					if ( LengthVector( vel) > 0.5)
					{
						float speed = vel.Length();
						speed -= acceleration;
						if (speed < 0.0f) speed = 0.0f;
						vel.Normalize();
						vel = vel*speed;
					}
					if (_agents[agent.id]._delta < 0.5 )
					{
						_agents[agent.id]._delta += deltaDelta;
						//_agents[agent.id]._delta = _agents[agent.id]._delta > 0.5f ? 0.5f : _agents[agent.id]._delta;
					}
				}
				else if (Angle < -5)
				{
					if (LengthVector(vel )> 0.5)
					{
						float speed = vel.Length();
						speed -= acceleration;
						if (speed < 0.0f) speed = 0.0f;
						vel.Normalize();
						vel = vel*speed;
					}

					if (_agents[agent.id]._delta > -0.5 )
					{
						_agents[agent.id]._delta -= deltaDelta;
						//_agents[agent.id]._delta = _agents[agent.id]._delta < -0.5f ? -0.5f : _agents[agent.id]._delta;
					}

				}
				else
				{
					if (LengthVector(vel) < agent.maxSpeed && Vector2::Distance(agent.prefVelocity.getTarget(), agent.pos) > 10.0f )
					{
						float speed = vel.Length();
						speed += acceleration;
						if (speed > agent.maxSpeed) speed = agent.maxSpeed;
						vel.Normalize();
						vel = vel*speed;
					}
					if (Vector2::Distance(agent.prefVelocity.getTarget(), agent.pos) < 10.0f) {
						float speed = vel.Length();
						speed -= acceleration;
						if (speed < 0.5) speed = 0.5f;
						vel.Normalize();
						vel = vel * speed;

					}
					_agents[agent.id]._delta = 0.0f;

					_agents[agent.id]._theta = atan2(normalizePrefVel.y, normalizePrefVel.x);
				}



				for (auto const & obst : _navSystem->GetClosestObstacles(agent.id)) {

					Vector2 nearPt;
					float sqDist;
					float SAFE_DIST2 = 0.03;
					if (obst.distanceSqToPoint(agent.pos, nearPt, sqDist) == Obstacle::LAST) continue;
					if (SAFE_DIST2 > sqDist)
					{
						_agents[agent.id]._delta = 0.0f;
						_agents[agent.id]._theta = atan2(normalizePrefVel.y, normalizePrefVel.x);
					}
				}


				_agents[agent.id]._theta +=LengthVector(vel) * tan(_agents[agent.id]._delta) / _agents[agent.id]._length;

				agent.velNew.x = LengthVector(vel) * cos(_agents[agent.id]._theta);
				agent.velNew.y = LengthVector(vel) * sin(_agents[agent.id]._theta);


				_agents[agent.id]._orintX = cos(_agents[agent.id]._theta);
				_agents[agent.id]._orintY = sin(_agents[agent.id]._theta);
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