#include "bicycleComponent.h"

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
	namespace bicycle
	{
		int step = 0;
		bicycleComponent::bicycleComponent(std::shared_ptr<NavSystem> navSystem) : _navSystem(navSystem), _agentScale(2000.f), _obstScale(2000.f), _reactionTime(0.5f), _bodyForse(1.2e5f), _friction(2.4e5f), _forceDistance(0.08f)
		{
		}

		bicycleComponent::bicycleComponent(std::shared_ptr<NavSystem> navSystem, float AGENT_SCALE, float OBST_SCALE, float REACTION_TIME, float BODY_FORCE, float FRICTION, float FORCE_DISTANCE) :
			_navSystem(navSystem), _agentScale(AGENT_SCALE), _obstScale(OBST_SCALE), _reactionTime(REACTION_TIME), _bodyForse(BODY_FORCE), _friction(FRICTION), _forceDistance(FORCE_DISTANCE)
		{
		}

		bicycleComponent::~bicycleComponent()
		{
		}

		void bicycleComponent::Update(float timeStep)
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



		void bicycleComponent::ComputeNewVelocity(AgentSpatialInfo & agent, float timeStep)
		{
			

			float Angel=0;
			Vector2 prefVel;
			Vector2 normalizePrefVel;
			Vector2 orint;
			float maxRul;
			prefVel = agent.prefVelocity.getPreferredVel();



				if(LengthVector(prefVel) != 0.0f)
				{

				
				if (LengthVector(agent.vel) != 0)
				{

					normalizePrefVel = normalizeVector(prefVel);

					orint.x = agent.orient.x;
					orint.y = agent.orient.y;

					Angel = (float)atan2(orint.x * normalizePrefVel.y - orint.y * normalizePrefVel.x, orint.x * normalizePrefVel.x + orint.y * normalizePrefVel.y);
					Angel = Angel * 180 / PI;




					maxRul = 0.5 / (1 + LengthVector(agent.vel));

					if (Angel > 5)
					{
						if ( LengthVector( agent.vel) > 0.5)
						{
							agent.vel = agent.vel / 1.02;
						}
						if (_agents[agent.id]._delta < 0.5 )
						{
							_agents[agent.id]._delta += 0.05f;
						
					
						}
					}
					else if (Angel < -5)
					{
						if (LengthVector(agent.vel )> 0.5)
						{
							agent.vel = agent.vel / 1.02;
						}
										
						if (_agents[agent.id]._delta > -0.5 )
						{							
							_agents[agent.id]._delta -= 0.05f;
						}
						
					}
					else
					{
						if (LengthVector(agent.vel) < 2)
						{
							agent.vel = agent.vel*1.01;
						}
						_agents[agent.id]._delta = 0.0f;
						
						_agents[agent.id]._theta = atan2(normalizePrefVel.y, normalizePrefVel.x);
							/*if (_agents[agent.id]._delta < -0.1f)
							{
								_agents[agent.id]._delta += 0.1f;
							}
							else if (_agents[agent.id]._delta > 0.1f)
							{
								_agents[agent.id]._delta -= 0.08f;
							}*/
							/*else
							{
								_agents[agent.id]._delta = 0.0f;
								
							}*/
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
							//std::cout << "Collisonh step: " << step << "\n";
						
						}
					}
					

					_agents[agent.id]._theta +=LengthVector(agent.vel) * tan(_agents[agent.id]._delta) / _agents[agent.id]._length;
					
					agent.velNew.x = LengthVector(agent.vel) * cos(_agents[agent.id]._theta);
					agent.velNew.y = LengthVector(agent.vel) * sin(_agents[agent.id]._theta);


					_agents[agent.id]._orintX = cos(_agents[agent.id]._theta);
					_agents[agent.id]._orintY = sin(_agents[agent.id]._theta);
					step++;
					

				}
	
				else
				{
					agent.velNew.x =agent.orient.x;
					agent.velNew.y = agent.orient.y;
				}
				
				
				}
			


				//dump(agent);
				/*
				std::cout << "step " << step <<"\n";
				std::cout << "++++++++++++++++++++++++++++++";
				std::cout << "prefVel "  << prefVel.x << "--" << prefVel.y << "\n";
				//std::cout << "normalizePrefVel " << normalizePrefVel.x << "--" << normalizePrefVel.y << "\n";
				//std::cout << "maxRUL " << maxRul << "\n";
				//std::cout << "orientMY " << orint.x << "--" << orint.y << "\n";
				std::cout << "orient " <<agent.orient.x << "--" << agent.orient.y << "\n";
				std::cout << "angel " << Angel << "\n";	
				std::cout << "vel " << LengthVector(agent.vel) << "\n";
				std::cout << "delta " << _agents[agent.id]._delta << "\n";
				//std::cout << "theta " << _agents[agent.id]._theta << "\n";
				std::cout << "pos " << agent.pos.x << "--" << agent.pos.y << "\n";
				std::cout << "target "<<agent.prefVelocity.getTarget().x<<"--"<<agent.prefVelocity.getTarget().y << "\n";
				std::cout << "distance to target " <<Vector2::Distance(agent.prefVelocity.getTarget(),agent.pos) << "\n";
				std::cout << "____________________________\n\n\n";
				getchar();*/
			
		

		}
	


		void bicycleComponent::AddAgent(size_t id, float mass)
		{
			_agents[id] = AgentParamentrs();
			_navSystem->GetSpatialInfo(id).inertiaEnabled = true;
		}

		void bicycleComponent::AddAgent(size_t id)
		{
			AddAgent(id, 80.0f);
		}

		bool bicycleComponent::DeleteAgent(size_t id)
		{
			_agents.erase(id);
			return true;
		}
	}
}