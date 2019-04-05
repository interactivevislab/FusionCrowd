#pragma once
#include "../IOperComponent.h"
#include "../Agent.h"
#include "../Config.h"

#include <map>

namespace FusionCrowd
{
	namespace Karamouzas
	{
		struct FUSION_CROWD_API AgentParamentrs
		{
			float _perSpace;
			float _anticipation;

			AgentParamentrs() :_perSpace(0.69f), _anticipation(8.f)
			{
			}
			AgentParamentrs(float perSpace, float anticipation) :_perSpace(perSpace), _anticipation(anticipation)
			{
			}
		};

		class FUSION_CROWD_API KaramouzasComponent :
			public IOperComponent
		{
		public:
			KaramouzasComponent();
			KaramouzasComponent(float ORIENT_WEIGHT, float COS_FOV_ANGLE, float REACTION_TIME, float WALL_STEEPNESS, float WALL_DISTANCE, int COLLIDING_COUNT,
				float D_MIN, float D_MID, float D_MAX, float AGENT_FORCE);
			~KaramouzasComponent();
			void ComputeNewVelocity(FusionCrowd::Agent* agent);
			void AddAgent(int idAgent, float perSpace, float anticipation);
			void DeleteAgent(int idAgent);
			void Update(FusionCrowd::Agent* agent, float timeStep);

		private:
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
	}
}

