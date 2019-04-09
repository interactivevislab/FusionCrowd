#pragma once
#include "../Agent.h"
#include "../Config.h"
#include "../IOperComponent.h"

#include <map>

namespace FusionCrowd
{
	namespace Zanlungo
	{
		struct FUSION_CROWD_API AgentParamentrs
		{
			float _mass;

			AgentParamentrs() :_mass(80)
			{
			}
			AgentParamentrs(float mass) :_mass(mass)
			{
			}
		};

		class FUSION_CROWD_API ZanlungoComponent :
			public IOperComponent
		{
		public:
			ZanlungoComponent();
			ZanlungoComponent(float agentScale, float obstScale, float reactionTime, float forceDistance);
			~ZanlungoComponent();

			void AddAgent(int idAgent, float mass);
			void DeleteAgent(int idAgent);

			void Update(FusionCrowd::Agent* agent, float timeStep);
			void ComputeNewVelocity(FusionCrowd::Agent* agent);
			bool ComputeTTI(FusionCrowd::Agent* agent, float& T_i) const;
			DirectX::SimpleMath::Vector2 AgentForce(FusionCrowd::Agent* agent, const FusionCrowd::Agent* other, float T_i) const;

			float RightOfWayVel(FusionCrowd::Agent* agent, DirectX::SimpleMath::Vector2& otherVel, const DirectX::SimpleMath::Vector2& otherPrefVel,
				float otherPriority, DirectX::SimpleMath::Vector2& vel) const;

		private:
			std::map<int, AgentParamentrs> _agents;
			float _agentScale;
			float _obstScale;
			float _reactionTime;
			float _forceDistance;
			float _timeStep;
		};
	}
}

