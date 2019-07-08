/*#pragma once
#include "OperationComponent/IOperationComponent.h"
#include "Agent.h"
#include "Config.h"
#include "Math/Util.h"
#include "Math/Line.h"
#include "AgentShape/Ellipse.h"

#include <map>

namespace FusionCrowd
{
	namespace GCF
	{
		struct FUSION_CROWD_API AgentParamentrs
		{
			AgentShape::Ellipse _ellipse;
			float _aMin;
			float _aRate;
			float _bMax;
			float _bGrowth;

			AgentParamentrs(): _ellipse()
			{
				_aMin = 0.18f;
				_aRate = 0.53f;
				_bMax = 0.25f;
				_bGrowth = 0.05f;
			}

			AgentParamentrs(float aMin, float aRate, float bMax, float bGrowth) : _ellipse()
			{
				_aMin = aMin;
				_aRate = aRate;
				_bMax = bMax;
				_bGrowth = bGrowth;
			}

		};

		class FUSION_CROWD_API GCFComponent
		{
		public:
			GCFComponent();
			GCFComponent(float reactionTime, float nuAgent, float maxAgentDist, float maxAgentForse, float agentInterpWidth, bool speedColor);
			~GCFComponent();

			void AddAgent(FusionCrowd::Agent* agent, float mass);
			void DeleteAgent(int idAgent);

			void Update(FusionCrowd::Agent* agent, float timeStep);
			void ComputeNewVelocity(FusionCrowd::Agent* agent);


			void UpdateEllipse(FusionCrowd::Agent* agent);
			DirectX::SimpleMath::Vector2 DriveForce(FusionCrowd::Agent* agent) const;
			int GetRepulsionParameters(FusionCrowd::Agent* agent, const Agent* other, float& effDist, DirectX::SimpleMath::Vector2& forceDir,
				float& K_ij, float& response, float& velScale, float& magnitude) const;
			DirectX::SimpleMath::Vector2 ObstacleForce(FusionCrowd::Agent* agent, const Obstacle* obst) const;
			float ComputeDistanceResponse(float effDist) const;

		private:
			std::map<int, AgentParamentrs> _agents;
			float _timeStep;

			float _reactionTime;
			float _nuAgent;
			float _maxAgentDist;
			float _maxAgentForse;
			float _agentInterpWidth;
			bool _speedColor;
		};
	}
}

*/