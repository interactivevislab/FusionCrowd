#pragma once

#include "Math/Util.h"
#include "Math/Line.h"
#include "AgentShape/Ellipse.h"
#include "OperationComponent/IOperationComponent.h"
#include "Navigation/NavSystem.h"
#include "Export/ComponentId.h"
#include "Navigation/NeighborInfo.h"
#include "Navigation/AgentSpatialInfo.h"

#include <map>
#include <memory>

namespace FusionCrowd
{
	namespace GCF
	{
		struct AgentParamentrs
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

		class GCFComponent : public IOperationComponent
		{
		public:
			GCFComponent(std::shared_ptr<NavSystem> navSystem);
			GCFComponent(std::shared_ptr<NavSystem> navSystem, float reactionTime, float nuAgent, float maxAgentDist, float maxAgentForse, float agentInterpWidth, bool speedColor);
			~GCFComponent();

			ComponentId GetId() override { return ComponentIds::GCF_ID; };
			void AddAgent(size_t id) override;
			bool DeleteAgent(size_t id) override;
			void Update(float timeStep) override;

			void ComputeNewVelocity(AgentSpatialInfo & agentInfo);


			void UpdateEllipse(const AgentSpatialInfo & agentInfo);
			DirectX::SimpleMath::Vector2 DriveForce(const AgentSpatialInfo & agentInfo) const;
			int GetRepulsionParameters(const AgentSpatialInfo & agent, const NeighborInfo & other,
				float& effDist, DirectX::SimpleMath::Vector2& forceDir,
				float& K_ij, float& response, float& velScale, float& magnitude) const;
			DirectX::SimpleMath::Vector2 ObstacleForce(const AgentSpatialInfo & agent, Obstacle & obst) const;
			float ComputeDistanceResponse(float effDist) const;

		private:
			std::shared_ptr<NavSystem> _navSystem;
			std::map<size_t, AgentParamentrs> _agents;
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
