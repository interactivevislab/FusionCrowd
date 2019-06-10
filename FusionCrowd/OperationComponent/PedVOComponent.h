#pragma once

#include "Simulator.h"
#include "Agent.h"
#include "Config.h"

#include "OperationComponent/IOperationComponent.h"

#include "Math/Util.h"
#include "Math/Line.h"

#include <map>

namespace FusionCrowd
{
	class AgentSpatialInfo;

	namespace PedVO
	{
		struct FUSION_CROWD_API AgentParamentrs
		{
			bool _denseAware;
			float _strideConst;
			float _speedConst;
			float _timeHorizon;
			float _timeHorizonObst;
			float _turningBias;

			float STRIDE_FACTOR;
			float STRIDE_BUFFER;

			AgentParamentrs()
			{
				STRIDE_FACTOR = 1.57f;
				STRIDE_BUFFER = 0.5f;

				_timeHorizon = 2.5f;
				_timeHorizonObst = 0.15f;
				_turningBias = 1.0f;;
				_denseAware = true;
				SetStrideParameters(STRIDE_FACTOR, STRIDE_BUFFER);
			}

			AgentParamentrs(float timeHorizon, float timeHorizonObst, float turningBias, bool denseAware, float factor, float buffer)
			{
				STRIDE_FACTOR = factor;
				STRIDE_BUFFER = buffer;

				_timeHorizon = timeHorizon;
				_timeHorizonObst = timeHorizonObst;
				_turningBias = turningBias;
				_denseAware = denseAware;
				SetStrideParameters(STRIDE_FACTOR, STRIDE_BUFFER);
			}

			void SetStrideParameters(float factor, float buffer)
			{
				_strideConst = 0.5f * (1.f + buffer) / factor;
				_speedConst = 1.f / (_strideConst * _strideConst);
			}
		};

		class FUSION_CROWD_API PedVOComponent : public IOperationComponent
		{
		public:
			PedVOComponent(Simulator & simulator);
			PedVOComponent(Simulator & simulator, float cosObstTurn, float sinObstTurn);
			~PedVOComponent();

			std::string GetName() { return "pedvo"; };
			void AddAgent(size_t agentId);
			bool DeleteAgent(size_t idAgent);
			void Update(float timeStep);

			void ComputeNewVelocity(AgentParamentrs & agentParams, AgentSpatialInfo & agentInfo);

			void AdaptPreferredVelocity(AgentParamentrs & agentParams, AgentSpatialInfo & agentInfo);
			size_t ComputeORCALinesTurning(AgentParamentrs & agentParams, AgentSpatialInfo & agentInfo, DirectX::SimpleMath::Vector2& optVel, DirectX::SimpleMath::Vector2& prefDir,
				float& prefSpeed);
			void ObstacleLine(AgentParamentrs & agentParams, AgentSpatialInfo & agentInfo, Obstacle & obst, const float invTau, bool flip);

			void AddAgent(size_t agentId, float timeHorizon, float timeHorizonObst, float turningBias, bool denseAware, float factor, float buffer);

			bool LinearProgram1(const std::vector<FusionCrowd::Math::Line>& lines, size_t lineNo, float radius,
				const DirectX::SimpleMath::Vector2& optVelocity, bool directionOpt, float turnBias,
				DirectX::SimpleMath::Vector2& result);

			size_t LinearProgram2(const std::vector<FusionCrowd::Math::Line>& lines, float radius,
				const DirectX::SimpleMath::Vector2& optVelocity, bool directionOpt, float turnBias,
				DirectX::SimpleMath::Vector2& result);

			void LinearProgram3(const std::vector<FusionCrowd::Math::Line>& lines, size_t numObstLines,
				size_t beginLine, float radius, float turnBias, DirectX::SimpleMath::Vector2& result);

		private:
			Simulator & _simulator;

			std::map<size_t, AgentParamentrs> _agents;
			std::vector<FusionCrowd::Math::Line> _orcaLines;
			float _cosObstTurn;
			float _sinObstTurn;
			float _timeStep;
		};
	}
}
