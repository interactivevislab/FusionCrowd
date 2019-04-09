#pragma once
#include "../IOperComponent.h"
#include "../Agent.h"
#include "../Config.h"
#include "../MathUtil.h"
#include "../Math/Line.h"

#include <map>

namespace FusionCrowd
{
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

		class FUSION_CROWD_API PedVOComponent
			:
			public IOperComponent
		{
		public:
			PedVOComponent();
			PedVOComponent(float cosObstTurn, float sinObstTurn);
			~PedVOComponent();

			void ComputeNewVelocity(FusionCrowd::Agent* agent);
			void AdaptPreferredVelocity(FusionCrowd::Agent* agent);
			size_t ComputeORCALinesTurning(FusionCrowd::Agent* agent, DirectX::SimpleMath::Vector2& optVel, DirectX::SimpleMath::Vector2& prefDir,
				float& prefSpeed);
			void ObstacleLine(FusionCrowd::Agent* agent, size_t obstNbrID, const float invTau, bool flip);
			void AddAgent(int idAgent, float timeHorizon, float timeHorizonObst, float turningBias, bool denseAware, float factor, float buffer);
			void DeleteAgent(int idAgent);
			void Update(FusionCrowd::Agent* agent, float timeStep);

			bool LinearProgram1(const std::vector<FusionCrowd::Math::Line>& lines, size_t lineNo, float radius,
				const DirectX::SimpleMath::Vector2& optVelocity, bool directionOpt, float turnBias,
				DirectX::SimpleMath::Vector2& result);

			size_t LinearProgram2(const std::vector<FusionCrowd::Math::Line>& lines, float radius,
				const DirectX::SimpleMath::Vector2& optVelocity, bool directionOpt, float turnBias,
				DirectX::SimpleMath::Vector2& result);

			void LinearProgram3(const std::vector<FusionCrowd::Math::Line>& lines, size_t numObstLines,
				size_t beginLine, float radius, float turnBias, DirectX::SimpleMath::Vector2& result);

		private:
			std::map<int, AgentParamentrs> _agents;
			std::vector<FusionCrowd::Math::Line> _orcaLines;
			float _cosObstTurn;
			float _sinObstTurn;
			float _timeStep;
		};
	}
}
