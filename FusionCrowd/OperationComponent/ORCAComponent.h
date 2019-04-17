#pragma once
#include "IOperationComponent.h"
#include "Agent.h"
#include "Config.h"
#include "Math/Line.h"

namespace FusionCrowd
{
	namespace ORCA
	{
		class FUSION_CROWD_API ORCAComponent :
			public IOperationComponent
		{
		public:
			ORCAComponent();
			ORCAComponent(float timeHorizon, float timeHorizonObst);
			void ComputeNewVelocity(FusionCrowd::Agent* agent);
			size_t ComputeORCALines(FusionCrowd::Agent* agent);
			void ObstacleLine(size_t obstNbrID, const float invTau, bool flip, FusionCrowd::Agent* agent);

			bool LinearProgram1(const std::vector<FusionCrowd::Math::Line>& lines, size_t lineNo,
				float radius, const DirectX::SimpleMath::Vector2 & optVelocity,
				bool directionOpt, DirectX::SimpleMath::Vector2& result);
			size_t LinearProgram2(const std::vector<FusionCrowd::Math::Line>& lines, float radius,
				const DirectX::SimpleMath::Vector2& optVelocity, bool directionOpt,
				DirectX::SimpleMath::Vector2& result);
			void LinearProgram3(const std::vector<FusionCrowd::Math::Line>& lines, size_t numObstLines,
				size_t beginLine, float radius, DirectX::SimpleMath::Vector2& result);

			void Update(FusionCrowd::Agent* agent, float timeStep);
			~ORCAComponent();

		private:
			float _timeHorizon;
			float _timeHorizonObst;

			std::vector<FusionCrowd::Math::Line> _orcaLines;

		};
	}
}
