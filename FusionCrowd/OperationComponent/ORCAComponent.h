#pragma once

#include <set>

#include "Agent.h"
#include "Config.h"
#include "Simulator.h"
#include "Math/Line.h"
#include "OperationComponent/IOperationComponent.h"

namespace FusionCrowd
{
	class Obstacle;

	namespace ORCA
	{
		class FUSION_CROWD_API ORCAComponent :
			public IOperationComponent
		{
		public:
			ORCAComponent(Simulator & simulator);
			ORCAComponent(Simulator & simulator, float timeHorizon, float timeHorizonObst);
			void ComputeNewVelocity(size_t agentId);
			size_t ComputeORCALines(size_t agentId);
			void ObstacleLine(Obstacle & obstacle, const float invTau, bool flip, size_t agentId);

			bool LinearProgram1(const std::vector<FusionCrowd::Math::Line>& lines, size_t lineNo,
				float radius, const DirectX::SimpleMath::Vector2 & optVelocity,
				bool directionOpt, DirectX::SimpleMath::Vector2& result);

			size_t LinearProgram2(const std::vector<FusionCrowd::Math::Line>& lines, float radius,
				const DirectX::SimpleMath::Vector2& optVelocity, bool directionOpt,
				DirectX::SimpleMath::Vector2& result);

			void LinearProgram3(const std::vector<FusionCrowd::Math::Line>& lines, size_t numObstLines,
				size_t beginLine, float radius, DirectX::SimpleMath::Vector2& result);

			std::string GetName() { return "orca"; };

			void AddAgent(size_t id);
			bool DeleteAgent(size_t id);

			void Update(float timeStep);


			~ORCAComponent();

		private:
			float _timeHorizon;
			float _timeHorizonObst;

			Simulator & _simulator;

			std::vector<FusionCrowd::Math::Line> _orcaLines;
			std::set<size_t> _agents;
		};
	}
}
