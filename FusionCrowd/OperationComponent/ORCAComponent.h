#pragma once

#include <memory>
#include <set>

#include "Simulator.h"

#include "Agent.h"
#include "Math/Line.h"
#include "Math/Util.h"
#include "Navigation/Obstacle.h"
#include "Navigation/AgentSpatialInfo.h"
#include "Navigation/NeighborInfo.h"
#include "Navigation/NavSystem.h"

#include "OperationComponent/IOperationComponent.h"
#include "Export/ComponentId.h"

#include "Util/ctpl_stl.h"

namespace FusionCrowd
{
	namespace ORCA
	{
		class ORCAComponent : public IOperationComponent
		{
		private:
			struct EnvironmentArgs
			{
				float timeHorizon;
				float timeHorizonObst;
				float timeStep;

				const AgentSpatialInfo& info;
				std::vector<Obstacle> obstacles;
				std::vector<NeighborInfo> neighbors;
			};

		public:
			ORCAComponent(std::shared_ptr<NavSystem> navSystem);
			ORCAComponent(std::shared_ptr<NavSystem> navSystem, float timeHorizon, float timeHorizonObst);

			ComponentId GetId() override { return ComponentIds::ORCA_ID; }

			void AddAgent(size_t id) override;
			bool DeleteAgent(size_t id) override;

			void Update(float timeStep) override;

		private:
			static DirectX::SimpleMath::Vector2 ComputeNewVelocity(EnvironmentArgs args);

			static size_t ComputeORCALines(std::vector<Math::Line>& _orcaLines, EnvironmentArgs & args);
			static void ObstacleLine(std::vector<Math::Line>& _orcaLines, Obstacle & obst, const float invTau, bool flip, const AgentSpatialInfo& info);

			static bool LinearProgram1(
				const std::vector<Math::Line>& lines, size_t lineNo,
				float radius, const DirectX::SimpleMath::Vector2 & optVelocity, bool directionOpt,
				DirectX::SimpleMath::Vector2& result
			);

			static size_t LinearProgram2(
				const std::vector<FusionCrowd::Math::Line>& lines,
				float radius, const DirectX::SimpleMath::Vector2& optVelocity, bool directionOpt,
				DirectX::SimpleMath::Vector2& result
			);

			static void LinearProgram3(
				const std::vector<Math::Line>& lines,
				size_t numObstLines, size_t beginLine, float radius,
				DirectX::SimpleMath::Vector2& result
			);

		private:
			float _timeHorizon;
			float _timeHorizonObst;

			std::shared_ptr<NavSystem> _navSystem;
			std::set<size_t> _agents;

			ctpl::thread_pool _pool;
		};
	}
}
