#pragma once

#include "Agent.h"
#include "Simulator.h"
#include "Navigation/NavSystem.h"
#include "OperationComponent/IOperationComponent.h"
#include "Export/ComponentId.h"
#include "Math/Util.h"

#include <map>

namespace FusionCrowd
{
	namespace StrictComp
	{
		struct AgentParamentrs
		{
			float _acceleration;
			float _negativeAcceleration;

			AgentParamentrs() : _acceleration(0.5f), _negativeAcceleration(0.5f)
			{
			}
		};

		class StrictComponent : public IOperationComponent
		{
		public:
			StrictComponent(std::shared_ptr<NavSystem> navSystem);

			ComponentId GetId() override { return ComponentIds::STRICT_ID; };

			void AddAgent(size_t id) override;
			bool DeleteAgent(size_t id) override;
			void Update(float timeStep) override;

		private:
			std::shared_ptr<NavSystem> _navSystem;
			std::map<int, AgentParamentrs> _agents;
			void ComputeNewVelocity(AgentSpatialInfo & spatialInfo, float timeStep);
		};
	}
}