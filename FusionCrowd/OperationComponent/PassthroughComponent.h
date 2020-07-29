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
	namespace PassthroughComp
	{

		class PassthroughComponent : public IOperationComponent
		{
		public:
			PassthroughComponent(std::shared_ptr<NavSystem> navSystem);

			ComponentId GetId() override { return ComponentIds::PASSTHROUGH_ID; };

			void AddAgent(size_t id) override;
			bool DeleteAgent(size_t id) override;
			void Update(float timeStep) override;

		private:
			std::shared_ptr<NavSystem> _navSystem;
			std::set<size_t> _agents;
		};
	}
}