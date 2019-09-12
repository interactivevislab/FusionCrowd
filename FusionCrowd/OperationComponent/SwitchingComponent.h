#pragma once

#include "Simulator.h"
#include "OperationComponent/IOperationComponent.h"
#include "Util/spimpl.h"
#include "ComponentId.h"

namespace FusionCrowd
{
	namespace SwitchingComp
	{
		class SwitchingComponent : public IOperationComponent
		{
		public:
			SwitchingComponent(std::shared_ptr<NavSystem> navSystem,
				std::shared_ptr<IOperationComponent> primaryComponent,
				std::shared_ptr<IOperationComponent> secondaryComponent);

			ComponentId GetId() override { return ComponentIds::SWITCHING; }

			void AddAgent(size_t id) override;
			bool DeleteAgent(size_t id) override;

			void Update(float timeStep) override;

			void SetNeighborsToSwitch(int neighborsToSwitch);

		private:
			class SwitchingComponentImpl;

			spimpl::unique_impl_ptr<SwitchingComponentImpl> pimpl;
		};
	}
}