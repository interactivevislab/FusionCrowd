#pragma once

#include "Simulator.h"
#include "OperationComponent/IOperationComponent.h"
#include "Util/spimpl.h"

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
			std::string GetName();

			void AddAgent(size_t id);
			bool DeleteAgent(size_t id);

			void Update(float timeStep);

			void SetNeighborsToSwitch(int neighborsToSwitch);

		private:
			class SwitchingComponentImpl;

			spimpl::unique_impl_ptr<SwitchingComponentImpl> pimpl;
		};
	}
}