#pragma once

#include "Simulator.h"
#include "OperationComponent/IOperationComponent.h"

namespace FusionCrowd
{
	namespace SwitchingComp
	{
		class FUSION_CROWD_API SwitchingComponent : public IOperationComponent
		{
		public:
			SwitchingComponent(Simulator & simulator,
				std::shared_ptr<IOperationComponent> primaryComponent,
				std::shared_ptr<IOperationComponent> secondaryComponent);
			std::string GetName();

			void AddAgent(size_t id);
			bool DeleteAgent(size_t id);

			void Update(float timeStep);
			~SwitchingComponent();

			void SetNeighborsToSwitch(int neighborsToSwitch);

		private:
			class SwitchingComponentImpl;

			std::unique_ptr<SwitchingComponentImpl> pimpl;
		};
	}
}