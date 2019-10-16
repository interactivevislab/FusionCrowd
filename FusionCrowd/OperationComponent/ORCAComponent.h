#pragma once

#include <memory>

#include "Simulator.h"

#include "OperationComponent/IOperationComponent.h"
#include "Util/spimpl.h"
#include "Export/ComponentId.h"

namespace FusionCrowd
{
	namespace ORCA
	{
		class ORCAComponent : public IOperationComponent
		{
		public:
			ORCAComponent(std::shared_ptr<NavSystem> navSystem);
			ORCAComponent(std::shared_ptr<NavSystem> navSystem, float timeHorizon, float timeHorizonObst);

			ComponentId GetId() override { return ComponentIds::ORCA_ID; }

			void AddAgent(size_t id) override;
			bool DeleteAgent(size_t id) override;

			void Update(float timeStep) override;

		private:
			class ORCAComponentImpl;

			spimpl::unique_impl_ptr<ORCAComponentImpl> pimpl;
		};
	}
}
