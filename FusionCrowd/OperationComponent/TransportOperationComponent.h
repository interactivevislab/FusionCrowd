#pragma once

#include "Simulator.h"

#include "OperationComponent/IOperationComponent.h"
#include "Navigation/NavGraph/NavGraph.h"

namespace FusionCrowd
{
	namespace Transport
	{
		class TransportOperationComponent : public IOperationComponent
		{
		public:
			TransportOperationComponent(std::shared_ptr<NavSystem> navSystem);

			ComponentId GetId() override { return ComponentIds::TRANSPORT_ID; }


			void AddAgent(size_t id) override;
			bool DeleteAgent(size_t id) override;

			void Update(float timeStep) override;

		private:
			class TransportOperationComponentImpl;

			spimpl::unique_impl_ptr<TransportOperationComponentImpl> pimpl;
		};
	}
}