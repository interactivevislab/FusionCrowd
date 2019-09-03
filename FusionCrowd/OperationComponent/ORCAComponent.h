#pragma once

#include <memory>

#include "Config.h"
#include "Simulator.h"

#include "OperationComponent/IOperationComponent.h"
#include "Util/spimpl.h"

namespace FusionCrowd
{
	namespace ORCA
	{
		class FUSION_CROWD_API ORCAComponent : public IOperationComponent
		{
		public:
			ORCAComponent(std::shared_ptr<NavSystem> navSystem);
			ORCAComponent(std::shared_ptr<NavSystem> navSystem, float timeHorizon, float timeHorizonObst);
			std::string GetName() { return "orca"; };

			void AddAgent(size_t id);
			bool DeleteAgent(size_t id);

			void Update(float timeStep);

		private:
			class ORCAComponentImpl;

			spimpl::unique_impl_ptr<ORCAComponentImpl> pimpl;
		};
	}
}
