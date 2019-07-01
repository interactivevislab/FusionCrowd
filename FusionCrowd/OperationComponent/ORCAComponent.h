#pragma once

#include <memory>

#include "Config.h"
#include "Simulator.h"

#include "OperationComponent/IOperationComponent.h"

namespace FusionCrowd
{
	namespace ORCA
	{
		class FUSION_CROWD_API ORCAComponent : public IOperationComponent
		{
		public:
			ORCAComponent(Simulator & simulator);
			ORCAComponent(Simulator & simulator, float timeHorizon, float timeHorizonObst);
			std::string GetName() { return "orca"; };

			void AddAgent(size_t id);
			bool DeleteAgent(size_t id);

			void Update(float timeStep);
			~ORCAComponent();

		private:
			class ORCAComponentImpl;

			std::unique_ptr<ORCAComponentImpl> pimpl;
		};
	}
}
