#pragma once

#include "Agent.h"
#include "Simulator.h"
#include "Navigation/NavSystem.h"
#include "Navigation/NeighborInfo.h"
#include "OperationComponent/IOperationComponent.h"
#include "Util/spimpl.h"
#include "Export/ComponentId.h"

#include <map>

namespace FusionCrowd
{
	namespace Zanlungo
	{
		struct ZAgentParamentrs
		{
			float _mass;

			ZAgentParamentrs() :_mass(80.0f)
			{
			}
			ZAgentParamentrs(float mass) :_mass(mass)
			{
			}
		};

		class ZanlungoComponent : public IOperationComponent
		{
		public:
			ZanlungoComponent(std::shared_ptr<NavSystem> navSystem);
			ZanlungoComponent(std::shared_ptr<NavSystem> navSystem, float agentScale, float obstScale, float reactionTime, float forceDistance);

			ComponentId GetId() override { return ComponentIds::ZANLUNGO_ID; }

			void AddAgent(size_t id) override;
			void AddAgent(size_t id, float mass);
			bool DeleteAgent(size_t id) override;

			void Update(float timeStep) override;

		private:
			class ZanlungoComponentImpl;

			spimpl::unique_impl_ptr<ZanlungoComponentImpl> pimpl;
		};
	}
}
