#pragma once

#include "Agent.h"
#include "Config.h"
#include "Simulator.h"
#include "Navigation/NavSystem.h"
#include "OperationComponent/IOperationComponent.h"
#include "Util/spimpl.h"

#include <map>

namespace FusionCrowd
{
	namespace Zanlungo
	{
		struct FUSION_CROWD_API ZAgentParamentrs
		{
			float _mass;

			ZAgentParamentrs() :_mass(80.0f)
			{
			}
			ZAgentParamentrs(float mass) :_mass(mass)
			{
			}
		};

		class FUSION_CROWD_API ZanlungoComponent : public IOperationComponent
		{
		public:
			ZanlungoComponent(std::shared_ptr<NavSystem> navSystem);
			ZanlungoComponent(std::shared_ptr<NavSystem> navSystem, float agentScale, float obstScale, float reactionTime, float forceDistance);

			std::string GetName();

			void AddAgent(size_t id);
			void AddAgent(size_t id, float mass);
			bool DeleteAgent(size_t id);

			void Update(float timeStep);

		private:
			class ZanlungoComponentImpl;

			spimpl::unique_impl_ptr<ZanlungoComponentImpl> pimpl;
		};
	}
}
