#pragma once

#include "Agent.h"
#include "Config.h"
#include "Simulator.h"
#include "Navigation/NavSystem.h"
#include "OperationComponent/IOperationComponent.h"

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
			ZanlungoComponent(Simulator & simulator);
			ZanlungoComponent(Simulator & simulator, float agentScale, float obstScale, float reactionTime, float forceDistance);
			~ZanlungoComponent();

			std::string GetName();

			void AddAgent(size_t id);
			void AddAgent(size_t id, float mass);
			bool DeleteAgent(size_t id);

			void Update(float timeStep);

		private:
			class ZanlungoComponentImpl;

			std::unique_ptr<ZanlungoComponentImpl> pimpl;
		};
	}
}
