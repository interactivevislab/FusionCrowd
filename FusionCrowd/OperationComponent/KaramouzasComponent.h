#pragma once

#include "Agent.h"
#include "Simulator.h"
#include "Navigation/NavSystem.h"
#include "OperationComponent/IOperationComponent.h"

#include "Util/spimpl.h"
#include "Export/ComponentId.h"

#include <memory>

namespace FusionCrowd
{
	namespace Karamouzas
	{
		struct AgentParamentrs
		{
			float _perSpace;
			float _anticipation;

			AgentParamentrs() :_perSpace(0.69f), _anticipation(8.f)
			{
			}
			AgentParamentrs(float perSpace, float anticipation) :_perSpace(perSpace), _anticipation(anticipation)
			{
			}
		};

		class KaramouzasComponent : public IOperationComponent
		{
		public:
			explicit KaramouzasComponent(std::shared_ptr<NavSystem> navSystem);
			KaramouzasComponent(std::shared_ptr<NavSystem> navSystem, float ORIENT_WEIGHT, float COS_FOV_ANGLE, float REACTION_TIME, float WALL_STEEPNESS, float WALL_DISTANCE, int COLLIDING_COUNT,
				float D_MIN, float D_MID, float D_MAX, float AGENT_FORCE);

			ComponentId GetId() override { return ComponentIds::KARAMOUZAS_ID; }

			void AddAgent(size_t id);
			void AddAgent(size_t id, float perSpace, float anticipation);
			bool DeleteAgent(size_t id);

			void Update(float timeStep);

		private:
			class KaramouzasComponentImpl;

			spimpl::unique_impl_ptr<KaramouzasComponentImpl> pimpl;
		};
	}
}

