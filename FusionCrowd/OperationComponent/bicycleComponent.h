#pragma once

#include "Agent.h"
#include "Simulator.h"
#include "Navigation/NavSystem.h"
#include "OperationComponent/IOperationComponent.h"
#include "Export/ComponentId.h"


#include <map>

namespace FusionCrowd
{
	namespace bicycle

	{

		


		struct AgentParamentrs
		{
			float _mass;
			float _theta;
			float _delta;
			float _length;
			float _orintX;
			float _orintY;
			float _acceleration;
			float _negativeAcceleration;

			AgentParamentrs() :_mass(80.0f), _theta(0.0f),_delta(0.0f),_length(2.0f),_orintX(1.0f),_orintY(0.0f), _acceleration(0.5f), _negativeAcceleration(0.5f)
			{
				
			}
			AgentParamentrs(float mass) : _mass(mass)
			{
			}
		};

		class bicycleComponent : public IOperationComponent
		{
		public:
			bicycleComponent(std::shared_ptr<NavSystem> navSystem);
			bicycleComponent(std::shared_ptr<NavSystem> navSystem, float AGENT_SCALE, float OBST_SCALE, float REACTION_TIME, float BODY_FORCE, float FRICTION, float FORCE_DISTANCE);
			~bicycleComponent();

			ComponentId GetId() override { return ComponentIds::BICYCLE; };

			void AddAgent(size_t id) override;
			void AddAgent(size_t id, float mass);
			bool DeleteAgent(size_t id) override;

			void Update(float timeStep) override;

		private:
			void ComputeNewVelocity(AgentSpatialInfo & agent, float timeStep);



			std::shared_ptr<NavSystem> _navSystem;
			std::map<int, AgentParamentrs> _agents;

			float _agentScale;
			float _obstScale;
			float _reactionTime;
			float _bodyForse;
			float _friction;
			float _forceDistance;
		};
	}
}