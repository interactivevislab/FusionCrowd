#pragma once

#include "ITacticComponent.h"
#include "Simulator.h"
#include "Navigation/NavGraph/NavGraph.h"
#include "Navigation/NavSystem.h"
#include "TacticComponent/NavGraph/NavGraphPathPlanner.h"
#include "Export/IExternalControlInterface.h"



namespace FusionCrowd
{
	class ExternalControl : public ITacticComponent, public IExternalControllInterface
	{
	public:
		ExternalControl(std::shared_ptr<Simulator> simulator);
		void AddAgent(size_t id) override;
		bool DeleteAgent(size_t id) override;
		bool AddInput(size_t agent_id, float x, float y) override;

		void Update(float timeStep) override;
		DirectX::SimpleMath::Vector2 GetClosestAvailablePoint(DirectX::SimpleMath::Vector2 p) override;

		ComponentId GetId() override { return ComponentIds::EXTERNAL_ID; }

	private:

		struct AgentStruct
		{
		public:
			size_t id;
			DirectX::SimpleMath::Vector2 input;
		};

		std::map<size_t, AgentStruct> _agents;
		std::shared_ptr<Simulator> _simulator;
	};
}