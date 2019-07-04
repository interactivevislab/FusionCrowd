#include "SwitchingComponent.h"

#include <set>

namespace FusionCrowd
{
	namespace SwitchingComp
	{
		class SwitchingComponent::SwitchingComponentImpl
		{
		public:
			SwitchingComponentImpl(Simulator & simulator,
				std::shared_ptr<IOperationComponent> primaryComponent,
				std::shared_ptr<IOperationComponent> secondaryComponent)
				: _simulator(simulator), _primaryComponent(primaryComponent), _secondaryComponent(secondaryComponent)
			{
			}

			~SwitchingComponentImpl() = default;

			std::string GetName() { 
				return "switching(" + _primaryComponent->GetName() + '&' + _secondaryComponent->GetName() + ')';
			};

			void AddAgent(size_t id)
			{
				_agents.insert(id);

				if (NeedSwitchToSecondary(id)) {
					_secondaryComponent->AddAgent(id);
				} else {
					_primaryComponent->AddAgent(id);
				}
			}

			bool DeleteAgent(size_t id)
			{
				_agents.erase(id);
				return _primaryComponent->DeleteAgent(id) || _secondaryComponent->DeleteAgent(id);
			}

			void Update(float timeStep)
			{
				UpdateAgentsDistribution();

				_primaryComponent->Update(timeStep);
				_secondaryComponent->Update(timeStep);
			}

		private:

			Simulator & _simulator;
			std::shared_ptr<IOperationComponent> _primaryComponent;
			std::shared_ptr<IOperationComponent> _secondaryComponent;
			std::set<size_t> _agents;

			//TEMP SOLUTION
			bool NeedSwitchToSecondary(size_t agentId) {
				return _simulator.GetNavSystem().CountNeighbors(agentId) < 4;
			}

			void UpdateAgentsDistribution() {

				std::set<size_t> switchToPrimary;
				std::set<size_t> switchToSecondary;

				for (auto agentId : _agents) {

					bool nowInSecondary = _simulator.GetById(agentId).opComponent == _secondaryComponent;
					bool mustBeInSecondary = NeedSwitchToSecondary(agentId);

					if (!nowInSecondary && mustBeInSecondary) {
						switchToSecondary.insert(agentId);
					}

					if (nowInSecondary && !mustBeInSecondary) {
						switchToPrimary.insert(agentId);
					}
				}

				for (auto agentId : switchToPrimary) {
					_secondaryComponent->DeleteAgent(agentId);
					_primaryComponent->AddAgent(agentId);
				}

				for (auto agentId : switchToSecondary) {
					_primaryComponent->DeleteAgent(agentId);
					_secondaryComponent->AddAgent(agentId);
				}
			}
		};

		SwitchingComponent::SwitchingComponent(Simulator & simulator,
			std::shared_ptr<IOperationComponent> primaryComponent,
			std::shared_ptr<IOperationComponent> secondaryComponent)
			: pimpl(std::make_unique<SwitchingComponentImpl>(simulator, primaryComponent, secondaryComponent))
		{
		}

		std::string SwitchingComponent::GetName()
		{
			return pimpl->GetName();
		}

		void SwitchingComponent::AddAgent(size_t id)
		{
			pimpl->AddAgent(id);
		}

		bool SwitchingComponent::DeleteAgent(size_t id)
		{
			return pimpl->DeleteAgent(id);
		}

		void SwitchingComponent::Update(float timeStep)
		{
			pimpl->Update(timeStep);
		}

		SwitchingComponent::~SwitchingComponent() = default;
	}
}