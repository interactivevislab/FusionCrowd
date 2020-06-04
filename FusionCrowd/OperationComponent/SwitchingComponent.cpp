#include "SwitchingComponent.h"

#include <set>
#include <map>

namespace FusionCrowd
{
	namespace SwitchingComp
	{
#pragma region Impl
		class SwitchingComponent::SwitchingComponentImpl
		{
		public:
			SwitchingComponentImpl(std::shared_ptr<NavSystem> navSystem,
				std::shared_ptr<IOperationComponent> primaryComponent,
				std::shared_ptr<IOperationComponent> secondaryComponent)
				: _navSystem(navSystem), _primaryComponent(primaryComponent), _secondaryComponent(secondaryComponent)
			{
			}

			~SwitchingComponentImpl() = default;

			void AddAgent(size_t id)
			{
				if (NeedSwitchToSecondary(id)) {
					_secondaryComponent->AddAgent(id);
					_agentsComponents.insert({ id, 1 });
				} else {
					_primaryComponent->AddAgent(id);
					_agentsComponents.insert({ id, 0 });
				}
			}

			bool DeleteAgent(size_t id)
			{
				_agentsComponents.erase(id);
				return _primaryComponent->DeleteAgent(id) || _secondaryComponent->DeleteAgent(id);
			}

			void Update(float timeStep)
			{
				UpdateAgentsDistribution();

				_primaryComponent->Update(timeStep);
				_secondaryComponent->Update(timeStep);
			}

			void SetNeighborsToSwitch(int neighborsToSwitch) {
				_neighborsToSwitch = neighborsToSwitch;
			}

		private:

			std::shared_ptr<NavSystem> _navSystem;
			std::shared_ptr<IOperationComponent> _primaryComponent;
			std::shared_ptr<IOperationComponent> _secondaryComponent;
			std::map<size_t, int> _agentsComponents;
			int _neighborsToSwitch = 3;

			//TEMP SOLUTION
			bool NeedSwitchToSecondary(size_t agentId) {
				return _navSystem->GetNeighbours(agentId).size() <= 3;
			}

			void UpdateAgentsDistribution() {

				for (auto agentData : _agentsComponents) {

					size_t agentId = agentData.first;
					int componentIndex = agentData.second;

					bool nowInSecondary = componentIndex == 1;
					bool mustBeInSecondary = NeedSwitchToSecondary(agentId);

					if (!nowInSecondary && mustBeInSecondary) {
						_primaryComponent->DeleteAgent(agentId);
						_secondaryComponent->AddAgent(agentId);
						agentData.second = 1;
					}

					if (nowInSecondary && !mustBeInSecondary) {
						_secondaryComponent->DeleteAgent(agentId);
						_primaryComponent->AddAgent(agentId);
						agentData.second = 0;
					}
				}
			}
		};
#pragma endregion

#pragma region proxy

		SwitchingComponent::SwitchingComponent(std::shared_ptr<NavSystem> navSystem,
			std::shared_ptr<IOperationComponent> primaryComponent,
			std::shared_ptr<IOperationComponent> secondaryComponent)
			: pimpl(spimpl::make_unique_impl<SwitchingComponentImpl>(navSystem, primaryComponent, secondaryComponent))
		{
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

		void SwitchingComponent::SetNeighborsToSwitch(int neighborsToSwitch)
		{
			pimpl->SetNeighborsToSwitch(neighborsToSwitch);
		}
#pragma endregion
	}
}