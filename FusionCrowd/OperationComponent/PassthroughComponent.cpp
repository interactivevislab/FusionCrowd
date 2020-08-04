#include "PassthroughComponent.h"

#include "Math/consts.h"
#include "Math/geomQuery.h"
#include "Math/Util.h"

#include "Navigation/AgentSpatialInfo.h"
#include "Navigation/Obstacle.h"
#include "Export/Export.h"

#include <algorithm>
#include <list>
#include <iostream>
#include <cmath>


using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	namespace PassthroughComp
	{
		PassthroughComponent::PassthroughComponent(std::shared_ptr<NavSystem> navSystem) : _navSystem(navSystem)
		{
		}

		void PassthroughComponent::Update(float timeStep)
		{
			for (auto id : _agents)
			{
				AgentSpatialInfo& agent = _navSystem->GetSpatialInfo(id);
				agent.velNew = agent.prefVelocity.getPreferredVel();
			}
		}

		void PassthroughComponent::AddAgent(size_t id)
		{
			_agents.emplace(id);
			//_navSystem->GetSpatialInfo(id).inertiaEnabled = true;
			_navSystem->GetSpatialInfo(id).inertiaEnabled = false;
			_navSystem->GetSpatialInfo(id).maxAngVel = 60.0f;
		}

		bool PassthroughComponent::DeleteAgent(size_t id)
		{
			_agents.erase(id);
			return true;
		}

	}
}