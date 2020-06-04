#include "ExternalControl.h"
#include "Navigation/AgentSpatialInfo.h"


using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	ExternalControl::ExternalControl(std::shared_ptr<Simulator> simulator) : _simulator(simulator)
	{ }

	void ExternalControl::AddAgent(size_t id)
	{
		AgentStruct agstuct;
		agstuct.id = id;
		agstuct.input = Vector2(0, 0);
		_agents.insert({ id, agstuct });
	}

	bool ExternalControl::DeleteAgent(size_t id)
	{
		_agents.erase(id);
		return true;;
	}

	void ExternalControl::Update(float timeStep)
	{
		for (auto & agtStruct : _agents)
		{
			size_t id = agtStruct.first;
			AgentSpatialInfo & info = _simulator->GetSpatialInfo(id);
			Vector2 dir = agtStruct.second.input;
			if (dir.Length() > 0) dir /= dir.Length();
			info.prefVelocity.setSingle(dir);
			info.prefVelocity.setSpeed(10.0f);
			agtStruct.second.input = Vector2(0, 0);
		}
	}

	DirectX::SimpleMath::Vector2 ExternalControl::GetClosestAvailablePoint(DirectX::SimpleMath::Vector2 p)
	{
		return p;
	}

	bool ExternalControl::AddInput(size_t agent_id, float x, float y) {
		if (_agents.count(agent_id) == 0) return false;
		_agents[agent_id].input += Vector2(x, y);
		return true;
	}
}