#include "FcComputationalServer.h"

#include "WsException.h"
#include "WebMessage.h"
#include "FcWebData.h"
#include "WebDataSerializer.h"
#include "FcFileWrapper.h"


namespace FusionCrowdWeb
{
	void FcComputationalServer::StartServer(WebAddress inAddress)
	{
		WebNode::StartServer(inAddress);
	}


	void FcComputationalServer::AcceptMainServerConnection()
	{
		_mainServerId = AcceptInputConnection();
	}


	void FcComputationalServer::InitComputation()
	{
		//reading data
		auto request = Receive(_mainServerId);
		if (request.first.AsRequestCode != RequestCode::InitSimulation)
		{
			throw FcWebException("RequestError");
		}
		auto data = WebDataSerializer<InitComputingData>::Deserialize(request.second);
		_navMeshRegion = data.NavMeshRegion;

		//initing simulation
		_builder = std::shared_ptr<ISimulatorBuilder>(BuildSimulator(), [](ISimulatorBuilder* inBuilder)
		{
			BuilderDeleter(inBuilder);
		});

		for (auto component : ComponentIds::allOperationComponentTypes)
		{
			_builder->WithOp(component);
		}

		data.NavMeshFile.Unwrap(FcFileWrapper::GetFullNameForResource("cs_navmesh.nav"));

		_builder->WithNavMesh(data.NavMeshFile.GetFileName());

		_simulator = std::shared_ptr<ISimulatorFacade>(_builder->Build(), [](ISimulatorFacade* inSimulatorFacade)
		{
			SimulatorFacadeDeleter(inSimulatorFacade);
		});
		_simulator->SetIsRecording(false);

		//initing agents
		auto agentsNum = data.AgentsData.size();
		AgentsIds agentIds(agentsNum);
		for (int i = 0; i < agentsNum; i++)
		{
			auto& agentData = data.AgentsData[i];
			auto agentId = _simulator->AddAgent(agentData.X, agentData.Y, 1, 10,
				ComponentIds::ORCA_ID, ComponentIds::NAVMESH_ID, ComponentIds::FSM_ID);
			_simulator->SetAgentGoal(agentId, Point(agentData.GoalX, agentData.GoalY));
			agentIds.Values[i] = agentId;
		}

		Send(_mainServerId, ResponseCode::Success, agentIds);
	}


	void FcComputationalServer::ProcessComputationRequest()
	{
		//reading data
		auto request = Receive(_mainServerId);
		if (request.first.AsRequestCode != RequestCode::DoStep)
		{
			throw FcWebException("RequestError");
		}
		auto inData = WebDataSerializer<InputComputingData>::Deserialize(request.second);

		//adding agents
		auto newAgentsNum = inData.NewAgents.size();
		AgentsIds newAgentIds(newAgentsNum);
		for (int i = 0; i < newAgentsNum; i++)
		{
			auto& agentData = inData.NewAgents[i];
			auto agentId = _simulator->AddAgent(agentData.posX, agentData.posY, 1, 10,
				ComponentIds::ORCA_ID, ComponentIds::NAVMESH_ID, ComponentIds::FSM_ID);
			_simulator->SetAgentGoal(agentId, Point(agentData.goalX, agentData.goalY));
			newAgentIds.Values[i] = agentId;
		}

		std::vector<size_t> boundaryAgentsIds;
		for (auto& agentData : inData.BoundaryAgents)
		{
			auto agentId = _simulator->AddAgent(agentData.posX, agentData.posY, 1, 10,
				ComponentIds::ORCA_ID, ComponentIds::NAVMESH_ID, ComponentIds::FSM_ID);
			_simulator->SetAgentGoal(agentId, Point(agentData.goalX, agentData.goalY));
			boundaryAgentsIds.push_back(agentId);
		}

		//step
		_simulator->DoStep(inData.TimeStep);

		for (auto boundaryAgentsId : boundaryAgentsIds)
		{
			_simulator->RemoveAgent(boundaryAgentsId);
		}

		//sending output data
		FCArray<AgentInfo> agents(_simulator->GetAgentCount());
		_simulator->GetAgents(agents);

		std::vector<AgentInfo> displacedAgents;
		for (auto& agent : agents)
		{
			if (!_navMeshRegion.IsPointInside(agent.posX, agent.posY))
			{
				displacedAgents.push_back(agent);
				_simulator->RemoveAgent(agent.id);
			}
		}

		agents = FCArray<AgentInfo>(_simulator->GetAgentCount());
		_simulator->GetAgents(agents);

		OutputComputingData outData;
		outData.AgentInfos = agents;
		outData.DisplacedAgents = FCArray<AgentInfo>(displacedAgents.size());
		for (int i = 0; i < displacedAgents.size(); i++)
		{
			outData.DisplacedAgents[i] = displacedAgents[i];
		}

		Send(_mainServerId, ResponseCode::Success, outData);
		Send(_mainServerId, ResponseCode::Success, newAgentIds);
	}
}
