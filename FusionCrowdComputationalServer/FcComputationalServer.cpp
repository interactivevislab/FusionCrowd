#include "FcComputationalServer.h"

#include "WsException.h"
#include "WebMessage.h"
#include "FcWebData.h"
#include "WebDataSerializer.h"
#include "FcFileWrapper.h"


namespace FusionCrowdWeb
{
	void FcComputationalServer::AcceptMainServerConnection()
	{
		_mainServerSocket = AcceptInputConnection();
	}


	void FcComputationalServer::InitComputation()
	{
		using namespace FusionCrowd;

		//reading data
		auto data = Receive<InitComputingData>(_mainServerSocket, RequestCode::InitSimulation, "RequestError");
		_navMeshRegion = data.NavMeshRegion;

		//initing simulation
		_builder = std::shared_ptr<ISimulatorBuilder>(BuildSimulator(), [](ISimulatorBuilder* inBuilder) {
			BuilderDeleter(inBuilder);
		});

		for (auto component : ComponentIds::allOperationComponentTypes)
		{
			_builder->WithOp(component);
		}

		auto navMeshFileName = FcFileWrapper::GetFullNameForResource("cs_navmesh.nav");
		data.NavMeshFile.Unwrap(navMeshFileName);
		_builder->WithNavMesh(navMeshFileName);
		delete navMeshFileName;

		_simulator = std::shared_ptr<ISimulatorFacade>(_builder->Build(), [](ISimulatorFacade* inSimulatorFacade) {
			SimulatorFacadeDeleter(inSimulatorFacade);
		});
		_simulator->SetIsRecording(false);

		//initing agents
		auto agentsNum = data.AgentsData.size();
		AgentsIds agentIds(agentsNum);
		for (int i = 0; i < agentsNum; i++)
		{
			auto& agentData = data.AgentsData[i];
			auto agentId = _simulator->AddAgent(agentData.X, agentData.Y, 0.25f, 10,
				0.5f, 0.3f, true, true,
				ComponentIds::ORCA_ID, ComponentIds::NAVMESH_ID, ComponentIds::FSM_ID);
			_simulator->SetAgentGoal(agentId, Point(agentData.GoalX, agentData.GoalY));
			agentIds.Values[i] = agentId;
		}

		Send(_mainServerSocket, ResponseCode::Success, agentIds);
	}


	void FcComputationalServer::ProcessComputationRequest()
	{
		using namespace FusionCrowd;

		//reading data
		auto inData = Receive<InputComputingData>(_mainServerSocket, RequestCode::DoStep, "RequestError");

		//adding agents
		auto newAgentsNum = inData.NewAgents.size();
		AgentsIds newAgentIds(newAgentsNum);
		for (int i = 0; i < newAgentsNum; i++)
		{
			auto& agentData = inData.NewAgents[i];
			auto agentId = _simulator->AddAgent(agentData, 10);
			newAgentIds.Values[i] = agentId;
		}

		std::vector<size_t> boundaryAgentsIds;
		for (auto& agentData : inData.BoundaryAgents)
		{
			auto agentId = _simulator->AddAgent(agentData, 10);
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

		Send(_mainServerSocket, ResponseCode::Success, outData);
		Send(_mainServerSocket, ResponseCode::Success, newAgentIds);
	}


	void FcComputationalServer::StartOrdinaryRun(u_short inPort)
	{
		StartServer(inPort);
		AcceptMainServerConnection();
		InitComputation();

		try
		{
			while (true)
			{
				ProcessComputationRequest();
			}
		}
		catch (FusionCrowdWeb::FcWebException e)
		{
			//do nothing
		}

		ShutdownServer();
	}
}
