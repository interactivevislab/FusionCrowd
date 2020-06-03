#include "FcComputationalServer.h"

#include "Export/ComponentId.h"
#include "WsException.h"
#include "WebMessage.h"
#include "FcWebData.h"

#include <iostream>


namespace FusionCrowdWeb
{
	void FcComputationalServer::StartServer(WebAddress inAddress)
	{
		WebNode::StartServer(inAddress);
		std::cout << "Successfully started on " << inAddress.IpAddress << ':' << inAddress.Port << std::endl << std::endl;
	}


	void FcComputationalServer::AcceptMainServerConnection()
	{
		_mainServerId = AcceptInputConnection();
		std::cout << "MainServer connected" << std::endl << std::endl;
	}


	void FcComputationalServer::InitComputation()
	{
		//reading data
		auto request = Receive(_mainServerId);
		auto data = WebDataSerializer<InitComputingData>::Deserialize(request.second);
		std::cout << "Init data received" << std::endl;

		//initing simulation
		const char* navMeshName = "verysimplenavmesh.nav";
		char exePath[MAX_PATH];
		GetModuleFileName(NULL, exePath, MAX_PATH);
		std::string::size_type pos = std::string(exePath).find_last_of("\\/");
		auto navMeshPath = (std::string(exePath).substr(0, pos + 1).append("Resources\\").append(navMeshName));

		_builder = std::shared_ptr<ISimulatorBuilder>(BuildSimulator(), [](ISimulatorBuilder* inBuilder)
		{
			BuilderDeleter(inBuilder);
		});

		_builder->WithNavMesh(navMeshPath.c_str());

		for (auto component : ComponentIds::allOperationComponentTypes)
		{
			_builder->WithOp(component);
		}

		_simulator = std::shared_ptr<ISimulatorFacade>(_builder->Build(), [](ISimulatorFacade* inSimulatorFacade)
		{
			SimulatorFacadeDeleter(inSimulatorFacade);
		});
		_simulator->SetIsRecording(false);

		//initing agents
		for (auto& agentData : data.AgentsData)
		{
			auto agentId = _simulator->AddAgent(agentData.X, agentData.Y, 1, 10,
				ComponentIds::ORCA_ID, ComponentIds::NAVMESH_ID, ComponentIds::FSM_ID);
			_simulator->SetAgentGoal(agentId, Point(agentData.GoalX, agentData.GoalY));
		}
	}


	void FcComputationalServer::ProcessComputationRequest()
	{
		auto request = Receive(_mainServerId);
		auto inData = WebDataSerializer<InputComputingData>::Deserialize(request.second);
		std::cout << "Computing data received" << std::endl;

		_simulator->DoStep(inData.TimeStep);

		FCArray<AgentInfo> agents(_simulator->GetAgentCount());
		_simulator->GetAgents(agents);
		OutputComputingData outData = OutputComputingData{ agents };

		char* rawData;
		auto dataSize = WebDataSerializer<OutputComputingData>::Serialize(outData, rawData);
		Send(_mainServerId, RequestCode(2), rawData, dataSize);
		std::cout << "Computing result sent" << std::endl;
		delete[] rawData;
	}
}
