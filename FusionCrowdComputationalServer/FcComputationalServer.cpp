#include "FcComputationalServer.h"

#include <iostream>

#include "Export/ComponentId.h"
#include "WsException.h"
#include "WebMessage.h"


namespace FusionCrowdWeb
{
	void FcComputationalServer::StartServer(const char* inIpAdress, short inPort)
	{
		WebNode::StartServer(inIpAdress, inPort);
		std::cout << "Successfully started on " << inIpAdress << ':' << inPort << std::endl << std::endl;
	}


	void FcComputationalServer::AcceptMainServerConnection()
	{
		_mainServerId = AcceptInputConnection();
		std::cout << "Client connected" << std::endl << std::endl;
	}


	void FcComputationalServer::InitComputation()
	{
		//stub
	}


	void FcComputationalServer::ProcessComputationRequest()
	{
		//stub
	}

	//FcComputationalServer::FcComputationalServer()
	//{
	//	_requestProcessors = {
	//		{
	//			DoStep,
	//			new RequestProcessor<void, float>(&ISimulatorFacade::DoStep)
	//		},
	//		{
	//			SetAgentOp,
	//			new RequestProcessor<OperationStatus, size_t, ComponentId>(&ISimulatorFacade::SetAgentOp)
	//		},
	//		{
	//			SetAgentStrategy,
	//			new RequestProcessor<OperationStatus, size_t, ComponentId>(&ISimulatorFacade::SetAgentStrategy)
	//		},
	//		{
	//			SetAgentGoal,
	//			new RequestProcessor<OperationStatus, size_t, Point>(&ISimulatorFacade::SetAgentGoal)
	//		},
	//		{
	//			GetAgentCount,
	//			new RequestProcessor<size_t>(&ISimulatorFacade::GetAgentCount)
	//		},
	//		//{
	//		//	GetAgents,
	//		//	new RequestProcessor<>(&ISimulatorFacade::GetAgents)
	//		//},
	//		{
	//			AddAgent,
	//			new RequestProcessor<size_t, float, float, ComponentId, ComponentId, ComponentId>(&ISimulatorFacade::AddAgent)
	//		},
	//		{
	//			RemoveAgent,
	//			new RequestProcessor<OperationStatus, size_t>(&ISimulatorFacade::RemoveAgent)
	//		}
	//	};
	//}


	//FcComputationalServer::~FcComputationalServer()
	//{
	//	for (auto processor : _requestProcessors)
	//	{
	//		delete processor.second;
	//	}
	//}


	//void FcComputationalServer::InitBuilderByNavMeshPath(const char* inNavMeshPath)
	//{
	//	using namespace FusionCrowd;

	//	_builder = std::shared_ptr<ISimulatorBuilder>(BuildSimulator(), [](ISimulatorBuilder* inBuilder)
	//	{ 
	//		BuilderDeleter(inBuilder);
	//	});
	//	_builder->WithNavMesh(inNavMeshPath);

	//	for (auto component : ComponentIds::allOperationComponentTypes)
	//	{
	//		_builder->WithOp(component);
	//	}
	//}


	//void FcComputationalServer::InitBuilderByNavMeshName(const char* inNavMeshName)
	//{
	//	using namespace std;

	//	char exePath[MAX_PATH];
	//	GetModuleFileName(NULL, exePath, MAX_PATH);
	//	string::size_type pos = string(exePath).find_last_of("\\/");
	//	auto navMeshpath = string(exePath).substr(0, pos + 1).append("Resources\\").append(inNavMeshName);

	//	InitBuilderByNavMeshPath(navMeshpath.c_str());
	//}


	//void FcComputationalServer::StartSimulation()
	//{
	//	using namespace FusionCrowd;

	//	_simulator = std::shared_ptr<ISimulatorFacade>(_builder->Build(), [](ISimulatorFacade* inSimulatorFacade)
	//	{ 
	//		SimulatorFacadeDeleter(inSimulatorFacade);
	//	});
	//	_simulator->SetIsRecording(false);

	//	_isSimulationStarted = true;
	//}


	//void FcComputationalServer::StartOn(const char* inIpAdress, short inPort)
	//{
	//	StartServer(inIpAdress, inPort);

	//	std::cout << "Successfully started on " << inIpAdress << ':' << inPort << std::endl << std::endl;

	//	while (true)
	//	{
	//		auto clientId = AcceptInputConnection();
	//		std::cout << "Client connected" << std::endl << std::endl;

	//		while (true)
	//		{
	//			try
	//			{
	//				std::cout << "Waiting for request..." << std::endl;
	//				ProcessRequest(clientId);
	//			}
	//			catch (WsException e)
	//			{
	//				std::cout << "Client disconnected" << std::endl << std::endl;
	//				Disconnect(clientId);
	//				break;
	//			}			
	//		}

	//		// some way to Shutdown()

	//	}
	//}


	//void FcComputationalServer::ProcessRequest(int inClientId)
	//{
	//	auto request = Receive(inClientId);
	//	auto requestCode = request.first.AsRequestCode;
	//	auto requestData = request.second;
	//	std::cout << "Request received" << std::endl;

	//	switch (requestCode)
	//	{
	//		case StartWithNavMesh:
	//		{
	//			try
	//			{
	//				InitBuilderByNavMeshName(requestData);
	//				StartSimulation();
	//				Send(inClientId, Success);
	//			}
	//			catch (...)
	//			{
	//				Send(inClientId, InnerFusionCrowdError);
	//			}

	//			break;
	//		}
	//		
	//		default:
	//		{
	//			if (!_isSimulationStarted)
	//			{
	//				Send(inClientId, NeedRunSimulation);
	//				return;
	//			}

	//			auto iter = _requestProcessors.find(requestCode);
	//			if (iter == _requestProcessors.end())
	//			{
	//				Send(inClientId, UnknowsRequestCode);
	//				return;
	//			}

	//			auto processor = iter->second;
	//			char *result = new char[processor->GetOutputSize()];

	//			try
	//			{
	//				processor->Process(_simulator, requestData, result);
	//				Send(inClientId, Success, result, processor->GetOutputSize());
	//			}
	//			catch (...)
	//			{
	//				Send(inClientId, InnerFusionCrowdError);
	//			}

	//			delete[] result;

	//			break;
	//		}
	//	}
	//}


	//void FcComputationalServer::Send(int inClientId, ResponseCode inResponseCode)
	//{
	//	WebNode::Send(inClientId, inResponseCode);
	//	std::cout << "Responce sent" << std::endl << std::endl;
	//}


	//void FcComputationalServer::Send(int inClientId, ResponseCode inResponseCode, const char * inResponseData, size_t inDataSize)
	//{
	//	WebNode::Send(inClientId, inResponseCode, inResponseData, inDataSize);
	//	std::cout << "Responce sent" << std::endl << std::endl;
	//}
}
