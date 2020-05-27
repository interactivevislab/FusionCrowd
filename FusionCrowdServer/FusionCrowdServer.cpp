#include "FusionCrowdServer.h"

#include <iostream>

#include "Export/ComponentId.h"
#include "WsException.h"
#include "WebMessage.h"


namespace FusionCrowdWeb
{
	FusionCrowdServer::FusionCrowdServer()
	{
		_requestProcessors = {
			{
				DoStep,
				new RequestProcessor<void, float>(&ISimulatorFacade::DoStep)
			},
			{
				SetAgentOp,
				new RequestProcessor<OperationStatus, size_t, ComponentId>(&ISimulatorFacade::SetAgentOp)
			},
			{
				SetAgentStrategy,
				new RequestProcessor<OperationStatus, size_t, ComponentId>(&ISimulatorFacade::SetAgentStrategy)
			},
			{
				SetAgentGoal,
				new RequestProcessor<OperationStatus, size_t, Point>(&ISimulatorFacade::SetAgentGoal)
			},
			{
				GetAgentCount,
				new RequestProcessor<size_t>(&ISimulatorFacade::GetAgentCount)
			},
			//{
			//	GetAgents,
			//	new RequestProcessor<>(&ISimulatorFacade::GetAgents)
			//},
			{
				AddAgent,
				new RequestProcessor<size_t, float, float, ComponentId, ComponentId, ComponentId>(&ISimulatorFacade::AddAgent)
			},
			{
				RemoveAgent,
				new RequestProcessor<OperationStatus, size_t>(&ISimulatorFacade::RemoveAgent)
			}
		};
	}


	FusionCrowdServer::~FusionCrowdServer()
	{
		for (auto processor : _requestProcessors)
		{
			delete processor.second;
		}
	}


	void FusionCrowdServer::InitBuilderByNavMeshPath(const char* inNavMeshPath)
	{
		using namespace FusionCrowd;

		_builder = std::shared_ptr<ISimulatorBuilder>(BuildSimulator(), [](ISimulatorBuilder* inBuilder)
		{ 
			BuilderDeleter(inBuilder);
		});
		_builder->WithNavMesh(inNavMeshPath);

		for (auto component : ComponentIds::allOperationComponentTypes)
		{
			_builder->WithOp(component);
		}
	}


	void FusionCrowdServer::InitBuilderByNavMeshName(const char* inNavMeshName)
	{
		using namespace std;

		char exePath[MAX_PATH];
		GetModuleFileName(NULL, exePath, MAX_PATH);
		string::size_type pos = string(exePath).find_last_of("\\/");
		auto navMeshpath = string(exePath).substr(0, pos + 1).append("Resources\\").append(inNavMeshName);

		InitBuilderByNavMeshPath(navMeshpath.c_str());
	}


	void FusionCrowdServer::StartSimulation()
	{
		using namespace FusionCrowd;

		_simulator = std::shared_ptr<ISimulatorFacade>(_builder->Build(), [](ISimulatorFacade* inSimulatorFacade)
		{ 
			SimulatorFacadeDeleter(inSimulatorFacade);
		});
		_simulator->SetIsRecording(false);

		_isSimulationStarted = true;
	}


	void FusionCrowdServer::StartOn(const char* inIpAdress, short inPort)
	{
		_serverCore.Start();
		_serverCore.Bind(inIpAdress, inPort);
		_serverCore.Listen();

		std::cout << "Successfully started on " << inIpAdress << ':' << inPort << std::endl << std::endl;

		while (true)
		{
			_serverCore.Accept();
			std::cout << "Client connected" << std::endl << std::endl;

			while (true)
			{
				try
				{
					std::cout << "Waiting for request..." << std::endl;
					ProcessRequest();
				}
				catch (WsException e)
				{
					std::cout << "Client disconnected" << std::endl << std::endl;
					_serverCore.Disconnect();
					break;
				}			
			}

			// some way to Shutdown()

		}
	}


	void FusionCrowdServer::ProcessRequest()
	{
		auto request = _serverCore.Receive();
		auto requestCode = request.first.AsRequestCode;
		auto requestData = request.second;
		std::cout << "Request received" << std::endl;

		switch (requestCode)
		{
			case StartWithNavMesh:
			{
				try
				{
					InitBuilderByNavMeshName(requestData);
					StartSimulation();
					SendResponce(Success);
				}
				catch (...)
				{
					SendResponce(InnerFusionCrowdError);
				}

				break;
			}
			
			default:
			{
				if (!_isSimulationStarted)
				{
					SendResponce(NeedRunSimulation);
					return;
				}

				auto iter = _requestProcessors.find(requestCode);
				if (iter == _requestProcessors.end())
				{
					SendResponce(UnknowsRequestCode);
					return;
				}

				auto processor = iter->second;
				char *result = new char[processor->GetOutputSize()];

				try
				{
					processor->Process(_simulator, requestData, result);
					SendResponce(Success, result, processor->GetOutputSize());
				}
				catch (...)
				{
					SendResponce(InnerFusionCrowdError);
				}

				delete[] result;

				break;
			}
		}
	}


	void FusionCrowdServer::SendResponce(ResponseCode inResponseCode)
	{
		_serverCore.Send(inResponseCode, nullptr, 0);
		std::cout << "Responce sent" << std::endl << std::endl;
	}


	void FusionCrowdServer::SendResponce(ResponseCode inResponseCode, const char * inResponseData, size_t inDataSize)
	{
		_serverCore.Send(inResponseCode, inResponseData, inDataSize);
		std::cout << "Responce sent" << std::endl << std::endl;
	}


	void FusionCrowdServer::Shutdown()
	{
		_serverCore.Shutdown();
	}
}