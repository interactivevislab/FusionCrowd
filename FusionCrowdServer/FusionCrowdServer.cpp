#include "FusionCrowdServer.h"

#include <string.h> 
#include <iostream>

#include "Export/ComponentId.h"
#include "WsException.h"
#include "MessageCodes.h"


#define CONSOLE_LOG(mes)  std::cout << (mes);
#define CONSOLE_LOG_ENDL(mes)  std::cout << (mes) << std::endl;
#define CONSOLE_ENDL()  std::cout << std::endl;


namespace FusionCrowdWeb {

	FusionCrowdServer::FusionCrowdServer()
	{
	}


	FusionCrowdServer::~FusionCrowdServer()
	{
		for (auto processor : _requestProcessors) {
			delete processor.second;
		}
	}


	void FusionCrowdServer::InitBuilderByNavMeshPath(const char* navMeshPath)
	{
		using namespace FusionCrowd;

		_builder = std::shared_ptr<ISimulatorBuilder>(BuildSimulator(), [](ISimulatorBuilder* Obj) { BuilderDeleter(Obj); });
		_builder->WithNavMesh(navMeshPath);

		for (auto component : ComponentIds::allOperationComponentTypes) {
			_builder->WithOp(component);
		}
	}


	void FusionCrowdServer::InitBuilderByNavMeshName(const char* navMeshName) {
		using namespace std;

		char exePath[MAX_PATH];
		GetModuleFileName(NULL, exePath, MAX_PATH);
		string::size_type pos = string(exePath).find_last_of("\\/");
		auto navMeshpath = string(exePath).substr(0, pos + 1).append("Resources\\").append(navMeshName);
		InitBuilderByNavMeshPath(navMeshpath.c_str());
	}


	void FusionCrowdServer::StartSimulation()
	{
		using namespace FusionCrowd;

		_simulator = std::shared_ptr<ISimulatorFacade>(_builder->Build(), [](ISimulatorFacade* Obj) { SimulatorFacadeDeleter(Obj); });
		_simulator->SetIsRecording(false);
	}


	void FusionCrowdServer::StartOn(const char* ipAdress, short port) {
		_serverCore.Start();
		_serverCore.Bind(ipAdress, port);
		_serverCore.Listen();

		CONSOLE_LOG("Successfully started on ");
		CONSOLE_LOG(ipAdress);
		CONSOLE_LOG(':');
		CONSOLE_LOG_ENDL(port);
		CONSOLE_ENDL()

		while (true) {

			_serverCore.Accept();
			CONSOLE_LOG_ENDL("Client connected");
			CONSOLE_ENDL()

			while (true) {
				try {
					CONSOLE_LOG_ENDL("Waiting for request...");
					ProcessRequest();
				}
				catch (WsException e) {
					_serverCore.Disconnect();
					CONSOLE_LOG_ENDL("Client disconnected");
					CONSOLE_ENDL()
					break;
				}			
			}

			// some way to Shutdown()

		}
	}


	void FusionCrowdServer::ProcessRequest() {
		const char* requestCodeAsData = _serverCore.Receive(sizeof(RequestCode));
		CONSOLE_LOG_ENDL("Request received");

		RequestCode requestType;
		memcpy(&requestType, requestCodeAsData, sizeof(RequestCode));

		if (requestType == RequestCode::StartWithNavMesh)
		{
			const char* request = _serverCore.ReceiveString();
			try {
				InitBuilderByNavMeshName(request);
				StartSimulation();
			}
			catch(...)
			{
				SendResponce(ResponseCode::InnerFusionCrowdError);
				return;
			}

			_isSimulationStarted = true;
			SendResponce(ResponseCode::Success);
		}
		else
		{
			if (!_isSimulationStarted)
			{
				SendResponce(ResponseCode::NeedRunSimulation);
			}
			else
			{
				auto iter = _requestProcessors.find(requestType);
				if (iter == _requestProcessors.end()) {
					SendResponce(ResponseCode::UnknowsRequestCode);
					return;
				}

				IRequestProcessor *processor = iter->second;

				const char* request = _serverCore.Receive(processor->GetInputSize());
				char *result = new char[processor->GetOutputSize()];

				try {
					processor->Process(_simulator, request, result);
				}
				catch (...) {
					SendResponce(ResponseCode::InnerFusionCrowdError);
					delete[] result;
					return;
				}

				SendResponce(ResponseCode::Success, result, processor->GetOutputSize());
				delete[] result;
			}	
		}
	}


	void FusionCrowdServer::SendResponce(ResponseCode responseCode) {
		SendResponce(responseCode, nullptr, 0);
	}


	void FusionCrowdServer::SendResponce(ResponseCode responseCode, const char * responseData, size_t dataSize) {
		size_t responseLenght = dataSize + sizeof(ResponseCode);
		char* response = new char[responseLenght];
		response[0] = responseCode;
		if (dataSize > 0) {
			memcpy_s(response + sizeof(ResponseCode), sizeof(dataSize), responseData, sizeof(dataSize));
		}
		_serverCore.Send(response, responseLenght);
		delete response;

		CONSOLE_LOG_ENDL("Responce sent");
		CONSOLE_ENDL()
	}


	void FusionCrowdServer::Shutdown() {
		_serverCore.Shutdown();
	}
}