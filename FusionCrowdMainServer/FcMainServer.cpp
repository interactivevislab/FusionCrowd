#include "FcMainServer.h"

#include "FcWebData.h"
#include "WebDataSerializer.h"
#include "WsException.h"

#include <algorithm>
#include <sstream>
#include <string>


namespace FusionCrowdWeb
{
	void FcMainServer::ConnectToComputationalServers(const std::vector<WebAddress>& inAddresses)
	{
		std::vector<WebAddress> failedConnections;
		for (auto address : inAddresses)
		{
			try
			{
				auto serverSocket = TryConnectToServer(address);
				_computationalServersSockets.push_back(serverSocket);
			}
			catch (WsException exception)
			{
				failedConnections.push_back(address);
			}
		}

		if (failedConnections.size() > 0)
		{
			std::ostringstream errorMessage;
			errorMessage << "Connection failed for computional servers: ";
			for (auto failedConnection : failedConnections)
			{
				errorMessage << failedConnection.IpAddress << ":" << failedConnection.Port << ", ";
			}
			throw WsException(errorMessage.str().c_str());
		}
	}


	void FcMainServer::DisconnectFromComputationalServers()
	{
		for (auto serverSocket : _computationalServersSockets)
		{
			Disconnect(serverSocket);
		}
		_computationalServersSockets.clear();
	}


	void FcMainServer::AcceptClientConnection()
	{
		_clientSocket = AcceptInputConnection();
	}


	void FcMainServer::InitComputation()
	{
		//reception init data	
		auto initData = Receive<InitComputingData>(_clientSocket, RequestCode::InitSimulation, "RequestError");

		//init data processing
		auto navMeshFileName = FcFileWrapper::GetFullNameForResource("ms_navmesh.nav");
		initData.NavMeshFile.Unwrap(navMeshFileName);
		initData.NavMeshRegion = NavMeshRegion(navMeshFileName);
		delete navMeshFileName;

		auto serversNum = _computationalServersSockets.size();
		auto navMeshRegionsBuffer = initData.NavMeshRegion.Split(serversNum);
		for (int i = 0; i < serversNum; i++)
		{
			_navMeshRegions[_computationalServersSockets[i]] = navMeshRegionsBuffer[i];
		}

		std::map<int, std::vector<AgentInitData>> initAgentsDataParts;
		for (auto& agentInitData : initData.AgentsData)
		{
			for (auto serverSocket : _computationalServersSockets)
			{
				if (_navMeshRegions[serverSocket].IsPointInside(agentInitData.X, agentInitData.Y))
				{
					initAgentsDataParts[serverSocket].push_back(agentInitData);
					break;
				}
			}
		}

		//sending init data
		for (auto serverSocket : _computationalServersSockets)
		{
			auto& agentsData = initAgentsDataParts[serverSocket];
			initData.AgentsData = FusionCrowd::FCArray<AgentInitData>(agentsData.size());
			for (int i = 0; i < agentsData.size(); i++)
			{
				initData.AgentsData[i] = agentsData[i];
			}

			initData.NavMeshRegion = _navMeshRegions[serverSocket];

			Send(serverSocket, RequestCode::InitSimulation, initData);
		}

		//reception agents ids
		for (auto serverSocket : _computationalServersSockets)
		{
			auto agentIds = Receive<AgentsIds>(serverSocket, ResponseCode::Success, "ResponseError");
			for (auto id : agentIds.Values)
			{
				_agentsIds[serverSocket][id] = _freeId++;
			}
		}

		_recording = std::shared_ptr<FusionCrowd::IRecording>(FusionCrowd::BuildRecord(),
			[](FusionCrowd::IRecording* inRecording) {
			RecordDeleter(inRecording);
		});
	}


	void FcMainServer::ProcessComputationRequest()
	{
		using FusionCrowd::FCArray;
		using FusionCrowd::AgentInfo;

		//reception input data
		auto inData = Receive<InputComputingData>(_clientSocket, RequestCode::DoStep, "RequestError");

		//input data processing
		std::map<int, std::vector<AgentInfo>> newAgentsDataParts;
		for (auto& displacedAgent : _displacedAgents)
		{
			for (auto serverSocket : _computationalServersSockets)
			{
				if (_navMeshRegions[serverSocket].IsPointInside(displacedAgent.posX, displacedAgent.posY))
				{
					newAgentsDataParts[serverSocket].push_back(displacedAgent);
					break;
				}
			}
		}

		std::map<int, std::vector<AgentInfo>> boundaryAgentsDataParts;
		for (auto& agent : _allAgents)
		{
			for (auto serverSocket : _computationalServersSockets)
			{
				if (_navMeshRegions[serverSocket].IsPointInsideBoundaryZone(agent.posX, agent.posY, _boundaryZoneDepth))
				{
					boundaryAgentsDataParts[serverSocket].push_back(agent);
				}
			}
		}

		//sending input data
		for (auto serverSocket : _computationalServersSockets)
		{
			auto& newAgentsData = newAgentsDataParts[serverSocket];
			inData.NewAgents = FCArray<AgentInfo>(newAgentsData.size());
			for (int i = 0; i < newAgentsData.size(); i++)
			{
				inData.NewAgents[i] = newAgentsData[i];
			}

			auto& boundaryAgentsData = boundaryAgentsDataParts[serverSocket];
			inData.BoundaryAgents = FCArray<AgentInfo>(boundaryAgentsData.size());
			for (int i = 0; i < boundaryAgentsData.size(); i++)
			{
				inData.BoundaryAgents[i] = boundaryAgentsData[i];
			}

			Send(serverSocket, RequestCode::DoStep, inData);
		}

		//reception output data
		std::map<int, OutputComputingData> outDataParts;
		std::map<int, AgentsIds> outNewAgentIds;
		size_t agentsNum = 0;
		auto serversNum = _computationalServersSockets.size();
		for (int i = 0; i < serversNum; i++)
		{
			auto serverSocket = _computationalServersSockets[i];
			
			auto outDataPart = Receive<OutputComputingData>(serverSocket, ResponseCode::Success, "ResponseError");
			for (auto& agentInfo : outDataPart.AgentInfos)
			{
				agentInfo.serverId = i;
			}
			for (auto& agentInfo : outDataPart.DisplacedAgents)
			{
				agentInfo.serverId = i;
			}
			outDataParts[serverSocket] = outDataPart;

			auto newAgentIds = Receive<AgentsIds>(serverSocket, ResponseCode::Success, "ResponseError");
			outNewAgentIds[serverSocket] = newAgentIds;

			agentsNum += outDataPart.AgentInfos.size() + outDataPart.DisplacedAgents.size();
		}

		//id updating
		for (auto serverSocket : _computationalServersSockets)
		{
			std::vector<AgentInfo>& currentDisplacedAgents = newAgentsDataParts[serverSocket];
			FCArray<size_t>& currentDisplacedAgentsIds = outNewAgentIds[serverSocket].Values;
			for (int i = 0; i < currentDisplacedAgents.size(); i++)
			{
				auto displacedAgent = currentDisplacedAgents[i];
				auto newId = currentDisplacedAgentsIds[i];
				_agentsIds[serverSocket][newId] = displacedAgent.id;
			}

			for (auto& newDisplacedAgent : outDataParts[serverSocket].DisplacedAgents)
			{
				auto oldId = newDisplacedAgent.id;
				newDisplacedAgent.id = _agentsIds[serverSocket][oldId];
				_agentsIds[serverSocket].erase(oldId);
			}
		}

		//output data processing
		_allAgents = FCArray<AgentInfo>(agentsNum);
		int infoIndex = 0;
		_displacedAgents.clear();
		for (auto& outDataPart : outDataParts)
		{
			auto serverSocket	= outDataPart.first;
			auto& data			= outDataPart.second;
			for (auto& agentInfo : data.AgentInfos)
			{
				agentInfo.id = _agentsIds[serverSocket][agentInfo.id];
				_allAgents[infoIndex++] = agentInfo;
			}
			for (auto& agentInfo : data.DisplacedAgents)
			{
				_allAgents[infoIndex++] = agentInfo;
				_displacedAgents.push_back(agentInfo);
			}
		}

		std::sort(_allAgents.begin(), _allAgents.end(), [](AgentInfo a, AgentInfo b) {
			return a.id < b.id;
		});

		_recording->MakeRecord(_allAgents, inData.TimeStep);

		//sending output data
		Send(_clientSocket, ResponseCode::Success, OutputComputingData{ _allAgents });
	}


	void FcMainServer::SaveRecording(const char* inRecordingFileName)
	{
		_recording->Serialize(inRecordingFileName, strlen(inRecordingFileName));
	}


	void FcMainServer::StartOrdinaryRun(u_short inPort, const std::vector<WebAddress>& computationalServersAddresses)
	{
		StartServer(inPort);

		while (true)
		{
			ConnectToComputationalServers(computationalServersAddresses);
			AcceptClientConnection();
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
				auto recordingFileName = FcFileWrapper::GetFullNameForResource("ordinary_run_recording.csv");
				SaveRecording(recordingFileName);
				delete recordingFileName;
			}

			DisconnectFromComputationalServers();
		}

		ShutdownServer();
	}
}
