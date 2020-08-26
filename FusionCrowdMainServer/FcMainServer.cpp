#include "FcMainServer.h"

#include "FcWebData.h"
#include "WebDataSerializer.h"
#include "WsException.h"

#include <algorithm>


namespace FusionCrowdWeb
{
	void FcMainServer::ConnectToComputationalServers(const std::vector<WebAddress>& inAddresses)
	{
		for (auto address : inAddresses)
		{
			auto serverId = WaitForConnectionToServer(address);
			_computationalServersIds.push_back(serverId);
		}
	}


	void FcMainServer::DisconnectFromComputationalServers()
	{
		for (auto serverId : _computationalServersIds)
		{
			Disconnect(serverId);
		}
		_computationalServersIds.clear();
	}


	void FcMainServer::AcceptClientConnection()
	{
		_clientId = AcceptInputConnection();
	}


	void FcMainServer::InitComputation()
	{
		//reception init data	
		auto initData = Receive<InitComputingData>(_clientId, RequestCode::InitSimulation, "RequestError");

		//init data processing
		auto navMeshFileName = FcFileWrapper::GetFullNameForResource("ms_navmesh.nav");
		initData.NavMeshFile.Unwrap(navMeshFileName);
		initData.NavMeshRegion = NavMeshRegion(navMeshFileName);

		auto serversNum = _computationalServersIds.size();
		auto navMeshRegionsBuffer = initData.NavMeshRegion.Split(serversNum);
		for (int i = 0; i < serversNum; i++)
		{
			_navMeshRegions[_computationalServersIds[i]] = navMeshRegionsBuffer[i];
		}

		std::map<int, std::vector<AgentInitData>> initAgentsDataParts;
		for (auto& agentInitData : initData.AgentsData)
		{
			for (auto serverId : _computationalServersIds)
			{
				if (_navMeshRegions[serverId].IsPointInside(agentInitData.X, agentInitData.Y))
				{
					initAgentsDataParts[serverId].push_back(agentInitData);
					break;
				}
			}
		}

		//sending init data
		for (auto serverId : _computationalServersIds)
		{
			auto& agentsData = initAgentsDataParts[serverId];
			initData.AgentsData = FusionCrowd::FCArray<AgentInitData>(agentsData.size());
			for (int i = 0; i < agentsData.size(); i++)
			{
				initData.AgentsData[i] = agentsData[i];
			}

			initData.NavMeshRegion = _navMeshRegions[serverId];

			Send(serverId, RequestCode::InitSimulation, initData);
		}

		//reception agents ids
		for (auto serverId : _computationalServersIds)
		{
			auto agentIds = Receive<AgentsIds>(serverId, ResponseCode::Success, "ResponseError");
			for (auto id : agentIds.Values)
			{
				_agentsIds[serverId][id] = _freeId++;
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
		auto inData = Receive<InputComputingData>(_clientId, RequestCode::DoStep, "RequestError");

		//input data processing
		std::map<int, std::vector<AgentInfo>> newAgentsDataParts;
		for (auto& displacedAgent : _displacedAgents)
		{
			for (auto serverId : _computationalServersIds)
			{
				if (_navMeshRegions[serverId].IsPointInside(displacedAgent.posX, displacedAgent.posY))
				{
					newAgentsDataParts[serverId].push_back(displacedAgent);
					break;
				}
			}
		}

		std::map<int, std::vector<AgentInfo>> boundaryAgentsDataParts;
		for (auto& agent : _allAgents)
		{
			for (auto serverId : _computationalServersIds)
			{
				if (_navMeshRegions[serverId].IsPointInsideBoundaryZone(agent.posX, agent.posY, _boundaryZoneDepth))
				{
					boundaryAgentsDataParts[serverId].push_back(agent);
				}
			}
		}

		//sending input data
		for (auto serverId : _computationalServersIds)
		{
			auto& newAgentsData = newAgentsDataParts[serverId];
			inData.NewAgents = FCArray<AgentInfo>(newAgentsData.size());
			for (int i = 0; i < newAgentsData.size(); i++)
			{
				inData.NewAgents[i] = newAgentsData[i];
			}

			auto& boundaryAgentsData = boundaryAgentsDataParts[serverId];
			inData.BoundaryAgents = FCArray<AgentInfo>(boundaryAgentsData.size());
			for (int i = 0; i < boundaryAgentsData.size(); i++)
			{
				inData.BoundaryAgents[i] = boundaryAgentsData[i];
			}

			Send(serverId, RequestCode::DoStep, inData);
		}

		//reception output data
		std::map<int, OutputComputingData> outDataParts;
		std::map<int, AgentsIds> outNewAgentIds;
		size_t agentsNum = 0;
		for (auto serverId : _computationalServersIds)
		{
			auto outDataPart = Receive<OutputComputingData>(serverId, ResponseCode::Success, "ResponseError");
			for (auto& agentInfo : outDataPart.AgentInfos)
			{
				agentInfo.serverId = serverId;
			}
			for (auto& agentInfo : outDataPart.DisplacedAgents)
			{
				agentInfo.serverId = serverId;
			}
			outDataParts[serverId] = outDataPart;

			auto newAgentIds = Receive<AgentsIds>(serverId, ResponseCode::Success, "ResponseError");
			outNewAgentIds[serverId] = newAgentIds;

			agentsNum += outDataPart.AgentInfos.size() + outDataPart.DisplacedAgents.size();
		}

		//id updating
		for (auto serverId : _computationalServersIds)
		{
			std::vector<AgentInfo>& currentDisplacedAgents = newAgentsDataParts[serverId];
			FCArray<size_t>& currentDisplacedAgentsIds = outNewAgentIds[serverId].Values;
			for (int i = 0; i < currentDisplacedAgents.size(); i++)
			{
				auto displacedAgent = currentDisplacedAgents[i];
				auto newId = currentDisplacedAgentsIds[i];
				_agentsIds[serverId][newId] = displacedAgent.id;
			}

			for (auto& newDisplacedAgent : outDataParts[serverId].DisplacedAgents)
			{
				auto oldId = newDisplacedAgent.id;
				newDisplacedAgent.id = _agentsIds[serverId][oldId];
				_agentsIds[serverId].erase(oldId);
			}
		}

		//output data processing
		_allAgents = FCArray<AgentInfo>(agentsNum);
		int infoIndex = 0;
		_displacedAgents.clear();
		for (auto& outDataPart : outDataParts)
		{
			auto serverId	= outDataPart.first;
			auto& data		= outDataPart.second;
			for (auto& agentInfo : data.AgentInfos)
			{
				agentInfo.id = _agentsIds[serverId][agentInfo.id];
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
		Send(_clientId, ResponseCode::Success, OutputComputingData{ _allAgents });
	}


	void FcMainServer::SaveRecording(const char* inRecordingFileName)
	{
		_recording->Serialize(inRecordingFileName, strlen(inRecordingFileName));
	}


	void FcMainServer::StartOrdinaryRun(u_short inPort, const std::vector<WebAddress>& computationalServersAddresses)
	{
		StartServer(inPort);
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
			SaveRecording(FcFileWrapper::GetFullNameForResource("ordinary_run_recording.csv"));
		}

		DisconnectFromComputationalServers();
		ShutdownServer();
	}
}
