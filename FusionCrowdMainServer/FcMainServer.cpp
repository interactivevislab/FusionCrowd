#include "FcMainServer.h"

#include "FcWebData.h"
#include "WebDataSerializer.h"
#include "WsException.h"

#include <algorithm>


namespace FusionCrowdWeb
{
	void FcMainServer::StartServer(WebAddress inAddress)
	{
		WebNode::StartServer(inAddress);
	}


	void FcMainServer::ConnectToComputationalServers(const std::vector<WebAddress>& inAddresses)
	{
		for (auto address : inAddresses)
		{
			bool connected = false;
			while (!connected)
			{
				try
				{
					auto serverId = ConnectToServer(address);
					_computationalServersIds.push_back(serverId);
					connected = true;
				}
				catch (...)
				{
					//connect error - try again
				}
			}
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
		auto request = Receive(_clientId);
		if (request.first.AsRequestCode != RequestCode::InitSimulation)
		{
			throw FcWebException("RequestError");
		}
		auto initData = WebDataSerializer<InitComputingData>::Deserialize(request.second);
		
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
		for (auto agentInitData : initData.AgentsData)
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
			initData.AgentsData = FCArray<AgentInitData>(agentsData.size());
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
			auto initResponce = Receive(serverId);
			if (request.first.AsResponseCode != ResponseCode::Success)
			{
				throw FcWebException("RequestError");
			}
			auto agentIds = WebDataSerializer<AgentsIds>::Deserialize(initResponce.second);

			for (auto id : agentIds.Values)
			{
				_agentsIds[serverId][id] = _freeId++;
			}
		}
	}


	void FcMainServer::ProcessComputationRequest()
	{
		//reception input data
		auto request = Receive(_clientId);
		if (request.first.AsRequestCode != RequestCode::DoStep)
		{
			throw FcWebException("RequestError");
		}
		auto inData = WebDataSerializer<InputComputingData>::Deserialize(request.second);

		//input data processing
		std::map<int, std::vector<AgentInfo>> newAgentsDataParts;
		for (auto displacedAgent : _displacedAgents)
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
		for (auto agent : _allAgents)
		{
			for (auto serverId : _computationalServersIds)
			{
				if (_navMeshRegions[serverId].IsPointInsideBoundaryZone(agent.posX, agent.posY, _boundaryZoneDepth))
				{
					boundaryAgentsDataParts[serverId].push_back(agent);
					break;
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
			request = Receive(serverId);
			if (request.first.AsResponseCode != ResponseCode::Success)
			{
				throw FcWebException("ResponseError");
			}
			auto outDataPart = WebDataSerializer<OutputComputingData>::Deserialize(request.second);
			agentsNum += outDataPart.AgentInfos.size() + outDataPart.DisplacedAgents.size();
			outDataParts[serverId] = outDataPart;

			request = Receive(serverId);
			if (request.first.AsResponseCode != ResponseCode::Success)
			{
				throw FcWebException("ResponseError");
			}
			auto newAgentIds = WebDataSerializer<AgentsIds>::Deserialize(request.second);
			outNewAgentIds[serverId] = newAgentIds;
		}

		//id updating
		for (auto serverId : _computationalServersIds)
		{
			std::vector<AgentInfo>& currentDisplacedAgents = newAgentsDataParts[serverId];
			FCArray<size_t>& currentDisplacedAgentsIds = outNewAgentIds[serverId].Values;
			FCArray<AgentInfo>& newDisplacedAgents = outDataParts[serverId].DisplacedAgents;

			for (int i = 0; i < currentDisplacedAgents.size(); i++)
			{
				auto displacedAgent = currentDisplacedAgents[i];
				auto newId = currentDisplacedAgentsIds[i];
				_agentsIds[serverId][newId] = displacedAgent.id;
			}

			for (auto& newDisplacedAgent : newDisplacedAgents)
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
			auto serverId = outDataPart.first;
			auto& data = outDataPart.second;
			for (auto agentInfo : data.AgentInfos)
			{
				agentInfo.id = _agentsIds[serverId][agentInfo.id];
				_allAgents[infoIndex++] = agentInfo;
			}
			for (auto agentInfo : data.DisplacedAgents)
			{
				_allAgents[infoIndex++] = agentInfo;
				_displacedAgents.push_back(agentInfo);
			}
		}

		std::sort(_allAgents.begin(), _allAgents.end(), [](AgentInfo a, AgentInfo b) {
			return a.id < b.id;
		});

		//sending output data
		Send(_clientId, ResponseCode::Success, OutputComputingData{ _allAgents });
	}
}
