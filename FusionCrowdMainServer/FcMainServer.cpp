#include "FcMainServer.h"

#include "FcWebData.h"
#include "WebDataSerializer.h"
#include "WsException.h"


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
		std::map<int, std::vector<AgentInitData>> newAgentsDataParts;
		for (auto displacedAgent : _displacedAgents)
		{
			for (auto serverId : _computationalServersIds)
			{
				if (_navMeshRegions[serverId].IsPointInside(displacedAgent.posX, displacedAgent.posX))
				{
					auto agentInitData = AgentInitData{ displacedAgent.posX, displacedAgent.posY, 
						displacedAgent.goalX, displacedAgent.goalY };
					newAgentsDataParts[serverId].push_back(agentInitData);
					break;
				}
			}
		}

		std::map<int, std::vector<AgentInitData>> boundaryAgentsDataParts;
		for (auto agent : _allAgents)
		{
			for (auto serverId : _computationalServersIds)
			{
				if (_navMeshRegions[serverId].IsPointInsideBoundaryZone(agent.posX, agent.posX, _boundaryZoneDepth))
				{
					auto agentInitData = AgentInitData{ agent.posX, agent.posY,
						agent.goalX, agent.goalY };
					boundaryAgentsDataParts[serverId].push_back(agentInitData);
					break;
				}
			}
		}

		//sending input data
		for (auto serverId : _computationalServersIds)
		{
			auto& newAgentsData = newAgentsDataParts[serverId];
			inData.NewAgents = FCArray<AgentInitData>(newAgentsData.size());
			for (int i = 0; i < newAgentsData.size(); i++)
			{
				inData.NewAgents[i] = newAgentsData[i];
			}

			auto& boundaryAgentsData = boundaryAgentsDataParts[serverId];
			inData.BoundaryAgents = FCArray<AgentInitData>(boundaryAgentsData.size());
			for (int i = 0; i < boundaryAgentsData.size(); i++)
			{
				inData.BoundaryAgents[i] = boundaryAgentsData[i];
			}

			Send(serverId, RequestCode::DoStep, inData);
		}

		//reception output data
		std::vector<OutputComputingData> outDataParts;
		int agentsNum = 0;
		for (auto serverId : _computationalServersIds)
		{
			request = Receive(serverId);
			if (request.first.AsResponseCode != ResponseCode::Success)
			{
				throw FcWebException("ResponseError");
			}
			auto outDataPart = WebDataSerializer<OutputComputingData>::Deserialize(request.second);
			agentsNum += outDataPart.AgentInfos.size();
			outDataParts.push_back(outDataPart);
		}

		//output data processing
		_allAgents = FCArray<AgentInfo>(agentsNum);
		int infoIndex = 0;
		_displacedAgents.clear();
		for (auto outDataPart : outDataParts)
		{
			for (auto agentInfo : outDataPart.AgentInfos)
			{
				_allAgents[infoIndex++] = agentInfo;
			}
			for (auto agentInfo : outDataPart.DisplacedAgents)
			{
				_displacedAgents.push_back(agentInfo);
			}
		}

		//sending output data
		Send(_clientId, ResponseCode::Success, OutputComputingData{ _allAgents });
	}
}
