#include "FcMainServer.h"

#include "FcWebData.h"
#include "WebDataSerializer.h"
#include "WsException.h"

#include <map>


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
		std::map<int, std::vector<AgentInitData>> initDataParts;
		for (auto serverId : _computationalServersIds)
		{
			initDataParts[serverId] = std::vector<AgentInitData>();
		}

		int stubConter = 0;
		auto serversNum = _computationalServersIds.size();
		for (auto agentInitData : initData.AgentsData)
		{
			int targetServerId = _computationalServersIds[stubConter++ % serversNum];	//stub logic
			initDataParts[targetServerId].push_back(agentInitData);
		}

		//sending init data
		for (auto serverId : _computationalServersIds)
		{
			auto& agentsData = initDataParts[serverId];
			initData.AgentsData = FCArray<AgentInitData>(agentsData.size());
			for (int i = 0; i < agentsData.size(); i++)
			{
				initData.AgentsData[i] = agentsData[i];
			}

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

		//TODO?: input data processing

		//sending input data
		for (auto serverId : _computationalServersIds)
		{
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
		FCArray<AgentInfo> agentInfos(agentsNum);
		int infoIndex = 0;
		for (auto outDataPart : outDataParts)
		{
			for (auto agentInfo : outDataPart.AgentInfos)
			{
				agentInfos[infoIndex++] = agentInfo;
			}
		}

		//sending output data
		Send(_clientId, ResponseCode::Success, OutputComputingData{ agentInfos });
	}
}
