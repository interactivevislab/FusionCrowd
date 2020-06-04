#include "FcClient.h"

#include <iostream>


int main()
{
	using namespace std;
	using namespace FusionCrowdWeb;

	FusionCrowdClient::GlobalStartup();
	cout << "---Fusion Crowd Client---" << endl;

	FusionCrowdClient client;

	client.ConnectToMainServer(WebAddress("127.0.0.1", 8000));

	FusionCrowd::FCArray<AgentInitData> agentsData(1);
	agentsData[0] = { -25.f, -25.f, 25.f, 25.f };
	InitComputingData initData{
		FcFileWrapper(FcFileWrapper::GetFullNameForResource("verysimplenavmesh.nav").c_str()),
		agentsData
	};

	client.InitComputation(initData);
	auto result = client.RequestComputation(InputComputingData{ 0.1f });

	auto agentInfo = result.AgentInfos[0];
	cout << "Computing result = { " << agentInfo.posX << ", "<< agentInfo.posY << " }" << endl;

	client.DisconnectFromMainServer();

	FusionCrowdClient::GlobalCleanup();

	system("pause");
	return 0;
}
