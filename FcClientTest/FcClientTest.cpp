#include "FcClient.h"

#include <iostream>


int main()
{
	using namespace std;
	using namespace FusionCrowdWeb;

	FusionCrowdClient::GlobalStartup();
	cout << "---Fusion Crowd Client---" << endl << endl;

	FusionCrowdClient client;

	std::cout << "Connecting to main server... ";
	client.ConnectToMainServer(WebAddress("127.0.0.1", 49000));
	std::cout << "success" << std::endl << std::endl;

	FusionCrowd::FCArray<AgentInitData> agentsData(4);
	agentsData[0] = { -25.f, -25.f, 25.f, 25.f };
	agentsData[1] = { 25.f, 25.f, -25.f, -25.f };
	agentsData[2] = { -25.f, 25.f, 25.f, -25.f };
	agentsData[3] = { 25.f, -25.f, -25.f, 25.f };

	InitComputingData initData{
		FcFileWrapper(FcFileWrapper::GetFullNameForResource("verysimplenavmesh.nav").c_str()),
		agentsData
	};

	client.InitComputation(initData);
	std::cout << "Computation initialization requested" << std::endl << std::endl;

	for (int i = 0; i < 10; i++)
	{
		cout << "Computing step requesting... ";
		auto result = client.RequestComputation(InputComputingData{ 0.1f });

		auto agentInfo = result.AgentInfos[0];
		cout << "result = { " << agentInfo.posX << ", " << agentInfo.posY << " }" << endl;
	}
	cout << endl;

	client.DisconnectFromMainServer();
	cout << "Disconnection from main server" << endl << endl;

	FusionCrowdClient::GlobalCleanup();

	system("pause");

	return 0;
}
