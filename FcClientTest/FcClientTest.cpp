#include "FcClient.h"

#include <vector>
#include <iostream>
#include <iomanip>


int main()
{
	using namespace std;
	using namespace FusionCrowdWeb;

	FusionCrowdClient::GlobalStartup();
	cout << "---Fusion Crowd Client---" << endl << endl;

	FusionCrowdClient client;

	cout << "Connecting to main server... ";
	client.ConnectToMainServer(WebAddress("127.0.0.1", 49000));
	cout << "success" << endl;

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
	cout << "Computation initialization requested" << endl;

	cout << "Computing steps requesting... ";
	const int stepsNum = 200;
	vector<OutputComputingData> results;
	for (int i = 0; i < stepsNum; i++)
	{
		auto result = client.RequestComputation(InputComputingData{ 0.1f });
		results.push_back(result);
	}
	cout << "success" << endl;

	client.DisconnectFromMainServer();
	cout << "Disconnection from main server" << endl << endl;

	FusionCrowdClient::GlobalCleanup();

	cout << "Results:" << endl << endl;
	cout << setprecision(3);
	for (int i = 0; i < results.size(); i += 5)
	{
		for (auto info : results[i].AgentInfos)
		{
			cout << setw(8) << info.posX << setw(8) << info.posY;
		}
		cout << endl;
	}
	cout << endl;

	system("pause");

	return 0;
}
