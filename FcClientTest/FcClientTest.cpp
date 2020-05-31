#include "FcClient.h"

#include <iostream>


int main()
{
	using namespace std;
	using namespace FusionCrowdWeb;

	FusionCrowdClient::GlobalStartup();
	cout << "---Fusion Crowd Client---" << endl;

	FusionCrowdClient client;

	client.ConnectToMainServer("127.0.0.1", 8000);
	client.InitComputation(InitComputingData());
	client.RequestComputation(5, 0.1f);
	auto result = client.GetComputationResult();

	client.DisconnectFromMainServer();

	FusionCrowdClient::GlobalCleanup();

	system("pause");
	return 0;
}
