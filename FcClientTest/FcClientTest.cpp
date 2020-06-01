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
	client.InitComputation(InitComputingData{5});
	auto result = client.RequestComputation(InputComputingData{0.1f});
	cout << "Computing result = " << result.StubData << endl;

	client.DisconnectFromMainServer();

	FusionCrowdClient::GlobalCleanup();

	system("pause");
	return 0;
}
