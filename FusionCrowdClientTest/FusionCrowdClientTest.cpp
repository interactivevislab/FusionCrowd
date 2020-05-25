#include "FusionCrowdClient.h"

#include <iostream>


int main()
{
	using namespace std;
	using namespace FusionCrowdWeb;

	cout << "---Fusion Crowd Client---" << endl;

	FusionCrowdClient client;
	client.StartOn("127.0.0.1", 8000);

	system("pause");
	return 0;
}
