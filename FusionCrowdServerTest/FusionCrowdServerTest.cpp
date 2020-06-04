#include "WebNode.h"
#include "FusionCrowdServer.h"

#include <iostream>


int main()
{
	using namespace std;
	using namespace FusionCrowdWeb;

	WebNode::GlobalStartup();
	cout << "---Fusion Crowd Server---" << endl;

	FusionCrowdServer server;
	server.StartOn("127.0.0.1", 8000);

	WebNode::GlobalCleanup();

	system("pause");
	return 0;
}
