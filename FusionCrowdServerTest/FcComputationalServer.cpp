#include "WebNode.h"
#include "FcComputationalServer.h"

#include <iostream>


int main()
{
	using namespace std;
	using namespace FusionCrowdWeb;

	WebNode::GlobalStartup();
	cout << "---Fusion Crowd Computational Server---" << endl;

	FcComputationalServer server;
	server.StartOn("127.0.0.1", 8000);

	WebNode::GlobalCleanup();

	system("pause");
	return 0;
}
