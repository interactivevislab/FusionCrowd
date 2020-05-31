#include "WebNode.h"
#include "FcMainServer.h"

#include <iostream>


int main()
{
	using namespace std;
	using namespace FusionCrowdWeb;

	WebNode::GlobalStartup();
	cout << "---Fusion Crowd Main Server---" << endl;

	FcMainServer server;
	server.StartOn("127.0.0.1", 8000);
	server.Shutdown();

	WebNode::GlobalCleanup();

	system("pause");
	return 0;
}
