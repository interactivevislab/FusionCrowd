#include "WebCore.h"
#include "FusionCrowdServer.h"

#include <iostream>


int main()
{
	using namespace std;
	using namespace FusionCrowdWeb;

	WebCore::GlobalStartup();
	cout << "---Fusion Crowd Server---" << endl;

	FusionCrowdServer server;
	server.StartOn("127.0.0.1", 8000);

	WebCore::GlobalCleanup();

	system("pause");
	return 0;
}
