#include "FcMainServer.h"

#include <iostream>


int main()
{
	using namespace std;
	using namespace FusionCrowdWeb;

	FcMainServer::GlobalStartup();
	cout << "---Fusion Crowd Main Server---" << endl << endl;

	u_short port = 49000;
	vector<WebAddress> computationalServersAddresses {
		WebAddress("127.0.0.1", 49001)
	};
	
	bool isVerboseRun = false;
	if (!isVerboseRun)
	{
		FcMainServer server;
		cout << "Ordinary run starts on localhost:" << port << endl;
		server.StartOrdinaryRun(port, computationalServersAddresses);
		cout << "Ordinary run ends" << endl << endl;
	}
	else
	{
		FcMainServer server;

		server.StartServer(port);
		cout << "Successfully started on localhost:" << port << endl;

		cout << "Connecting to computational servers... ";
		server.ConnectToComputationalServers(computationalServersAddresses);
		cout << "success" << endl;

		server.AcceptClientConnection();
		cout << "Client connected" << endl;

		cout << "Processing init data... ";
		server.InitComputation();
		cout << "success" << endl;

		cout << "Processing computation data... ";
		try
		{
			while (true)
			{
				server.ProcessComputationRequest();
			}
		}
		catch (FusionCrowdWeb::FcWebException e)
		{
			cout << "success" << endl;
			//cout << "error: " << e.What() << endl;
			cout << "Client disconnected" << endl;

			auto recordingFileName = FcFileWrapper::GetFullNameForResource("debug_recording.csv");
			server.SaveRecording(recordingFileName);
			delete recordingFileName;
		}

		server.DisconnectFromComputationalServers();
		cout << "Disconnection from computational servers" << endl;

		server.ShutdownServer();
		cout << "Server shutdown" << endl << endl;
	}

	FcMainServer::GlobalCleanup();

	system("pause");
	
	return 0;
}
