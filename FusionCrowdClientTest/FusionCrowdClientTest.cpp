#include "FCClientCore.h"
#include "MessageCodes.h"
#include "WsException.h"

#include <string.h>
#include <iostream>


int main()
{
	using namespace std;
	using namespace FusionCrowdWeb;

	cout << "---Fusion Crowd Client---" << endl;

	try
	{
		FCClientCore client;
		cout << "Starting..." << endl;
		client.Start();

		cout << "Trying to connect to server...";
		bool connected = false;
		while (!connected)
		{
			try
			{
				client.Connect("127.0.0.1", 8000);
				connected = true;
				cout << " success" << endl;
			}
			catch (...)
			{
				//
			}
		}

		cout << endl << "Session started" << endl << endl;

		char request[100];
		while (strcmp(request, "exit"))
		{
			request[0] = StartWithNavMesh;
			cin >> request + sizeof(RequestCode);

			cout << "Sending request..." << endl;
			size_t requestSize = sizeof(RequestCode) + strlen(request + sizeof(RequestCode)) + 1;
			client.Send(request, requestSize);

			auto response = client.Receive(1);
			ResponseCode messageCode = (ResponseCode)response[0];
			response += sizeof(ResponseCode);
			cout << "Received: " << response << endl << endl;
		}

		cout << "Session ended" << endl << endl;

		cout << "Shutting down..." << endl;
		client.Shutdown();
	}
	catch (WsException e)
	{
		cout << e.What() << " | Error code: " << e.GetWsErrorCode() << endl;
	}
	catch (...)
	{
		cout << "Unknown exception" << endl;
	}

	system("pause");
	return 0;
}
