#include "FusionCrowdClient.h"

#include <string.h> 
#include <iostream>

#include "WsException.h"
#include "WebMessage.h"


namespace FusionCrowdWeb
{
	void FusionCrowdClient::StartOn(const char* inIpAdress, short inPort)
	{
		using namespace std;

		_clientCore.Initialize();

		bool connected = false;
		while (!connected)
		{
			try
			{
				_clientCore.Connect(inIpAdress, inPort);
				connected = true;
				cout << "Successfully connected to " << inIpAdress << ':' << inPort << endl << endl;
			}
			catch (...)
			{
				//
			}
		}

		cout << endl << "Session started" << endl << endl;

		char request[100];
		while (true)
		{
			cin >> request;

			if (strcmp(request, "exit") == 0)
			{
				break;
			}

			cout << "Sending request..." << endl;
			_clientCore.Send(StartWithNavMesh, request, strlen(request) + 1);

			auto response = _clientCore.Receive();
			auto responseCode = response.first.AsRequestCode;
			auto responseData = response.second;
			cout << "Received code: " << responseCode << endl;
			cout << "Received data: " << responseData << endl << endl;
		}
	}

	void FusionCrowdClient::Shutdown()
	{
		_clientCore.Finalize();
	}
}