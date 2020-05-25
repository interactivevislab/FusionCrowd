#include "FusionCrowdClient.h"

#include <string.h> 
#include <iostream>

#include "WsException.h"
#include "MessageCodes.h"


namespace FusionCrowdWeb
{
	void FusionCrowdClient::StartOn(const char* inIpAdress, short inPort)
	{
		using namespace std;

		_clientCore.Start();

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
		while (strcmp(request, "exit"))
		{
			request[0] = StartWithNavMesh;
			cin >> request + sizeof(RequestCode);

			cout << "Sending request..." << endl;
			size_t requestSize = sizeof(RequestCode) + strlen(request + sizeof(RequestCode)) + 1;
			_clientCore.Send(request, requestSize);

			auto response = _clientCore.Receive(1);
			ResponseCode messageCode = (ResponseCode)response[0];
			response += sizeof(ResponseCode);
			cout << "Received: " << response << endl << endl;
		}
	}

	void FusionCrowdClient::Shutdown()
	{
		_clientCore.Shutdown();
	}
}