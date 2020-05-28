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

		int clientId;
		bool connected = false;
		while (!connected)
		{
			try
			{
				clientId = _webNode.ConnectToServer(inIpAdress, inPort);
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
			_webNode.Send(clientId, StartWithNavMesh, request, strlen(request) + 1);

			auto response = _webNode.Receive(clientId);
			auto responseCode = response.first.AsRequestCode;
			auto responseData = response.second;

			cout << "Received code: " << responseCode << endl;
			if (responseData != nullptr)
			{
				cout << "Received data: " << responseData << endl;
			}
			cout << endl;
		}
	}
}