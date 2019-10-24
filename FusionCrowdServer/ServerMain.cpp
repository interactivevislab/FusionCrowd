#include <iostream>
#include "FCServerCore.h"
#include "WsException.h"
#include <string.h>
#include "FusionCrowdServer.h"


int main()
{
	using namespace std;
	using namespace FusionCrowdWeb;

	cout << "---Fusion Crowd Server---" << endl;

	//try {
	//	FCServerCore server;
	//	cout << "Starting..." << endl;
	//	server.Start();
	//	cout << "Binging..." << endl;
	//	server.Bind("127.0.0.1", 8000);
	//	cout << "Listenning..." << endl;
	//	server.Listen();
	//	cout << "Waiting for connection..." << endl;
	//	server.Accept();

	//	cout << endl << "Session started" << endl << endl;

	//	char *request;
	//	do {
	//		request = server.Receive();
	//		cout << "Received: " << request << endl;

	//		size_t answerMaxSize = 100;
	//		char *response = new char[answerMaxSize];
	//		strcpy_s(response, answerMaxSize, "Server receive request: ");
	//		strcat_s(response, answerMaxSize, request);

	//		cout << "Sending response..." << endl << endl;
	//		server.Send(response);
	//	} while (strcmp(request, "exit"));

	//	cout << "Session ended" << endl << endl;

	//	cout << "Shutting down..." << endl;
	//	server.Shutdown();
	//}
	//catch (WsException e) {
	//	cout << e.what() << endl;
	//}
	//catch (...) {
	//	cout << "Unknown exception" << endl;
	//}

	FusionCrowdServer server;
	server.StartOn("127.0.0.1", 8000);

	system("pause");
	return 0;
}