#include <iostream>
#include "FCServerCore.h"
#include "WsException.h"
#include <string.h>


int main()
{
    std::cout << "---Fusion Crowd Server---" << std::endl;

	try {
		FCServerCore server;
		std::cout << "Starting..." << std::endl;
		server.Start();
		std::cout << "Binging..." << std::endl;
		server.Bind("127.0.0.1", 8000);
		std::cout << "Listenning..." << std::endl;
		server.Listen();
		std::cout << "Waiting for connection..." << std::endl;
		server.Accept();

		std::cout << std::endl << "Session started" << std::endl << std::endl;

		char *request;
		do {
			request = server.Receive();
			std::cout << "Received: " << request << std::endl;

			size_t answerMaxSize = 100;
			char *response = new char[answerMaxSize];
			strcpy_s(response, answerMaxSize, "Server receive request: ");
			strcat_s(response, answerMaxSize, request);

			std::cout << "Sending response..." << std::endl << std::endl;
			server.Send(response);
		} while (strcmp(request, "exit"));

		std::cout << "Session ended" << std::endl << std::endl;

		std::cout << "Shutting down..." << std::endl;
		server.Shutdown();
	}
	catch (WsException e) {
		std::cout << e.what() << std::endl;
	}
	catch (...) {
		std::cout << "Unknown exception" << std::endl;
	}

	system("pause");
	return 0;
}