#include <iostream>
#include "FCClientCore.h"
#include "WsException.h"
#include <string.h>


int main()
{
    std::cout << "---Fusion Crowd Client---" << std::endl;

	try {
		FCClientCore client;
		std::cout << "Starting..." << std::endl;
		client.Start();

		std::cout << "Trying to connect to server...";
		bool connected = false;
		while (!connected) {
			try {
				client.Connect("127.0.0.1", 8000);
				connected = true;
				std::cout << " success" << std::endl;
			}
			catch (...) {}
		}

		std::cout << std::endl << "Session started" << std::endl << std::endl;

		char request[100];
		while (strcmp(request, "exit")) {
			std::cin >> request;

			std::cout << "Sending request..." << std::endl;
			client.Send(request);

			auto response = client.Receive();
			std::cout << "Received: " << response << std::endl << std::endl;
		}

		std::cout << "Session ended" << std::endl << std::endl;

		std::cout << "Shutting down..." << std::endl;
		client.Shutdown();
	}
	catch (WsException e) {
		std::cout << e.what() << " | Error code: " << e.GetWSErrorCode() << std::endl;
	}
	catch (...) {
		std::cout << "Unknown exception" << std::endl;
	}

	system("pause");
	return 0;
}
