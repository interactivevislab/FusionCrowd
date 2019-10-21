#pragma once

#include <exception>
#include<WinSock2.h>
#include <cstring>
#include <string.h>

using RuntimeError = std::exception;


struct  WsException : RuntimeError
{
	WsException() {
		WsException("WsException");
	}

	WsException(const char* message) {
		size_t fullMessageSize = std::strlen(message) + 20;
		char *fullMessage = new char[fullMessageSize];
		strcpy_s(fullMessage, fullMessageSize, message);
		strcat_s(fullMessage, fullMessageSize, "; Error Code: " + WSAGetLastError());
		RuntimeError::exception(fullMessage);
		delete[] fullMessage;
	}
};