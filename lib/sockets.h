#pragma once

#include <stdio.h>
#include <stdlib.h>
// windows
#ifdef _WIN32
#include <windows.h>
#include <winsock2.h>
#pragma comment(lib,"ws2_32.lib")
#pragma warning(disable:4996)
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#define SOCKET int // linux函数不同于windows
#define INVALID_SOCKET -1 // linux函数返回值不同
#endif


bool OpenSocket(SOCKET& sock, const char IP[], const unsigned short Port)
{

#ifdef _WIN32
    WSADATA wsaData;
    SOCKADDR_IN addrSrv;

    if(!WSAStartup(MAKEWORD(1, 1), &wsaData))
    {
        if( (sock = socket(AF_INET, SOCK_STREAM ,0)) != INVALID_SOCKET )
        {
            addrSrv.sin_addr.S_un.S_addr = inet_addr(IP);
            addrSrv.sin_family = AF_INET;
            addrSrv.sin_port = htons(Port);
            connect(sock, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR));
            return true;
        }
    }
#else
    sockaddr_in addrSrv;
    if( (sock = socket(AF_INET, SOCK_STREAM ,0)) != INVALID_SOCKET )
    {
        addrSrv.sin_addr.s_addr = inet_addr(IP);
        addrSrv.sin_family = AF_INET;
        addrSrv.sin_port = htons(Port);
        connect(sock, (sockaddr*)&addrSrv, sizeof(addrSrv));
        return true;
    }
#endif
    return false;
}

void CloseSocket(SOCKET& sock)
{
#ifdef _WIN32
    closesocket(sock);
    WSACleanup();
#else
    close(sock);
#endif
}
