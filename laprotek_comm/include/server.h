#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
// #include <time.h>
#include <stdint.h>
// #include <fstream>
#include <iomanip>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <vector>

// #define IP "192.168.1.205"
#define IP "127.0.0.1"

class Server{
private:
  int client, server;
  int portNum;
  struct sockaddr_in server_addr;

  // int in_bufsize;
  // int out_bufsize;
  // char out_buf[out_bufsize];
  // char in_buf[in_bufsize];

  double LT[4][4];
  double RT[4][4];
  double LJ[7];
  double RJ[7];
  double MotionScale;
  int clutch;
  int frozen;
public:
  Server(int port)
  {
    portNum=port;

    socklen_t size;
    client = socket(AF_INET, SOCK_STREAM, 0);

    if (client < 0)
    {
        std::cout << "\nError establishing socket..." << std::endl;
        exit(1);
    }

    std::cout << "\n=> Socket server has been created..." << std::endl;


    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr =inet_addr(IP);
    server_addr.sin_port = htons(portNum);
  };
  void setPortNumber(int port);
  int ConnectToClient();
  void closeConnection();
  // setInData(int size);

};

void Server::setPortNumber(int port)
{
  portNum=port;
}

int Server::ConnectToClient()
{
  if ((bind(client, (struct sockaddr*)&server_addr,sizeof(server_addr))) < 0)
  {
      std::cout << "=> Error binding connection, the socket has already been established..." << std::endl;
      return 0;
  }
  else
  {
    std::cout<< "=> CLient Binded to the Socket" << std::endl;
    std::cout << "=> Looking for clients..." << std::endl;

    listen(client, 1);
    socklen_t size;
    size = sizeof(server_addr);
    server = accept(client,(struct sockaddr *)&server_addr,&size);
    std::cout << "=>Found Client, waiting for server to accept..." << '\n';
    if (server<0)
    {
      std::cout << "Error on accepting" << '\n';
    }
    else
    {
      std::cout << "=> Connection with Client established" << std::endl;
    }
    return 1;
  }
}

void Server::closeConnection()
{
  std::cout << "\n\n=> Connection terminated with IP " << inet_ntoa(server_addr.sin_addr);
  close(server);
  std::cout << "\nGoodbye..." << std::endl;
}
