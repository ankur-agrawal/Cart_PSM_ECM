#include "RosBridge.h"

// #define MAX_ITERATION 100000

void siginthandler(int param)
{
  exit(1);
}

int main()
{
  Server server(1500);


  RosBridge<Server> Bridge(&server);
  Bridge.init();
  while(ros::ok())
  {
    signal(SIGINT, siginthandler);
    server.initSocket();
    if (!server.ConnectToClient())
    {
      return -1;
    }
    while(ros::ok())
    {
      server.PackData();
      if (!server.SendData())
        break;
      server.ReceiveData();
      server.UnpackData();
      // server.DebugPrint();

      Bridge.setFuncPtr(&Server::getRosPoses);
      Bridge.setPoses();
      Bridge.setFuncPtr2(&Server::getRosJoints);
      Bridge.setJoints();
      Bridge.print();

      Bridge.publishPoses();
      Bridge.publishJoints();
    }
    server.closeConnection();
  }
}
