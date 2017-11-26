#include "RosBridge.h"

// #define MAX_ITERATION 100000
#define DATATYPE 0

int main()
{
  Server server(1500);
  RosBridge<Server> Bridge(&server);
  Bridge.init();

  if (!server.ConnectToClient())
  {
    return -1;
  }

  while(ros::ok())
  {
    std::cout << "Hi" << '\n';
  }
  server.closeConnection();
}
