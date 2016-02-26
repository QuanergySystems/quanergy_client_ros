/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy_client_ros/client_node.h>

int main(int argc, char** argv)
{
  if (ClientNode::checkArgs(argc, argv))
  {
    ClientNode node(argc, argv);

    node.run();
  }

  return 0;
}
