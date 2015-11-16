/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include "m8_client_node.h"


int main (int argc, char ** argv)
{
  M8ClientNode node (argc, argv);

  node.publish();

  return (0);
}
