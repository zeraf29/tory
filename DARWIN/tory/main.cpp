// File:          main.cpp
// Date:          January 2013
// Description:   Manage the entree point function
// Author:        david.mansolino@epfl.ch

#include "walk.hpp"

#include <cstdlib>

using namespace webots;

int main(int argc, char **argv)
{
  Walk *controller = new Walk();
  controller->conSocket();
  delete controller;
  return EXIT_FAILURE;
}

