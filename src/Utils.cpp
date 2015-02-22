//	Copyright (c) 2014, Francesco Wanderlingh. 					//
//	All rights reserved.										//
//	License: BSD (http://opensource.org/licenses/BSD-3-Clause)	//

/*
 * Utils.cpp
 *
 *  Created on: Jun 8, 2014
 *      Author: francescow
 */

#include "Utils.h"
#include <iostream>
#include <cstdio>
#include <ctime>


Utils::Utils()
{
  // TODO Auto-generated constructor stub

}

Utils::~Utils()
{
  // TODO Auto-generated destructor stub
}

void Utils::spaced_cout(int value){
  if(value >=0 && value < 10)
    std::cout << "   " << value;
  else if( (value >=10 && value < 100) || (value >-10 && value < 0) )
    std::cout << "  " << value;
  else if( (value >=100 && value < 1000) || (value >-100 && value <= -10) )
    std::cout << " " << value;
}


void Utils::spaced_cout(double value){
  if(value == 0) printf("    0");
  else if(value < 10)  printf("  %2.1f", value);
  else printf(" %2.1f", value);
}

std::string Utils::add_argv(std::string str, char* argvalue)
{
  std::string suffix(argvalue);
  str = str + "_" + suffix;

  return str;

}

std::string Utils::GetCurrentDateFormatted(){

	std::time_t t = std::time(NULL);
	char mbstr[20];
	std::strftime(mbstr, sizeof(mbstr), "%Y-%m-%d_%H.%M.%S", std::localtime(&t));
    std::string currentDate(mbstr);

	return currentDate;
}
