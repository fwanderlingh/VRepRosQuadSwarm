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
