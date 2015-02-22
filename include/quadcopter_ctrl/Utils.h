/*
 * Utils.h
 *
 *  Created on: Jun 8, 2014
 *      Author: francescow
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <string>

class Utils
{
public:
  Utils();
  virtual ~Utils();

  static void spaced_cout(int value);
  static void spaced_cout(double value);
  static std::string add_argv(std::string str, char* argvalue);
  static std::string GetCurrentDateFormatted();

};

#endif /* UTILS_H_ */
