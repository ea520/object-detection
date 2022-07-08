/*
 This programme listens for arrays of string messages on the input topic
 these include most of the info on a detected object (text,x,y,z,robot,mode,type).
 A unique id and the time are added to the message and saved to a file
 (/path/to/object_detection/output.csv)
*/
#include "ros/ros.h"
#include <ctime>
#include <fstream>
#include <ros/package.h>
#include <object_detection/string_arr.h>

std::ofstream outfile;
const std::string input_topic = "/detection/text_csv";

void print_detections_csv(const object_detection::string_arr::ConstPtr &msg)
// Listen for arrays of strings as messages.
// Add a unique ID and a time to the strings.
// Print the strings to a file (and to the screen if in debug mode).
{
  static int id;
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  for (const std::string &str : msg->data)
  {
    outfile << "\n"
            << id << "," << std::put_time(&tm, "%H:%M:%S") << "," << str;
    id++;

    // If its compiled in debug mode, print the message to the screen as well as to the file
#ifndef NDEBUG
    std::cerr << id << "," << std::put_time(&tm, "%H:%M:%S") << "," << str << "\n";
#endif
  }
  outfile.flush(); // make sure everyting is written to the file at this point
}

int main(int argc, char **argv)
{
  // find the path of the output file
  std::string path = ros::package::getPath("object_detection");
  outfile.open(path + "/output.csv", std::ofstream::out);

  // get the current time (required as the date is needed in the header)
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  // Print the header to a file
  outfile << "pois\n"
          << "1.2\n"
          << "CUR\n"
          << "England\n"
          << std::put_time(&tm, "%Y-%m-%d\n")
          << std::put_time(&tm, "%H:%M:%S\n")
          << "Prelim1\n"
          << "id,time,text,x,y,z,robot,mode,type";

  ros::init(argc, argv, "csv_printer");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe(input_topic, 3, print_detections_csv);
  ros::spin();
  outfile.close();
}