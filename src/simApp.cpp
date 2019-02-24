/*
 * @file: simApp.h
 *
 * @Created on: Apr 5, 2017
 * @Author: Kamran Shamaei
 *
 * @brief - It instantiates the tarsim class.

 * @copyright Copyright Kamran Shamaei 
 * All Rights Reserved.
 *
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 ****************************************************************************/
#include <string>
#include <cstdarg>
#include <stdexcept>
#include <unistd.h>

#include "tarsim.h"
#include "ipcMessages.h"


void print_usage() {
    printf("\nTarsim Usage Options: \n"
            "-c /path/to/config/folder   [Default = None. Must be provided]\n"
            "-l realtime_thread_policy   [Default = %d] \n"
            "-r realtime_thread_priority [Default = %d] \n"
            "-m message_priority         [Default = %d] \n\n",
            tarsim::DEFAULT_RT_THREAD_POLICY,
            tarsim::DEFAULT_RT_THREAD_PRIORITY,
            tarsim::DEFAULT_MSG_PRIORITY);
}

/*
 * @brief creates following server .
 * @param argc - number of arguments
 * @param argv - list of arguments
 * @return EXIT_SUCCESS
 */
int main(int argc, char **argv)
{
    int option = 0;
    std::string configFolderName = "";
    int policy = tarsim::DEFAULT_RT_THREAD_POLICY;
    int priority = tarsim::DEFAULT_RT_THREAD_PRIORITY;
    unsigned int msgPriority = tarsim::DEFAULT_MSG_PRIORITY;

    //Specifying the expected options
    //The two options l and b expect numbers as argument
    while ((option = getopt(argc, argv,"c:l:r:m:h")) != -1) {
        switch (option) {
             case 'c' : configFolderName = std::string(optarg);
                 break;
             case 'l' : policy = atoi(optarg);
                 break;
             case 'r' : priority = atoi(optarg);
                 break;
             case 'm' : msgPriority = atoi(optarg);
                  break;
             case 'h' :
             default: print_usage();
                 exit(EXIT_FAILURE);
        }
    }


    try {
      tarsim::Tarsim sim(configFolderName, policy, priority, msgPriority);
      sim.start();
    } catch (const std::invalid_argument& e) {
      printf("Error: %s\n", e.what());
      printf("For instructions, run with -h option\n");
      return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}


