
//
// @file: main.cpp
//
// @Created on: Jul 13, 2018
// @Author: Kamran Shamaei
//
//
// @brief - Sample program to talk to scara robot
//
// @copyright Copyright Kamran Shamaei
// All Rights Reserved.
//
// This file is subject to the terms and conditions defined in
// file 'LICENSE', which is part of this source code package.
// 
//


//INCLUDES
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <signal.h>
#include <tarsimClient.h>

bool forever = true;

/**
 * A handler function to set the promise and close the logger gracefully
 */
void eventHandler(int signum) {
  printf("\nCTRL+C was pressed, closing application...\n");
  forever = false;
}

/**
 * Displays a matrix in a readable format
 */
void printFrame(std::string name, float mij[tarsim::FRAME_INDICES])
{
    printf("%s Frame\n", name.c_str());
    printf("%.3f %.3f %.3f %.3f\n", mij[ 0], mij[ 1], mij[ 2], mij[ 3]);
    printf("%.3f %.3f %.3f %.3f\n", mij[ 4], mij[ 5], mij[ 6], mij[ 7]);
    printf("%.3f %.3f %.3f %.3f\n", mij[ 8], mij[ 9], mij[10], mij[11]);
    printf("%.3f %.3f %.3f %.3f\n", mij[12], mij[13], mij[14], mij[15]);
}

int main(int argc, char **argv)
{
    // First run Tarsim using a terminal or inside your code

	// Instantiate TarsimClient and connect to Tarsim
	tarsim::TarsimClient tc;
    printf("Connecting to Tarsim...\n");
    if (!tc.connect()) {
      printf("Failed to connect to Tarsim\n");
      return EXIT_FAILURE;
    }

    // Set up CTRL+C to terminate (optional)
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = eventHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, nullptr);

    int c = 0;
    while (forever) {
        // If you need to get the speed rate from the user, you can get it as:
    	printf("Running robot at speed rate of %.1f percent\n", tc.getSpeed());

        // Lets display a message for the user every 10 sec on Tarsim, each time for 5 sec
        if (0 == c % 1000) {
            std::string txt = "Running Tarsim using TarsimClient";
            tc.displayMessage(txt.c_str(), tarsim::FaultLevels::FAULT_LEVEL_INFO);
        }

        // Check if there is any incremental commands from the user
        int32_t type = -1;
        int32_t index = -1;
        int32_t incCmd = -1;
        tc.getIncrementalCommand(type, index, incCmd);
        if (-1 != incCmd) {
            printf("Incremental Command %d %d %d\n", type, index, incCmd);
        }

        // Check for robot self-collisions
        std::vector<std::pair<int32_t, int32_t>> selfCollisions =
                tc.getSelfCollisions();
        if (selfCollisions.size() > 0) {
            for (size_t i = 0; i < selfCollisions.size(); i++) {
            printf("Self collision was detected between links %d and %d\n",
                    (int)selfCollisions.at(i).first,
                    (int)selfCollisions.at(i).second);
            }
        }

        // Check for robot collisions with external objects
        std::vector<std::pair<int32_t, int32_t>> externalCollisions =
                tc.getExternalCollisions();
        if (externalCollisions.size() > 0) {
            for (size_t i = 0; i < externalCollisions.size(); i++) {
            printf("External collision was detected between link %d and object %d\n",
                    (int)externalCollisions.at(i).first,
                    (int)externalCollisions.at(i).second);
            }
        }

        // Get current joint values
        tarsim::JointPositions_t jnt;
        if (!tc.getJointValues(jnt)) {
            printf("Failed to get joint values\n");
            return EXIT_FAILURE;
        } else {
            printf("Joint Values:\n");
            for (int i = 0; i < jnt.numJoints; i++) {
                printf("Joint (%d): %.2f\n", jnt.indices[i], jnt.positions[i]);
            }
        }
        
        // Set target joint values as a dummy trajectory.
        for (int32_t i = 0; i < jnt.numJoints; i++) {
            jnt.positions[i] += M_PI * sin(0.01 * (double)c * M_PI);
        }

        // Send the new set of joint position to Tarsim.
        if (!tc.sendJointPositions(jnt)) {
            printf("Failed to send joint values\n");
        }

        // Send a message to Tarsim to execute forward kinematics using the new joint positions
        if (!tc.executeForwardKinematics()) {
            printf("Failed to send execute forward kinematics\n");
        }

        // Get a rigid body (index of 1 for example) coordinate frame
        tarsim::Frame_t f;
        if (!tc.getRigidBodyFrame(1, 0, f)) {
            printf("Failed to get rigid body frame\n");
        } else {
            printFrame("Rigid Body", f.mij);
        }

        // Get end-effector coordinate frame
        if (!tc.getEndEffectorFrame(f)) {
            printf("Failed to get end-effector frame\n");
        } else {
            printFrame("End-Effector", f.mij);
        }

        // Get an object's coordinate frame. Here, object 1 is chosen as an example
        if (!tc.getObjectFrame(1, f)) {
            printf("Failed to get object frame\n");
        } else {
            printFrame("Object", f.mij);
        }

        // If you like to lock an object to a robot link (grasp) you can do it as:
        tc.lockObjectToRigidBody(1, 1);

        // If you like to unlock an object from the robot (release) you can do it as:
        tc.unlockObjectFromRigidBody(1);

        usleep(10000);
        c++;
    }

    // Optional: If you like to close tarsim, you can call this
    tc.shutdown();

    return EXIT_SUCCESS;
}



