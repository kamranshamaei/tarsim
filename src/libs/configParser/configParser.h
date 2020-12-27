/**
 *
 * @file: configParser.h
 *
 * @Created on: March 31, 2018
 * @Author: Kamran Shamaei
 *
 *
 * @brief -
 * <Requirement Doc Reference>
 * <Design Doc Reference>
 *
 * @copyright Copyright [2017-2018] Kamran Shamaei .
 * All Rights Reserved.
 *
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE', which is part of this source code package.
 *
 */

// IFNDEF
#ifndef CONFIG_PARSER_H
#define CONFIG_PARSER_H

//INCLUDES
#include "node.h"
#include "rbs.pb.h"
#include "win.pb.h"
#include "object.h"
#include <mutex>
#include <fstream>
namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
class ConfigParser
{
public:
    // FUNCTIONS
    ConfigParser(std::string configFolderName);
    virtual ~ConfigParser();

    RigidBodySystem* getRbs();
    Window* getWin();
    Node* getRoot();
    Node* getNodeOfRigidBody(int index);
    Node* getNodeOfMate(int index);
    Node* getEndEffectorNode();
    unsigned int getEndEffectorFrameNumber();
    std::string getConfigFolderName() {return m_configFolderName;}
    Errors loadTool(const std::string &toolName);
    Object* getTool() {return m_tool;}

    // MEMBERS
private:
    // FUNCTIONS
    Errors parseRigidBodySystem();
    Errors verifyRigidBodySystem();
    Errors verifyBase();
    Errors verifyMates();
    Errors verifyMate(const Mate &mate);
    Errors createRigidBodySystemTree();
    Errors readPreviousJointValues();
    Errors createChildren(Node* root, Node* currentNode);
    bool isInTree(int childIndex, Node* node);
    void printTree(Node* node);
    void depthPush(std::string c);
    void depthPop();
    Errors findEndEffectorNode();
    Errors findEndEffectorNodeHere(Node* node);
    void cleanTree();
    void deleteNodes(Node* node, std::ofstream &file);

    // MEMBERS
    RigidBodySystem* m_rbs = nullptr;
    Object* m_tool = nullptr;
    Window* m_win = nullptr;
    Node* m_root = nullptr;
    Node* m_endEffectorNode = nullptr;
    unsigned int m_endEffectorFrameNumber = 0;
    int m_rootRigidBodyIndex = -1;

    mutable std::mutex m_mutexMapNodes;
    std::map<int, Node*> m_mapRbToNode {};
    std::map<int, Node*> m_mapMateToNode {};

    std::string m_depth = "";
    int m_depthCounter = 0;

    unsigned int k_depthMaxLength = 2056;

    std::string m_configFolderName = "";
    // TODO: Specify during installation/packaging
    std::string m_winDefaultFileName = "";

    std::map<int32_t, double> m_mapPreviousJointValues {};
    const std::string k_previousJointValuesFile = "prevJointValues.txt";
};
} // end of namespace tarsim
// ENDIF
#endif /* CONFIG_PARSER_H */
