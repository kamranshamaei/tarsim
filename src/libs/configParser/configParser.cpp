/*
 *
 * @file: configParser.cpp
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

//INCLUDES
#include "configParser.h"
#include "fileSystem.h"
#include "logClient.h"
namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
ConfigParser::ConfigParser(std::string configFolderName)
{
    m_configFolderName = configFolderName;
    std::string dummyString = "";

    if (!FileSystem::splitFilename(
        FileSystem::getexepath(), m_winDefaultFileName, dummyString)) {
      m_winDefaultFileName = "";
    }
    m_winDefaultFileName = m_winDefaultFileName + "default/win.txt";

    // Load rigid body system configuration data
    m_rbs = new RigidBodySystem();

    std::string configFileName = configFolderName + "/rbs.txt";
    if (!FileSystem::loadProtoFile(configFileName, m_rbs)) {
        throw std::invalid_argument("Failed to load the simulator config data");
    }

    if (m_rbs == nullptr) {
        throw std::invalid_argument("Specified file has no rigid body system data");
    }

    LOG_INFO("Rigid body system data were loaded from file %s\n",
            configFileName.c_str());

    // Load gui configuration data
    m_win = new Window();

    configFileName = configFolderName + "/win.txt";
    if (!FileSystem::loadProtoFile(configFileName, m_win)) {
        LOG_INFO("Failed to load window config data from the specified folder, "
                "trying default %s", m_winDefaultFileName.c_str());
        if (!FileSystem::loadProtoFile(m_winDefaultFileName, m_win)) {
            throw std::invalid_argument("Failed to load the simulator config data");
        }
    }

    if (m_win == nullptr) {
        throw std::invalid_argument("Specified file has no window data");
    }

    LOG_INFO("Window data were loaded from file %s\n",
            configFileName.c_str());

    if (parseRigidBodySystem() != NO_ERR) {
        throw std::invalid_argument("No rigid body specified for current node");
    }

    if (NO_ERR != findEndEffectorNode()) {
        throw std::invalid_argument("Failed to find end effector node");
    }

    LOG_INFO("Rigid body system was successfully represented as a tree:\n");
    printf("Robot %s was processed as a rigid body system:\n", m_rbs->name().c_str());
    usleep(10000);
    printTree(m_root);
}

ConfigParser::~ConfigParser()
{
    delete m_tool;
    m_tool = nullptr;

    cleanTree();

    delete m_rbs;
    m_rbs = nullptr;

    delete m_win;
    m_win = nullptr;
}

Errors ConfigParser::parseRigidBodySystem()
{
    if (verifyRigidBodySystem() != NO_ERR) {
        LOG_FAILURE("Failed to verify rigid body system\n");
        return ERR_INVALID;
    }

    if (createRigidBodySystemTree() != NO_ERR) {
        LOG_FAILURE("Failed to verify rigid body system\n");
        return ERR_INVALID;
    }

    return NO_ERR;
}

Errors ConfigParser::verifyRigidBodySystem()
{
    if (verifyBase() != NO_ERR) {
        LOG_FAILURE("Failed to verify base rigid body\n");
        return ERR_INVALID;
    }

    if (verifyMates() != NO_ERR) {
        LOG_FAILURE("Failed to verify base rigid body\n");
        return ERR_INVALID;
    }

    return NO_ERR;
}

Errors ConfigParser::verifyBase()
{
    int rbCounter = 0;
    int index = -1;
    for (int r = 0; r < m_rbs->rigid_bodies_size(); r++) {
        if (m_rbs->rigid_bodies(r).is_fixed()) {
            index = r;
            rbCounter++;
        }
    }

    if (rbCounter != 1) {
        LOG_FAILURE("Multiple rigid bodies (%d) are fixed\n", rbCounter);
        return ERR_INVALID;
    }

    m_root = new Node(
        nullptr, m_rbs->rigid_bodies(index), Mate(), true, m_configFolderName, 0.0);

    m_mapRbToNode.insert(std::pair<int, Node*>(
            m_rbs->rigid_bodies(index).index(), m_root));

    return NO_ERR;
}

Errors ConfigParser::verifyMates()
{
    for (int i = 0; i < m_rbs->mates_size(); i++) {
        // Make sure mate indices exist
        if (verifyMate(m_rbs->mates(i))) {
            LOG_FAILURE("At least one mate index cannot be verified\n");
            return ERR_INVALID;
        }
    }

    return NO_ERR;
}

Errors ConfigParser::verifyMate(const Mate &mate)
{
    if (mate.sides_size() != 2) {
        LOG_FAILURE("Number of mates sides (%d) must be 2\n", mate.sides_size());
        return ERR_INVALID;
    }

    // Make sure mate's indices are not equal
    if (mate.sides(0).rigid_body_index() ==
        mate.sides(1).rigid_body_index()) {
        LOG_FAILURE("Mate bearing and shaft indices are equal");
        return ERR_INVALID;
    }

    // Check if the rigid body and joint exist
    bool doesRigidBodyExist[2] = {false, false};
    bool doesJointExist[2] = {false, false};

    for (int s = 0; s < 2; s++) {
        int rbCounter = 0;
        for (int r = 0; r < m_rbs->rigid_bodies_size(); r++) {
            if (mate.sides(s).rigid_body_index() == m_rbs->rigid_bodies(r).index()) {
                rbCounter++;

                int jointCounter = 0;
                for (int j = 0; j < m_rbs->rigid_bodies(r).joints_size(); j++) {
                    if (mate.sides(s).joint_index() == m_rbs->rigid_bodies(r).joints(j).index()) {
                        jointCounter++;
                    }
                }

                if (jointCounter == 1) {
                    doesJointExist[s] = true;
                } else {
                    LOG_FAILURE("Number of matched joints (%d) must be 1\n",
                            jointCounter);
                }
            }
        }

        if (rbCounter == 1) {
            doesRigidBodyExist[s] = true;
        } else {
            LOG_INFO("Number of matched rigid bodies (%d) must be 1\n",
                    rbCounter);
        }
    }

    if (!doesRigidBodyExist[0] || !doesJointExist[0] ||
        !doesRigidBodyExist[1] || !doesJointExist[1]) {
        LOG_FAILURE("Mate bearing and shaft indices are equal");
        return ERR_INVALID;
    }
    return NO_ERR;
}


Errors ConfigParser::createRigidBodySystemTree()
{
    readPreviousJointValues();
    if (createChildren(m_root, m_root) != NO_ERR) {
        LOG_FAILURE("Failed to create the tree");
        return ERR_INVALID;
    }
    return NO_ERR;
}

Errors ConfigParser::readPreviousJointValues()
{
    std::ifstream file(m_configFolderName + "/"+ k_previousJointValuesFile);
    if (file) {
        if (file.is_open()) {
            std::string indexStr, valueStr;
            while ( getline (file, indexStr, ' ') && getline (file, valueStr)) {
                m_mapPreviousJointValues[std::stoi(indexStr)] = std::stod(valueStr);
            }
            file.close();
        }
    }
    return NO_ERR;
}

Errors ConfigParser::createChildren(
        Node* root, Node* currentNode)
{
    if ((root == nullptr) || (currentNode == nullptr)) {
        LOG_FAILURE("Null pointer received\n");
        return ERR_INVALID;
    }

    for (int i = 0; i < m_rbs->mates_size(); i++) {
        // If the mates is not already in the tree (mate <--> tree branch)
        if (m_mapMateToNode.count(m_rbs->mates(i).index()) == 0) {
            int childIndex = -1;
            // If the current node the bearing, then the child is the shaft
            if (currentNode->getRigidBody()->index() ==
                    m_rbs->mates(i).sides(0).rigid_body_index()) {
                childIndex = m_rbs->mates(i).sides(1).rigid_body_index();
            // Else if the current node is the shaft of the mate, then the child
            // is the bearing side
            } else if (currentNode->getRigidBody()->index() ==
                    m_rbs->mates(i).sides(1).rigid_body_index()) {
                childIndex = m_rbs->mates(i).sides(0).rigid_body_index();
            }

            // If the child index was found using the mate sides
            if (childIndex >= 0) {
                if (isInTree(childIndex, root)) {
                    LOG_FAILURE("Rigid body %d causes a closed chain\n", childIndex);
                    return ERR_INVALID;
                } else {
                    double jntValue = 0.0;
                    if ((m_mapPreviousJointValues.end() !=
                        m_mapPreviousJointValues.find(
                                m_rbs->mates(i).index())) &&
                        (m_rbs->should_start_with_previous_joint_values())) {
                        jntValue = m_mapPreviousJointValues[m_rbs->mates(i).index()];
                    } else {
                        jntValue = m_rbs->mates(i).value();
                    }

                    size_t index = 0;
                    for (size_t j = 0; j < m_rbs->rigid_bodies_size(); j++) {
                      if (childIndex == (int)m_rbs->rigid_bodies(j).index()) {
                        index = j;
                      }
                    }
                    // Generate a new child (node)
                    Node* newNode = new Node(
                            currentNode,
                            m_rbs->rigid_bodies(index),
                            m_rbs->mates(i),
                            false,
                            m_configFolderName,
                            jntValue);

                    // Add the child rigid body to the map so that we know
                    // where it is
                    m_mapRbToNode.insert(std::pair<int, Node*>(
                            newNode->getRigidBody()->index(), newNode));

                    // Map the mate so that we know where it is when we want
                    // to set the value of the mate
                    m_mapMateToNode.insert(std::pair<int, Node*>(
                            newNode->getMateToParent()->index(), newNode));

                    if (createChildren(root, newNode) != NO_ERR) {
                        LOG_FAILURE("Failed to create children for node %d\n",
                                newNode->getRigidBody()->index());
                        return ERR_INVALID;
                    }
                }
            }
        }
    }

    return NO_ERR;
}

bool ConfigParser::isInTree(int index, Node* node)
{
    if (node->getRigidBody()->index() == index) {
        return true;
    }

    for (unsigned int i = 0; i < node->getChildren().size(); i++) {
        return isInTree(index, node->getChildren().at(i));
    }

    return false;
}

RigidBodySystem* ConfigParser::getRbs()
{
    return m_rbs;
}

Window* ConfigParser::getWin()
{
    return m_win;
}

Node* ConfigParser::getRoot()
{
    return m_root;
}

Node* ConfigParser::getNodeOfRigidBody(int index)
{
    std::unique_lock<std::mutex> lock(m_mutexMapNodes);
    auto it = m_mapRbToNode.find(index);
    if (it == m_mapRbToNode.end()) {
        LOG_FAILURE("Rigid body %d does not exist", index);
        return nullptr;
    }
    Node* node = it->second;
    return node;
}


Node* ConfigParser::getNodeOfMate(int index)
{
    std::unique_lock<std::mutex> lock(m_mutexMapNodes);
    auto it = m_mapMateToNode.find(index);
    if (it == m_mapMateToNode.end()) {
        LOG_FAILURE("Mate %d does not exist", index);
        return nullptr;
    }
    Node* node = it->second;
    return node;
}

void ConfigParser::depthPush(std::string c)
{
    m_depth.insert(m_depthCounter++, " ");
    m_depth.insert(m_depthCounter++,  c );
    m_depth.insert(m_depthCounter++, " ");
    m_depth.insert(m_depthCounter++, " ");
    m_depth.erase(m_depthCounter, k_depthMaxLength - m_depthCounter);
}

void ConfigParser::depthPop( )
{
    m_depth.erase(m_depthCounter -= 4, k_depthMaxLength - m_depthCounter);
}

void ConfigParser::printTree(Node* node)
{
    printf("(%d)\n", node->getRigidBody()->index());

    for (unsigned int i = 0; i < node->getChildren().size(); i++) {
        printf("%s └─", m_depth.c_str());

        if (i + 1 < node->getChildren().size()) {
            depthPush("|");
        } else {
            depthPush(" ");
        }

        printTree(node->getChildren().at(i));
        depthPop();
    }
}

Errors ConfigParser::findEndEffectorNode()
{
    if (NO_ERR != findEndEffectorNodeHere(m_root)) {
        LOG_FAILURE("Failed to add joint values actor to the scene");
        return ERR_INVALID;
    }

    return NO_ERR;
}

Errors ConfigParser::findEndEffectorNodeHere(Node* node)
{
    if (node == nullptr) {
        LOG_FAILURE("Empty node was received");
        return ERR_INVALID;
    }

    for (unsigned int i = 0; i < node->getRigidBody()->frames_size(); i++) {
        if (node->getRigidBody()->frames(i).is_end_effector()) {
            m_endEffectorNode = node;
            m_endEffectorFrameNumber = i;
            return NO_ERR;
        }
    }

    for (unsigned int i = 0; i < node->getChildren().size(); i++) {
        if (NO_ERR != findEndEffectorNodeHere(node->getChildren().at(i))) {
            LOG_FAILURE("Failed to find end effector");
            return ERR_INVALID;
        }
    }
    return NO_ERR;
}

Node* ConfigParser::getEndEffectorNode()
{
    return m_endEffectorNode;
}

unsigned int ConfigParser::getEndEffectorFrameNumber()
{
    return m_endEffectorFrameNumber;
}

void ConfigParser::cleanTree()
{
    if (m_rbs->should_start_with_previous_joint_values()) {
        std::ofstream file;
        std::string fileName = m_configFolderName + "/"+ k_previousJointValuesFile;
        file.open (fileName);
        deleteNodes(m_root, file);
        file.close();
    }
}

void ConfigParser::deleteNodes(Node* node, std::ofstream &file)
{
    for (unsigned int i = 0; i < node->getChildren().size(); i++) {
        deleteNodes(node->getChildren().at(i), file);
    }
    if (node != m_root) {
        file << node->getMateToParent()->index() << " " << node->getCurrentJointValue() << "\n";
    }
    delete node;
    node = nullptr;
}

Errors ConfigParser::loadTool(const std::string &toolName)
{
    std::string toolFileName = m_configFolderName + "/" + toolName;


    ExternalObject obj;
    if (!FileSystem::loadProtoFile(toolFileName, &obj)) {
        LOG_FAILURE("Failed to load the simulator config data from %s",
            toolFileName.c_str());
        return ERR_INVALID;
    }


    // Generate a new child (node)
    delete m_tool;
    m_tool = new Object(obj, m_configFolderName);

    return NO_ERR;
}

} // end of namespace tarsim










