/**
 * @file: tarsimClientInterface.cpp
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
 */

//INCLUDES
#include "Python.h"

#include <cmath>
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/ndarrayobject.h>
#include <vector>
#include <wrappers/python/exposed/tarsimClientExposed.h>

namespace tarsim {
// FORWARD DECLARATIONS
// TYPEDEFS AND DEFINES
// ENUMS
// NAMESPACES AND STRUCTS
// CLASS DEFINITION
extern "C" {

struct module_state {
    PyObject *error;
};

#if PY_MAJOR_VERSION >= 3
#define PY3K
#define IS_PY3K
#define GETSTATE(m) ((struct module_state*)PyModule_GetState(m))
#else
#define GETSTATE(m) (&_state)
static struct module_state _state;
#endif

static PyObject* g_wrapperError = nullptr;

// Function that wraps python excpetion throwing
PyObject* InterfaceError(std::string msg)
{
    if (g_wrapperError == nullptr) {
        printf("Exceptions not initialized.");
        fflush(stdout);
        return nullptr;
    }

    PyErr_SetString(g_wrapperError, msg.c_str());
    return nullptr;
}

static PyObject* initialize_wrap(PyObject* /*self*/, PyObject* args)
{
    if (!initialize()) {
        return InterfaceError("Failed to initialize network.");
    }

    Py_RETURN_TRUE;
}

static PyObject* stop_wrap(PyObject* /*self*/, PyObject* args)
{
    if (!stop()) {
        return InterfaceError("Failed to stop network.");
    }

    Py_RETURN_TRUE;
}

static PyObject* connect_wrap(PyObject* /*self*/, PyObject* args)
{
    if (!connect()) {
        return InterfaceError("Failed to connect to simulator.");
    }

    Py_RETURN_TRUE;
}

static PyObject* shutdown_wrap(PyObject* /*self*/, PyObject* args)
{
    if (!shutdown()) {
        return InterfaceError("Failed to shutdown simulator.");
    }

    Py_RETURN_TRUE;
}

static PyObject* isSimulatorRunning_wrap(PyObject* /*self*/, PyObject* args)
{
    if (isSimulatorRunning()) {
        Py_RETURN_TRUE;
    }

    Py_RETURN_FALSE;
}

static PyObject* startRecording_wrap(PyObject* /*self*/, PyObject* args)
{
    if (!startRecording()) {
        return InterfaceError("Failed to start recording.");
    }

    Py_RETURN_TRUE;
}

static PyObject* stopRecording_wrap(PyObject* /*self*/, PyObject* args)
{
    if (!stopRecording()) {
        return InterfaceError("Failed to stop recording.");
    }

    Py_RETURN_TRUE;
}

static PyObject* sendJointPositions_wrap(PyObject* /*self*/, PyObject* args)
{
    int msgCounter = 0;
    unsigned int numJoints = 0;
    PyObject* jointIndices;
    PyObject* jointPositions;
    if (!PyArg_ParseTuple(args, "iiO!O!",
            &msgCounter, &numJoints, &PyTuple_Type, &jointIndices,
            &PyTuple_Type, &jointPositions)) {
        PyErr_Print();
        fflush(stderr);
        return InterfaceError("Failed to parse arguments.");
    }

    JointPositions_t msg;
    msg.msgCounter = (int32_t)msgCounter;
    msg.numJoints = numJoints;
    PyObject* obj;
    for (int i = 0; i < PyTuple_Size(jointIndices); i++) {
        obj = PyTuple_GetItem(jointIndices, i);
        int32_t val = PyFloat_AsDouble(obj);
        msg.indices[i] = val;
        if (PyErr_Occurred()) {
            return InterfaceError("Bad data type in joint indices.");
        }
    }

    for (int i = 0; i < PyTuple_Size(jointPositions); i++) {
        obj = PyTuple_GetItem(jointPositions, i);
        float val = PyFloat_AsDouble(obj);
        msg.positions[i] = val;
        if (PyErr_Occurred()) {
            return InterfaceError("Bad data type in joint indices.");
        }
    }

    if (!sendJointPositions(msg)) {
        return InterfaceError("Failed to send joint positions.");
    }

    Py_RETURN_TRUE;
}

static PyObject* getJointValues_wrap(PyObject* /*self*/, PyObject* args)
{
    size_t numJoints = 0;
    int timeout_period_us = 0;
    PyObject* jointIndices;
    PyObject* jointPositions;
    if (!PyArg_ParseTuple(args, "ii!O!i",
            &numJoints, &PyTuple_Type, &jointIndices,
            &PyTuple_Type, &jointPositions, &timeout_period_us)) {
        PyErr_Print();
        fflush(stderr);
        return InterfaceError("Failed to parse arguments.");
    }

    JointPositions_t msg;
    msg.numJoints = numJoints;
    PyObject* obj;
    for (int i = 0; i < PyTuple_Size(jointIndices); i++) {
        obj = PyTuple_GetItem(jointIndices, i);
        int32_t val = PyFloat_AsDouble(obj);
        msg.indices[i] = val;
        if (PyErr_Occurred()) {
            return InterfaceError("Bad data type in joint indices.");
        }
    }

    for (int i = 0; i < PyTuple_Size(jointPositions); i++) {
        obj = PyTuple_GetItem(jointPositions, i);
        float val = PyFloat_AsDouble(obj);
        msg.positions[i] = val;
        if (PyErr_Occurred()) {
            return InterfaceError("Bad data type in joint indices.");
        }
    }

    if (!getJointValues(msg, timeout_period_us)) {
        return InterfaceError("Failed to send joint positions.");
    }

    Py_RETURN_TRUE;
}

static PyObject* sendJointPosition_wrap(PyObject* /*self*/, PyObject* args)
{
    int index, msgCounter;
    double position;
    if (!PyArg_ParseTuple(args, "iid", &msgCounter, &index, &position)) {
        PyErr_Print();
        fflush(stderr);
        return InterfaceError("Failed to parse arguments.");
    }

    JointPosition_t msg;
    msg.index = index;
    msg.position = position;
    msg.msgCounter = (int32_t)msgCounter;

    if (!sendJointPosition(msg)) {
        return InterfaceError("Failed to send joint positions.");
    }

    Py_RETURN_TRUE;
}

static PyObject* sendBaseFrame_wrap(PyObject* /*self*/, PyObject* args)
{
    double rxx, rxy, rxz, tx, ryx, ryy, ryz, ty, rzx, rzy, rzz, tz;
    if (!PyArg_ParseTuple(args, "dddddddddddd",
            &rxx, &rxy, &rxz, &tx,
            &ryx, &ryy, &ryz, &ty,
            &rzx, &rzy, &rzz, &tz)) {
        PyErr_Print();
        fflush(stderr);
        return InterfaceError("Failed to parse arguments.");
    }

    Frame_t msg;
    msg.mij[0] = rxx;
    msg.mij[1] = rxy;
    msg.mij[2] = rxz;
    msg.mij[3] = tx;

    msg.mij[4] = ryx;
    msg.mij[5] = ryy;
    msg.mij[6] = ryz;
    msg.mij[7] = ty;

    msg.mij[8] = rzx;
    msg.mij[9] = rzy;
    msg.mij[10] = rzz;
    msg.mij[11] = tz;

    msg.mij[12] = 0.0;
    msg.mij[13] = 0.0;
    msg.mij[14] = 0.0;
    msg.mij[15] = 1.0;

    if (!sendBaseFrame(msg)) {
        return InterfaceError("Failed to send base frame.");
    }

    Py_RETURN_TRUE;
}


static PyObject* sendCamera_wrap(PyObject* /*self*/, PyObject* args)
{
    PyObject* position;
    PyObject* focalPoint;
    PyObject* viewUp;
    PyObject* clippingRange;

    if (!PyArg_ParseTuple(args, "O!O!O!O!",
            &PyTuple_Type, position,
            &PyTuple_Type, focalPoint,
            &PyTuple_Type, viewUp,
            &PyTuple_Type, clippingRange)) {
        PyErr_Print();
        fflush(stderr);
        return InterfaceError("Failed to parse arguments.");
    }

    Camera_t msg;
    PyObject* obj;
    for (int i = 0; i < PyTuple_Size(position); i++) {
        obj = PyTuple_GetItem(position, i);
        float val = PyFloat_AsDouble(obj);
        msg.position[i] = val;
        if (PyErr_Occurred()) {
            return InterfaceError("Bad data type in camera position.");
        }
    }

    for (int i = 0; i < PyTuple_Size(focalPoint); i++) {
        obj = PyTuple_GetItem(focalPoint, i);
        float val = PyFloat_AsDouble(obj);
        msg.focalPoint[i] = val;
        if (PyErr_Occurred()) {
            return InterfaceError("Bad data type in camera focalPoint.");
        }
    }

    for (int i = 0; i < PyTuple_Size(viewUp); i++) {
        obj = PyTuple_GetItem(viewUp, i);
        float val = PyFloat_AsDouble(obj);
        msg.viewUp[i] = val;
        if (PyErr_Occurred()) {
            return InterfaceError("Bad data type in camera viewUp.");
        }
    }

    for (int i = 0; i < PyTuple_Size(clippingRange); i++) {
        obj = PyTuple_GetItem(clippingRange, i);
        float val = PyFloat_AsDouble(obj);
        msg.clippingRange[i] = val;
        if (PyErr_Occurred()) {
            return InterfaceError("Bad data type in camera clippingRange.");
        }
    }

    if (!sendCamera(msg)) {
        return InterfaceError("Failed to send base frame.");
    }

    Py_RETURN_TRUE;
}

static PyObject* sendLockObjectToRigidBody_wrap(PyObject* /*self*/, PyObject* args)
{
    int indexObject, indexRigidBody;
    if (!PyArg_ParseTuple(args, "ii", indexObject, indexRigidBody)) {
        PyErr_Print();
        fflush(stderr);
        return InterfaceError("Failed to parse arguments.");
    }

    if (!lockObjectToRigidBody(indexObject, indexRigidBody)) {
        return InterfaceError("Failed to send lock object.");
    }

    Py_RETURN_TRUE;
}

static PyObject* sendUnlockObjectFromRigidBody_wrap(PyObject* /*self*/, PyObject* args)
{
    int indexObject;
    if (!PyArg_ParseTuple(args, "i", indexObject)) {
        PyErr_Print();
        fflush(stderr);
        return InterfaceError("Failed to parse arguments.");
    }

    if (!unlockObjectFromRigidBody(indexObject)) {
        return InterfaceError("Failed to send lock object.");
    }

    Py_RETURN_TRUE;
}

static PyObject* executeForwardKinematics_wrap(
        PyObject* /*self*/, PyObject* args)
{
    if (!executeForwardKinematics()) {
        return InterfaceError(
                "Failed to send request to execute forward kinematics.");
    }

    Py_RETURN_TRUE;
}

static PyObject* getEndEffectorFrame_wrap(
        PyObject* /*self*/, PyObject* args)
{
    Frame_t msg;

    if (!getEndEffectorFrame(msg)) {
        return InterfaceError(
                "Failed to send request to get end-effector frame");
    }

    Py_RETURN_TRUE;
}

static PyObject* getRigidBodyFrame_wrap(PyObject* /*self*/, PyObject* args)
{
    int indexRigidBody, indexFrame;
    if (!PyArg_ParseTuple(args, "ii", &indexRigidBody, &indexFrame)) {
        PyErr_Print();
        fflush(stderr);
        return InterfaceError("Failed to parse arguments.");
    }

    Frame_t msg;
    if (!getRigidBodyFrame((int32_t)indexRigidBody, (int32_t)indexFrame, msg)) {
        return InterfaceError(
                "Failed to send request to get rigid body frame.");
    }

    Py_RETURN_TRUE;
}

static PyObject* getObjectFrame_wrap(
        PyObject* /*self*/, PyObject* args)
{
    int indexObject;
    if (!PyArg_ParseTuple(args, "i", &indexObject)) {
        PyErr_Print();
        fflush(stderr);
        return InterfaceError("Failed to parse arguments.");
    }

    Frame_t msg;

    if (!getObjectFrame((int32_t)indexObject, msg)) {
        return InterfaceError(
                "Failed to send request to get object frame.");
    }

    Py_RETURN_TRUE;
}

static PyObject* setObjectFrame_wrap(
        PyObject* /*self*/, PyObject* args)
{
    int indexObject;
    double rxx, rxy, rxz, tx, ryx, ryy, ryz, ty, rzx, rzy, rzz, tz;
    if (!PyArg_ParseTuple(args, "idddddddddddd", &indexObject,
            &rxx, &rxy, &rxz, &tx,
            &ryx, &ryy, &ryz, &ty,
            &rzx, &rzy, &rzz, &tz)) {
        PyErr_Print();
        fflush(stderr);
        return InterfaceError("Failed to parse arguments.");
    }

    Frame_t msg;
    msg.mij[0] = rxx;
    msg.mij[1] = rxy;
    msg.mij[2] = rxz;
    msg.mij[3] = tx;

    msg.mij[4] = ryx;
    msg.mij[5] = ryy;
    msg.mij[6] = ryz;
    msg.mij[7] = ty;

    msg.mij[8] = rzx;
    msg.mij[9] = rzy;
    msg.mij[10] = rzz;
    msg.mij[11] = tz;

    msg.mij[12] = 0.0;
    msg.mij[13] = 0.0;
    msg.mij[14] = 0.0;
    msg.mij[15] = 1.0;

    if (!setObjectFrame((int32_t)indexObject, msg)) {
        return InterfaceError(
                "Failed to send request to get object frame.");
    }

    Py_RETURN_TRUE;
}

static PyObject* getErrorMessage_wrap(
        PyObject* /*self*/, PyObject* args)
{
    ErrorMessage_t m = getErrorMessage();
    return Py_BuildValue("iis#", m.msgCounter, m.errorId, m.errorMsg, m.msgLength);
}

static PyObject* displayMessage_wrap(
        PyObject* /*self*/, PyObject* args)
{
	const char* message;
	int level;
    if (!PyArg_ParseTuple(args, "si", message, &level)) {
        PyErr_Print();
        fflush(stderr);
        return InterfaceError("Failed to parse arguments.");
    }

    if (!displayMessage(message, level)) {
        return InterfaceError(
                "Failed to send request to get object frame.");
    }

    Py_RETURN_TRUE;
}

// An array specifying exactly which methods are wrappers.
// The syntax for each item is:
// {String for the method name when converted to python code,
//  Name of C++ wrapper method,
//  METH_VARARGS,
//  String which will go into the methods' docstrings in python}
static PyMethodDef TarsimClientInterface_methods[] = {
    {"initialize", initialize_wrap, METH_VARARGS,
    "Initializes communication"},

    {"stop", stop_wrap, METH_VARARGS,
    "Closes communication"},

    {"connect", connect_wrap, METH_VARARGS,
    "Connects to simulator"},

    {"shutdown", shutdown_wrap, METH_VARARGS,
    "Shuts down the simulator"},

    {"startRecording", startRecording_wrap, METH_VARARGS,
    "Start recording the simulator window"},

    {"stopRecording", stopRecording_wrap, METH_VARARGS,
    "Stop recording the simulator window"},

    {"isSimulatorRunning", isSimulatorRunning_wrap, METH_VARARGS,
    "Whether the simulator is running"},

    {"sendJointPositions", sendJointPositions_wrap, METH_VARARGS,
    "Sends joints positions"},

    {"sendJointPosition", sendJointPosition_wrap, METH_VARARGS,
    "Sends joint position"},

    {"getJointValues", getJointValues_wrap, METH_VARARGS,
    "Sends message to request current joint values and waits to receive them"},

    {"sendBaseFrame", sendBaseFrame_wrap, METH_VARARGS,
    "Sends base frame"},

    {"sendCamera", sendCamera_wrap, METH_VARARGS,
    "Sends camera data"},

    {"sendLockObjectToRigidBody", sendLockObjectToRigidBody_wrap, METH_VARARGS,
    "Sends message to lock an object to a rigid body"},

    {"sendUnlockObjectFromRigidBody", sendUnlockObjectFromRigidBody_wrap, METH_VARARGS,
    "Sends message to unlock an object from a rigid body"},

    {"executeForwardKinematics", executeForwardKinematics_wrap, METH_VARARGS,
    "Sends message to execute forward kinematics"},

    {"getEndEffectorFrame", getEndEffectorFrame_wrap, METH_VARARGS,
    "Sends message to request end-effector frame"},

    {"getRigidBodyFrame", getRigidBodyFrame_wrap, METH_VARARGS,
    "Sends message to request a rigid-body frame"},

    {"getObjectFrame", getObjectFrame_wrap, METH_VARARGS,
    "Sends message to request an object frame"},

    {"setObjectFrame", setObjectFrame_wrap, METH_VARARGS,
    "Sends message to set an object frame"},

    {"getErrorMessage", getErrorMessage_wrap, METH_VARARGS,
    "Gets the current fault message"},

    {"displayMessage", displayMessage_wrap, METH_VARARGS,
    "Displays a text message on the simulator fault scene"},

    {nullptr, nullptr, 0, nullptr}};

#if PY_MAJOR_VERSION >= 3

static int TarsimClientInterface_traverse(PyObject *m, visitproc visit, void *arg) {
    Py_VISIT(GETSTATE(m)->error);
    return 0;
}

static int TarsimClientInterface_clear(PyObject *m) {
    Py_CLEAR(GETSTATE(m)->error);
    return 0;
}

// Define module info. Note: See python docs for syntax.
static struct PyModuleDef moduledef = {PyModuleDef_HEAD_INIT,
                                         "TarsimClientInterface",
                                         nullptr, // Documention.
                                         sizeof(struct module_state),
                                         TarsimClientInterface_methods,
                                         nullptr,
                                         TarsimClientInterface_traverse,
                                         TarsimClientInterface_clear,
                                         nullptr};

#define INITERROR return NULL
PyMODINIT_FUNC PyInit_TarsimClientInterface(void)
#else
#define INITERROR return
void initTarsimClientInterface(void)
#endif
{
    import_array();

#if PY_MAJOR_VERSION >= 3
    PyObject* module = PyModule_Create(&moduledef);
#else
    PyObject *module = Py_InitModule("TarsimClientInterface", TarsimClientInterface_methods);
#endif

    if (module == nullptr)
        INITERROR;

    struct module_state *st = GETSTATE(module);

    char errorString[] = "TarsimClient.Error";
    st->error = PyErr_NewException(errorString, nullptr, nullptr);
    if (st->error == NULL) {
        Py_DECREF(module);
        INITERROR;
    }

    Py_INCREF(st->error);
    PyModule_AddObject(module, "error", st->error);

#if PY_MAJOR_VERSION >= 3
    return module;
#endif
}

}; // end of namespace tarsim

} //extern "C"









