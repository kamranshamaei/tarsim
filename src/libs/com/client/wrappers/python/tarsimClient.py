import libtarsimClientInterfaceLib

class Simulator():
    def __init__(self):
        self.interface = tarsimClientInterface
        if not self.initialize():
            raise Exception('Failed to initialize connection')

    def __del__(self):
        if not self.interface.stop():
            print("Failed to stop network")

    def initialize(self):
        return self.interface.initialize()

    def isSimulatorRunning(self):
        return self.interface.isSimulatorRunning()

    def startRecording(self):
        return self.interface.startRecording()
    
    def stopRecording(self):
        return self.interface.stopRecording()
    
    def shutdown(self):
        return self.interface.shutdown()

    def sendJointPositions(self, msgCounter, numJoints, jointIndices, jointValues):
        return self.interface.sendJointPositions(
            msgCounter, numJoints, tuple(jointIndices), tuple(jointValues))

    def sendJointPosition(self, msgCounter, jointIndex, jointValue):
        return self.interface.sendJointPosition(msgCounter, jointIndex, jointValue)

    def sendJointPosition(self, jointIndex, jointValue):
        return self.interface.sendJointPosition(jointIndex, jointValue)

    def getJointValues(self, numJoints, jointIndices, jointPositions):
        return self.interface.getJointValues(jointIndices, jointPositions)

    def sendBaseFrame(self, 
        rxx, rxy, rxz, tx, 
        ryx, ryy, ryz, ty, 
        rzx, rzy, rzz, tz):
        return self.interface.sendBaseFrame(
            rxx, rxy, rxz, tx, ryx, ryy, ryz, ty, rzx, rzy, rzz, tz)

    def setObjectFrame(self, 
        rxx, rxy, rxz, tx, 
        ryx, ryy, ryz, ty, 
        rzx, rzy, rzz, tz):
        return self.interface.setObjectFrame(
            indexObject, rxx, rxy, rxz, tx, ryx, ryy, ryz, ty, rzx, rzy, rzz, tz)

    def sendCamera(self, 
        position=[1000.0, 0.0, 0.0], 
        focalPoint=[0.0, 0.0, 0.0], 
        viewUp=[0.0, 0.0, 1.0], 
        clippingRange=[0.0, 5000.0]):
        return self.interface.sendCamera(
            position, focalPoint, viewUp, clippingRange)

    def sendLockObjectToRigidBody(self, indexObject, indexRigidBody):
        return self.interface.sendLockObjectToRigidBody(indexObject, indexRigidBody)

    def sendUnlockObjectFromRigidBody(self, indexObject):
        return self.interface.sendUnlockObjectFromRigidBody(indexObject)

    def executeForwardKinematics(self):
        return self.interface.executeForwardKinematics()

    def sendRequestEndEffectorFrame(self):
        return self.interface.sendRequestEndEffectorFrame()

    def sendRequestRigidBodyFrame(self, indexRigidBody, indexFrame):
        return self.interface.sendRequestRigidBodyFrame(indexRigidBody, indexFrame)

    def sendRequestObjectFrame(self, indexObject):
        return self.interface.sendRequestObjectFrame(indexObject)

    def getFrameEndEffector(self):
        return self.interface.getFrameEndEffector()

    def getFrameRigidBody(self):
        return self.interface.getFrameRigidBody()

    def getFrameObject(self):
        return self.interface.getFrameObject()

    def getErrorMessage(self):
        return self.interface.getErrorMessage()





