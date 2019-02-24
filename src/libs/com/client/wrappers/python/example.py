from tarsimClient import Simulator
import time

sim = None
try:
    sim = Simulator()
except Exception as error:
    print('Caught this error: ' + repr(error))
except:
    print('Unknown error')

if sim is not None:
    counter = -10000
    while counter < 20000:
        if not sim.sendJointPositions(counter, 4, [0, 1, 2, 3], [0.01 * counter, 0.01 * counter, 0.01 * counter, 0.01 * counter]):
            print("Failed to send joint position")

        print(sim.getErrorMessage())

        if not sim.executeForwardKinematics():
            print("Failed to send execute forward kinematics")

        if not sim.sendRequestRigidBodyFrame(4, 0):
            print("Failed to send request rigid body frame")

        print(sim.getFrameRigidBody())

        counter += 200
        time.sleep(0.1)

del sim