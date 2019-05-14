# Introduction
Tarsim is a kinematic realtime simulator that can simulate all open-chain 
robots. An open-chain robot is a robot that its links do not create a closed
chain. For example a serial manipulator or a robotic hand are open-chain robots, 
whereas a delta robot is a closed-chain robot. Tarsim models a robot as a
rigid-body system, which is a combination of rigid bodies that are mated 
together.

# What Makes it Realtime
Tarsim is designed to be extremely fast in that it can calculate the robot
pose very quickly. It also has time measurement calculations that can report
issue in timing. For example if the data are received later or sooner than
expected.

# How to Use
Tarsim is designed to be almost fully reconfigurable. In order to use Tarsim, 
you need to follow these steps:

## Configure
Tarsim uses two configuration files to operate: rbs.txt that includes the
rigid-body definition of the robot and win.txt that includes the attributes
of its gui. The first step to run Tarsim is to configure your robot. Tarsim
looks at any robot as a combination of a series of rigid bodies that are mated 
together. So, the user should define all the rigid bodies and their attributes
first and then define the mates that define the robot joints. Each mate is 
composed of two joints; one from each mate side. Each rigid body also has a
series of attributes that define how it should appear in the scene. 

To configure your robot, create a folder with robot name "my_robot" and copy 
rbs.txt in it. Fill out the rbs.txt fields as described in it. 
You can find an example in the samples folder. You don't need to define win.txt
because if the simulator cannot find it, it will use a default. But, if you'd
like to change the way the gui looks, you can almost change every attribute of
the gui such as colors, location of scenes, camera view, etc.

## Run   
To run the simulator, you need to run tarsim and point to where it should find
the configuration files of your robot. For example:
```
cd path/to/tarsim
./tarsim -c path/to/my_robot
```

# Developer
Tarsim was developed by Kamran Shamaei, PhD. Please send your questions to
shamaei@hotmail.com
