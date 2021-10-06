#!/usr/bin/env/python
# coding: utf-8

import sys
import cv2
import rospy
import pybullet
import pybullet_data
from qibullet import SimulationManager
from qibullet import PepperVirtual
from qibullet import PepperRosWrapper
from math import *

if __name__ == "__main__":
    simulation_manager = SimulationManager()

    #Pokretanje instance simulacije s grafičkim prikazom
    client_id = simulation_manager.launchSimulation(gui=True)

    #stvaranje Peppera u ishodištu koordinatnog sustava simulacijskog svijeta
    pepper = simulation_manager.spawnPepper(
          client_id,
          translation=[0, 0, 0],
          quaternion=[0, 0, 0, 1],
          spawn_ground_plane=True)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    for i in range(3):
        z=0.4*(i+1)
        blok1 = pybullet.loadURDF(
            "block.urdf",
            basePosition=[1, -1, z],
            globalScaling=20.0)
        q=pybullet.getQuaternionFromEuler((0,0,pi/2))
    
        blok2 = pybullet.loadURDF(
            "block.urdf",
            basePosition=[1, 1, z], 
            globalScaling=20.0)    
        blok3 = pybullet.loadURDF(
            "block.urdf",
            basePosition=[-2, -2, z],
            globalScaling=20.0)
        pybullet.resetBasePositionAndOrientation(blok3,[1.82,2.18,z],q)
        blok4 = pybullet.loadURDF(
            "block.urdf",
            basePosition=[-2, -2, z],
            globalScaling=20.0)
        pybullet.resetBasePositionAndOrientation(blok4,[1.82,-2.18,z],q)

        for j in range(3):
            blok5 = pybullet.loadURDF(
                "block.urdf",
                basePosition=[2.64+2*j, 3.36, z],
                globalScaling=20.0)
            blok6 = pybullet.loadURDF(
                "block.urdf",
                basePosition=[2.64+2*j, -3.36, z],
                globalScaling=20.0)

        blok7 = pybullet.loadURDF(
            "block.urdf",
            basePosition=[-2, -2, z],
            globalScaling=20.0)
        pybullet.resetBasePositionAndOrientation(blok7,[7.46,2.18,z],q)
        blok8 = pybullet.loadURDF(
            "block.urdf",
            basePosition=[-2, -2, z],
            globalScaling=20.0)
        pybullet.resetBasePositionAndOrientation(blok8,[7.46,-2.18,z],q)
        blok9 = pybullet.loadURDF(
            "block.urdf",
            basePosition=[6.64, 1, z],
            globalScaling=20.0)
        blok10 = pybullet.loadURDF(
            "block.urdf",
            basePosition=[-2, -2, z],
            globalScaling=20.0)
        pybullet.resetBasePositionAndOrientation(blok10,[5.46,-2.18,z],q)
        blok10 = pybullet.loadURDF(
            "block.urdf",
            basePosition=[-2, -2, z],
            globalScaling=20.0)
        pybullet.resetBasePositionAndOrientation(blok10,[3.46,-2.18,z],q)
        blok10 = pybullet.loadURDF(
            "block.urdf",
            basePosition=[-2, -2, z],
            globalScaling=20.0)
        pybullet.resetBasePositionAndOrientation(blok10,[4,2.18,z],q)


        blok9 = pybullet.loadURDF(
            "block.urdf",
            basePosition=[-2, -2, z],
            globalScaling=20.0)
        pybullet.resetBasePositionAndOrientation(blok9,[7.46,-0.18,z],q)
        

    pepper.setAngles("HeadPitch",  0.2, 0.5) #zakret pepperove glave, kako bi željene prepreke korištenje kod mapiranja dubinskom kamerom bile u vidnom polju
    wrap=PepperRosWrapper()
    wrap.launchWrapper(pepper, "/naoqi_driver") #pokretanje ROS Wrappera
    camera_id = PepperVirtual.ID_CAMERA_DEPTH
    handle = pepper.subscribeCamera(camera_id)#subscribe na dubinsku kameru, kako bismo dobili slike
    pepper.showLaser(True) #Prikaz lasera u grafičkom sučelju simulatora
    pepper.subscribeLaser()#subscribe na lasere, kako bismo dobili iz simulatora njihova očitanja

    try:
        rospy.spin()
    
    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass
    finally:
        #zaustavljanje Wrappera i simulatora u slučaju kraja 
        wrap.stopWrapper() 
        simulation_manager.stopSimulation(client_id)





    
