#!/usr/bin/env/python
# coding: utf-8

import sys
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
    for i in range(3):#petlja koja dodaje blokove koji predstavljaju prostoriju
        blok1 = pybullet.loadURDF(
            "block.urdf",
            basePosition=[1, -1, 0.5+i],
            globalScaling=20.0)
        q=pybullet.getQuaternionFromEuler((0,0,pi/2))
    
        blok2 = pybullet.loadURDF(
            "block.urdf",
            basePosition=[1, 1, 0.5+i], 
            globalScaling=20.0)
        pybullet.resetBasePositionAndOrientation(blok2,[4.3,0,0.5+i],q)
        blok3 = pybullet.loadURDF(
            "block.urdf",
            basePosition=[1, 1, 0.5+i],
            globalScaling=20.0)
        blok4 = pybullet.loadURDF(
            "block.urdf",
            basePosition=[3, 1, 0.5+i],
            globalScaling=20.0)
        blok5 = pybullet.loadURDF(
            "block.urdf",
            basePosition=[-3, 1, 0.5+i],
            globalScaling=20.0)
        pybullet.resetBasePositionAndOrientation(blok5,[1.9,-2.2,0.5+i],q)
        blok6 = pybullet.loadURDF(
            "block.urdf",
            basePosition=[-3, 1, 0.5+i],
            globalScaling=20.0)
        pybullet.resetBasePositionAndOrientation(blok6,[4.3,-2.2,0.5+i],q)
        blok7 = pybullet.loadURDF(
            "block.urdf",
            basePosition=[3, -3.5, 0.5+i],
            globalScaling=20.0)





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
