import numpy as np
import pybullet as p
import pybullet_data
import time

# Basic setup to test pybullet
try:
    print("Initializing PyBullet...")
    # Connect to PyBullet in GUI mode
    p.connect(p.GUI)

    p.resetSimulation()
    
    # Set search path for URDFs
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.setGravity(0, 0, -9.8)

    p.setRealTimeSimulation(0)

    
    # Load a ground plane
    plane_id = p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])

    
    # Load a cube
    # cube_id = p.loadURDF("cube.urdf", [0, 0, 1], [0, 0, 0, 1])
    
    # Set the cube's position and orientation
    # Note: p.setPosition and p.setQuaternion are not standard PyBullet functions.
    # Using p.resetBasePositionAndOrientation instead.
    # p.resetBasePositionAndOrientation(cube_id, [0, 0, 1], [0, 0, 0, 1])
    
    target_id = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], [0, 0, 0, 1], useFixedBase=True)

    obj_of_focus = target_id

    p.getNumJoints(target_id)

    #for i in range(p.getNumJoints(target_id)):
    #    print(p.getJointInfo(target_id, i))
    #    print("--------------------------------")

    jointid = 4
    jtype = p.getJointInfo(target_id, jointid)[2]
    jlower = p.getJointInfo(target_id, jointid)[8]
    jupper = p.getJointInfo(target_id, jointid)[9]
    jmaxforce = p.getJointInfo(target_id, jointid)[10]
    jmaxvel = p.getJointInfo(target_id, jointid)[11]
    jinfo = p.getJointInfo(target_id, jointid)
    # print(jlower, jupper)
    # print("--------------------------------")
    
    # changing the joint angles

    for step in range(500):
        joint_two_target = np.random.uniform(jlower, jupper)
        joint_four_target = np.random.uniform(jlower, jupper)
        p.setJointMotorControlArray(target_id, [2,4], p.POSITION_CONTROL, targetPositions= [joint_two_target, joint_four_target])
        p.stepSimulation()
        # if you need to query joint states or link states to update the observation
        # print(p.getLinkStates(target_id), [2, 4])
    


    
    print("PyBullet initialized successfully. Running simulation for 2 seconds...")
    
    # Run simulation loop
    for _ in range(240 * 2):  # Run for 2 seconds (assuming 240Hz default)

        focus_position, _ = p.getBasePositionAndOrientation(obj_of_focus)
        p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=focus_position)
        p.stepSimulation()
        time.sleep(1./240.)
        
    p.disconnect()
    print("Test completed successfully.")
    
except Exception as e:
    print(f"An error occurred: {e}")
