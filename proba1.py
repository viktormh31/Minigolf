
import time
import pybullet as p
import pybullet_data
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())


plane_1 = p.loadURDF("plane.urdf")
p.changeDynamics(plane_1, -1 , mass = 0.0, restitution = 0.9)

#creating a sphere
def create_sphere():
    sphere_radius = 0.6
    sphere_collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius = sphere_radius)
    sphere_visual_shape = -1
    sphere_position = [0,0,.6]
    sphere_orientation = p.getQuaternionFromEuler([0,0,0])
    
    sphere_1 = p.createMultiBody(baseMass = 1.0, baseCollisionShapeIndex = sphere_collision_shape,
                                 baseVisualShapeIndex = sphere_visual_shape,
                                 basePosition = sphere_position,baseOrientation = sphere_orientation)
    p.changeDynamics(sphere_1,-1, restitution = .9, linearDamping = 0.1, 
                     angularDamping = 0.01,rollingFriction =.4, spinningFriction = 1,
                     lateralFriction = 0.01, mass = 5)
    return sphere_1

#creatin a sphere

"""
# importing a box
box_start_position = [2,0,2]
box_start_orientation = p.getQuaternionFromEuler([0,0,0])
urdf_file_path = '/home/viktor/Documents/custom urdf/box.urdf'
box_1 = p.loadURDF(urdf_file_path, basePosition = box_start_position, baseOrientation = box_start_orientation)
#importing a box
"""

def create_box():
    wall_dimensions = [1, 1, 1]
    wall_start_position = [3, 0, 1]
    wall_start_orientation = [0, 0, 0, 1]
    wall_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=wall_dimensions)
    wall_body_id = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=wall_id,
                                basePosition=wall_start_position, baseOrientation=wall_start_orientation)
    p.changeDynamics(wall_body_id, -1, restitution=.9)


#creating a box

def import_box(start_position):
    box_start_position = start_position
    box_start_orientation = p.getQuaternionFromEuler([0,0,0])
    urdf_file_path = '/home/viktor/Documents/custom urdf/box.urdf'
    box_1 = p.loadURDF(urdf_file_path, basePosition = box_start_position, baseOrientation = box_start_orientation)
    p.changeDynamics(box_1,-1,mass = 0.0, restitution = .9)
    return box_1


def create_soccerball():
    ball = p.loadURDF("soccerball.urdf",[0,0,1])
    p.changeDynamics(ball,-1,restitution = .9,linearDamping=0.001, angularDamping=0.001, rollingFriction=0.001, spinningFriction=0.001,mass = 2)
    p.changeVisualShape(ball,-1,rgbaColor=[0.8,0.8,0.8,1])
    #p.applyExternalForce(ball,-1, forceObj=[force_magnitude,0,0],posObj=[0,0,0], flags= p.WORLD_FRAME)

    return ball


#pomeranje
force_magnitude = 6000
time_step = 1 / 800.0

#p.setTimeStep(time_step)
#bola = create_soccerball()
bola2 = create_sphere()
box = create_box()
p.setGravity(0,0,-9.81)
#time.sleep(2)
p.applyExternalForce(bola2,-1, forceObj=[force_magnitude,0,0],posObj=[0,0,1], flags= p.WORLD_FRAME)

time.sleep(1)
#p.applyExternalForce(bola,-1, forceObj=[force_magnitude,0,0],posObj=[0,0,0], flags= p.WORLD_FRAME)
#p.changeDynamics(sphere_1,-1, rollingFriction=0.001, spinningFriction=0.001)
#p.applyExternalForce(sphere_1, -1, forceObj=[force_magnitude,0,0],posObj=[0,0,0], flags= p.WORLD_FRAME)
#pomeranje

for _ in range(1000):
    time.sleep(time_step)
    p.stepSimulation()
    print("hello")
input() 

