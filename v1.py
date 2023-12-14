
import time
import pybullet as p
import pybullet_data
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())


plane_1 = p.loadURDF("plane.urdf")
p.changeDynamics(plane_1, -1 , mass = 0.0, restitution = 0.9)

#creating a sphere
def create_sphere(radius, start_position):
    sphere_radius = radius
    sphere_collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius = sphere_radius)
    sphere_visual_shape = -1
    sphere_position = start_position
    sphere_orientation = p.getQuaternionFromEuler([0,0,0])
    
    sphere_1 = p.createMultiBody(baseMass = 1.0, baseCollisionShapeIndex = sphere_collision_shape,
                                 baseVisualShapeIndex = sphere_visual_shape,
                                 basePosition = sphere_position,baseOrientation = sphere_orientation)
    p.changeDynamics(sphere_1,-1, restitution = .9, linearDamping = 0.1, 
                     angularDamping = 0.01,rollingFriction =.4, spinningFriction = 1,
                     lateralFriction = 0.01, mass = 5)
    return sphere_1

def import_teren(start_position):
    urdf_file_path= '/home/viktor/Documents/DIPLOMSKI/custom urdf/podloga.urdf'
    teren = p.loadURDF(urdf_file_path,start_position,p.getQuaternionFromEuler([0,0,0]))
    p.changeDynamics(teren,-1,mass = 0.0, restitution = .3)
    return teren

#creating a box
def create_box(dimensions, start_position, start_orientation):
    wall_dimensions = dimensions
    wall_start_position = start_position
    wall_start_orientation = start_orientation
    wall_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=wall_dimensions)
    wall_body_id = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=wall_id,
                                basePosition=wall_start_position, baseOrientation=wall_start_orientation)
    p.changeDynamics(wall_body_id, -1, restitution=.9)


#importing a box
def import_box(start_position):
    box_start_position = start_position
    box_start_orientation = p.getQuaternionFromEuler([0,0,0])
    urdf_file_path = '/home/viktor/Documents/DIPLOMSKI/custom urdf/box.urdf'
    box_1 = p.loadURDF(urdf_file_path, basePosition = box_start_position, baseOrientation = box_start_orientation)
    p.changeDynamics(box_1,-1,mass = 0.0, restitution = .9)
    return box_1

#importing a soccerball
def create_soccerball():
    ball = p.loadURDF("soccerball.urdf",[0,0,1])
    p.changeDynamics(ball,-1,restitution = .9,linearDamping=0.001, angularDamping=0.001, rollingFriction=0.001, spinningFriction=0.001,mass = 2)
    p.changeVisualShape(ball,-1,rgbaColor=[0.8,0.8,0.8,1])
    #p.applyExternalForce(ball,-1, forceObj=[force_magnitude,0,0],posObj=[0,0,0], flags= p.WORLD_FRAME)

    return ball

def import_cylinder(start_position):
    cylinder_start_position = start_position
    cylinder_start_orientation = p.getQuaternionFromEuler([0,0,0])
    urdf_file_path = '/home/viktor/Documents/DIPLOMSKI/custom urdf/cilindar.urdf'
    box_1 = p.loadURDF(urdf_file_path,basePosition = cylinder_start_position)
    p.changeDynamics(box_1,-1,mass = 0.0, restitution = .9)
    return box_1



#globalni parametri
force_magnitude = 6000
time_step = 1 / 80.0
p.setGravity(0,0,-9.81)


#p.setTimeStep(time_step)



#---------------main funkcije-------------------
lopta = create_sphere(.1,[0,0.2,3.3])
teren = import_teren([0,0,1])
#box = create_box([1, 1, 1],[3, 0, 1],[0, 0, 1.7, 1])
#box_2 = create_box([1,1,1],[0,-3,1],[0,0,0,2])
box3 = import_box([1,0,0])

#box4 = import_cylinder([2,0,-.055])
#visual_shape_1 = p.createVisualShape(p.GEOM_BOX, rgbaColor=[0, 0, 0, 0])
#p.changeVisualShape(box3,-1,baseVisualShape = visual_shape_1)
#proba = p.loadSoftBody('/home/viktor/Documents/DIPLOMSKI/proba1/probaobj.obj')

#p.applyExternalForce(lopta,-1, forceObj=[force_magnitude,0,0],posObj=[0,0,1], flags= p.LINK_FRAME)


time.sleep(4)

#custom = p.loadURDF('/home/viktor/Documents/Minigolf/custom urdf/customurdf1.urdf')

flag = 0 #koristimo da bi loop samo jednom ispisao Upala u rupu.




while(p.isConnected()):
    time.sleep(time_step)
    p.stepSimulation()
    
    contact = p.getContactPoints(lopta,box3)
    
    #prvo ovaj if, zato sto ukoliko lopta upadne i mi izadjemo iz simulacije, vise ne moze da se ocita velocity pa izbacuje error
    if p.getBaseVelocity(lopta)[0][0] < 0.01 and p.getBaseVelocity(lopta)[0][1] < 0.01:
        print("Lopta stala\n\n\n")
        #time.sleep(1)
        #p.disconnect()
    
    if len(contact) > 0 and flag == 0:
        print("Upala u rupu")
        flag = 1
        
        x = p.getBaseVelocity(lopta)[0][0]
        y = p.getBaseVelocity(lopta)[0][1]
        z = p.getBaseVelocity(lopta)[0][2]
        print(f"""Brzina po x osi: {x:.2f}\nBrzina po y osi: {y:.2f}\nBrzina po z osi: {z:.2f}""")
        #time.sleep(1)
        #p.resetBaseVelocity(lopta,[0,0,0])
        #p.disconnect()
        

input() 
