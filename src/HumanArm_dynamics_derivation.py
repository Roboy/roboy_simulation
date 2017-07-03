#%% 
from __future__ import print_function, division
from sympy import symbols, simplify, Matrix
from sympy import trigsimp
from sympy.physics.mechanics import dynamicsymbols, ReferenceFrame, Point, Particle, inertia, RigidBody, KanesMethod
from numpy import deg2rad, rad2deg, array, zeros, linspace
from sympy.physics.vector import init_vprinting, vlatex
import numpy as np

from sympy.physics.vector import init_vprinting
init_vprinting(use_latex='mathjax', pretty_print=False)
#%% 
# define all reference frames of all individually moving links of the robot
print("Defining reference frames")
inertial_frame = ReferenceFrame('I')
shoulder_frame = ReferenceFrame('R_1');
arm_frame = ReferenceFrame('R_2');
# fore_arm_frame = ReferenceFrame('R_3');
# hand_frame = ReferenceFrame('R_4');
# finger_base_frame = ReferenceFrame('R_5');
# finger_1_frame = ReferenceFrame('R_6');


#%% Angles
# everything is symbolic, so create all angles of your robot
# NOTE: the angle phi is the angle between your robot and the inertial frame
theta0, phi = dynamicsymbols('theta0, phi')

shoulder_frame.orient(inertial_frame, 'Axis', (phi, inertial_frame.z))
simplify(shoulder_frame.dcm(inertial_frame))

#%% Points and Locations
# define the kinematical chain of your robot
print("Defining kinematical chain")

# these can be arbitray points on each of you robots links
# later these points are visualized, so you can verify the correct build of your robot
origin = Point('Origin')
shoulder_joint = Point('ShoulderJoint')
elbow_joint = Point('ElbowJoint')

arm_length = symbols('l1')

shoulder_joint.set_pos(origin, (0 * inertial_frame.y)+(0 * inertial_frame.x))
elbow_joint.set_pos(shoulder_joint, arm_length * arm_frame.y)

#%%
# The following defines the full robots kinematics, if you only want to do 
# inverse kinematics you can skip this whole part
print("calculating full kinematics, this might take a while ...")

#%% COMs
print("Defining center of mass")

arm_com_length = arm_length / 2

shoulder_mass_center = Point('ShoulderCOM')
arm_mass_center = Point('armCOM')

shoulder_mass_center.set_pos(shoulder_joint, (0 * inertial_frame.y)+(0 * inertial_frame.x))
arm_mass_center.set_pos(shoulder_joint, arm_com_length * arm_frame.y)

#%% Kinematical Differential Equations
omega0, psi = dynamicsymbols('omega0, psi')
kinematical_differential_equations = [omega0 - theta0.diff(),
                                        psi - phi.diff(),                                
                                        ]

kinematical_differential_equations

#%% Angular velocities
print("Defining angular velocities")

shoulder_frame.set_ang_vel(inertial_frame, 0 * inertial_frame.z)

arm_frame.set_ang_vel(arm_frame, omega0 * inertial_frame.z)

#%% Linear Velocities
print("Defining linear velocities")

origin.set_vel(inertial_frame, 0)

shoulder_joint.set_vel(inertial_frame, 0)

elbow_joint.v2pt_theory(shoulder_joint, inertial_frame, arm_frame)

shoulder_mass_center.v2pt_theory(shoulder_joint, inertial_frame, shoulder_frame)

arm_mass_center.v2pt_theory(elbow_joint, inertial_frame, arm_frame)

#%% Masses, Inertia, Rigid Bodies
print("Defining Masses, Inertia, Rigid Bodies")

shoulder_mass, arm_mass = symbols('m_S', 'm_H')

shoulder_inertia, arm_inertia = symbols('I_Sz', 'I_Az')

shoulder_inertia_dyadic = inertia(shoulder_frame, shoulder_inertia, shoulder_inertia, shoulder_inertia)
shoulder_central_inertia = (shoulder_inertia_dyadic, shoulder_mass_center)
shoulder_inertia_dyadic.to_matrix(shoulder_frame)

arm_inertia_dyadic = inertia(arm_frame, arm_inertia, arm_inertia, arm_inertia)
arm_central_inertia = (arm_inertia_dyadic, arm_mass_center)
arm_inertia_dyadic.to_matrix(arm_frame)

arm = RigidBody('Arm', arm_mass_center, arm_frame, arm_mass, arm_central_inertia)

shoulder = RigidBody('Shoulder', shoulder_mass_center, shoulder_frame, shoulder_mass, shoulder_central_inertia)

particles = []
particles.append(Particle('shoulder_joint', shoulder_joint, 0))
particles.append(Particle('elbow_joint', elbow_joint, 0))
particles

mass_centers = []
mass_centers.append(Particle('shoulder_mass_center', shoulder_mass_center, shoulder_mass))
mass_centers.append(Particle('arm_mass_center', arm_mass_center, arm_mass))
mass_centers

#%% Forces and Torques
print("Defining Forces and Torques")
g = symbols('g')

shoulder_grav_force = (shoulder_mass_center, -shoulder_mass * g * inertial_frame.y)
arm_grav_force = (arm_mass_center, -arm_mass * g * inertial_frame.y)

elbow_torque0, shoulder_torque0 = dynamicsymbols('T_e', 'T_s')

arm_torque_vector = elbow_torque0 * inertial_frame.z - shoulder_torque0 * inertial_frame.z
shoulder_torque_vector = shoulder_torque0 * inertial_frame.z

shoulder_torque = (shoulder_frame, shoulder_torque_vector)
arm_torque = (arm_frame, arm_torque_vector)

#%% Equations of Motion
# you only need this if you want a full dynamics simulation, with forces etc.
print("Calculating equations of motion")
coordinates = [theta0, phi]
coordinates

speeds = [omega0, psi]
speeds

kinematical_differential_equations

kane = KanesMethod(inertial_frame, coordinates, speeds, kinematical_differential_equations)

loads = [shoulder_grav_force,
         arm_grav_force,
         shoulder_torque,
         arm_torque]

loads

bodies = [shoulder, arm]
bodies

print("evaluating kanes equation")
fr, frstar = kane.kanes_equations(loads, bodies)

print("simplifying mass_matrix")
mass_matrix = trigsimp(kane.mass_matrix_full)

print("simplifying forcing_vector")
forcing_vector = trigsimp(kane.forcing_full)

