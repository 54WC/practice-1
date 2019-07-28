# -*- coding: utf8 -*-

import argparse
import math
import os.path
import time

import numpy as np

from pydrake.all import (
    RigidBodyTree,
    AddModelInstancesFromSdfString, FloatingBaseType,
    DiagramBuilder, 
    Simulator, VectorSystem,
    ConstantVectorSource, 
    SignalLogger,
    AbstractValue,
    Parser,
    PortDataType,
    MultibodyPlant,
    UniformGravityFieldElement
)
from IPython.display import HTML
import matplotlib.pyplot as plt
from drake import lcmt_viewer_load_robot
from pydrake.common.eigen_geometry import Quaternion
from pydrake.geometry import DispatchLoadMessage, SceneGraph
from pydrake.lcm import DrakeMockLcm
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.rendering import PoseBundle

from underactuated.planar_multibody_visualizer import PlanarMultibodyVisualizer


class Hopper2dController(VectorSystem):
    def __init__(self, hopper, desired_theta2 = 0,
        desired_lateral_velocity = 0.0,
        print_period = 0.0,k10=np.array([30,5,0.12,0.3,0.4]),k20=np.array([30,5,0.12,0.3,0.4])):
        # hopper = a rigid body tree representing the 1D hopper
        # desired_lateral_velocity = How fast should the controller
        #  aim to run sideways?
        # print_period = print to console to indicate sim progress
        #  if nonzero
        VectorSystem.__init__(self,
            10, # 10 inputs: x, z, theta, alpha, l, and their derivatives
            2) # 2 outputs: Torque on thigh, and force on the leg extension
               #  link. (The passive spring for on the leg is calculated as
               #  part of this output.)
        self.hopper = hopper
        self.desired_lateral_velocity = desired_lateral_velocity
        self.desired_theta2 = desired_theta2 
        self.print_period = print_period
        self.last_print_time = -print_period
        self.kp10=k10[0]
        self.kv10=k10[1]
        self.k10=k10[2]
        self.kpp10=k10[3]
        self.kvv10=k10[4]
        self.kp20=k20[0]
        self.kv20=k20[1]
        self.k20=k20[2]
        self.kpp20=k20[3]
        self.kvv20=k20[4]
    
        # Remember what the index of the foot is
        # self.foot_body_index = hopper.GetFrameByName("foot").get_body_index()
        self.foot_body_frame = hopper.GetFrameByName("foot")
        self.world_frame = hopper.world_frame()
        
        # The context for the hopper
        self.plant_context = self.hopper.CreateDefaultContext()

        # Default parameters for the hopper -- should match
        # raibert_hopper_1d.sdf, where applicable.
        # You're welcome to use these, but you probably don't need them.
        self.hopper_leg_length = 1.0
        self.m_b = 1.0
        self.m_f = 0.1
        self.l_max = 0.5
        self.theta1_desire = 0
        self.in_contact_last = 0
        self.Tst=0.35
        self.stance_start=0
        self.stance_time=0 
        self.Thop=0.6845
        self.hop_start = 0 
        self.hop_time =0
        # This is arbitrary choose of k
        self.K_l = 100
        self.tow =0 
        self.theta1_in =0
        self.theta1_out=0
        self.cost = 0
        self.x_fix = 0
        self.XTD = 0
        self.first_contact =1
        
    def ChooseSpringRestLength(self, X):
        ''' Given the system state X,
            returns a (scalar) rest length of the leg spring.
            We can command this instantaneously, as
            the actual system being simulated has perfect
            force control of its leg extension. '''
        # Unpack states
        x, z, theta, alpha, l = X[0:5]
        zd = X[6]

        # Run out the forward kinematics of the robot
        # to figure out where the foot is in world frame.
        foot_point = np.array([0.0, 0.0, -self.hopper_leg_length])
        foot_point_in_world = self.hopper.CalcPointsPositions(self.plant_context, 
                              self.foot_body_frame, foot_point, self.world_frame)
        in_contact = foot_point_in_world[2] <= 0.01
        
        # Feel free to play with these values!
        # These should work pretty well for this problem set,
        # though.
        if (in_contact):
            if (zd > 0):
                # On the way back up,
                # "push" harder by increasing the effective
                # spring constant.
                l_rest = 1.05
            else:
                # On the way down,
                # "push" less hard by decreasing the effective
                # spring constant.
                l_rest = 1.0
        else:
            # Keep l_rest large to make sure the leg
            # is pushed back out to full extension quickly.
            l_rest = 1.0 

        # See "Hopping in Legged Systems-Modeling and
        # Simulation for the Two-Dimensional One-Legged Case"
        # Section III for a great description of why
        # this works. (It has to do with a natural balance
        # arising between the energy lost from dissipation
        # during ground contact, and the energy injected by
        # this control.)

        return l_rest

    def ChooseThighTorque(self,t,X):
        ''' Given the system state X,
            returns a (scalar) leg angle torque to exert. '''
        x, z, theta, alpha, l = X[0:5]
        xd, zd, thetad, alphad, ld = X[5:10]
        
        # Run out the forward kinematics of the robot
        # to figure out where the foot is in world frame.
        foot_point = np.array([0.0, 0.0, -self.hopper_leg_length])
        foot_point_in_world = self.hopper.CalcPointsPositions(self.plant_context, 
                              self.foot_body_frame, foot_point, self.world_frame)
        in_contact = foot_point_in_world[2] <= 0.01
        

        body_length = 1
        theta2=theta
        theta2d=thetad
        theta1=alpha+theta
        theta1d = alphad+thetad
        r1=self.hopper_leg_length
        r2=body_length
        m1=self.m_f
        m2=self.m_b
       
        #time = context.get_time() 
        
        
 
        
        # 0.0001,   -693.0,   -515.0,     0.15,   1736.0,    142.0,   -706.0,   -523.0,     0.15,   1629.8,    262.0
        if t < 2:
            kp1=-693
            kv1=-515
            k1=0.15    #0.12
            kp2=1736    #0.3
            kv2=142    #0.4      
        if t>= 2 and t <5.5: 
            kp1=-706
            kv1=-523
            k1=0.15    #0.12
            kp2=1629.8    #0.3
            kv2=262 
        if t >=5.5 and t <7.5:
            kp1=self.kp10
            kv1=self.kv10
            k1=self.k10    #0.12
            kp2=self.kpp10    #0.3
            kv2=self.kvv10    #0.4      
        if t>= 7.5: 
            kp1=self.kp20
            kv1=self.kv20
            k1=self.k20    #0.12
            kp2=self.kpp20    #0.3
            kv2=self.kvv20 
        
        
        
        # Method 3, Servo Attitude    
      
        XERR = k1*(xd-self.desired_lateral_velocity)
        XCG = (r1*m1*math.sin(theta1)+r2*m2*math.sin(theta2))/(m1+m2)
        Xstance = xd*self.Tst
        XTD = XCG+XERR+0.5*Xstance
        
        if t >= 5.5 and self.first_contact == 1: 
            if in_contact ==1 and self.in_contact_last == 0:  
                    self.first_contact =0
                    self.x_fix = x
                    
            self.XTD = self.x_fix-x
            self.desired_lateral_velocity=0
            self.cost+= (self.x_fix-x)**2
                
            #print  'self.theta1_out', self.theta1_out
            with open("guru9.txt", "w+") as file1: 
                           file1.write("%10.6s"%str(self.cost))
                           file1.close 
                
        if in_contact == 0 and self.in_contact_last ==1:
                
                self.in_contact_last = 0 
                self.Tst = t-self.stance_start
                self.hop_start = t 
                
                self.theta1_out = theta1

        
        if in_contact == 0 and self.in_contact_last == 0: 
                
                self.hop_time = t- self.hop_start
                
                if XTD >=0.866 or XTD <=-0.866:
                       self.theta1_desire = -math.asin(0.64*XTD/abs(XTD)/r1)
                else: 
                       self.theta1_desire = -math.asin(XTD/r1)
                self.tow =kp1*(theta1-self.theta1_desire) +kv1*theta1d
                
                if self.tow >50 or self.tow <-50:
                     self.tow=50*self.tow/abs(self.tow)
                return self.tow
                
     
        if in_contact ==1 and self.in_contact_last == 0:  
                
                self.in_contact_last = 1 
                self.Thop = t- self.hop_start
                self.stance_start=t
                self.theta1_in = theta1

                #print  'self.theta1_in', self.theta1_in

        if in_contact == 1 and self.in_contact_last ==1:
                
                self.tow = kp2*(theta2-self.desired_theta2) +kv2*theta2d
                if self.tow >50 or self.tow <-50:
                        self.tow=50*self.tow/abs(self.tow)
                return self.tow
                
              

    
    def DoCalcVectorOutput(self, context, u, x, y):
        # The naming if inputs is confusing, as this is a separate
        # system with its own state (x) and input (u), but the input
        # here is the state of the hopper.
        # Empty now
        t=context.get_time()
        if (self.print_period and
            context.get_time() - self.last_print_time >= self.print_period):
            t=context.get_time()
            #print "t: ", context.get_time()
            self.last_print_time = context.get_time()
        
        # Update the internal context
        plant = self.hopper
        context = self.plant_context
        x_ref = plant.GetMutablePositionsAndVelocities(context)
        x_ref[:] = u
        
        # OK
        l_rest = self.ChooseSpringRestLength(X = u)

        # Passive spring force
        leg_compression_amount = l_rest - u[4]

        
        y[:] = [  self.ChooseThighTorque(t,X=u),
                  self.K_l * leg_compression_amount]


'''
Simulates a 2d hopper from initial conditions x0 (which
should be a 10x1 np array) for duration seconds,
targeting a specified lateral velocity and printing to the
console every print_period seconds (as an indicator of
progress, only if print_period is nonzero).
'''
def Simulate2dHopper(x0, duration,desired_theta2 = 0,
        desired_lateral_velocity = 0.0,
        print_period = 0.0,k10=np.array([30,5,0.12,0.3,0.4]),k20=np.array([30,5,0.12,0.3,0.4])):
    builder = DiagramBuilder()
    
    plant = builder.AddSystem(MultibodyPlant(0.0005))
    scene_graph = builder.AddSystem(SceneGraph())
    plant.RegisterAsSourceForSceneGraph(scene_graph)
    builder.Connect(plant.get_geometry_poses_output_port(),
                    scene_graph.get_source_pose_port(
                        plant.get_source_id()))
    builder.Connect(scene_graph.get_query_output_port(),
                    plant.get_geometry_query_input_port())
    
    # Build the plant
    parser = Parser(plant)
    parser.AddModelFromFile("raibert_hopper_2d.sdf")
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("ground"))
    plant.AddForceElement(UniformGravityFieldElement())
    plant.Finalize()
    
    # Create a logger to log at 30hz
    state_dim = plant.num_positions() + plant.num_velocities()
    state_log = builder.AddSystem(SignalLogger(state_dim))
    state_log.DeclarePeriodicPublish(0.0333, 0.0) # 30hz logging
    builder.Connect(plant.get_continuous_state_output_port(), state_log.get_input_port(0))
    
    # The controller
    controller = builder.AddSystem(
        Hopper2dController(plant,desired_theta2 = desired_theta2,
            desired_lateral_velocity = desired_lateral_velocity,
            print_period = print_period,k10 = k10,k20=k20))
    builder.Connect(plant.get_continuous_state_output_port(), controller.get_input_port(0))
    builder.Connect(controller.get_output_port(0), plant.get_actuation_input_port())
    
    # The diagram
    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.Initialize()
    
    plant_context = diagram.GetMutableSubsystemContext(
        plant, simulator.get_mutable_context())
    plant_context.get_mutable_discrete_state_vector().SetFromVector(x0)

    simulator.StepTo(duration)
    return plant, controller, state_log


def ConstructVisualizer():
    from underactuated import PlanarRigidBodyVisualizer
    tree = RigidBodyTree()
    AddModelInstancesFromSdfString(
        open("raibert_hopper_2d.sdf", 'r').read(),
        FloatingBaseType.kFixed,
        None, tree)
    viz = PlanarRigidBodyVisualizer(tree, xlim=[-5, 5], ylim=[-5, 5])
    viz.fig.set_size_inches(10, 5)
    return viz


if __name__ == '__main__':
    x0 = np.zeros(10)
    x0[1] = 2
    x0[4] = 0.5
    hopper, controller, state_log = Simulate2dHopper(x0 = x0,
                               duration=20,desired_theta2 = 0,
                               desired_lateral_velocity = 0.5,
                               print_period = 1.0,k10=np.array([30,5,0.12,0.3,0.4]),k20=np.array([30,5,0.12,0.3,0.4]))