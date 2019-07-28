# -*- coding: utf8 -*-

import argparse
import math
import os.path
import time
import random
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

from underactuated.planar_rigid_body_visualizer import (FindResource, PlanarRigidBodyVisualizer)


class Hopper2dController(VectorSystem):
    def __init__(self, hopper,
                 print_period = 0.0,f_try=np.array([-54,-51,0.15,123,12,-0.14,0.015])):
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
        self.desired_lateral_velocity = 0.2
        self.ki = 1.01    # best 0.992
        self.desire_theta2 = 0
        self.print_period = print_period
        self.last_print_time = -print_period
        self.count=0
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
        self.in_contact_last = 1
        self.Tst=0
        self.stance_start=0
        self.stance_time=0
        # This is arbitrary choose of k
        self.K_l = 100
        self.first_hop=1
        self.tow =0
        self.pre_theta1_in=0
        self.pre_theta1_out=0
        self.pre_theta2_in = 0
        self.pre_theta2d_in = 0
        self.dance_counter=1  #for dance, donot delete
        #self.k_walk=np.array([-614,-542,0.15,1795,220])            # V =0.5
        #self.k_stay=np.array([-108*0.5,-101*0.5,0.0231*13*0.5,245.7*0.5,23.318*0.5,0.1*0.0231])      # V =0

        self.k_stay = np.array([-54,-50.5,0.150,123,11.7,0.015015])
        self.cost = 0
        self.count = 0
        self.countcount = 0
        self.f_try = f_try  # f_try[0] theta1_bias,  f_try[1] theta2_bias
        self.step_count = 0
        self.first_time = 1
        self.tilt_A = 0.2


    
    def ChooseSpringRestLength(self, t,X):
        ''' Given the system state X,
            returns a (scalar) rest length of the leg spring.
            We can command this instantaneously, as
            the actual system being simulated has perfect
            force control of its leg extension. '''
        # Unpack states
        x, z, theta, alpha, l = X[0:5]
        xd, zd, thetad, alphad, ld = X[5:10]
        
        body_length = 1
        theta2=theta
        theta2d=thetad
        theta1=alpha+theta
        theta1d = alphad+thetad
        r1=self.hopper_leg_length
        r2=body_length
        m1=self.m_f
        m2=self.m_b
        
        # Run out the forward kinematics of the robot
        # to figure out where the foot is in world frame.
        foot_point = np.array([0.0, 0.0, -self.hopper_leg_length])
        foot_point_in_world = self.hopper.CalcPointsPositions(self.plant_context, 
                              self.foot_body_frame, foot_point, self.world_frame)
        in_contact_ = foot_point_in_world[2] <= 0.01
        in_contact = (foot_point_in_world[2] <= 0.01 + foot_point_in_world[0] * math.tan(self.tilt_A))
        

        # Feel free to play with these values!
        # These should work pretty well for this problem set,
        # though.
      
        if (in_contact):
            if (zd > 0):
                # On the way back up,
                # "push" harder by increasing the effective
                # spring constant.
                # original  l_rest = 1.05

                l_rest = 1.05-0.013*(zd-3.5)
                
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

    def jump_adapt(self,t,X,desired_lateral_velocity):
        ''' Given the system state X,
             returns a (scalar) leg angle torque to exert. '''
        x, z, theta, alpha, l = X[0:5]
        xd, zd, thetad, alphad, ld = X[5:10]
        
        body_length = 1
        theta2=theta
        theta2d=thetad
        theta1=alpha+theta
        theta1d = alphad+thetad
        r1=self.hopper_leg_length
        r2=body_length
        m1=self.m_f
        m2=self.m_b
        
        
        # Run out the forward kinematics of the robot
        # to figure out where the foot is in world frame.
        foot_point = np.array([0.0, 0.0, -self.hopper_leg_length])
        foot_point_in_world = self.hopper.CalcPointsPositions(self.plant_context,
                              self.foot_body_frame, foot_point, self.world_frame)


        in_contact_ = foot_point_in_world[2] <= 0.01
        in_contact = (foot_point_in_world[2] <= 0.01 + foot_point_in_world[0] * math.tan(self.tilt_A))


        
        
        #time = context.get_time()
        
        kp1=self.f_try[0]
        kv1=self.f_try[1]
        k1=self.f_try[2]*1.5
        kp2=self.f_try[3]
        kv2=self.f_try[4]
        stance_reudce = -0.14
        k_tilt= 0.015
        kp3 = self.f_try[5]
        kv3 = self.f_try[6]
        
        
        self.countcount +=1
        
        if in_contact == 0 and self.in_contact_last ==1:
            self.Tst = t-self.stance_start
            self.hop_start = t
            self.cost+= abs(theta1-self.pre_theta1_out)
            self.count +=1
            self.pre_theta1_out = theta1

    
        if in_contact == 0:
            self.in_contact_last = 0
            Xstance = xd*self.Tst
            XERR = k1*(xd-desired_lateral_velocity)   #-k_tilt
            XCG = (r1*m1*math.sin(theta1)+r2*m2*math.sin(theta2))/(m1+m2)
            Xstance = xd*self.Tst
            XTD = XCG+XERR+0.5*Xstance
            Angle=-(-0.2)
            tt=math.tan(Angle)
            oo = XTD*tt
            kk = oo / (1-oo)
            #XTD=XTD*(1-kk)
            
            if XTD >=1 or XTD <=-1:
                self.theta1_desire = -math.asin(1*XTD/abs(XTD)/r1)
            else:
                self.theta1_desire = -math.asin(XTD/r1)
            self.tow =kp1*(theta1-self.theta1_desire) +kv1*theta1d
            realTD = -r1*m1*math.sin(theta1)
            XCG = (r1*m1*math.sin(theta1)+r2*m2*math.sin(theta2))/(m1+m2)

            if self.tow >500 or self.tow <-500:
                self.tow=500*self.tow/abs(self.tow)
            return self.tow

        if in_contact ==1 and self.in_contact_last == 0:

            self.step_count +=1
            self.stance_start=t
            self.in_contact_last = 1
            #self.cost+=abs(xd-desired_lateral_velocity)
            self.cost+= abs(theta1-self.pre_theta1_in)
            self.count +=1

            
            self.pre_theta1_in = theta1
            if theta1 < 0:
               self.pre_theta2_in = theta2
               self.pre_theta2d_in = theta2d


        if in_contact == 1:
            self.in_contact_last = 1
            self.tow = kp2*(theta2-self.desire_theta2) + kv2*theta2d    # <----- refer to 0.5*theta1

            realTD = -r1*m1*math.sin(theta1)
            XCG = (r1*m1*math.sin(theta1)+r2*m2*math.sin(theta2))/(m1+m2)

            if self.tow >500 or self.tow <-500:
                self.tow=500*self.tow/abs(self.tow)
            return self.tow
                
                
    def jump_tilt(self,t,X,desired_lateral_velocity):
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
        k_walk=np.array([-614,-542,0.15,1795,220])            # V =0.5
        k_stay=np.array([-108,-101,0.0231,245.7,23.318])      # V =0
      
        kp1=k_stay[0]*0.5
        kv1=k_stay[1]*0.5
        k1=k_stay[2]*13*0.5
        kp2=k_stay[3]*0.5
        kv2=k_stay[4]*0.5
    
        
        #time = context.get_time()
        
        Angle=-(-0.2)
        XERR = k1*(xd-desired_lateral_velocity)-0.1*k1
        XCG = (r1*m1*math.sin(theta1)+r2*m2*math.sin(theta2))/(m1+m2)
        Xstance = xd*self.Tst
        XTD = XCG+XERR+0.5*Xstance
        tt=math.tan(Angle)
        oo = XTD*tt
        kk = oo / (1-oo)
        XTD=XTD*(1-kk)

        
        
        if in_contact == 0 and self.in_contact_last ==1:
            self.Tst = t-self.stance_start
            self.hop_start = t
            self.theta1_out = theta1
            if self.first_hop ==1:
                self.Tst = 0.35
    
        if in_contact == 0:
            self.in_contact_last = 0
            Xstance = xd*self.Tst

            if XTD >=0.866 or XTD <=-0.866:
                self.theta1_desire = -math.asin(0.64*XTD/abs(XTD)/r1)
            else:
                self.theta1_desire = -math.asin(XTD/r1)
            self.tow =kp1*(theta1-self.theta1_desire) +kv1*theta1d
            
            if self.tow >50 or self.tow <-50:
                self.tow=50*self.tow/abs(self.tow)
            return self.tow

        if in_contact ==1 and self.in_contact_last == 0:
            self.first_hop = 0
            self.stance_start=t
            self.in_contact_last = 1



        if in_contact == 1:
            self.in_contact_last = 1
            self.tow = kp2*(theta2-self.desire_theta2) +kv2*theta2d    # <----- refer to 0.5*theta1
            if self.tow >50 or self.tow <-50:
                self.tow=50*self.tow/abs(self.tow)
            return self.tow


    def jump(self,t,X,desired_lateral_velocity):
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
        k_walk=np.array([-614,-542,0.15,1795,220])            # V =0.5
        k_stay=np.array([-108,-101,0.0231,245.7,23.318])      # V =0
        if desired_lateral_velocity == 0:
            kp1=k_stay[0]
            kv1=k_stay[1]
            k1=k_stay[2]
            kp2=k_stay[3]
            kv2=k_stay[4]
        else:
            kp1=k_walk[0]
            kv1=k_walk[1]
            k1=k_walk[2]
            kp2=k_walk[3]
            kv2=k_walk[4]
        
        #time = context.get_time()
    
        XERR = k1*(xd-desired_lateral_velocity)
        XCG = (r1*m1*math.sin(theta1)+r2*m2*math.sin(theta2))/(m1+m2)
        Xstance = xd*self.Tst
        XTD = XCG+XERR+0.5*Xstance
        
        if in_contact == 0 and self.in_contact_last ==1:
            self.Tst = t-self.stance_start
            self.hop_start = t
            self.theta1_out = theta1
            if self.first_hop ==1:
                self.Tst = 0.35

        if in_contact == 0:
            self.in_contact_last = 0
            Xstance = xd*self.Tst
            
            if XTD >=0.866 or XTD <=-0.866:
                self.theta1_desire = -math.asin(0.64*XTD/abs(XTD)/r1)
            else:
                self.theta1_desire = -math.asin(XTD/r1)
            self.tow =kp1*(theta1-self.theta1_desire) +kv1*theta1d
            
            if self.tow >50 or self.tow <-50:
                self.tow=50*self.tow/abs(self.tow)
            return self.tow

        if in_contact ==1 and self.in_contact_last == 0:
            self.first_hop = 0
            self.stance_start=t
            self.in_contact_last = 1

        if in_contact == 1:
            self.in_contact_last = 1
            self.tow = kp2*(theta2-0.5*theta1) +kv2*theta2d    # <----- refer to 0.5*theta1
            if self.tow >50 or self.tow <-50:
                self.tow=50*self.tow/abs(self.tow)
            return self.tow

    def dance(self,t,X,desired_lateral_velocity):
        ''' Auto Opimal K,
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
        k_walk=np.array([-614,-542,0.15,1795,220])            # V =0.5
        k_stay=np.array([-108,-101,0.0231,245.7,23.318])      # V =0
        if desired_lateral_velocity == 0:
            kp1=k_stay[0]
            kv1=k_stay[1]
            k1=k_stay[2]
            kp2=k_stay[3]
            kv2=k_stay[4]
        else:
            kp1=k_walk[0]
            kv1=k_walk[1]
            k1=k_walk[2]
            kp2=k_walk[3]
            kv2=k_walk[4]
        

        XERR = k1*(0-desired_lateral_velocity*((-1)**self.dance_counter))
        XCG = (r1*m1*math.sin(theta1)+r2*m2*math.sin(theta2))/(m1+m2)
        Xstance = xd*self.Tst
        XTD = XCG+XERR+0.5*Xstance
        
        if in_contact == 0 and self.in_contact_last ==1:
            self.Tst = t-self.stance_start
            self.hop_start = t
            self.theta1_out = theta1
            if self.first_hop ==1:
                self.Tst = 0.35

        
        if in_contact == 0:
            self.in_contact_last = 0
            Xstance = xd*self.Tst
            
            if XTD >=0.866 or XTD <=-0.866:
                self.theta1_desire = -math.asin(0.64*XTD/abs(XTD)/r1)
            else:
                self.theta1_desire = -math.asin(XTD/r1)
            self.tow =kp1*(theta1-self.theta1_desire) +kv1*theta1d
            
            if self.tow >50 or self.tow <-50:
                self.tow=50*self.tow/abs(self.tow)
            return self.tow
        

        if in_contact ==1 and self.in_contact_last == 0:
            self.dance_counter +=1
            self.first_hop = 0
            self.stance_start=t
            self.in_contact_last = 1

        
        if in_contact == 1:
            self.in_contact_last = 1
            self.tow = kp2*(theta2-0.5*theta1) +kv2*theta2d    # <----- refer to 0.5*theta1
            if self.tow >50 or self.tow <-50:
                self.tow=50*self.tow/abs(self.tow)
            return self.tow
    
    def ChooseThighTorque(self,t,X):
           return self.jump_adapt(t,X,desired_lateral_velocity= 0.2)



            
    
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
        l_rest = self.ChooseSpringRestLength(t,X = u)

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
def Simulate2dHopper(x0, duration,
                     print_period = 1.0,f_try=np.array([-54,-51,0.15,123,12,-0.14,0.015])):           # <---------
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
        Hopper2dController(plant,print_period = print_period,f_try=f_try))                                                    # <---------
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
                               duration=20,
                               print_period = 1.0,f_try=np.array([-54,-51,0.15,123,12,-0.14,0.015]))         # <---------
