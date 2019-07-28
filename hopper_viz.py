import hopper_2d_limit
import numpy as np
import matplotlib.pyplot as plt
import random


x0 = np.zeros(10)
x0[0]=0        #x
x0[1] = 2      #z
x0[2]=0   #theta
x0[3]=0     #alfa
x0[4] = 0.5  # L
x0[5] = 0   # xd
x0[6] =0    #zd
x0[7] =0   #thetad
x0[8] =0    #alfad
x0[9] =0.    # Ld

desired_theta2 =0

with open("guru.txt", "w+") as file1:
    file1.write("t,")
    file1.write("in_contact,")
    file1.write("foot_2,")
    file1.write("z,")
    file1.write("zd\n")
    file1.close

#k_walk=np.array([-614,-542,0.15,1795,220])            # V =0.5
#k_stay=np.array([-108,-101,0.0231,245.7,23.318,0.1])
#k_try = np.array([-54,-50.5,0.150,123,11.7,0.015015]) # tilt -0.2 , velocity = 0



new_f=np.array([-54,-51,0.15,123,11.7,0.001,0.01])
#with open("record.txt", "r+") as file3:
    #f_list = [float(i) for line in file3 for i in line.split(',') if i.strip()]
    #file3.close()
#new_f=np.array([0,0])

hopper, controller, state_log = hopper_2d_limit.Simulate2dHopper(x0 = x0,duration=10,print_period = 1.0,f_try=new_f)



#from IPython.display import HTML
viz = hopper_2d_limit.ConstructVisualizer()
ani = viz.animate(state_log, 30, repeat=True)
plt.show()
#plt.close(viz.fig)
#HTML(ani.to_html5_video())



plt.figure().set_size_inches(5, 5)
plt.plot(state_log.data()[1, :], state_log.data()[1+5, :])
plt.legend(["z-zd"])
plt.grid(True)
plt.show()

plt.figure().set_size_inches(5, 5)
plt.plot(state_log.data()[3, :]+state_log.data()[2, :], state_log.data()[3+5, :]+state_log.data()[2+5, :])
plt.legend(["theta1-theta1d"])
plt.grid(True)
plt.show()


plt.figure().set_size_inches(10, 5)
plt.plot(state_log.sample_times(), state_log.data()[2, :])
plt.plot(state_log.sample_times(), state_log.data()[3, :]+state_log.data()[2, :])
plt.legend(["theta2", "theta1"])
plt.grid(True)
plt.show()


# Plot traces of certain states
plt.figure().set_size_inches(10, 5)
plt.plot(state_log.sample_times(), state_log.data()[0, :])
plt.plot(state_log.sample_times(), state_log.data()[0+5, :])
plt.grid(True)
plt.legend(["body_x", "body_x_d"])

plt.figure().set_size_inches(10, 5)
plt.plot(state_log.sample_times(), state_log.data()[1, :])
plt.plot(state_log.sample_times(), state_log.data()[1+5, :])
plt.grid(True)
plt.legend(["body_z", "body_z_d"])

plt.figure().set_size_inches(10, 5)
plt.plot(state_log.sample_times(), state_log.data()[2, :])
plt.plot(state_log.sample_times(), state_log.data()[2+5, :])
plt.legend(["body_theta", "body_theta_d"])
plt.grid(True)

plt.figure().set_size_inches(10, 5)
plt.plot(state_log.sample_times(), state_log.data()[3, :]+state_log.data()[2, :])
plt.plot(state_log.sample_times(), state_log.data()[3+5, :]+state_log.data()[2+5, :])
plt.legend(["leg_angle", "led_angle_d"])
plt.grid(True)
plt.show()

print('Done')
