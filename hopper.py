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
cost_min = 99999

#k_walk=np.array([-614,-542,0.15,1795,220])            # V =0.5
#k_stay=np.array([-108*0.5,-101*0.5,0.0231*13*0.5,245.7*0.5,23.318*0.5,0.1*0.0231])
#k_try = np.array([-54,-50.5,0.150,123,11.7,0.015])



with open("record.txt", "r+") as file3:
    f_list = [float(i) for line in file3 for i in line.split(',') if i.strip()]
    file3.close()
#k_try=np.array([f_list[0],f_list[1],f_list[2],f_list[3],f_list[4],f_list[5]])
#k_try = np.array([-54,-50.5,0.150,123,11.7,0.015])
k_try = np.array([-64.8,-60.6,0.18,147.42,13.99,0.018])
hopper, controller, state_log = hopper_2d_limit.Simulate2dHopper(x0 = x0,duration=30,print_period = 1.0,k_try=k_try)



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
