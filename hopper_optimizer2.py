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
k_try = np.array([-54,-50.5,0.150,123,11.7,0.015015]) # tilt -0.2 , velocity = 0

new_f=np.array([-54,-50.5,0.150,123,11.7,-0.14,0.015])

with open("record.txt", "w+") as file2:
    file2.write("t,")
    file2.write("XTD,")
    file2.write("Xerro,")
    file2.write("X\n")
    file2.close()

with open("guru9.txt", "w+") as file1:
    file1.write("%10.6s"%str(999999))
    file1.close

with open("guru9.txt", "r+") as file1:
    f_list = [float(i) for line in file1 for i in line.split(',') if i.strip()]
    file1.close()

# [-5.30e+01 -5.05e+01  1.51e-01  1.24e+02  1.27e+01 -1.39e-01  1.60e-02]
#[-4.80e+01 -5.05e+01  1.51e-01  1.19e+02  1.27e+01 -1.39e-01  1.60e-02]
#[-4.750e+02 -5.550e+01  3.000e-01  1.185e+03  1.220e+02 -1.390e-01 1.600e-02]
#[-4.70e+02 -6.00e+01  3.00e-01  1.18e+03  1.17e+02 -1.40e-01  1.50e-02]

new_f=np.array([-60,-55,0.20,120,18,-0.139,0.016])   #lope -0.2, v = 0
f_try =np.array([-60,-55,0.20,120,18,-0.139,0.016])

#hopper, controller, state_log = hopper_2d_limit.Simulate2dHopper(x0 = x0,duration=10,print_period = 1.0,f_try=f_try)

b=1

while b>0:
    count =0
    for a in range (-1,2,1):
        for b in range (-1,2,1):
            for c in range (0,1,1):
                for d in range (-1,2,2):
                    for e in range (-1,2,1):
                        for f in range (0,1,1):
                            for g in range (0,1,1):
                                    try_direction=np.array([5*a,5*b,0.001*c,5*d,5*e,0.001*f,0.001*g])
                                    f_try = new_f+try_direction
                                    print(f_try)
                                  
                                    hopper, controller, state_log = hopper_2d_limit.Simulate2dHopper(x0 = x0,duration=16,print_period = 1.0,f_try=f_try)
                                    with open("guru9.txt", "r+") as file1:
                                      f_list = [float(i) for line in file1 for i in line.split(',') if i.strip()]
                                      file1.close()
                                    if f_list[0] <= cost_min:
                                       print('good',f_try)
                                       good_direction = try_direction
                                       good_f_try = f_try
                                       cost_min = f_list[0]
                                       
                                       if abs(f_list[0]-cost_min) < 0.0001:
                                          count=count+1
                                   #else:
                                       #print('NG',f_try)
                                    if count > 3:
                                      b=0

    new_f =good_f_try
    #print('new_f',new_f)
    #with open("record.txt", "w+") as file2:
        #file2.write("%10.6s,"%str(new_f[0]))
        #file2.write("%10.6s,"%str(new_f[1]))
        #file2.close()





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
