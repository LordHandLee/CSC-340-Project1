
import numpy as np # linear algebra
import pandas as pd # data processing, CSV file I/O (e.g. pd.read_csv)
import matplotlib.pyplot as plt
np.set_printoptions(precision = 2, suppress = True)
# Part 2, problem 4.a.
print("Part 2, problem 4.a.: ")
def unit_step(time, actuation_time):
    if time>= actuation_time:
        value = 500
    else:
        value = 0
    return value

def unit_impulse(time, actuation_time, dt):
    return unit_step(time, actuation_time) - unit_step(time, actuation_time+dt)

def state_space_model(A,B,C, x_k, u_k, r_prime, dt):  
    # calculate the next state
    #step_values = [unit_step(n, t_s) for n in time_values]
    x_k1 = dt*(np.matmul(A,x_k) + np.matmul(B,u_k))+r_prime# + np.matmul(C, r_prime)) + x_k# np.matmul(C, r_prime) + x_k
    # calculate the current outputs
    #y = np.matmul(C,x_k) + np.matmul(D,u_k)
    # return the results
    return x_k1#, y

# Dynamic model of the 1D motion of a car
Ms = 290 #kg
Mus = 59 #kg
Ka = 16812 #N/m
K1 = 190000 # N/m
Ca = 1000 #N/(m/s)

xs = 10 # cm
xw = 5 # cm

rprime = 0 # specify ourselves?
ua = 20000 # specify ourselves?

x1 = xs-xw
x2 = xs # xs prime 
x3 = xw - rprime
x4 = xw # xw prime



# l = 0.5 #m
# g = 9.81 #m/s^2
dt = 0.1 #seconds
# tf = 2 #seconds
# u = 0
# f = u # force


x_0 = np.array([[x1, x2, x3, x4]]).reshape(4,1)
# define the A, B, C, and D matrices
A = np.array([[0, 1, 0, -1],
              [(-Ka)/Ms, (-Ca)/Ms, 0, (Ca)/Ms],
              [0, 0, 0, 1],
              [(Ka)/Mus, (Ca)/Mus, (-K1)/Mus, (-Ca)/Mus]])
print('A:\n',A)
B = np.array([[0, 1/Ms, 0, -1/(Ms)]]).T
print('B:\n',B)
C = np.array([[0, 0, -1, 0]]).T
# C = np.array([[1,0,0,0],
#               [0,0,1,0]])
print('C:\n',C)
# D = np.array([[0, 0]]).reshape(2,1)
# print('D:\n',D)

# let's redo it for a longer time horizon
t_0 = 0 # seconds
t_f = 3 # seconds
# variable for tracking the states
x_values = np.array(x_0.T)
time_values = np.array([[t_0]])
print(x_values)
# initialize the solver
x_k = x_0
print('x_0\n',x_0)
u_k = np.array([[ua]]) # remains fixed for this example
print('u_k\n', u_k)
t = t_0
# simulate the system
while t < t_f:
    # update the states x_{k+1}
    step_values = unit_impulse(t, 2, dt)
    #print(step_values)
    rprime = np.array([[step_values]])
    x_k1 = state_space_model(A,B,C, x_k, u_k, rprime, dt) # y_k
    #print(x_k1[0])
    # update the simulation time stamp
    t += dt
    # update the arrays
    x_values = np.append(x_values, x_k1.T,axis = 0)
    time_values= np.append(time_values, np.array([[t]]),axis = 0)
    # set up the loop for the next iteration
    x_k = x_k1
    # print("x: ")
    # print(x_k)
    # print("y: ")
    # print(y_k)

print('dimentions of x_values\n',x_values.shape)
print('dimentions of time_values\n',time_values.shape)
print(x_values[:,0])


# create some subplots and display the results
fig, axs = plt.subplots(4, 1,figsize=(9,9))
# plot the first state vs time
fig.tight_layout(pad=5.0)
mini = np.min(x_values[:,0])
maxi = np.max(x_values[:,0])
axs[0].plot(time_values, x_values[:,0], label='x')
axs[0].set_ylim(mini, maxi)
axs[0].set_xlabel('time (seconds)')
axs[0].set_ylabel('Position')
axs[0].grid()
# plot the second state vs time
axs[1].semilogy(time_values, x_values[:,1], label='x prime')
axs[1].set_xlabel('time (seconds)')
axs[1].set_ylabel('Velocity')
axs[1].grid()

axs[2].semilogy(time_values, x_values[:,2], label='θ')
axs[2].set_xlabel('time (seconds)')
axs[2].set_ylabel('Angle')
axs[2].grid()
# plot the second state vs time
axs[3].semilogy(time_values, x_values[:,3], label='θ prime')
axs[3].set_xlabel('time (seconds)')
axs[3].set_ylabel('Angular velocity')
axs[3].grid()


plt.show()
