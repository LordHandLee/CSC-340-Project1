import numpy as np # linear algebra
import pandas as pd # data processing, CSV file I/O (e.g. pd.read_csv)
import matplotlib.pyplot as plt
import math

np.set_printoptions(precision = 2, suppress = True)
# Part 2, problem 4.a.
print("Part 2, problem 4.a.: ")
def unit_step(time, actuation_time):
    if time>= actuation_time:
        value = 20
    else:
        value = 0
    return value

def unit_impulse(time, actuation_time, dt):
    return unit_step(time, actuation_time) - unit_step(time, actuation_time+dt)

def sin_wave(time, actuation_time, freq, amplitude):
    if time >= actuation_time:
        value = amplitude*math.sin(2*math.pi*freq*(time - actuation_time))
    else:
        value = 0
    # return the output
    return value

def state_space_model(initial, A,B,C, x_k, u_k, r_prime, dt):  
    print("rrime: ",r_prime[0][0])
    initial[2] = xw - r_prime[0][0]
    #initial[1] = x_k[1][0]
    #initial[3] = x_k[3][0]
    x_k1 = dt*(np.matmul(A,initial) + np.matmul(B,u_k) + r_prime)# + x_k  #+ r_prime# + np.matmul(C, r_prime)) + x_k# np.matmul(C, r_prime) + x_k
    #print("disturbance: ", r_prime)# calculate the current outputs
    #print(np.matmul(B,u_k))
    # return the results
    return x_k1#, y

# Dynamic model of the 1D motion of a car
Ms = 290 #kg
Mus = 59 #kg
Ka = 16812 #N/m
Kt = 190000 # N/m
Ca = 1000 #N/(m/s)

xs = 5 # cm
xw = 2 # cm

rprime = 0 # specify ourselves?
#ua = 36000 # specify ourselves?
ua = 2000

# x1 = xs-xw # Suspension travel
# x2 = 0 # xs prime  Car body velocity
# x3 = xw - 0 # Wheel deflection
# x4 = 0 # xw prime Wheel velocity

x1 = 0
x2 = 0 # xs prime 
x3 = 0
x4 = 0 # xw prime

dt = 0.1 #seconds

x_0 = np.array([[x1, x2, x3, x4]]).reshape(4,1)
# define the A, B, C, and D matrices
A = np.array([[0, 1, 0, -1],
              [(-Ka)/Ms, (-Ca)/Ms, 0, (Ca)/Ms],
              [0, 0, 0, 1],
              [(Ka)/Mus, (Ca)/Mus, (-Kt)/Mus, (-Ca)/Mus]])
print('A:\n',A)
B = np.array([[0, 1/Ms, 0, -1/(Ms)]]).T
print('B:\n',B)
C = np.array([[0, 0, -1, 0]]).T
print('C:\n',C)
print('X0: ', x_0)
u_k = np.array([[ua]])


# let's redo it for a longer time horizon
t_0 = 0 # seconds
t_f = 10 # seconds
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
    step_values = sin_wave(t, 2, 1, 5)
    rprime = np.array([[step_values]])
    x_k1 = state_space_model(x_0, A,B,C, x_k, u_k, rprime, dt) # y_k
    #print("x_value: ", x_k1)
    # update the simulation time stamp
    t += dt
    # update the arrays
    x_values = np.append(x_values, x_k1.T,axis = 0)
    time_values= np.append(time_values, np.array([[t]]),axis = 0)
    # set up the loop for the next iteration
    x_k = x_k1

print('dimentions of x_values\n',x_values.shape)
print('dimentions of time_values\n',time_values.shape)

# create some subplots and display the results
fig, axs = plt.subplots(4, 1,figsize=(9,9))
# plot the first state vs time
fig.tight_layout(pad=5.0)
axs[0].plot(time_values, x_values[:,0], label='x')
axs[0].set_xlabel('time (seconds)')
axs[0].set_ylabel('Suspension travel')
axs[0].grid()
# plot the second state vs time
axs[1].plot(time_values, x_values[:,1], label='x prime')
axs[1].set_xlabel('time (seconds)')
axs[1].set_ylabel('Car body velocity')
axs[1].grid()

axs[2].plot(time_values, x_values[:,2], label='θ')
axs[2].set_xlabel('time (seconds)')
axs[2].set_ylabel('Wheel deflection')
axs[2].grid()
# plot the second state vs time
axs[3].plot(time_values, x_values[:,3], label='θ prime')
axs[3].set_xlabel('time (seconds)')
axs[3].set_ylabel('Wheel velocity')
axs[3].grid()
plt.show()
