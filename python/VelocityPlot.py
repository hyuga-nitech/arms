import matplotlib.pyplot as plt
import pandas as pd

filepath = "data/"
filename = "xArm1_data1.csv"

input_csv = pd.read_csv(filepath + filename)
time = input_csv[input_csv.keys()[0]]
current_vel = input_csv[input_csv.keys()[1]]
at_time_vel = input_csv[input_csv.keys()[2]]

fig = plt.figure(dpi = 1200)

ax = fig.add_subplot(111)
ax.plot(time, current_vel, label='Current_Vel', color='red')
ax.plot(time, at_time_vel, label='Predict_Vel', color='brue')

ax.set_xlabel('Time [s]')
ax.set_ylabel('Velocity [m/s]')

ax.legend(loc='upper left', bbox_to_anchor=(1,1))
fig.show()
fig.save('Velocity.png', bbox_inches='tight')