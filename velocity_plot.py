import numpy as np
import pandas as pd
import matplotlib.pyplot as plt 

# 0/1 col: displacement <x, y>
# 2/3 col: disp_velocity <dx, dy>
# 4 col  : angular disp 
# 5 col  : angular speed
# 6 col  : timestamp ms

# Returns DataFrame or TextParser
df_vel = pd.read_csv("velocity_data.csv", header=None)
#df.drop([0, 2], axis=1, inplace=True)

df_disp = df_vel.copy()

i = 0
dfvel_length = len(df_vel[0])



while(i < dfvel_length):
    if(df_vel[0][i] == "disp"):
        df_vel.drop([i], axis=0, inplace=True)
    i += 1

j = 0
dfdisp_length = len(df_disp[0])

while(j < dfdisp_length):
    if(df_disp[0][j] == "vel"):
        df_disp.drop([j], axis=0, inplace=True)
    j += 1


fig = plt.figure(dpi=300)

s = np.ones(len(df_vel[0]))
r = np.ones(len(df_disp[0]))

#           x-axis,y-axis
plt.scatter(df_vel[3], df_vel[2], s)
# plt.show()
plt.savefig("velocity_result.png", dpi=300, format="png")

plt.clf()        
# plt.subplot(131)

plt.scatter(df_disp[4], df_disp[2], r)
plt.savefig("displacement_result.png", dpi=300, format="png")



