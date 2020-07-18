import numpy as np
import pandas as pd
import matplotlib.pyplot as plt 

# First two columns contains location
# Third column is orientation
# Fourth is time

# Returns DataFrame or TextParser
df = pd.read_csv("data.csv", header=None)
#df.drop([0, 2], axis=1, inplace=True)

fig = plt.figure(dpi=300)
s = np.ones(len(df[3]))
plt.scatter(df[3], df[1], s)
# plt.show()
plt.savefig("result.png", dpi=300, format="png")