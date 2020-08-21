import json
import matplotlib.pyplot as plt
import numpy as np
with open("./data.json", 'r') as f:
    dick = json.load(f)
data = np.array(dick["data"])

xlin = np.linspace(-1, 1, 100)
ylin = np.linspace(-1, 1, 100)
X, Y = np.meshgrid(xlin, ylin)

fig, ax = plt.subplots()
cplt = ax.contourf(X, Y, data.T)
cbar = fig.colorbar(cplt)

plt.show()
