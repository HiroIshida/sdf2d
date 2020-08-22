import json
import matplotlib.pyplot as plt
import numpy as np
import sdf2d

with open("./test.json", 'r') as f:
    problem = json.load(f)
    b_min = np.array(problem['b_min'])
    b_max = np.array(problem['b_max'])
    N = np.array(problem["N"]) 
    V = np.array(problem["V"]) 
    E = np.array(problem["E"])

with open("./data.json", 'r') as f:
    dick = json.load(f)
data = np.array(dick["data"])


xlin = np.linspace(b_min[0], b_max[0], N[0])
ylin = np.linspace(b_min[1], b_max[1], N[1])
X, Y = np.meshgrid(xlin, ylin)

data = np.array(sdf2d.convert2sdf(V, E, b_min, b_max, N))
fig, ax = plt.subplots()
cplt = ax.contourf(X, Y, data.T)
cbar = fig.colorbar(cplt)
ax.scatter(V[:, 0], V[:, 1])
plt.show()
