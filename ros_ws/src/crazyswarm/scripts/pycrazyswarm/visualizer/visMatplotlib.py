import warnings

from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import matplotlib.pyplot as plt
import numpy as np

#TODO: Detta bör läsas ur en csv fil (lösning finns i planner simulator)
obs1 = np.array([0,1,0,2,1,1])
#obs2 = np.array([1,1,0,1,1,1])

# Define dimensions of Visionen
VISIONEN_X_DIM = 11.70
VISIONEN_Y_DIM = 11.70
VISIONEN_Z_DIM = 3.0

#TODO: Kanske bör läggas in i ett separat script för tydlighet?
def add_obs(x,y,z,w,h,d):
    w,h,d = np.indices((w+x, h+y, d+z))
    cube1 = (w >= x) & (h >= y) & (d >= z)
    voxels = cube1
    return voxels

class VisMatplotlib:
    def __init__(self, flags=None):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-VISIONEN_X_DIM/2, VISIONEN_X_DIM/2]) 
        self.ax.set_ylim([-VISIONEN_Y_DIM/2, VISIONEN_Y_DIM/2])
        self.ax.set_zlim([0, VISIONEN_Z_DIM])
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.set_title("Drone Simulation")
        self.plot = None
        self.timeAnnotation = self.ax.annotate("Time", xy=(3, 0), xycoords='axes fraction', fontsize=12, ha='right', va='bottom')

        self.line_color = 0.3 * np.ones(3)

        # Lazy-constructed data for connectivity graph gfx.
        self.graph_edges = None
        self.graph_lines = None
        self.graph = None

        #TODO: Detta bör ligga under if flags == "--obs"
        # Vizualisation of obstacles
        self.ax.voxels(add_obs(obs1[0], obs1[1], obs1[2], obs1[3], obs1[4], obs1[5]), facecolors='red', zorder = 0)
        #self.ax.voxels(add_obs(obs2[0], obs2[1], obs2[2], obs2[3], obs2[4], obs2[5]), facecolors='blue', zorder = 1)
        if flags == "--obs":
            #TODO: Hur lägger man till flaggor i crazyswarm_py.py?
            pass
    
    def setGraph(self, edges):
        """Set edges of graph visualization - sequence of (i,j) tuples."""

        # Only allocate new memory if we need to.
        n_edges = len(edges)
        if self.graph_edges is None or n_edges != len(self.graph_edges):
            self.graph_lines = np.zeros((n_edges, 2, 3))
        self.graph_edges = edges

        # Lazily construct Matplotlib object for graph.
        if self.graph is None:
            self.graph = Line3DCollection(self.graph_lines, edgecolor=self.line_color)
            self.ax.add_collection(self.graph)

    def showEllipsoids(self, radii):
        warnings.warn("showEllipsoids not implemented in Matplotlib visualizer.")

    def update(self, t, crazyflies):
        xs = []
        ys = []
        zs = []
        cs = []
        for i in range(0,len(crazyflies)):
            cf = crazyflies[i]
            x, y, z = cf.position()
            color = "green"
            xs.append(x)
            ys.append(y)
            zs.append(z)
            cs.append(color)

        if self.plot is None:
            self.plot = self.ax.scatter(xs, ys, zs, c=cs)
        else:
            print("YES")
            # TODO: Don't use protected members.
            self.plot._offsets3d = (xs, ys, zs)
            self.plot.set_facecolors(cs)
            self.plot.set_edgecolors(cs)
            self.plot._facecolor3d = self.plot.get_facecolor()
            self.plot._edgecolor3d = self.plot.get_edgecolor()

        if self.graph is not None:
            # Update graph line segments to match new Crazyflie positions.
            for k, (i, j) in enumerate(self.graph_edges):
                self.graph_lines[k, 0, :] = xs[i], ys[i], zs[i]
                self.graph_lines[k, 1, :] = xs[j], ys[j], zs[j]
                self.graph.set_segments(self.graph_lines)
                print(self.graph_lines)

        self.timeAnnotation.set_text("{} s".format(t))
        plt.pause(0.0001)

    def render(self):
        warnings.warn("Rendering video not supported in VisMatplotlib yet.")
        return None
