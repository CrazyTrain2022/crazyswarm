import warnings

from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import yaml
import csv
import uav_trajectory
import os

# Drone colors
COLORS = ["black", "blue", "red", "green"]

# Define dimensions of Visionen
VISIONEN_X_DIM = 8
VISIONEN_Y_DIM = 8
VISIONEN_Z_DIM = 3.0

def add_obs(ax,x1,y1,z1,x2,y2,z2):
    """Graphical help function to add obstacles"""

    # Defining corner coordinates of obstacles
    x = [x1,x2,x2,x1],[x2,x2,x2,x2],[x1,x2,x2,x1],[x1,x1,x1,x1],[x1,x2,x2,x1],[x1,x2,x2,x1]
    y = [y1,y1,y2,y2],[y1,y1,y2,y2],[y1,y1,y2,y2],[y1,y1,y2,y2],[y1,y1,y1,y1],[y2,y2,y2,y2]
    z = [z1,z1,z1,z1],[z1,z2,z2,z1],[z2,z2,z2,z2],[z1,z2,z2,z1],[z1,z1,z2,z2],[z1,z1,z2,z2]

    # List of obstacle surfaces
    surfaces = []

    # Loop for creating surfaces
    for i in range(len(x)):
        surfaces.append( [list(zip(x[i],y[i],z[i]))] )

    # Plot obstacle, one surface at a time
    for surface in surfaces:
        ax.add_collection3d(Poly3DCollection(surface, facecolors='red', alpha = 0.7, zorder=0))

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
        self.line_color = 1 * np.ones(3)

        # Lazy-constructed data for connectivity graph gfx.
        self.graph_edges = None
        self.graph_lines = None
        self.graph = None

        #Reading manual or autonomous control
        self.manual_mode = 0
        if flags == "--manual":
            self.manual_mode = 1

        #Read "plot_trajectories.yaml" for graphical settings originated from GUI
        file = "plot_trajectories.yaml"
        stream = open(file, 'r')
        data = yaml.safe_load_all(stream)
        for settings in data:
            self.plot_traj = settings["traj"]
            self.plot_obs = settings["obs"]

        # Obstacle plot
        if self.plot_obs[0]==1:
            file_obs = "pycrazyswarm/visualizer/obstacles.csv"
            with open(file_obs, 'r') as file:
                reader = csv.reader(file, skipinitialspace=True)
                obs = np.empty((0,6),int)
                for coord in reader:
                    step = np.array([[float(coord[0]),float(coord[1]),float(coord[2]),float(coord[3]),float(coord[4]),float(coord[5])]])
                    obs = np.append(obs ,step, axis = 0)
            for i in range(0,len(obs)):
                add_obs(self.ax,obs[i][0],obs[i][1],obs[i][2],obs[i][3],obs[i][4],obs[i][5])
            

    def Show_traj(self):
        """Graphical help function to add trajectories"""

        if self.plot_traj[0] == 1:
            path = "."
            self.traj = []
            i = 0
            trajectory_files = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
            for file in trajectory_files:
                if("trajectory.csv" in file):
                    self.Traj_exist = True
                    self.traj.append(uav_trajectory.Trajectory())
                    self.traj[i].loadcsv(file)
                    idx = file[5] 
                    ts = np.arange(0, self.traj[i].duration, 0.01)
                    self.evals = np.empty((len(ts), 15))
                    for t, k in zip(ts, range(0, len(ts))):
                        e = self.traj[i].eval(t)
                        self.evals[k, 0:3]  = e.pos
                    self.traj_pos = np.array([self.evals[:,0], self.evals[:,1], self.evals[:,2]])
                    self.traj[i]  = self.ax.plot3D(self.evals[:,0], self.evals[:,1], self.evals[:,2], color = COLORS[int(idx)-1], linestyle = 'dashed', alpha=0.7)
                    self.ax.plot3D([self.evals[0,0], self.evals[0,0]], [self.evals[0,1], self.evals[0,1]], [0, self.evals[0,2]],color = COLORS[int(idx)-1], linestyle = 'dashed', alpha=0.7)
                    i += 1
        else:
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

    def update(self, t, crazyflies):
        """Function for updating the simulation"""

        xs = []
        ys = []
        zs = []
        cs = []
        labels = []

        for i in range(0,len(crazyflies)):
            cf = crazyflies[i]
            x, y, z = cf.position()
            color = COLORS[cf.id-1]
            labels.append("CF" + str(cf.id))
            xs.append(x)
            ys.append(y)
            zs.append(z)
            cs.append(color)

        if self.plot is None:
            self.plot = self.ax.scatter(xs, ys, zs, c=cs)
            patches = []
            for i in range(0,len(cs)):
                patches.append(mpatches.Patch(color=cs[i], label=labels[i]))
            self.ax.legend(handles = patches)       
        else:
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

        plt.pause(0.0001)
