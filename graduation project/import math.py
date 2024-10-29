import tkinter as tk
from tkinter import ttk, messagebox
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon
from matplotlib.widgets import Button
from collections import namedtuple
from queue import PriorityQueue
import math
from matplotlib.path import Path

# Define a named tuple 'Point' to represent 2D points
#by defining x,y this rebresent  our environment in 2D
# the reason why we represent the envirenment in 2D is becuse the robot cannot work in 3D or 4D 
#we rebresent it in 2D by input the coordenates of each polygon as x,y not by names 

Point = namedtuple('Point', ['x', 'y'])

#this function  cheecks if if tow lines intersect (vedges)
#-_by calculating distance to each vedge #and also calculate orientation 
# Check if edges intersect
def do_edges_intersect(p1, q1, p2, q2):
    def orientation(p, q, r):
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if val == 0:
            return 0  # collinear 
        elif val > 0:
            return 1  # clockwise
        else:
            return 2  # counterclockwise
     # 
    def on_segment(p, q, r):
        if q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and \
           q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]):
            return True
        return False

    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    if o1 != o2 and o3 != o4:
        return True

    if o1 == 0 and on_segment(p1, p2, q1):
        return True
    if o2 == 0 and on_segment(p1, q2, q1):
        return True
    if o3 == 0 and on_segment(p2, p1, q2):
        return True
    if o4 == 0 and on_segment(p2, q1, q2):
        return True

    return False

# Check if edge is obstructed
def is_edge_obstructed(p1, p2, obstacles):
    for obstacle in obstacles:
        if do_edges_intersect(p1, p2, obstacle[0], obstacle[1]):
            return True
    return False

# Generate visibility graph with start and goal included
def generate_visibility_graph(vertices, obstacles, start=None, goal=None):
    vertices = list(vertices)  # Convert vertices to a list
    if start:  # Add start to vertices
        vertices.append(Point(*start))
    if goal:  # Add goal to vertices
        vertices.append(Point(*goal))
    
    edges = []
    for i in range(len(vertices)):
        for j in range(i + 1, len(vertices)):
            if not is_edge_obstructed(vertices[i], vertices[j], obstacles):
                edges.append((vertices[i], vertices[j]))
    return edges

# Create a dictionary representation of the visibility graph
def create_visibility_graph(edges):
    visibility_graph = {}
    for edge in edges:
        if edge[0] not in visibility_graph:
            visibility_graph[edge[0]] = []
        if edge[1] not in visibility_graph:
            visibility_graph[edge[1]] = []
        distance = np.linalg.norm(np.array(edge[1]) - np.array(edge[0]))
        visibility_graph[edge[0]].append((edge[1], distance))
        visibility_graph[edge[1]].append((edge[0], distance))
    return visibility_graph

# Dijkstra's algorithm to find the shortest path
def dijkstra(graph, start, goal):
    distances = {vertex: float('infinity') for vertex in graph}
    distances[start] = 0
    priority_queue = PriorityQueue()
    priority_queue.put((0, start))

    while not priority_queue.empty():
        current_distance, current_vertex = priority_queue.get()

        for neighbor, weight in graph[current_vertex]:
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                priority_queue.put((distance, neighbor))

    return distances[goal]

# Function to find the parent vertices in the shortest path
def dijkstra_with_parents(graph, start, goal):
    distances = {vertex: float('infinity') for vertex in graph}
    distances[start] = 0
    priority_queue = PriorityQueue()
    priority_queue.put((0, start))

    parents = {vertex: None for vertex in graph}

    while not priority_queue.empty():
        current_distance, current_vertex = priority_queue.get()

        for neighbor, weight in graph[current_vertex]:
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                priority_queue.put((distance, neighbor))
                parents[neighbor] = current_vertex

    return parents
def is_point_in_polygon(point, polygon):
    x, y = point
    n = len(polygon)
    inside = False
    p1x, p1y = polygon[0]
    for i in range(1, n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside


# Improved identify_traversed_polygons function
def identify_traversed_polygons(self, path):
    traversed_polygons = []
    transitions = []
    for i in range(1, len(path)):
        point_prev = path[i - 1]
        point_curr = path[i]
        start_polygon = None
        end_polygon = None
        for polygon_name, polygon_edges in self.polygon_edges.items():
            for edge in polygon_edges:
                if do_edges_intersect(edge[0], edge[1], point_prev, point_curr):
                    if start_polygon is None:
                        start_polygon = polygon_name
                    else:
                        end_polygon = polygon_name
                        break
        if start_polygon is not None and end_polygon is not None:
            transition = f"{start_polygon} -> {end_polygon}"
            transitions.append(transition)
            print(f"Transition from {start_polygon} to {end_polygon}")  # Debugging statement
    return transitions




# Class to represent the GUI
class PathPlanningGUI:
    def __init__(self, root):
        # Initialization
        self.root = root
        self.start = None
        self.goal = None
        self.setting_start = False
        self.setting_goal = False
        self.polygon_vertices = [
            (1, 1), (1, 5), (1, 8), (3, 10),
            (5, 1), (5, 5), (5, 10), (7, 1),
            (10, 4), (10, 5), (10, 10)
        ]
        self.obstacle_edges = [
            ((4, 4), (4, 2)), ((4, 4), (2, 4)), ((2, 4), (4, 2)),
            ((7, 2), (9.5, 4)), ((9.5, 4), (6, 4)), ((6, 4), (7, 2)),
            ((6, 6), (9, 6)), ((9, 6), (9, 9)), ((9, 9), (6, 9)),
            ((6, 9), (6, 6)), ((3, 6), (4, 7)), ((4, 7), (4, 8)),
            ((4, 8), (3, 9)), ((3, 9), (2, 8)), ((2, 8), (2, 7)),
            ((2, 7), (3, 6)),
            ((0, 0), (11, 0)), ((11, 11), (11, 0)), ((11, 11), (0, 11)), ((0, 11), (0, 0))
        ]
        self.polygons = {
            "b01": [(0, 0), (0, 11)],  # Left edge of the environment
            "b02": [(0, 11), (11, 11)],  # Top edge of the environment
            "b03": [(11, 11), (11, 0)],  # Right edge of the environment
            "b04": [(11, 0), (0, 0)],  # Bottom edge of the environment
            "b1": [(4, 4), (4, 2), (2, 4)],
            "b2": [(7, 2), (9.5, 4), (6, 4)],
            "b3": [(6, 6), (9, 6), (9, 9), (6, 9)],
            "b4": [(3, 6), (4, 7), (4, 8), (3, 9), (2, 8), (2, 7)]
        }

        # Setup GUI
        self.frame = ttk.Frame(root)
        self.frame.pack(fill=tk.BOTH, expand=True)
        # Button Frame
        self.button_frame = ttk.Frame(self.frame)
        self.button_frame.pack(side=tk.TOP, fill=tk.X)

        # Button for running path planning
        self.run_path_button = ttk.Button(self.button_frame, text="Run Path Planning", command=self.run_path_planning)
        self.run_path_button.pack(side=tk.LEFT, padx=5, pady=5)

        # Button for resetting
        self.reset_button = ttk.Button(self.button_frame, text="Reset", command=self.reset_gui)
        self.reset_button.pack(side=tk.LEFT, padx=5, pady=5)

        # Status Label
        self.status_label = ttk.Label(self.button_frame, text="Ready", foreground="green")
        self.status_label.pack(side=tk.LEFT, padx=5, pady=5)

        # Canvas for Matplotlib plot
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.canvas.mpl_connect("button_press_event", self.on_plot_click)

        # Sub-window to display start, goal, shortest path value, and path details
        self.info_frame = ttk.Frame(self.frame)
        self.info_frame.place(relx=1.0, rely=0.0, anchor='ne', x=10, y=10)

        self.info_label = ttk.Label(self.info_frame, text="Start: None\nGoal: None\nDistance: None\nPath: None")
        self.info_label.pack(side=tk.TOP, padx=5, pady=5)

        self.plot_obstacles()

    def reset_gui(self):
        self.start = None
        self.goal = None
        self.ax.clear()
        self.plot_obstacles()
        self.info_label.configure(text="Start: None\nGoal: None\nDistance: None\nPath: None")
        self.canvas.draw()

    def plot_obstacles(self):
        # Draw and label the polygons
        for name, vertices in self.polygons.items():
            polygon = Polygon(vertices, fill=None, edgecolor='k')
            self.ax.add_patch(polygon)
            centroid = np.mean(vertices, axis=0)
            self.ax.text(centroid[0], centroid[1], name, fontsize=12, ha='center')

            self.ax.set_xlim(-1, 12)
            self.ax.set_ylim(-1, 12)
            self.ax.set_xlabel("X")
            self.ax.set_ylabel("Y")
            plt.axhline(y=10, xmin=0.31, xmax=0.847, color='red', ls='-')
            plt.axhline(y=5, xmin=0.155, xmax=0.847, color='red', ls='-')
            plt.axhline(y=1, xmin=0.39, xmax=0.61, color='red', ls='-')
            plt.axvline(x=10, ymin=0.39, ymax=0.85, color='red', ls='-')
            plt.axvline(x=5, ymin=0.157, ymax=0.849, color='red', ls='-')
            plt.axvline(x=1, ymin=0.39, ymax=0.69, color='red', ls='-')
            
            plt.plot(1, 5,'go', markersize=12.5, linestyle='dotted', alpha=0.4)
           
            plt.plot([1, 3], [8, 10],color='red',)  # Add line between (1,8) and (3,10)
            
            plt.plot([7, 10], [1, 4],color='red')  # Add line between (7,1) and (10,4)
            
            plt.plot(5, 1,'go', markersize=12.5, linestyle='dotted', alpha=0.4)
            plt.plot(5, 5,'go', markersize=12.5, linestyle='dotted', alpha=0.4)
            plt.plot(5, 10,'go', markersize=12.5, linestyle='dotted', alpha=0.4)
        
            plt.plot([7, 10], [1, 4],color='red',)  # Add line between (7,1) and (10,4)
            
            plt.plot([1, 4], [4, 1],color='red',)  # Add line between (1,4) and (4,1)
            plt.plot(10, 10,'go', markersize=12.5, linestyle='dotted', alpha=0.4)
            plt.plot(10, 5,'go', markersize=12.5, linestyle='dotted', alpha=0.4)
        
    
    def on_plot_click(self, event):
        if event.inaxes is None:
            return

        point = (round(event.xdata), round(event.ydata))

        if self.start is None:
            self.start = point
            self.ax.add_patch(Circle(self.start, 0.2, color='g', label="Start"))
            self.canvas.draw()
        elif self.goal is None:
            self.goal = point
            self.ax.add_patch(Circle(self.goal, 0.2, color='r', label="Goal"))
            self.canvas.draw()

            # Update info label with start and goal coordinates
            self.info_label.configure(text=f"Start: {self.start}\nGoal: {self.goal}\nDistance: Calculating...\nPath: Calculating...")

    def run_path_planning(self):
        if self.start and self.goal:
            self.plot_visibility_graph_and_path()
        else:
            messagebox.showerror("Error", "Please set both start and goal points.")
        if self.start and self.goal:
            # Check for path existence using Dijkstra's algorithm
            visibility_graph = self.create_visibility_graph()
            shortest_path_value = dijkstra(visibility_graph, self.start, self.goal)
            if shortest_path_value == float('infinity'):
                messagebox.showerror("Error", "No path found between start and goal.")
                return
    


    def plot_visibility_graph_and_path(self):
        # Generate the visibility graph
        visibility_edges = generate_visibility_graph(
            [Point(*vertex) for vertex in self.polygon_vertices],
            [(Point(*obstacle[0]), Point(*obstacle[1])) for obstacle in self.obstacle_edges],
            start=self.start,
            goal=self.goal
        )

        visibility_graph = create_visibility_graph(visibility_edges)

        parents = dijkstra_with_parents(visibility_graph, self.start, self.goal)

        # Backtrack the path
        current = self.goal
        path = [current]

        while current in parents and parents[current] is not None:
            path.append(parents[current])
            current = parents[current]

        path.reverse()
        path_coordinates = np.array(path)

        self.ax.plot(path_coordinates[:, 0], path_coordinates[:, 1], 'r-', label='Shortest Path')

        # Annotate shortest path
        shortest_path_value = dijkstra(visibility_graph, self.start, self.goal)
        traversed_polygons_transitions = self.identify_traversed_polygons(path)
        traversed_polygons_text = ' -> '.join(traversed_polygons_transitions)

        # Update info label with calculated details
        self.info_label.configure(text=f"Start: {self.start}\nGoal: {self.goal}\nDistance: {shortest_path_value:.3f}\nPath: {traversed_polygons_text}")

        # Update path details subwindow
        #path_details_text = "\n".join(traversed_polygons_transitions)
        #self.path_details_label.configure(text=path_details_text)

        self.canvas.draw()

    def identify_traversed_polygons(self, path):
        traversed_polygons = []
        transitions = []
        for i in range(1, len(path)):
            point_prev = path[i - 1]
            point_curr = path[i]
            for name, vertices in self.polygons.items():
                if do_edges_intersect(point_prev, point_curr, vertices[0], vertices[-1]):
                    traversed_polygons.append(name)
                    transitions.append(f"{name}")
        return transitions

# Run the GUI
def run_gui():
    root = tk.Tk()
    app = PathPlanningGUI(root)
    root.mainloop()

# Start the GUI
if __name__ == "__main__":
    run_gui()
