# Import packages
import numpy as np
import matplotlib.pyplot as plt
import random


# RRT Implementation of 2D path planning with no obstacles/no path smoothing


# Define the parameters of the RRT implementation
xdim=10
ydim=10
epsilon=1.0      # Max length of branch
numNodes=500
goal_region=0.35   
div=20

init_node=(2,2)
fin_node=(8,8)

# Define functions for RRT

#Find the distance between
def dist(p1,p2):
    d=np.sqrt((p2[0]-p1[0])**2+(p2[1]-p1[1])**2)
    return d

# If the edge between two points are longer than epsilon, shorten the edge to match epsilon.
# p2 is the newly sampled random point
def set_edge_pt(p1,p2):
    d=dist(p1,p2)
    if d<epsilon:
        return p2
    else:
        new_pt=((p2[0]-p1[0])/d*epsilon+p1[0],(p2[1]-p1[1])/d*epsilon+p1[1])
        return new_pt

# Find the neighbor nodes from E
def find_neighbor(node,E):
    neighbor=[]
    for e in E:
        if e[0]==node:
            neighbor.append(e[1])
        elif e[1]==node:
            neighbor.append(e[0])
    return neighbor

# Initialize the figure
fig=plt.figure(figsize=(8.333,6),dpi=80)
ax=fig.add_subplot(111,autoscale_on=False, xlim=(0, xdim), ylim=(0, ydim))
ax.set_aspect('equal')

# Initialize a list of nodes and edges
nodes=[]
nodes.append(init_node)
E=[]
cost=[]
cost.append(0)

for i in range(numNodes):
    node=nodes[0]
    rand_pt=(random.random()*xdim, random.random()*ydim)
    for p in nodes:
        if dist(p,rand_pt)<dist(node,rand_pt):
            node=p
    new_rand_pt=set_edge_pt(node,rand_pt)
    nodes.append(new_rand_pt)
    cost.append(float('inf'))
    E.append((node,new_rand_pt))
    ax.plot([node[0], new_rand_pt[0]],[node[1], new_rand_pt[1]],'y-')

g_neighbor=[]
for vertex in nodes:
    if dist(vertex,fin_node)<goal_region:
        g_neighbor.append(vertex)
for node in g_neighbor:
    E.append((node,fin_node))
nodes.append(fin_node)
cost.append(float("inf"))

# Use Djikstra's algorithm to find the path
unvisited=list(nodes)
unvisited_cost=list(cost)
prev=list(nodes)
current_node=init_node

while unvisited:
    if current_node==fin_node:
        break
    neighbors=find_neighbor(current_node,E)
    for node in neighbors:
        d=dist(node,current_node)
        next_cost=cost[nodes.index(current_node)]+d
        ind=nodes.index(node)
        if next_cost<cost[ind]:
            prev[ind]=current_node
            cost[ind]=next_cost
            unvisited_cost[unvisited.index(node)]=next_cost
    ind_visited=unvisited.index(current_node)
    unvisited.pop(ind_visited)
    unvisited_cost.pop(ind_visited)
    if not unvisited_cost:
        print("empty")
    current_node=unvisited[unvisited_cost.index(min(unvisited_cost))]

# Find the path
path=[]
path.append(fin_node)
path_ind=nodes.index(fin_node)
while True:
    path.append(prev[path_ind])
    if prev[path_ind] is init_node:
        break
    path_ind=nodes.index(prev[path_ind])
path.reverse()

# Plot the final path
ax.plot([path_x[0] for path_x in path],[path_y[1] for path_y in path],'g-',label='Path')


# Print the details of the PRM implementation
total=0
for j in range(len(path)-1):
    total+=dist(path[j],path[j+1])
print("Start: {0}, Goal: {1}".format(init_node,fin_node))
print("Number of branches: {0:1}".format(len(E)))
print("Original path distance: {0:2.4f}".format(total))

# Signal the user to close the figure to continue on with the code.
print("\nClose the figure to continue on with the code.\n\n")
        
# Plot the initial and final points
ax.plot(init_node[0],init_node[1],'ro',label='Initial Point')
ax.plot(fin_node[0],fin_node[1],'bo',label='Final Point')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title("RRT Implementation of 2D path planning with no obstacles/no path smoothing")
ax.legend(bbox_to_anchor=(1,1))
plt.show()



# RRT Implementation in 2D path planning with obstacles and path smoothing (path smoothing currently does not have collision detection implemented)



# Create a class for obstacles that are square
class Obstacle:
    def __init__(self,center,angle,length):
        self.center=(center[0],center[1])
        self.ang=angle                       # Angle in rads, positive direction CCW from +x axis
        self.length=length
        corner_length=np.sqrt(2)*self.length/2
        cos_val=np.cos(self.ang+np.pi/4)*corner_length
        sin_val=np.sin(self.ang+np.pi/4)*corner_length
        self.corner=np.array([[cos_val+self.center[0],sin_val+self.center[1]],
                              [-sin_val+self.center[0],cos_val+self.center[1]],
                              [-cos_val+self.center[0],-sin_val+self.center[1]],
                              [sin_val+self.center[0],-cos_val+self.center[1]],
                              [cos_val+self.center[0],sin_val+self.center[1]]])

# Create two square obstacles
obs1=Obstacle((5,5),-np.pi/8,1.5)
obs2=Obstacle((4.5,2.5),np.pi/4,1.5)

# Check collision of the point with an obstacle
def collision(obs,pt):
    box_to_pt=np.array([pt[0]-obs.center[0],pt[1]-obs.center[1]])
    if np.linalg.norm(box_to_pt)==0:
        return True
    else:
        box_to_pt_norm=box_to_pt/np.linalg.norm(box_to_pt)
    max_val=float("-inf")
    for i in range(4):
        obs_pt=obs.corner[i,:]
        v=np.array([obs_pt[0]-obs.center[0],obs_pt[1]-obs.center[1]])
        current_proj=np.matmul(v,box_to_pt_norm)
        if max_val<current_proj:
            max_val=current_proj
    if np.linalg.norm(box_to_pt)-max_val>0:
        return False
    return True


# Initialize the figure
fig=plt.figure(figsize=(8.333,6),dpi=80)
ax=fig.add_subplot(111,autoscale_on=False, xlim=(0, xdim), ylim=(0, ydim))
ax.set_aspect('equal')

# Initialize a list of nodes and edges
nodes=[]
cost=[]
nodes.append(init_node)
cost.append(0)
E=[]

# Create the random tree
for i in range(numNodes):
    node=nodes[0]
    rand_pt=(random.random()*xdim, random.random()*ydim)
    for p in nodes:
        if dist(p,rand_pt)<dist(node,rand_pt):
            node=p
    new_rand_pt=set_edge_pt(node,rand_pt)
    if not collision(obs1,new_rand_pt) and not collision(obs2,new_rand_pt):
        intersect=False
        for j in range(div):
            line_point=((j/div)*node[0]+(1-j/div)*new_rand_pt[0],(j/div)*node[1]+(1-j/div)*new_rand_pt[1])
            if collision(obs1,line_point) or collision(obs2,line_point):
                intersect=True
                break
        if not intersect:
            nodes.append(new_rand_pt)
            cost.append(float('inf'))
            E.append((node,new_rand_pt))
            ax.plot([node[0], new_rand_pt[0]],[node[1], new_rand_pt[1]],'k-')

g_neighbor=[]
for vertex in nodes:
    if dist(vertex,fin_node)<goal_region:
        g_neighbor.append(vertex)
for node in g_neighbor:
    E.append((node,fin_node))
nodes.append(fin_node)
cost.append(float("inf"))

# Use Djikstra's algorithm to find the path
unvisited=list(nodes)
unvisited_cost=list(cost)
prev=list(nodes)
current_node=init_node

while unvisited:
    if current_node==fin_node:
        break
    neighbors=find_neighbor(current_node,E)
    for node in neighbors:
        d=dist(node,current_node)
        next_cost=cost[nodes.index(current_node)]+d
        ind=nodes.index(node)
        if next_cost<cost[ind]:
            prev[ind]=current_node
            cost[ind]=next_cost
            unvisited_cost[unvisited.index(node)]=next_cost
    ind_visited=unvisited.index(current_node)
    unvisited.pop(ind_visited)
    unvisited_cost.pop(ind_visited)
    if not unvisited_cost:
        print("empty")
    current_node=unvisited[unvisited_cost.index(min(unvisited_cost))]

# Find the path
path=[]
path.append(fin_node)
path_ind=nodes.index(fin_node)
while True:
    path.append(prev[path_ind])
    if prev[path_ind] is init_node:
        break
    path_ind=nodes.index(prev[path_ind])
path.reverse()

# Perform shortcut of path
shortcut=list(path)
i=0
div=30
while i<len(shortcut)-2:
    intersect=False
    n1=shortcut[i]
    n2=shortcut[i+2]
    for j in range(div):
        line_point=((j/div)*n1[0]+(1-j/div)*n2[0],(j/div)*n1[1]+(1-j/div)*n2[1])
        if collision(obs1,line_point) or collision(obs2,line_point):
            intersect=True
    if not intersect:
        shortcut.pop(i+1)
        if i>0:
            i-=1
    else:
        i+=1

# Smoothe the path using Quadratic B-Spline Curve       
i=0
div=30
M=np.array([[1,1,0],[-2,2,0],[1,-2,1]])
smooth=np.zeros((1,2))
smooth=np.vstack((smooth,np.array(shortcut[0])))
while i<len(shortcut)-2:
    s=np.array([list(shortcut[i]),list(shortcut[i+1]),list(shortcut[i+2])])
    ms=np.matmul(0.5*M,s)
    sm=np.zeros((30,2))
    for j in range(30):
        t=j/30
        t_arr=np.array([1,t,t**2])
        sm[j,:]=np.matmul(t_arr,ms)
    smooth=np.vstack((smooth,sm))
    i+=1
smooth=np.vstack((smooth,np.array(shortcut[-1])))

# Plot the path
ax.plot([path_x[0] for path_x in path],[path_y[1] for path_y in path],'-',color='lightgreen',label='Path')

# Plot the shortcut path
ax.plot([sc_x[0] for sc_x in shortcut],[sc_y[1] for sc_y in shortcut],'-',color='orange',label='Shortcut')

# Plot the shortcut path
ax.plot(smooth[1:,0],smooth[1:,1],'-',color='purple',label='Smooth Path')

# Plot the obstacles
ax.plot([obs[0] for obs in obs1.corner],[obs[1] for obs in obs1.corner],'y-',
        [obs[0] for obs in obs2.corner],[obs[1] for obs in obs2.corner],'y-')

# Print the details for the RRT implementation
total=0
for j in range(len(path)-1):
    total+=dist(path[j],path[j+1])
total_shortcut=0
for j in range(len(shortcut)-1):
    total_shortcut+=dist(shortcut[j],shortcut[j+1])
total_smooth=0
for j in range(smooth.shape[0]-2):
    total_smooth+=dist(smooth[j+1],smooth[j+2])
print("Start: {0}, Goal: {1}".format(init_node,fin_node))
print('Number of branches: '+str(len(E)))
print("Original path distance: {0:2.4f}".format(total))
print("Shortcut path distance: {0:2.4f}".format(total_shortcut))
print("Smooth path distance: {0:2.4f}".format(total_smooth))

# Tell user that path smoothing doesn't have collision detection and recalculation of the B-spline curve implemented yet
print("\nThe current code doesn't check collision along the smooth path and won't recalculate the B-spline curve that avoids collisions yet.")

# Plot the initial and final points
ax.plot(init_node[0],init_node[1],'ro',label='Initial Point')
ax.plot(fin_node[0],fin_node[1],'bo',label='Final Point')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title("RRT Implementation of 2D path planning with obstacles and path smoothing")
ax.legend(bbox_to_anchor=(1,1))
plt.show()
