import numpy as np
import math
import cmath
import heapq
from nav_msgs.msg import OccupancyGrid


class Point:
    def __init__(self,x,y):
        self.point = np.array([[x,y]])
    def __eq__(self, other):
        return (self.point == other.point)
    def __str__(self):
        ret_string = '(' + str(self.point[0][0]) + ',' + str(self.point[0][1]) + ')'
        return ret_string

class Edge:

    def __init__(self,src,dest):
        self.src = Point(src[0],src[1])
        self.dest = Point(dest[0],dest[1])
        self.h_signature = None

    def to_complex(self,arr):
        return complex(arr[0],arr[1])

    def helper_compute_signature(self,src,dest):
        
        src_abs = abs(src)
        dest_abs = abs(dest)
     
        h_signature = np.zeros(src_abs.shape[0],dtype=np.complex128)
        for i in range(src_abs.shape[0]):
            real_part = math.log(dest_abs[i]) - math.log(src_abs[i])
            src_phase = cmath.phase(src[i])
            dest_phase = cmath.phase(dest[i])
            min_value = dest_phase - src_phase

            for j in range(-8,9,1):
                current_value = (dest_phase - src_phase) + 2*math.pi*j

                if abs(min_value) > abs(current_value):
                    min_value = current_value
          
            h_signature[i] = complex(real_part,min_value)
        #print(h_signature)
        return h_signature
    
    def helper_compute_signature_2(self,src,dest):

        h_signature = np.zeros(src.shape[0])
        values_to_test = [i for i in range(-2,3)]

        for i in range(src.shape[0]):
            minimum_phase_difference = (cmath.phase(dest[i])) - (cmath.phase(src[i]))
            for j in values_to_test: 
                for k in values_to_test:
                    current_phase_difference = (cmath.phase(dest[i]) +2*math.pi*j)  - (cmath.phase(src[i]) + 2*math.pi*k)
                    if abs(current_phase_difference) < abs(minimum_phase_difference):
                        minimum_phase_difference = current_phase_difference
            h_signature[i] = minimum_phase_difference
            
        return h_signature
        
    def compute_h_signature(self,obstacle_coordinates):
        src_diff = self.src.point - obstacle_coordinates
        dest_diff = self.dest.point - obstacle_coordinates
        src_complex = np.apply_along_axis(self.to_complex,1,src_diff)
        dest_complex = np.apply_along_axis(self.to_complex,1,dest_diff)
        self.h_signature = self.helper_compute_signature_2(src_complex,dest_complex)
        return self.h_signature


class DijkstraNode:
    def __init__(self,point,cost,edge : Edge):
        self.point = point
        self.h_signature = None
        self.cost = cost
        self.parent = None
        self.edge = edge

    def __str__(self):
        return (str(self.point) + ";" + str(self.h_signature))
    
    def compute_h_signature(self,parent,obstacle_coordinates):
        self.edge.compute_h_signature(obstacle_coordinates)
        if parent is None:
            self.h_signature =  self.edge.h_signature
        else:
            self.h_signature = parent.h_signature + self.edge.h_signature
        self.parent = parent

    def __lt__(self,other):
        return (self.cost < other.cost)
    def __le__(self, other):
        return (self.cost < other.cost)
    def __eq__(self,other):
        return (self.cost == other.cost)
    def __ne__(self, other):
        return (self.cost != other.cost)
    def __gt__(self, other):
        return (self.cost > other.cost)
    def __ge__(self, other):
        return (self.cost >= other.cost)

def dijkstra_algorithm(src,dest,obstacle_coordinates,map):
    
    
    TRAJECTORY_LIMIT = 2
    COUNT_LIMIT = 2
    width = map.shape[1]
    height = map.shape[0]

    directions = np.array([[-1,1],[0,1],[1,1],[1,0],[1,-1],[0,-1],[-1,-1],[-1,0]])
    start_node = np.array([[src[0],src[1]]])
    end_node = np.array([[dest[0],dest[1]]])

    heap = []
    new_points = start_node + directions
    
    
    distance_count = {}
    trajectories_found = set()
    trajectories = []

    counter = 0
    for i in range(8):
        current_point = np.expand_dims(new_points[i],0)
        cost = 0 if map[new_points[i][0],new_points[i][1]]==-1 else map[new_points[i][0],new_points[i][1]]
        d = DijkstraNode(current_point,cost + np.linalg.norm(current_point-end_node),Edge((start_node[0][0],start_node[0][1]),(new_points[i][0],new_points[i][1])))
        d.compute_h_signature(None,obstacle_coordinates)
        heapq.heappush(heap,d)
    
    while heap:
        item = heapq.heappop(heap)
        if((0 > item.point[0][0] or item.point[0][0]>height or 0 > item.point[0][1] or item.point[0][1]>width)):
            continue
        
        if(np.any(np.all(item.point==obstacle_coordinates,axis=1))):
            continue
        
        if(np.all(item.point==end_node)):
            
            if(str(item) not in trajectories_found):
                print(str(counter) + ":"+ str(item.h_signature))
                counter+=1
                
                '''if counter==13:
                    node = item
                    
                    while(node.parent is not None):
                        print(str(node.point) + "--" + str(node.h_signature))
                        node = node.parent
                    print(node.h_signature)'''
                if counter >= COUNT_LIMIT:
                    break
                greater_than_two = (1.0/(2*math.pi))*(item.h_signature)
                #print(np.any(greater_than_two>=1.0) or np.any(greater_than_two <= -1.0))
                if np.any(greater_than_two>=1.0) or np.any(greater_than_two <= -1.0):
                    continue

                trajectories_found.add(str(item))
        
                current_trajectory = []
                node = item
                while(node.parent is not None):
                    current_trajectory.append(node.point.squeeze(0).tolist())
                    node = node.parent
                current_trajectory.append(node.point.squeeze(0).tolist())
                current_trajectory.append(start_node.squeeze(0).tolist())

                trajectories.append(current_trajectory)

                if(len(trajectories)>=TRAJECTORY_LIMIT):
                    return trajectories

            continue
            print("continue")
    
        new_points = item.point + directions
        for i in range(8):
            current_point = np.expand_dims(new_points[i],0)
            if((0 > new_points[0][0] or new_points[0][0]>height or 0 > new_points[0][1] or new_points[0][1]>width)):
                continue
            cost = 0 if map[new_points[i][0],new_points[i][1]]==-1 else map[new_points[i][0],new_points[i][1]]
            d = DijkstraNode(current_point,item.cost+ cost + np.linalg.norm(current_point-end_node),Edge((item.point[0][0],item.point[0][1]),(new_points[i][0],new_points[i][1])))
              
            if(np.any(np.all(d.point==obstacle_coordinates,axis=1))):
                continue
            
            #pygame.draw.circle(screen,(0,0,0),(offset + scale_x*d.point[0][0],offset +scale_y*d.point[0][1]),scale/8)

            d.compute_h_signature(item,obstacle_coordinates)

            if (str(d) in distance_count):
                if distance_count[str(d)] > d.cost:
                    distance_count[str(d)] = d.cost
                    heapq.heappush(heap,d)
            else:
                distance_count[str(d)] = d.cost
                heapq.heappush(heap,d)
            
    return trajectories


def get_trajectories(start_point,dest_point,obstacle_coordinates,map):
    trajectories = dijkstra_algorithm(start_point,dest_point,obstacle_coordinates,map)
    return trajectories
    

#obstacle_coordinates = np.array([[3,3],[7,7],[4,3]])
#traj = dijkstra_algorithm((0,0),(9,8))
#print(len(traj))

#trajectories_to_display = [i for i in range(len(traj))]
#trajectories_to_display = [i for i in range(len(traj))]