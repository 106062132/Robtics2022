import cv2
import sys
sys.path.append("..")
import PathPlanning.utils as utils
from PathPlanning.planner import Planner

class PlannerAStar(Planner):
    def __init__(self, m, inter=10):
        super().__init__(m)
        self.inter = inter
        self.initialize()

    def initialize(self):
        self.queue = []
        self.parent = {}
        self.h = {} # Distance from start to node
        self.g = {} # Distance from node to goal
        self.goal_node = None

    def planning(self, start=(100,200), goal=(375,520), inter=None, img=None):
        if inter is None:
            inter = self.inter
        start = (int(start[0]), int(start[1]))
        goal = (int(goal[0]), int(goal[1]))
        # Initialize 
        self.initialize()
        self.queue.append(start)
        self.parent[start] = None
        self.g[start] = 0
        self.h[start] = utils.distance(start, goal)
        while(1):
            # TODO: A Star Algorithm
            cost = 10000000
            index_pop = 0
            for index in range(len(self.queue)):
                item = self.queue[index]
                if(self.g[item] + self.h[item] < cost):
                    cost = self.g[item] + self.h[item]
                    index_pop = index
            now_pts = self.queue.pop(index_pop)
            cv2.circle(img,now_pts,2,(0,0,1),3)
            if(utils.distance(now_pts, goal) < self.inter):
                self.goal_node = now_pts   
                break

            for i in range(9):
                if(i != 4):
                    next_pts = (now_pts[0]+(i%3-1)*inter, now_pts[1]+(int(i/3)-1)*inter)
                    if(self.map[next_pts[1],next_pts[0]]>0.5):
                        if(next_pts in self.queue):
                            g = self.g[now_pts] + self.inter
                            h = utils.distance(next_pts, goal)
                            if(g + h < self.g[next_pts] + self.h[next_pts]):
                                self.g[next_pts] = g
                                self.h[next_pts] = h
                        if(next_pts not in self.parent):
                            g = self.g[now_pts] + self.inter
                            h = utils.distance(next_pts, goal)
                            self.g[next_pts] = g
                            self.h[next_pts] = h
                            self.parent[next_pts] = now_pts
                            self.queue.append(next_pts)
        
        # Extract path
        path = []
        p = self.goal_node
        if p is None:
            return path
        while(True):
            path.insert(0,p)
            if self.parent[p] is None:
                break
            p = self.parent[p]
        if path[-1] != goal:
            path.append(goal)
        return path
