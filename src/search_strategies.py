'''
Created on Sep 15, 2016

@author: Anuradha
'''
from pip._vendor.distlib.compat import raw_input
import sys
import re
import ast
import heapq
from queue import PriorityQueue  , LifoQueue
from _ast import Num
from telnetlib import theNULL
from math import sqrt
import math

time_complexity=0
space_complexity_frontier=0
space_complexity_explored=0


def heuristic(a, b,type):
   # Manhattan distance on a square grid
    if type == 'euclidean':
        return (sqrt(sum( (a - b)**2 for a, b in zip(a, b))))
    if type == "manhattan":
        return (abs(a[0] - b[0]) + abs(a[1] - b[1]))
    if type == "jaccard":
        intersection_cardinality = len(set.intersection(*[set(a), set(b)]))
        union_cardinality = len(set.union(*[set(a), set(b)]))
        return 1-(intersection_cardinality/float(union_cardinality))
    if type == 'logical_heuristic':
        if b[0] == 0:
            if b[1] in a:
                return 0
            else :
                return 1
        elif b[1]==0:
            if b[0] in a:
                return 0
            else :
                return 1
        else:
            if a[0] or a[1] not in b:
                return 0
            else :
                return 1
    return null

def idastar_paths(start,goal):
    cut_off=getheuristic(start,goal);
    while 1:
        cut_off,solution=cost_limited_dfs(start,goal,cut_off)
        if solution != 'fail': 
            return solution
        if cut_off == math.inf:
            return 'fail'
               
def cost_limited_dfs(start,goal,cut_off):
    global time_complexity
    global space_complexity_frontier
    global space_complexity_explored
    next_min=math.inf  
    stack=LifoQueue()
    stack.put((0,[start]))

    while 1:       
        if stack.empty():
            return next_min,'fail'

        cost,path=stack.get()
        f_score=cost+getheuristic(path[-1], goal)
        print("f(n): ",f_score,"cost: ",cost,"h(n): ",getheuristic(path[-1], goal),"frontier:",path)
        if f_score <= cut_off:
            if path[-1]==goal:
                print("TIME COMPLEXITY",time_complexity)
                print("SPACE COMPLEXITY frontier",space_complexity_frontier)  
                print("=========================================================================================")
                return next_min, path

            nodes=nextstep(path[-1])
            for next in nodes:
                if next not in path:
                    total_cost=cost+getcost(path[-1], next)
                    time_complexity=time_complexity+1
                    stack.put((total_cost,path+[next]))
        else:
            if f_score < next_min:
                next_min=f_score   
                 
        if space_complexity_frontier < stack.qsize():
            space_complexity_frontier=stack.qsize() 
                          
def greedy_paths(start, goal):
    visited=set()
    cost=0 
    queue=PriorityQueue()
    global time_complexity
    global space_complexity_frontier
    global space_complexity_explored

    queue.put((getheuristic(start, goal),[start]))
    while queue != []:        
        cost,path= queue.get()
        print("h(n): ",cost,"frontier :",path)
        if path[-1] not in visited:
            visited.add(path[-1])
        if path[-1]==goal: 
            print("TIME COMPLEXITY",time_complexity)
            print("SPACE COMPLEXITY explored",space_complexity_explored)
            print("SPACE COMPLEXITY frontier",space_complexity_frontier) 
            print("=========================================================================================")                          
            return path,cost
        nodes=nextstep(path[-1])
        for next in nodes:
            if next not in visited:
                total_cost=getheuristic(next,goal)
                time_complexity=time_complexity+1
                queue.put((total_cost,path+[next]))
        
        if space_complexity_frontier < queue.qsize():
            space_complexity_frontier=queue.qsize()
        if space_complexity_explored <len(visited):
            space_complexity_explored=len(visited)
    return []

def astar_paths(start, goal):
    visited=set()
    cost=0 
    queue=PriorityQueue()
    global time_complexity
    global space_complexity_frontier
    global space_complexity_explored

    queue.put((getheuristic(start, goal),[start]))
    while queue != []:
        cost,path= queue.get()
        print("f(n): ",cost,"h(n) :",getheuristic(path[-1], goal),"frontier: ",path)
        if path[-1] not in visited:
            visited.add(path[-1])
        if path[-1]==goal: 
            print("TIME COMPLEXITY",time_complexity)
            print("SPACE COMPLEXITY explored",space_complexity_explored)
            print("SPACE COMPLEXITY frontier",space_complexity_frontier)
            print("=========================================================================================")                            
            return path,cost

        nodes=nextstep(path[-1])
        for next in nodes:
            if next not in visited:
                total_cost=cost+getcost(path[-1],next)+getheuristic(next,goal)-getheuristic(path[-1], goal)
                time_complexity=time_complexity+1
                queue.put((total_cost,path+[next]))
                
        if space_complexity_frontier < queue.qsize():
            space_complexity_frontier=queue.qsize()
        if space_complexity_explored <len(visited):
            space_complexity_explored=len(visited)

    return []

def ucs_paths(start, goal):
    visited=set()
    cost=0 
    queue=PriorityQueue()
    global time_complexity
    global space_complexity_frontier
    global space_complexity_explored
    queue.put((cost,[start]))
    while queue != []:
        cost,path= queue.get()
        print("g(n):",cost,"frontier:",path)
        if path[-1] not in visited:
            visited.add(path[-1])
        if path[-1]==goal: 
            print("TIME COMPLEXITY",time_complexity)
            print("SPACE COMPLEXITY explored",space_complexity_explored)
            print("SPACE COMPLEXITY frontier",space_complexity_frontier) 
            print("=========================================================================================")            
            return path,cost

        nodes=nextstep(path[-1])
        for next in nodes:
            if next not in visited:
                total_cost=cost+getcost(path[-1],next)
                time_complexity=time_complexity+1
                queue.put((total_cost,path+[next]))
                
        if space_complexity_frontier < queue.qsize():
            space_complexity_frontier=queue.qsize()
        if space_complexity_explored <len(visited):
            space_complexity_explored=len(visited)

    return []

def iddfs_paths(start,goal):
    global time_complexity
    global space_complexity_frontier
    depth=0
    stack=[]
    while 1:       
        if stack==[]:
            depth=depth+1
            stack=[[start]]
        path=stack.pop()

        if path[-1]==goal:
            print("TIME COMPLEXITY",time_complexity)
            print("SPACE COMPLEXITY frontier",space_complexity_frontier)
            print("=========================================================================================")
            return path
        if len(path)>depth:
            continue
        nodes=nextstep(path[-1])
        for next in nodes:
            if next not in path:
                time_complexity=time_complexity+1
                stack.append((path + [next]))
                
        if space_complexity_frontier < len(stack):
            space_complexity_frontier=len(stack)   
    return []

def dfs_paths(start, goal):
    global time_complexity
    global space_complexity_frontier
    stack = [[start]]
    while stack != []:
        path = stack.pop()
        if path[-1]==goal:
            print("TIME COMPLEXITY",time_complexity)
            print("SPACE COMPLEXITY frontier",space_complexity_frontier)
            print("=========================================================================================")
            return path

        nodes=nextstep(path[-1])
        for next in nodes:
            if next not in path:
                time_complexity=time_complexity+1
                stack.append((path + [next]))
                
        if space_complexity_frontier < len(stack):
            space_complexity_frontier=len(stack)
    return []


                
def bfs_paths(start, goal):
    global time_complexity
    global space_complexity_frontier
    global space_complexity_explored
    visited=set()
    queue = [[start]]

    while queue != []:
        path = queue.pop(0)
        if path[-1] not in visited:
            visited.add(path[-1])
        
        if path[-1]==goal:
            print("TIME COMPLEXITY",time_complexity)
            print("SPACE COMPLEXITY explored",space_complexity_explored)
            print("SPACE COMPLEXITY frontier",space_complexity_frontier)
            print("=========================================================================================")
            return path
        nodes=nextstep(path[-1])
        for next in nodes:
            if next not in visited:
                time_complexity=time_complexity+1
                queue.append((path + [next]))
        if space_complexity_frontier < len(queue):
            space_complexity_frontier=len(queue)
        if space_complexity_explored <len(visited):
            space_complexity_explored=len(visited)
    return[]

if __name__ == '__main__':
    
    print("=========================================================================================")
    filename=sys.argv[1]
    searchAlgo=sys.argv[2]
    print("FILENAME : ",filename,"SEARCH AlGORITHM : ",searchAlgo)
    if len(sys.argv)>3:
        heuristicFunc=sys.argv[3]
        print("HEURISTIC :", heuristicFunc)
    else:
        if searchAlgo=='greedy' or searchAlgo=='astar' or searchAlgo=='idastar':
            heuristicFunc=raw_input("Please provide heuristic \n")
            print("HEURISTIC :", heuristicFunc)
     
    array = []
    with open(filename, "r") as configFile:
        dict={}
        i=1;
        for line in configFile:
            dict[i]=line.strip('\n')
            i=i+1


    puzzle=dict.pop(1)
    print("=========================================================================================")
    if puzzle == 'cities':
        cities = dict.pop(2).strip('"')
        cities=eval(cities)
        startingCity=dict.pop(3).strip('"')
        print("START",startingCity)
        destinationCity=dict.pop(4).strip('"')
        print("GOAL",destinationCity) 
        
        def nextstep(path): 
            cost=0
            nextpath=[]
            for x in dict:
                input=ast.literal_eval(dict[x])
                if path==input[0]:
                    nextpath.append(input[1])
                elif path == input[1]: 
                    nextpath.append(input[0])
            return nextpath 

        def getcost(from_node,to_node):
            cost=0;
            for x in dict:
                input=ast.literal_eval(dict[x])
                if from_node==input[0] and to_node==input[1]:
                    cost=input[2]
                elif from_node==input[1] and to_node==input[0]:
                    cost=input[2]
            return cost
        
        def getheuristic(from_node,to_node):
            cost=0
            a=(0,0)
            x=0

            for x,y,z in cities:
                if from_node==x:
                    a=(y,z)
                if to_node==x:
                    b=(y,z)
                 
            cost=heuristic(a,b,heuristicFunc)
            return cost
        print("=========================================================================================")        
        if searchAlgo == 'bfs':
            print("BFS",bfs_paths(startingCity, destinationCity))
        elif searchAlgo == 'dfs':
            print("DFS:",dfs_paths(startingCity, destinationCity))
        elif searchAlgo == 'iddfs':
            print("IDDFS:",iddfs_paths(startingCity, destinationCity))
        elif searchAlgo == 'ucs':
            print("UCS",ucs_paths(startingCity, destinationCity))
        elif searchAlgo == 'greedy':
            print("Greedy",greedy_paths(startingCity, destinationCity))
        elif searchAlgo == 'astar':
            print("Astar",astar_paths(startingCity, destinationCity))
        elif searchAlgo == 'idastar':
            print("IDA star",idastar_paths(startingCity, destinationCity))
             
        print("=========================================================================================")
    elif puzzle == 'jugs':
        size=ast.literal_eval(dict.pop(2))
        jug1_size=size[0]
        jug2_size=size[1]
        initial=ast.literal_eval(dict.pop(3))
        jug1_init=initial[0]
        jug2_init=initial[1]
        final=ast.literal_eval(dict.pop(4))
        jug1_final=final[0]
        jug2_final=final[1]

        print("INITIAL",initial)
        print("FINAL",final)
        def nextstep(state): 
            x,y=state
            states=[(x,jug2_size),(jug1_size,y),(0,y),(x,0)]
            if x+y>=jug1_size:
                states=states+[(jug1_size,y-(jug1_size-x))] 
            else:
                states=states+[(x+y,0)]
            if x+y >=jug2_size:
                states=states+[(x-(jug2_size-y),jug2_size)]
            else:
                states=states+[(0,x+y)]
            return list(set(states))
        
        def getcost(from_state,to_state):
            cost=1;      
            return cost
        
        def getheuristic(from_node,to_node):              
            cost=heuristic(from_node,to_node, heuristicFunc)
            return cost
        
        if searchAlgo == 'bfs':
            print("BFS",bfs_paths(initial, final))
        elif searchAlgo =="dfs" :
            print("DFS",dfs_paths(initial, final))
        elif searchAlgo == 'iddfs':
            print("IDDFS",iddfs_paths(initial, final))
        elif searchAlgo == 'ucs':
            print("UCS",ucs_paths(initial, final))
        elif searchAlgo == 'greedy':
            print("Greedy",greedy_paths(initial, final))
        elif searchAlgo == 'astar':
            print("Astar",astar_paths(initial, final))
        elif searchAlgo == 'idastar':
            print("IDA star",idastar_paths(initial, final))
        
        print("=========================================================================================")
            
    elif puzzle == 'pancakes':
        initial=ast.literal_eval(dict.pop(2))
        print("INITIAL",initial)
        goal=ast.literal_eval(dict.pop(3))
        print("GOAL",goal)

        def nextstep(state):                                            
            h=0;        
            states=[]
            for i in range(1,len(state)+1):              
                h= ([-x for x in list(reversed(state[:i]))]) + list(state[i:len(state)])   
                h=tuple(h)
                states.append(h)
            return states
        
        def getcost(from_state,to_state):
            cost=1;         
            return cost
        
        def getheuristic(from_node,to_node):              
            cost=heuristic(from_node,to_node, heuristicFunc)
            return cost
        print("=========================================================================================")
        if searchAlgo == 'bfs':
            print("BFS",bfs_paths(initial, goal))
        elif searchAlgo =="dfs" :
            print("DFS",dfs_paths(initial, goal))
        elif searchAlgo == 'iddfs':
            print("IDDFS",iddfs_paths(initial, goal))
        elif searchAlgo == 'ucs':
            print("UCS",ucs_paths(initial, goal))
        elif searchAlgo == 'greedy':
            print("Greedy",greedy_paths(initial, goal))
        elif searchAlgo == 'astar':
            print("Astar",astar_paths(initial, goal))
        elif searchAlgo == 'idastar':
            print("IDA star",idastar_paths(initial, goal))
        print("=========================================================================================")