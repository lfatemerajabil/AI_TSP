import math
import numpy as np
import time
import matplotlib.pyplot as plt
#import random
#import numpy

n = 10
nodes = [(0.896918430605464, 0.6419761311064077),
         (0.6075246366936281, 0.3960013606982353),
         (0.3969583644464204, 0.212708507511278),
         (0.39751738302281725, 0.3066567678288378),
         (0.8446848510696021, 0.07659781531199383),
         (0.3708941589005984, 0.6168880380108651),
         (0.14453230079185886, 0.7025146550918926),
         (0.3428149407738543, 0.7761737287849915),
         (0.6446911196740392, 0.8208398743347229),
         (0.04190157392120375, 0.25698001415977967)]
'''
nodes = []
for i in range(n):
    x = random.random()
    y = random.random()
    nodes.append((x,y))
'''
initial = []
for i in range(n):
    initial.append(i)

H = 0
for i in range(len(initial)-1):
    h = math.sqrt(math.pow(nodes[i][0]-nodes[i+1][0] , 2)+math.pow(nodes[i][1]-nodes[i+1][1] , 2))
    H += h
end = math.sqrt(math.pow(nodes[n-1][0]-nodes[0][0] , 2)+math.pow(nodes[n-1][1]-nodes[0][1] , 2))
H += end

def action(initial, H, nodes, n):
    cycles = []
    for i in range(n):
        cycles.append([np.array(initial), H])
    for i in range(n-1):
        cycles[i][0][i] , cycles[i][0][i+1] = cycles[i][0][i+1] , cycles[i][0][i]
        C = 0
        for j in range(n-1):
            cost = math.sqrt(math.pow(nodes[cycles[i][0][j]][0]-nodes[cycles[i][0][j+1]][0] , 2)+math.pow(nodes[cycles[i][0][j]][1]-nodes[cycles[i][0][j+1]][1] , 2))
            C += cost
        end = math.sqrt(math.pow(nodes[cycles[i][0][n-1]][0]-nodes[cycles[i][0][0]][0] , 2)+math.pow(nodes[cycles[i][0][n-1]][1]-nodes[cycles[i][0][0]][1] , 2))
        C += end
        cycles[i][1] = C
    cycles[n-1][0][n-1] , cycles[n-1][0][0] = cycles[n-1][0][0] , cycles[n-1][0][n-1]
    C = 0
    for j in range(n-1):
        cost = math.sqrt(math.pow(nodes[cycles[n-1][0][j]][0]-nodes[cycles[n-1][0][j+1]][0] , 2)+math.pow(nodes[cycles[n-1][0][j]][1]-nodes[cycles[n-1][0][j+1]][1] , 2))
        C += cost
    end = math.sqrt(math.pow(nodes[cycles[n-1][0][n-1]][0]-nodes[cycles[n-1][0][0]][0] , 2)+math.pow(nodes[cycles[n-1][0][n-1]][1]-nodes[cycles[n-1][0][0]][1] , 2))
    C += end
    cycles[n-1][1] = C
    return cycles

def tsp(initial , H , nodes , n):
    time1 = time.perf_counter()
    
    path = np.array(initial)
    while True:
        x = path
        X, Y = [], []
        for i in range(len(path)):
            X.append(nodes[path[i]][0])
            Y.append(nodes[path[i]][1])
        X.append(nodes[path[0]][0])
        Y.append(nodes[path[0]][1])
        plt.figure()
        plt.plot(X, Y, color='green', linestyle='solid', linewidth = 3, 
                 marker='o', markerfacecolor='blue', markersize=12)
        plt.ylim(0,1)
        plt.xlim(0,1)
        plt.show()
        time.sleep(1)
        
        Action = action(path.tolist(), H, nodes, n)
        for act in Action:
            if(act[1]< H):
                H = act[1]
                path = act[0]
        if(x.tolist() != path.tolist()):
            continue
        if(x.tolist() == path.tolist()):
            break
    
    time2 = time.perf_counter()
    print("time for solving TSP with local search:",(time2-time1))
    return path.tolist(), H

print(tsp(initial, H, nodes, n))