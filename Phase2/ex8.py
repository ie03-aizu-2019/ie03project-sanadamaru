from heapq import heappush, heappop
import math
import copy

class Point:    
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return "{:.5f} {:.5f}".format(self.x, self.y)
        
    def __lt__(self, other):
        return (self.x, self.y) < (other.x, other.y)
    
class Segment:
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2

def get_distance(p1, p2):
    x1 = p1.x
    y1 = p1.y
    x2 = p2.x
    y2 = p2.y
    d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return d
    
# (p1, p2) と (p3, p4) が交差する座標(Point)を返す。
# 交差しない場合は、None を返す。
def get_cross_point(p1, q1, p2, q2):
    EPS = 10**(-9)
    A = (q1.x-p1.x)*(p2.y-q2.y) + (q2.x-p2.x)*(q1.y-p1.y)

    if ( abs(A) < EPS ):
        return None
    
    s = ((p2.y-q2.y)*(p2.x-p1.x) + (q2.x-p2.x)*(p2.y-p1.y)) / A
    t = ((p1.y-q1.y)*(p2.x-p1.x) + (q1.x-p1.x)*(p2.y-p1.y)) / A
    # print(A, s, t)
    if (EPS < s < 1-EPS) and (EPS < t < 1-EPS):
        x = p2.x + (q2.x-p2.x)*t
        y = p2.y + (q2.y-p2.y)*t
        return Point(x, y)
    else:
        return None
    

class Graph:
    class Edge:
        def __init__(self, to, cost):
            self.to = to
            self.cost = cost

    # V : 頂点数
    def __init__(self, V):
        self.V = V        
        self.graph = [[]for _ in range(V)]
        self.INF = 10**9
        self.previous_node = [self.INF] * V
        self.dist_from_start = [self.INF] * V
        self.k = 0
        self.bridges = []
        self.ords = [0] * V
        self.low = [0] * V
        self.vis = [False] * V
        

    def add_edge(self, v, to, cost):
        self.graph[v].append(self.Edge(to, cost))
        self.graph[to].append(self.Edge(v, cost))

    # start から goal への最短距離を求める。ダイクストラ法
    def calc_shortest_distance(self, start):
        self.previous_node = [self.INF]*self.V
        self.dist_from_start = [self.INF]*self.V
        que = []
        self.dist_from_start[start] = 0
        heappush(que, (0, start))        
        while len(que):
            dist, v = heappop(que)

            if self.dist_from_start[v] < dist:
                continue
            
            for e in self.graph[v]:
                to = e.to
                cost = e.cost
                if (dist+cost, v) < (self.dist_from_start[to], self.previous_node[to]):
                    self.dist_from_start[to] = dist+cost
                    self.previous_node[to] = v
                    heappush(que, (self.dist_from_start[to], to))
    # 最短経路を求める
    def get_shortest_path(self, goal):
        ans = []
        now = goal
        ans.append(v)
        while self.previous_node[now] != self.INF:            
            tmp = self.previous_node[now]
            ans.append(tmp)
            now = self.previous_node[now]        
        ans.reverse()
        return ans

    def get_k_shortest_path(self, start, goal, k):
        ans = []
        que = []
        tmp = copy.deepcopy(self.graph)
        self.previous_node = [self.INF] * self.V
        self.dist_from_start = [self.INF] * self.V
        self.calc_shortest_distance(start)
        d = self.dist_from_start[goal]
        path = self.get_shortest_path(goal)
        heappush(que, (d, path))
        S = []
        for i in range(k):
            if len(que) == 0:
                break
            d, path = heappop(que)            
            S.append(path)
            ans.append((d, path))
            squa_root = []
            sum_dist = 0
            pre = -1
            self.graph = copy.deepcopy(tmp)
            for v in path[0:len(path)-1]:                
                squa_root.append(v)
                
                pre = squa_root[0]                
                
                if pre != -1:
                    for e in tmp[pre]:
                        if e.to == v:
                            sum_dist += e.cost     
                            break
                pre = v                
                for b, a in ans:                    
                    if len(a) < len(squa_root)+1:
                        continue                    
                    if squa_root == a[0:len(squa_root)]:
                        u = a[len(squa_root)]                  
                        for j, e in enumerate(self.graph[v]):
                            if e.to == u:
                                self.graph[v][j].cost = self.INF
                                
                self.previous_node = [self.INF] * self.V
                self.dist_from_start = [self.INF] * self.V
                self.calc_shortest_distance(v)
                d = self.dist_from_start[goal]
                if d < self.INF:
                    path = self.get_shortest_path(goal)
                    # print(d+sum_dist, squa_root[0:len(squa_root)-1]+path)
                    new_path = squa_root[0:len(squa_root)-1]+path
                    flag = True
                    for e in S:
                        if e == new_path:
                            flag = False
                            break
                    if flag:
                        heappush(que, (d+sum_dist, new_path))
                        S.append(new_path)

                for e in self.graph[v]:
                    e.cost = self.INF
        self.graph = tmp
        return ans

    def dfs(self, v, p):
        self.vis[v] = True
        self.ords[v] = self.k
        self.k = self.k+1
        self.low[v] = self.ords[v]

        ct = 0
        for i in self.graph[v]:
            u = i.to
            if self.vis[u] == False:
                ct += 1
                self.dfs(u, v)
                self.low[v] = min(self.low[v], self.low[u])
                if self.ords[v] < self.low[u]:
                    self.bridges.append((v, u))
            elif u != p:
                self.low[v] = min(self.low[v], self.ords[u])

    def calc_bridges(self):
        for i in range(self.V):
            if self.vis[i] == False:                
                self.dfs(i, -1)

        return self.bridges
                    

# C10 のような交差点を表す入力を頂点番号を表す整数に変換
def ctoi(a, N):
    if a[0] == 'C':
        u = N + int(a[1:len(a)]) - 1
    else:
        u = int(a)-1
    return u

# def itoc(i):    
    

# 線分群からグラフ(Graph)を生成する。
# def segment_arrangement(segments):
if __name__ == '__main__':

    N, M = map(int,input().split())

    graph = Graph(N)
    for i in range(M):
        x, y = map(int, input().split())
        x -= 1
        y -= 1
        graph.add_edge(x, y, 1)
        
    bridges = graph.calc_bridges()
    print(len(bridges))
    for a, b in bridges:
        print(a+1, b+1)
