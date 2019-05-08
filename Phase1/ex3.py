from heapq import heappush, heappop
import math

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
    # def get_shortest_path(self):
        
        

# 線分群からグラフ(Graph)を生成する。
# def segment_arrangement(segments):

N, M, P, Q = map(int,input().split())

points = [Point(0, 0)]*N
for i in range(N):
    x, y = map(int, input().split())
    points[i] = Point(x, y)

segments = [(0, 0)]*M
for i in range(M):
    p1, p2 = map(int, input().split())
    p1 -= 1
    p2 -= 1
    segments[i] = (p1, p2)

cross_points = []

for i in range(0, M-1):
    for j in range(i+1, M):
        a1, b1 = segments[i]
        a2, b2 = segments[j]
        p1 = points[a1]
        q1 = points[b1]
        p2 = points[a2]
        q2 = points[b2]        
        cross_point = get_cross_point(p1, q1, p2, q2)
        if cross_point != None:
            cross_points.append((cross_point, i, j))

cross_points.sort()

G = [[]for _ in range(M)]
for i, (p, a, b) in enumerate(cross_points):
    G[a].append((p, b, N+i))
    G[b].append((p, a, N+i))

graph = Graph(N+len(cross_points))

for i in range(M):
    l = len(G[i])
    a1, b1 = segments[i]
    p1 = points[a1]
    q1 = points[b1]
    graph.add_edge(a1, b1, get_distance(p1, q1))
    for j in range(0, l):
        cross_point1, u1, c1 = G[i][j]
        graph.add_edge(a1, c1, get_distance(p1, cross_point1))
        graph.add_edge(b1, c1, get_distance(q1, cross_point1))
        for k in range(j+1, l):
            cross_point2, u2, c2 = G[i][k]
            graph.add_edge(c1, c2, get_distance(cross_point1, cross_point2))    

for q in range(Q):
    a, b, k = map(str, input().split())
    if a[0] == 'C':
        u = N + int(a[1:len(a)]) - 1
    else:
        u = int(a)-1
    if b[0] == 'C':
        v = N + int(b[1:len(b)]) - 1
    else:
        v = int(b)-1
    if u >= N+len(cross_points) or v >= N+len(cross_points):
        print("NA")
    else:
        graph.calc_shortest_distance(u)
        print(round(graph.dist_from_start[v], 5))
