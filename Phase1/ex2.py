class Point:    
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return "{:.5f} {:.5f}".format(self.x, self.y)

    def __repr__(self):
        return "{x}, {y}".format(x = self.x, y = self.y)
        
    def __lt__(self, other):
        return (self.x, self.y) < (other.x, other.y)

points = []

def askxy(i,j):
    A = [0] * M
    A[i] = tmpx[i]*-tmpy[j] + tmpx[j]*tmpy[i]

    # print(A[i])
    
    s = 0.0
    s = ((Y[p[j]-1]-Y[q[j]-1])*(X[p[j]-1]-X[p[i]-1])+(X[q[j]-1]-X[p[j]-1])*(Y[p[j]-1]-Y[p[i]-1]))/A[i]
    t = 0.0
    t = ((Y[p[i]-1]-Y[q[i]-1])*(X[p[j]-1]-X[p[i]-1])+(X[q[i]-1]-X[p[i]-1])*(Y[p[j]-1]-Y[p[i]-1]))/A[i]

    # print(s, t)
    
    eps = 10**(-20)

    if (s>eps and s<1) and (t>eps and t<1) and A[i]!=0:
        resx = X[p[i]-1]+(X[q[i]-1]-X[p[i]-1])*s
        resy = Y[p[i]-1]+(Y[q[i]-1]-Y[p[i]-1])*s
        # print("{0:.5f}".format(resx),end=' ')
        # print("{0:.5f}".format(resy))
        points.append(Point(resx, resy))

    
N,M,P,Q = map(int,input().split())

X = [0]*N
Y = [0]*N

for i in range(N):
    x,y = map(int,input().split())
    X[i] = x
    Y[i] = y

tmpx = [0]*M
tmpy = [0]*M
p = [0]*M
q = [0]*M
for i in range(M):
    b,e = map(int,input().split())
    tmpx[i] = X[e-1]-X[b-1]
    tmpy[i] = Y[e-1]-Y[b-1]
    p[i] = b
    q[i] = e

for i in range(0,M-1):
    for j in range(i+1,M):
        askxy(i,j)

# print(type(points[0]))

points.sort()
        
for i in points:
    # print("{0:.5f}".format(points[i].resx),end=' ')
    # print("{0:.5f}".format(points[i].resy))
    print(i)
