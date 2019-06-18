#追加の座標(x0,y0)
#線分の端点の座標1(x1,y1)
#線分の端点の座標2(x2,y2)


import math

x0 = int(input());
y0 = int(input());
x1 = int(input());
y1 = int(input());
x2 = int(input());
y2 = int(input());


def distance(x0,y0,x1,y1,x2,y2): 
    a = x2 - x1;
    b = y2 - y1;
    a2 = a * a;
    b2 = b * b;
    r2 = a2 + b2;
    tt = -(a*(x1-x0)+b*(y1-y0));
    if tt<0:
        return math.sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0));

    if tt>r2:
        return math.sqrt((x2-x0)*(x2-x0) + (y2-y0)*(y2-y0));


    f1 = a*(y1-y0)-b*(x1-x0);
    return math.sqrt((f1*f1)/r2);



def cross(x0,y0,x1,y1,x2,y2):
    d1 = math.sqrt((x0-x1)*(x0-x1)+(y0-y1)*(y0-y1));
    d2 = distance(x0,y0,x1,y1,x2,y2);
    sin = d2/d1;
    cos = math.sqrt(1-sin*sin);
    d3 = d1*cos;
    ab = math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    x = (x1*(ab-d3)+x2*d3)/ab;
    y = (y1*(ab-d3)+y2*d3)/ab;
    print(x);
    print(y);

    return #xとy両方を返すポインタ

cross(x0,y0,x1,y1,x2,y2);

    
