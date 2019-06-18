import random
print("200 199 0 100")

for i in range(0,200):
    print("%d %d" % (random.randint(0,10000),random.randint(0,10000)));
          
for i in range(0,199):
    print("%d %d" % (random.randint(1,200),random.randint(1,200)));

for i in range(0,100):
    flag = random.randint(0,3);
    if flag == 0:
        print("%d %d %d" %(random.randint(1,200),random.randint(1,200),random.randint(1,10)));
    elif flag == 1:
        print("%d C%d %d" %(random.randint(1,200),random.randint(1,1000),random.randint(1,10)));
    elif flag == 2:
        print("C%d %d %d" %(random.randint(1,1000),random.randint(1,200),random.randint(1,10)));
    elif flag == 3:
         print("C%d C%d %d" %(random.randint(1,1000),random.randint(1,1000),random.randint(1,10)));
