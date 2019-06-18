import random
print("1000 500 0 100")

for i in range(0,1000):
    print("%d %d" % (random.randint(0,10000),random.randint(0,10000)));
          
for i in range(0,500):
    print("%d %d" % (random.randint(1,1000),random.randint(1,1000)));

for i in range(0,100):
    flag = random.randint(0,3);
    if flag == 0:
        print("%d %d 1" %(random.randint(1,1000),random.randint(1,1000)));
    elif flag == 1:
        print("%d C%d 1" %(random.randint(1,1000),random.randint(1,1000)));
    elif flag == 2:
        print("C%d %d 1" %(random.randint(1,1000),random.randint(1,1000)));
    elif flag == 3:
         print("C%d C%d 1" %(random.randint(1,1000),random.randint(1,1000)));
        
        
