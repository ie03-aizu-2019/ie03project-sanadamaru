import random
print("1000 1999 100 0")

for i in range(0,1000):
    print("%d %d" % (random.randint(0,10000),random.randint(0,10000)));
          
for i in range(0,1999):
    print("%d %d" % (random.randint(1,1000),random.randint(1,1000)));

for i in range(0,100):
    print("%d %d" % (random.randint(1,10000),random.randint(1,10000)));
