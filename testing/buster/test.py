from numpy import number


a = (1,1,1,1)
b = (2,1,2,3)
c = []
for idx, number in enumerate(a):
    c.append(number + b[idx])
print(tuple(c))