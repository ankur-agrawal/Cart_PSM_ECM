#!/usr/bin/env python
import csv

b=[1, 2, 3, 4, 5 ]
c= [3, 4,5, 6, 7 ]
a = [None] * 2

a[0]=b
a[1]=c
print(a[0])
print(a[1])

with open('qdd.csv', 'wb') as myfile:
      wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
      for i in range(0,2):
        wr.writerow(a[i])
with open('qddd.csv', 'wb') as myfile:
      wr = csv.writer(myfile, quoting=csv.QUOTE_NONE)
      for i in range(0,2):
        wr.writerow(a[i])

print(len(a))