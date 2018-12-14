#!/usr/bin/python
# -*- coding:utf-8 -*-

import csv
import numpy as np
import matplotlib.pyplot as plt

dataSet = []

fr = open("result.csv",'r')
for line in fr.readlines():
    dataSet.append(map(float,line.strip().split(',')))
length = len(dataSet)
print("length of dataSet is {}".format(length))
fr.close()

plt.plot(dataSet)
plt.show()