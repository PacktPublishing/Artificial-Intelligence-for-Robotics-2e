# -*- coding: utf-8 -*-
"""
Created on Mon Jul  9 23:45:19 2018

@author: bh47612
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import matplotlib.plot as plt
def func(x, a, b, c):
    return a * np.exp(-b * x) + c

x = np.array([0,453,481,498.5,508.2,515])
y = np.array([2.5,24,36,48,60,72])

popt, pcov = curve_fit(func, x, y)

print popt
a = popt[0]
b = popt[1]
c = popt[2]
# test curve
for testx in x:
    yy =func(testx,*popt)
    print yy
    
print y
