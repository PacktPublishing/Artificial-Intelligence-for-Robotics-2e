#!/usr/bin/env python
import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt

mu, sigma = 3.0, 0.9
s = sigma*np.random.lognormal(mu, sigma,10000)

# the histogram of the data
n, bins, patches = plt.hist(s, 200, normed=True, align='mid')
x = np.linspace(min(bins), max(bins), 10000)


# add a 'best fit' line
#y = mlab.normpdf( bins, mu, sigma)
#l = plt.plot(x, 'r--', linewidth=1)

plt.xlabel('Samples')
plt.title('Histogram of Random Number Generator: Lognormal')
plt.axis([0, 100,0, 0.035])
plt.grid(True)

plt.show()

# x = np.random.random(10000)*1000.0
# newX = []
# for bb in x:
	# bb = 1.0/bb
	# newX.append(bb)
x = np.reciprocal(x)

# n, bins, patches = plt.hist(newX, 200, normed=True, align='mid')
# plt.axis([0, 1,0, 0.5])
# plt.show()