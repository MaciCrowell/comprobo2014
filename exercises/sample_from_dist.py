import random
from math import exp, pi, sqrt
import matplotlib.pyplot as plt
from numpy import linspace

def sample_from_dist(n,dist='uniform',sigma=1):
	if dist == 'uniform':
		generator = random.random
	elif dist == 'gaussian':
		generator = lambda: random.gauss(0,sigma)
	else:
		print "Invalid distribution"
		return []
	samples = []
	for i in range(n):
		samples.append(generator())
	return samples

def plot_density(plt,dist,sigma=1):
	if dist == 'uniform':
		xi = linspace(0,1,100)
		yi = [1 for i in xi]
		plt.plot(xi,yi)
		plt.xlim([-.1, 1.1])
		plt.ylim([-.1, 1.1])
	elif dist == 'gaussian':
		xi = linspace(-5,5,100)
		yi = [exp(-i**2/(2*sigma**2))/(sigma*sqrt(2.0*pi)) for i in xi]
		plt.plot(xi,yi)
		plt.xlim([-5, 5])
		plt.ylim([-.1, 2])
	else:
		print "Invalid distribution"
		return []
	plt.xlabel('x')
	plt.ylabel('p(x)')

if __name__ == '__main__':
	dist = 'uniform'
	sigma = 1
	plt.figure()
	samples = sample_from_dist(50,dist,sigma)
	h = plt.plot(samples,[0]*len(samples),'bx')
	plt.setp(h,'markersize',5)
	plt.hold(True)
	plot_density(plt,dist,sigma)
	plt.show()