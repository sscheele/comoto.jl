import matplotlib.pyplot as plt
import numpy as np
import sys

def plot(title):
    distance = open("distance.txt", "r").read().split(", ")
    legibility = open("legibility.txt", "r").read().split(", ")
    visiblity = open("visibility.txt", "r").read().split(", ")
    jointvel = open("jointvel.txt").read().split(", ")
    nominal = open("nominal.txt").read().split(", ")

    distance = [float(x) for x in distance]
    legibility = [float(x) for x in legibility]
    visiblity = [float(x) for x in visiblity]
    jointvel = [float(x) for x in jointvel]
    nominal = [float(x) for x in nominal]

    x = np.arange(len(distance))*0.25
    plt.plot(x, distance,  label = "distance")
    plt.plot(x, legibility,  label = "legibility")
    plt.plot(x, visiblity,  label = "visibility")
    plt.plot(x, jointvel,  label = "joint velocity")
    plt.plot(x, nominal,  label = "nominal")

    plt.legend()
    plt.xlabel("Time in Seconds")
    plt.ylabel("Magnitude of cost")
    plt.title(title + " Calculator CoMOTO")
    plt.savefig('costs-'+title+'.png')

if __name__ == "__main__":
    plot(sys.argv[1])    

