# This code generates A^-1 and b matrices that is used to calibrate the magnetometer sensor
# readings as following:
#     h = A^-1 * (h_m - b)
#     where,
#       h_m is measured heading vector pointing to magnetic north in [x y z]^T
#       h is the estimated TRUE heading vector pointing to magnetic north

import numpy as np
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import quadrics
from scipy import linalg

if __name__=="__main__":

    data = np.loadtxt('magdata_xyz.txt', delimiter=',')

    X = data[:,[0]]
    Y = data[:,[1]]
    Z = data[:,[2]]

    num = len(X)


    # fig = plt.figure()
    # ax = fig.gca(projection='3d')
    # ax = fig.add_subplot(projection='3d')

    # ax.scatter(X,Y,Z,marker='.',color='r')

    Q = quadrics.quadric_equation(X,Y,Z)
    M = Q[0:3, 0:3]
    n = Q[0:3, 3]
    n = np.transpose(n)
    d = 1.0

    M_1 = linalg.inv(M)
    b = -np.dot(M_1, n)
    A_1 = np.real(40 / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d + 0j) * linalg.sqrtm(M))

    print("\nA^-1=")
    print(A_1)
    print("\nb=")
    print(b)

    # xmax = np.max(X) + 10
    # xmin = np.min(X) - 10
    # ymax = np.max(Y) + 10
    # ymin = np.min(Y) - 10
    # zmax = np.max(Z) + 10
    # zmin = np.min(Z) - 10
    # 
    # ax = quadrics.plot_quadric(ax,Q,xmax,xmin,ymax,ymin,zmax,zmin)
    # ax.scatter(X,Y,Z,marker='.',color='b')
    # plt.show()

