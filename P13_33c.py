import numpy as np
from modern_robotics import *

np.set_printoptions(precision=3,suppress=True)

def t_sb(phi,x,y):
    t = [[cos(phi),-sin(phi),0.0,x],[sin(phi),cos(phi),0,y],[0.0,0.0,1.0, 0.0963],[0,0,0,1]]
    return t

def main():
    x0,y0,phi0 = 0,0,0
    Tsb = t_sb(phi0,x0,y0)
    Tb0 = [[1.0,0.0,0.0, 0.1662],[0.0,1.0,0.0,0.0],[0.0,0.0,1.0,0.0026],[0,0,0,1]]

    M0e=[[1.0,0.0,0.0,0.0330],[0.0,1.0,0.0,0.0],[0.0,0.0,1.0,0.6546],[0,0,0,1]]

    M = np.array(np.dot(np.dot(Tsb,Tb0),M0e),float)

    Blist = np.array([[0, 0,  1, 0, 0.1992, 0],
                  [0, 0,  0, 1, 0, 0],
                  [0, 0,  0, 0, 1, 0],
                  [0,0,1,0,0.0330,0],
                  [0, -1,  0, -0.5076, 0,   0],
                  [0, -1,  0, -0.3526, 0, 0],
                  [0, -1,  0, -0.2176, 0, 0],
                  [0, 0,  1, 0, 0, 0]],float).T

    X = np.array([[1.0,0.0,0.0, 0.0],[0.0, 0.0, 1.0, 1.0],[0.0, -1,  0.0, 0.4],[  0,   0,    0,   1]],float)


    thetalist0 = [phi0,x0,y0,0,0,-pi/2,0,0]
    thetalist0 = np.array(thetalist0,float)
    eomg = 0.01
    ev = 0.001

    thetalist = IKinBody(Blist,M,X,thetalist0,eomg,ev)

    print thetalist

if __name__ == '__main__':
    main()
