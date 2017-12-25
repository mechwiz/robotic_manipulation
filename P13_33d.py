import numpy as np
from modern_robotics import *
import csv
import matplotlib.pyplot as mp

np.set_printoptions(precision=3,suppress=True)

def t_sb(phi,x,y):
  t = [[cos(phi),-sin(phi),0.0,x],[sin(phi),cos(phi),0,y],[0.0,0.0,1.0, 0.0963],[0,0,0,1]]
  return t

def X_d(s):
  t = [[sin(s*pi/2),0,cos(s*pi/2),s],[0,1,0,0],[-cos(s*pi/2),0,sin(s*pi/2),0.491],[0,0,0,1]]
  return t

def X_dp(s,sp):
  t = [[pi/2*cos(s*pi/2)*sp,0,-pi/2*sin(s*pi/2)*sp,sp],[0,0,0,0],[pi/2*sin(s*pi/2)*sp,0,pi/2*cos(s*pi/2)*sp,0],[0,0,0,0]]
  return t

def spr(t):
  return 6.0/25.0*t-6.0/125.0*(t**2)

def dq(phi):
  q = [[1,0,0],[0,cos(phi),-sin(phi)],[0,sin(phi),cos(phi)]]
  return q

def main():
  Tsb = t_sb(0,0,0)
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


  thetalist = np.array([-pi/8,-0.5,0.5,0,-pi/4,pi/4,-pi/2,0],float)
  # thetalist = np.array([0,-0.526,0,0,-pi/4,pi/4,-pi/2,0],float)


  r = 0.0475
  l=0.47/2
  w=0.3/2
  F6 = r/4*np.array([[0,0,0,0],[0,0,0,0],[-1/(l+w),1/(l+w),1/(l+w),-1/(l+w)],[1,1,1,1],[-1,1,-1,1],[0,0,0,0]],float)

  F = r/4*np.array([[-1/(l+w),1/(l+w),1/(l+w),-1/(l+w)],[1,1,1,1],[-1,1,-1,1]],float)

  Tf = 5
  s=[]
  sp=[]
  t = 0
  Xerr = []
  thetamaster = []
  thetareorder = np.zeros(12)
  thetareorder[0:2]=thetalist[1:3]
  thetareorder[2] = thetalist[0]
  thetareorder[3:8] = thetalist[3:8]
  thetamaster.append(thetareorder.tolist())
  Kp = 5
  Ki = 13
  ff = 1
  toterr = np.zeros(6)
  Xse = []

  for i in range(500):
      s.append(CubicTimeScaling(Tf, t))
      sp.append(spr(t))
      t+=0.01

  for i in range(len(s)):
    T0e = FKinBody(M0e, Blist[:,3:8], thetalist[3:8])
    Tsb = t_sb(thetalist[0],thetalist[1],thetalist[2])
    X = np.array(np.dot(np.dot(Tsb,Tb0),T0e),float)

    # X = FKinBody(M,Blist,thetalist)
    Xd = X_d(s[i])
    Xdp = X_dp(s[i],sp[i])
    Vd = se3ToVec(np.dot(TransInv(Xd),Xdp))
    Xe = se3ToVec(MatrixLog6(np.dot(TransInv(X),Xd)))
    Xse.append(Xe)
    toterr = toterr + np.dot(np.array(Xe),0.01)

    V = ff*np.dot(Adjoint(np.dot(TransInv(X),Xd)),Vd.T)+ Kp*np.array(Xe,float).T+Ki*toterr.T

    T0e_inv = TransInv(FKinBody(M0e,Blist[:,3:8],thetalist[3:8]))
    Tb0_inv = TransInv(Tb0)

    Jarm = JacobianBody(Blist[:,3:8],thetalist[3:8])
    Jbase = np.dot(Adjoint(np.dot(T0e_inv,Tb0_inv)),F6)

    Je = np.zeros((6,9),dtype=np.float)
    Je[:,0:4] = Jbase
    Je[:,4:9] = Jarm

    Ve = np.dot(np.linalg.pinv(Je),np.array(V,float))

    Vwh = np.dot(np.dot(F,np.array(Ve[0:4],float).T),0.01)

    wbz,vbx,vby = Vwh[0],Vwh[1],Vwh[2]

    if wbz == 0:
      qb = np.array([[0],[vbx],[vby]],float)
    else:
      qb = np.array([[wbz],[(vbx*sin(wbz)+vby*(cos(wbz)-1))/wbz],[(vby*sin(wbz)+vbx*(1-cos(wbz)))/wbz]],float)

    q2 = thetalist[0:3]+np.dot(np.array(dq(thetalist[0]),float),np.array(qb,float)).T

    thetaarm = thetalist[3:8] + np.dot(np.array(Ve[4:9],float),0.01).T
    thetalist[0:3] = q2[0]
    thetalist[3:8] = thetaarm

    uv = thetareorder[8:12]+np.dot(np.array(Ve[0:4],float),0.01).T

    thetareorder[0:2]=thetalist[1:3]
    thetareorder[2] = thetalist[0]
    thetareorder[3:8] = thetalist[3:8]
    thetareorder[8:12] = uv
    thetamaster.append(thetareorder.tolist())

  xtime = np.arange(0,5,0.01)

  Xse = np.copy(Xse)
  mp.plot(xtime,Xse[:,0],label='wx')
  mp.plot(xtime,Xse[:,1],label='wy')
  mp.plot(xtime,Xse[:,2],label='wz')
  mp.plot(xtime,Xse[:,3],label='vx')
  mp.plot(xtime,Xse[:,4],label='vy')
  mp.plot(xtime,Xse[:,5],label='vz')
  mp.title('Error Plot: Kp = %.2f and Ki = %.2f'% (Kp,Ki))
  mp.xlabel('Time (s)')
  mp.legend(bbox_to_anchor=(1,0.5))

  myFile = open('kukabot_kpki1.csv','w')
  with myFile:
    writer = csv.writer(myFile)
    writer.writerows(thetamaster)

  mp.show()

if __name__ == '__main__':
    main()
