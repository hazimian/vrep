import numpy as np

pi = np.pi

def sin(x):
    return np.sin(x)
def cos(x):
    return np.cos(x)

def JacoFK(q):




    D1 = .2755
    D2 = .41
    D3 = .2073
    D4 = .0743
    D5 = D4
    D6 = .1687
    e2 = .0098
    aa = .48#30*pi/180
    ca = cos(aa)
    sa = sin(aa)
    c2a = cos(2 * aa);
    s2a = sin(2 * aa);
    d4b = D3 +( (sa / s2a) * D4);
    d5b = (sa / s2a) * D4 + (sa / s2a) * D5;
    d6b = (sa / s2a) * D5 + D6;



    # q1=pi-q[0,0]
    # q2=-q[1,0]+1.5*pi
    # q3=-q[2,0]+pi/2
    # q4=-(pi-q[3,0])
    # q5=-(pi-q[4,0])
    # q6=260/180*pi-q[5,0]
    q1 = - q[0, 0]
    q2 = q[1, 0] - .5 * pi
    q3 = q[2, 0] + pi / 2
    q4 = q[3, 0]
    q5 = q[4, 0]-pi
    q6 = q[5, 0]+pi/2
    A=np.diag([-1,1,1,1,1,1])
    # A[1,1]=1




    p=np.zeros([3,1])
    R=np.zeros([3,3])
    p[0,0]=e2 * sin(q1) + D2 * cos(q1) * cos(q2) + d4b * sin(q2 - q3) * cos(q1) + d6b * sin(q2 - q3) * cos(2 * aa) * cos(2 * aa) * cos(
        q1) - d5b * sin(2 * aa) * cos(q4) * sin(q1) + d5b * sin(q2 - q3) * cos(2 * aa) * cos(q1) - d6b * sin(
        q2 - q3) * sin(2 * aa) *sin(2 * aa) * cos(q1) * cos(q5) + d6b * sin(2 * aa) * sin(q1) * sin(q4) * sin(q5) - d5b * cos(
        q2 - q3) * sin(2 * aa) *sin(2 * aa)* cos(q1) * sin(q4) - d6b * cos(2 * aa) * sin(2 * aa) * cos(q4) * sin(q1) - d6b * cos(
        2 * aa) * sin(2 * aa) * cos(q4) * cos(q5) * sin(q1) - d6b * cos(q2 - q3) * cos(2 * aa) * sin(2 * aa) * cos(
        q1) * sin(q4) - d6b * sin(2 * aa) * cos(q1) * cos(q2) * cos(q3) * cos(q4) * sin(q5) - d6b * sin(2 * aa) * cos(
        q1) * cos(q4) * sin(q2) * sin(q3) * sin(q5) - d6b * cos(q2 - q3) * cos(2 * aa) * sin(2 * aa) * cos(q1) * cos(
        q5) * sin(q4)
    p[1,0]=D2 * cos(q2) * sin(q1) - e2 * cos(q1) + d4b * sin(q2 - q3) * sin(q1) + d6b * sin(q2 - q3) * cos(2 * aa) * cos(2 * aa)* sin(
        q1) + d5b * sin(2 * aa) * cos(q1) * cos(q4) + d5b * sin(q2 - q3) * cos(2 * aa) * sin(q1) - d6b * sin(
        q2 - q3) * sin(2 * aa) *sin(2 * aa) * cos(q5) * sin(q1) - d6b * sin(2 * aa) * cos(q1) * sin(q4) * sin(q5) + d6b * cos(
        2 * aa) * sin(2 * aa) * cos(q1) * cos(q4) - d5b * cos(q2 - q3) * sin(2 * aa) * sin(q1) * sin(q4) + d6b * cos(
        2 * aa) * sin(2 * aa) * cos(q1) * cos(q4) * cos(q5) - d6b * cos(q2 - q3) * cos(2 * aa) * sin(2 * aa) * sin(
        q1) * sin(q4) - d6b * sin(2 * aa) * cos(q2) * cos(q3) * cos(q4) * sin(q1) * sin(q5) - d6b * sin(2 * aa) * cos(
        q4) * sin(q1) * sin(q2) * sin(q3) * sin(q5) - d6b * cos(q2 - q3) * cos(2 * aa) * sin(2 * aa) * cos(q5) * sin(
        q1) * sin(q4)
    p[2,0]=D1 + D2 * sin(q2) - d4b * cos(q2 - q3) - d6b * cos(q2 - q3) * cos(2 * aa) *cos(2 * aa)- d5b * cos(q2 - q3) * cos(
        2 * aa) + d6b * cos(q2 - q3) * sin(2 * aa) * sin(2 * aa) * cos(q5) - d5b * sin(q2 - q3) * sin(2 * aa) * sin(
        q4) - d6b * sin(q2 - q3) * cos(2 * aa) * sin(2 * aa) * sin(q4) - d6b * sin(q2 - q3) * sin(2 * aa) * cos(
        q4) * sin(q5) - d6b * sin(q2 - q3) * cos(2 * aa) * sin(2 * aa) * cos(q5) * sin(q4)


    R[0,0]=cos(q6) * (cos(q5) * (
                cos(q1) * cos(q2) * cos(q3) * cos(q4) - sin(q1) * sin(q4) + cos(q1) * cos(q4) * sin(q2) * sin(
            q3)) - sin(q5) * (
                            cos(2 * aa) * cos(q4) * sin(q1) + sin(q2 - q3) * sin(2 * aa) * cos(q1) + cos(q2 - q3) * cos(
                        2 * aa) * cos(q1) * sin(q4))) - sin(q6) * (cos(2 * aa) * sin(q5) * (
                cos(q1) * cos(q2) * cos(q3) * cos(q4) - sin(q1) * sin(q4) + cos(q1) * cos(q4) * sin(q2) * sin(
            q3)) - sin(2 * aa) * (sin(2 * aa) * cos(q4) * sin(q1) - sin(q2 - q3) * cos(2 * aa) * cos(q1) + cos(
        q2 - q3) * sin(2 * aa) * cos(q1) * sin(q4)) + cos(2 * aa) * cos(q5) * (cos(2 * aa) * cos(q4) * sin(q1) + sin(
        q2 - q3) * sin(2 * aa) * cos(q1) + cos(q2 - q3) * cos(2 * aa) * cos(q1) * sin(q4)))
    R[0,1]=sin(q6) * (cos(q5) * (
                cos(q1) * cos(q2) * cos(q3) * cos(q4) - sin(q1) * sin(q4) + cos(q1) * cos(q4) * sin(q2) * sin(
            q3)) - sin(q5) * (cos(2 * aa) * cos(q4) * sin(q1) + sin(q2 - q3) * sin(2 * aa) * cos(q1) + cos(
        q2 - q3) * cos(2 * aa) * cos(q1) * sin(q4))) + cos(q6) * (cos(2 * aa) * sin(q5) * (
                cos(q1) * cos(q2) * cos(q3) * cos(q4) - sin(q1) * sin(q4) + cos(q1) * cos(q4) * sin(q2) * sin(
            q3)) - sin(2 * aa) * (sin(2 * aa) * cos(q4) * sin(q1) - sin(q2 - q3) * cos(2 * aa) * cos(q1) + cos(
        q2 - q3) * sin(2 * aa) * cos(q1) * sin(q4)) + cos(2 * aa) * cos(q5) * (cos(2 * aa) * cos(q4) * sin(q1) + sin(
        q2 - q3) * sin(2 * aa) * cos(q1) + cos(q2 - q3) * cos(2 * aa) * cos(q1) * sin(q4)))
    R[0,2]=- cos(2 * aa) * (
                 sin(2 * aa) * cos(q4) * sin(q1) - sin(q2 - q3) * cos(2 * aa) * cos(q1) + cos(q2 - q3) * sin(
             2 * aa) * cos(q1) * sin(q4)) - sin(2 * aa) * sin(q5) * (
                 cos(q1) * cos(q2) * cos(q3) * cos(q4) - sin(q1) * sin(q4) + cos(q1) * cos(q4) * sin(q2) * sin(
             q3)) - sin(2 * aa) * cos(q5) * (
                 cos(2 * aa) * cos(q4) * sin(q1) + sin(q2 - q3) * sin(2 * aa) * cos(q1) + cos(q2 - q3) * cos(
             2 * aa) * cos(q1) * sin(q4))
    R[1,0]=cos(q6) * (cos(q5) * (
                cos(q1) * sin(q4) + cos(q2) * cos(q3) * cos(q4) * sin(q1) + cos(q4) * sin(q1) * sin(q2) * sin(
            q3)) - sin(q5) * (
                            sin(q2 - q3) * sin(2 * aa) * sin(q1) - cos(2 * aa) * cos(q1) * cos(q4) + cos(q2 - q3) * cos(
                        2 * aa) * sin(q1) * sin(q4))) - sin(q6) * (sin(2 * aa) * (
                sin(2 * aa) * cos(q1) * cos(q4) + sin(q2 - q3) * cos(2 * aa) * sin(q1) - cos(q2 - q3) * sin(
            2 * aa) * sin(q1) * sin(q4)) + cos(2 * aa) * sin(q5) * (cos(q1) * sin(q4) + cos(q2) * cos(q3) * cos(
        q4) * sin(q1) + cos(q4) * sin(q1) * sin(q2) * sin(q3)) + cos(2 * aa) * cos(q5) * (
                                                                               sin(q2 - q3) * sin(2 * aa) * sin(
                                                                           q1) - cos(2 * aa) * cos(q1) * cos(q4) + cos(
                                                                           q2 - q3) * cos(2 * aa) * sin(q1) * sin(q4)))

    R[1,1]=sin(q6) * (cos(q5) * (
                 cos(q1) * sin(q4) + cos(q2) * cos(q3) * cos(q4) * sin(q1) + cos(q4) * sin(q1) * sin(q2) * sin(
             q3)) - sin(q5) * (
                            sin(q2 - q3) * sin(2 * aa) * sin(q1) - cos(2 * aa) * cos(q1) * cos(q4) + cos(q2 - q3) * cos(
                        2 * aa) * sin(q1) * sin(q4))) + cos(q6) * (sin(2 * aa) * (
                 sin(2 * aa) * cos(q1) * cos(q4) + sin(q2 - q3) * cos(2 * aa) * sin(q1) - cos(q2 - q3) * sin(
             2 * aa) * sin(q1) * sin(q4)) + cos(2 * aa) * sin(q5) * (cos(q1) * sin(q4) + cos(q2) * cos(q3) * cos(
         q4) * sin(q1) + cos(q4) * sin(q1) * sin(q2) * sin(q3)) + cos(2 * aa) * cos(q5) * (
                                                                               sin(q2 - q3) * sin(2 * aa) * sin(
                                                                           q1) - cos(2 * aa) * cos(q1) * cos(q4) + cos(
                                                                           q2 - q3) * cos(2 * aa) * sin(q1) * sin(q4)))

    R[1,2]=cos(2 * aa) * (sin(2 * aa) * cos(q1) * cos(q4) + sin(q2 - q3) * cos(2 * aa) * sin(q1) - cos(q2 - q3) * sin(
         2 * aa) * sin(q1) * sin(q4)) - sin(2 * aa) * sin(q5) * (
                 cos(q1) * sin(q4) + cos(q2) * cos(q3) * cos(q4) * sin(q1) + cos(q4) * sin(q1) * sin(q2) * sin(
             q3)) - sin(2 * aa) * cos(q5) * (
                 sin(q2 - q3) * sin(2 * aa) * sin(q1) - cos(2 * aa) * cos(q1) * cos(q4) + cos(q2 - q3) * cos(
             2 * aa) * sin(q1) * sin(q4))

    R[2,0]=cos(q6) * (sin(q2 - q3) * cos(q4) * cos(q5) + cos(q2 - q3) * sin(2 * aa) * sin(q5) - sin(q2 - q3) * cos(
        2 * aa) * sin(q4) * sin(q5)) + sin(q6) * (
                 sin(2 * aa) * (cos(q2 - q3) * cos(2 * aa) + sin(q2 - q3) * sin(2 * aa) * sin(q4)) + cos(2 * aa) * cos(
             q5) * (cos(q2 - q3) * sin(2 * aa) - sin(q2 - q3) * cos(2 * aa) * sin(q4)) - sin(q2 - q3) * cos(
             2 * aa) * cos(q4) * sin(q5))
    R[2,1]=sin(q6) * (
                 sin(q2 - q3) * cos(q4) * cos(q5) + cos(q2 - q3) * sin(2 * aa) * sin(q5) - sin(q2 - q3) * cos(
             2 * aa) * sin(q4) * sin(q5)) - cos(q6) * (
                 sin(2 * aa) * (cos(q2 - q3) * cos(2 * aa) + sin(q2 - q3) * sin(2 * aa) * sin(q4)) + cos(2 * aa) * cos(
             q5) * (cos(q2 - q3) * sin(2 * aa) - sin(q2 - q3) * cos(2 * aa) * sin(q4)) - sin(q2 - q3) * cos(
             2 * aa) * cos(q4) * sin(q5))

    R[2,2]=sin(2 * aa) * cos(q5) * (cos(q2 - q3) * sin(2 * aa) - sin(q2 - q3) * cos(2 * aa) * sin(q4)) - cos(2 * aa) * (
                 cos(q2 - q3) * cos(2 * aa) + sin(q2 - q3) * sin(2 * aa) * sin(q4)) - sin(q2 - q3) * sin(2 * aa) * cos(
         q4) * sin(q5)



    Jp=np.zeros([3,6]);

    Jp[0,0]=e2 * cos(q1) - D2 * cos(q2) * sin(q1) - d4b * sin(q2 - q3) * sin(q1) - d6b * sin(q2 - q3) * cos(2 * aa) * cos(2 * aa) * sin(
        q1) - d5b * sin(2 * aa) * cos(q1) * cos(q4) - d5b * sin(q2 - q3) * cos(2 * aa) * sin(q1) + d6b * sin(
        q2 - q3) * sin(2 * aa) * sin(2 * aa) * cos(q5) * sin(q1) + d6b * sin(2 * aa) * cos(q1) * sin(q4) * sin(q5) - d6b * cos(
        2 * aa) * sin(2 * aa) * cos(q1) * cos(q4) + d5b * cos(q2 - q3) * sin(2 * aa) * sin(q1) * sin(q4) - d6b * cos(
        2 * aa) * sin(2 * aa) * cos(q1) * cos(q4) * cos(q5) + d6b * cos(q2 - q3) * cos(2 * aa) * sin(2 * aa) * sin(
        q1) * sin(q4) + d6b * sin(2 * aa) * cos(q2) * cos(q3) * cos(q4) * sin(q1) * sin(q5) + d6b * sin(2 * aa) * cos(
        q4) * sin(q1) * sin(q2) * sin(q3) * sin(q5) + d6b * cos(q2 - q3) * cos(2 * aa) * sin(2 * aa) * cos(q5) * sin(
        q1) * sin(q4)
    Jp[0,1]=cos(q1) * (
                 d4b * cos(q2 - q3) - D2 * sin(q2) + d6b * cos(q2 - q3) * cos(2 * aa) * cos(2 * aa) + d5b * cos(q2 - q3) * cos(
             2 * aa) - d6b * cos(q2 - q3) * sin(2 * aa)  * sin(2 * aa) * cos(q5) + d5b * sin(q2 - q3) * sin(2 * aa) * sin(
             q4) + d6b * sin(q2 - q3) * cos(2 * aa) * sin(2 * aa) * sin(q4) - d6b * sin(2 * aa) * cos(q2) * cos(
             q4) * sin(q3) * sin(q5) + d6b * sin(2 * aa) * cos(q3) * cos(q4) * sin(q2) * sin(q5) + d6b * sin(
             q2 - q3) * cos(2 * aa) * sin(2 * aa) * cos(q5) * sin(q4))
    Jp[0,2]=-cos(q1) * (
                 d4b * cos(q2 - q3) + d6b * cos(q2 - q3) * cos(2 * aa) * cos(2 * aa) + d5b * cos(q2 - q3) * cos(
             2 * aa) - d6b * cos(q2 - q3) * sin(2 * aa)  * sin(2 * aa) * cos(q5) + d5b * sin(q2 - q3) * sin(2 * aa) * sin(
             q4) + d6b * sin(q2 - q3) * cos(2 * aa) * sin(2 * aa) * sin(q4) - d6b * sin(2 * aa) * cos(q2) * cos(
             q4) * sin(q3) * sin(q5) + d6b * sin(2 * aa) * cos(q3) * cos(q4) * sin(q2) * sin(q5) + d6b * sin(
             q2 - q3) * cos(2 * aa) * sin(2 * aa) * cos(q5) * sin(q4))
    Jp[0,3]=sin(2 * aa) * (
                 d5b * sin(q1) * sin(q4) + d6b * cos(q4) * sin(q1) * sin(q5) - d5b * cos(q2 - q3) * cos(q1) * cos(
             q4) + d6b * cos(2 * aa) * sin(q1) * sin(q4) + d6b * cos(2 * aa) * cos(q5) * sin(q1) * sin(q4) - d6b * cos(
             q2 - q3) * cos(2 * aa) * cos(q1) * cos(q4) + d6b * cos(q1) * cos(q2) * cos(q3) * sin(q4) * sin(
             q5) + d6b * cos(q1) * sin(q2) * sin(q3) * sin(q4) * sin(q5) - d6b * cos(q2 - q3) * cos(2 * aa) * cos(
             q1) * cos(q4) * cos(q5))
    Jp[0,4]=d6b * sin(2 * aa) * (
                 cos(q5) * sin(q1) * sin(q4) + cos(2 * aa) * cos(q4) * sin(q1) * sin(q5) + sin(q2 - q3) * sin(
             2 * aa) * cos(q1) * sin(q5) + cos(q2 - q3) * cos(2 * aa) * cos(q1) * sin(q4) * sin(q5) - cos(q1) * cos(
             q2) * cos(q3) * cos(q4) * cos(q5) - cos(q1) * cos(q4) * cos(q5) * sin(q2) * sin(q3))
    Jp[0,5]=0

    Jp[1,0]=e2 * sin(q1) + D2 * cos(q1) * cos(q2) + d4b * sin(q2 - q3) * cos(q1) + d6b * sin(q2 - q3) * cos(2 * aa) * cos(2 * aa) * cos(
        q1) - d5b * sin(2 * aa) * cos(q4) * sin(q1) + d5b * sin(q2 - q3) * cos(2 * aa) * cos(q1) - d6b * sin(
        q2 - q3) * sin(2 * aa)  * sin(2 * aa) * cos(q1) * cos(q5) + d6b * sin(2 * aa) * sin(q1) * sin(q4) * sin(q5) - d5b * cos(
        q2 - q3) * sin(2 * aa) * cos(q1) * sin(q4) - d6b * cos(2 * aa) * sin(2 * aa) * cos(q4) * sin(q1) - d6b * cos(
        2 * aa) * sin(2 * aa) * cos(q4) * cos(q5) * sin(q1) - d6b * cos(q2 - q3) * cos(2 * aa) * sin(2 * aa) * cos(
        q1) * sin(q4) - d6b * sin(2 * aa) * cos(q1) * cos(q2) * cos(q3) * cos(q4) * sin(q5) - d6b * sin(2 * aa) * cos(
        q1) * cos(q4) * sin(q2) * sin(q3) * sin(q5) - d6b * cos(q2 - q3) * cos(2 * aa) * sin(2 * aa) * cos(q1) * cos(
        q5) * sin(q4)
    Jp[1,1]=sin(q1) * (
                 d4b * cos(q2 - q3) - D2 * sin(q2) + d6b * cos(q2 - q3) * cos(2 * aa) * cos(2 * aa) + d5b * cos(q2 - q3) * cos(
             2 * aa) - d6b * cos(q2 - q3) * sin(2 * aa)  * sin(2 * aa) * cos(q5) + d5b * sin(q2 - q3) * sin(2 * aa) * sin(
             q4) + d6b * sin(q2 - q3) * cos(2 * aa) * sin(2 * aa) * sin(q4) - d6b * sin(2 * aa) * cos(q2) * cos(
             q4) * sin(q3) * sin(q5) + d6b * sin(2 * aa) * cos(q3) * cos(q4) * sin(q2) * sin(q5) + d6b * sin(
             q2 - q3) * cos(2 * aa) * sin(2 * aa) * cos(q5) * sin(q4))
    Jp[1,2]=-sin(q1) * (
                 d4b * cos(q2 - q3) + d6b * cos(q2 - q3) * cos(2 * aa) * cos(2 * aa) + d5b * cos(q2 - q3) * cos(
             2 * aa) - d6b * cos(q2 - q3) * sin(2 * aa)  * sin(2 * aa) * cos(q5) + d5b * sin(q2 - q3) * sin(2 * aa) * sin(
             q4) + d6b * sin(q2 - q3) * cos(2 * aa) * sin(2 * aa) * sin(q4) - d6b * sin(2 * aa) * cos(q2) * cos(
             q4) * sin(q3) * sin(q5) + d6b * sin(2 * aa) * cos(q3) * cos(q4) * sin(q2) * sin(q5) + d6b * sin(
             q2 - q3) * cos(2 * aa) * sin(2 * aa) * cos(q5) * sin(q4))
    Jp[1,3]=-sin(2 * aa) * (
                 d5b * cos(q1) * sin(q4) + d6b * cos(q1) * cos(q4) * sin(q5) + d5b * cos(q2 - q3) * cos(q4) * sin(
             q1) + d6b * cos(2 * aa) * cos(q1) * sin(q4) + d6b * cos(2 * aa) * cos(q1) * cos(q5) * sin(q4) + d6b * cos(
             q2 - q3) * cos(2 * aa) * cos(q4) * sin(q1) - d6b * cos(q2) * cos(q3) * sin(q1) * sin(q4) * sin(
             q5) - d6b * sin(q1) * sin(q2) * sin(q3) * sin(q4) * sin(q5) + d6b * cos(q2 - q3) * cos(2 * aa) * cos(
             q4) * cos(q5) * sin(q1))
    Jp[1,4]=-d6b * sin(2 * aa) * (
                 cos(q1) * cos(q5) * sin(q4) + cos(2 * aa) * cos(q1) * cos(q4) * sin(q5) - sin(q2 - q3) * sin(
             2 * aa) * sin(q1) * sin(q5) - cos(q2 - q3) * cos(2 * aa) * sin(q1) * sin(q4) * sin(q5) + cos(q2) * cos(
             q3) * cos(q4) * cos(q5) * sin(q1) + cos(q4) * cos(q5) * sin(q1) * sin(q2) * sin(q3))
    Jp[1,5]=0

    Jp[2,0]=0
    Jp[2,1]=D2 * cos(q2) + d4b * sin(q2 - q3) + d6b * sin(q2 - q3) * cos(2 * aa) * cos(2 * aa) + d5b * sin(q2 - q3) * cos(
        2 * aa) - d6b * sin(q2 - q3) * sin(2 * aa)  * sin(2 * aa) * cos(q5) - d5b * cos(q2 - q3) * sin(2 * aa) * sin(
        q4) - d6b * cos(q2 - q3) * cos(2 * aa) * sin(2 * aa) * sin(q4) - d6b * cos(q2 - q3) * sin(2 * aa) * cos(
        q4) * sin(q5) - d6b * cos(q2 - q3) * cos(2 * aa) * sin(2 * aa) * cos(q5) * sin(q4)

    Jp[2,2]=d6b * sin(q2 - q3) * sin(2 * aa)  * sin(2 * aa) * cos(q5) - d6b * sin(q2 - q3) * cos(2 * aa) * cos(2 * aa) - d5b * sin(q2 - q3) * cos(
         2 * aa) - d4b * sin(q2 - q3) + d5b * cos(q2 - q3) * sin(2 * aa) * sin(q4) + d6b * cos(q2 - q3) * cos(
         2 * aa) * sin(2 * aa) * sin(q4) + d6b * cos(q2 - q3) * sin(2 * aa) * cos(q4) * sin(q5) + d6b * cos(
         q2 - q3) * cos(2 * aa) * sin(2 * aa) * cos(q5) * sin(q4)
    Jp[2,3]=-sin(q2 - q3) * sin(2 * aa) * (
                 d5b * cos(q4) - d6b * sin(q4) * sin(q5) + d6b * cos(2 * aa) * cos(q4) + d6b * cos(2 * aa) * cos(
             q4) * cos(q5))
    Jp[2,4]=-d6b * sin(2 * aa) * (
                 sin(q2 - q3) * cos(q4) * cos(q5) + cos(q2 - q3) * sin(2 * aa) * sin(q5) - sin(q2 - q3) * cos(
             2 * aa) * sin(q4) * sin(q5))
    Jp[2,5]=0



    Jo=np.zeros([3,6])
    Jo[0, 0] = 0

    Jo[0, 1] = sin(q1)

    Jo[0, 2] = -sin(q1)

    Jo[0, 3] = -sin(q2 - q3) * cos(q1)

    Jo[0, 4] = sin(2 * aa) * cos(q4) * sin(q1) + cos(2 * aa) * cos(q1) * cos(q2) * sin(q3) - cos(2 * aa) * cos(
        q1) * cos(q3) * sin(q2) + sin(2 * aa) * cos(q1) * sin(q2) * sin(q3) * sin(q4) + sin(2 * aa) * cos(q1) * cos(
        q2) * cos(q3) * sin(q4)

    Jo[0, 5] = cos(2 * aa) ** 2 * cos(q1) * cos(q2) * sin(q3) - sin(2 * aa) * sin(q1) * sin(q4) * sin(q5) - cos(
        2 * aa) ** 2 * cos(q1) * cos(q3) * sin(q2) + cos(2 * aa) * sin(2 * aa) * cos(q4) * sin(q1) - cos(q1) * cos(
        q2) * cos(q5) * sin(q3) + cos(q1) * cos(q3) * cos(q5) * sin(q2) + cos(2 * aa) ** 2 * cos(q1) * cos(q2) * cos(
        q5) * sin(q3) - cos(2 * aa) ** 2 * cos(q1) * cos(q3) * cos(q5) * sin(q2) + cos(2 * aa) * sin(2 * aa) * cos(
        q4) * cos(q5) * sin(q1) + cos(2 * aa) * sin(2 * aa) * cos(q1) * cos(q2) * cos(q3) * sin(q4) + cos(2 * aa) * sin(
        2 * aa) * cos(q1) * sin(q2) * sin(q3) * sin(q4) + sin(2 * aa) * cos(q1) * cos(q2) * cos(q3) * cos(q4) * sin(
        q5) + sin(2 * aa) * cos(q1) * cos(q4) * sin(q2) * sin(q3) * sin(q5) + cos(2 * aa) * sin(2 * aa) * cos(q1) * cos(
        q5) * sin(q2) * sin(q3) * sin(q4) + cos(2 * aa) * sin(2 * aa) * cos(q1) * cos(q2) * cos(q3) * cos(q5) * sin(q4)

    Jo[1, 0] = 0

    Jo[1, 1] = -cos(q1)

    Jo[1, 2] = cos(q1)

    Jo[1, 3] = -sin(q2 - q3) * sin(q1)

    Jo[1, 4] = cos(2 * aa) * cos(q2) * sin(q1) * sin(q3) - sin(2 * aa) * cos(q1) * cos(q4) - cos(2 * aa) * cos(
        q3) * sin(q1) * sin(q2) + sin(2 * aa) * cos(q2) * cos(q3) * sin(q1) * sin(q4) + sin(2 * aa) * sin(q1) * sin(
        q2) * sin(q3) * sin(q4)

    Jo[1, 5] = sin(2 * aa) * cos(q1) * sin(q4) * sin(q5) - cos(2 * aa) * sin(2 * aa) * cos(q1) * cos(q4) + cos(
        2 * aa) ** 2 * cos(q2) * sin(q1) * sin(q3) - cos(2 * aa) ** 2 * cos(q3) * sin(q1) * sin(q2) - cos(q2) * cos(
        q5) * sin(q1) * sin(q3) + cos(q3) * cos(q5) * sin(q1) * sin(q2) - cos(2 * aa) * sin(2 * aa) * cos(q1) * cos(
        q4) * cos(q5) + cos(2 * aa) ** 2 * cos(q2) * cos(q5) * sin(q1) * sin(q3) - cos(2 * aa) ** 2 * cos(q3) * cos(
        q5) * sin(q1) * sin(q2) + cos(2 * aa) * sin(2 * aa) * cos(q2) * cos(q3) * sin(q1) * sin(q4) + cos(2 * aa) * sin(
        2 * aa) * sin(q1) * sin(q2) * sin(q3) * sin(q4) + sin(2 * aa) * cos(q2) * cos(q3) * cos(q4) * sin(q1) * sin(
        q5) + sin(2 * aa) * cos(q4) * sin(q1) * sin(q2) * sin(q3) * sin(q5) + cos(2 * aa) * sin(2 * aa) * cos(q5) * sin(
        q1) * sin(q2) * sin(q3) * sin(q4) + cos(2 * aa) * sin(2 * aa) * cos(q2) * cos(q3) * cos(q5) * sin(q1) * sin(q4)

    Jo[2, 0] = 1

    Jo[2, 1] = 0

    Jo[2, 2] = 0

    Jo[2, 3] = cos(q2 - q3)

    Jo[2, 4] = cos(2 * aa) * cos(q2) * cos(q3) + cos(2 * aa) * sin(q2) * sin(q3) - sin(2 * aa) * cos(q2) * sin(
        q3) * sin(q4) + sin(2 * aa) * cos(q3) * sin(q2) * sin(q4)

    Jo[2, 5] = cos(2 * aa) ** 2 * cos(q2) * cos(q3) + cos(2 * aa) ** 2 * sin(q2) * sin(q3) - cos(q2) * cos(q3) * cos(
        q5) - cos(q5) * sin(q2) * sin(q3) + cos(2 * aa) ** 2 * cos(q2) * cos(q3) * cos(q5) + cos(2 * aa) ** 2 * cos(
        q5) * sin(q2) * sin(q3) - sin(2 * aa) * cos(q2) * cos(q4) * sin(q3) * sin(q5) + sin(2 * aa) * cos(q3) * cos(
        q4) * sin(q2) * sin(q5) - cos(2 * aa) * sin(2 * aa) * cos(q2) * sin(q3) * sin(q4) + cos(2 * aa) * sin(
        2 * aa) * cos(q3) * sin(q2) * sin(q4) - cos(2 * aa) * sin(2 * aa) * cos(q2) * cos(q5) * sin(q3) * sin(q4) + cos(
        2 * aa) * sin(2 * aa) * cos(q3) * cos(q5) * sin(q2) * sin(q4)

    Jp=np.matmul(Jp,A)
    Jo=np.matmul(Jo,A)
    return p, R, Jp,Jo


def JacoIK(pd,q0,kp,ko):
    q=q0
    dt=.01
    Kp=kp*np.diag([1,1,1])
    Ko=ko*np.diag([1,1,1])
    for i in range(1):
        p, R, Jp,Jo = JacoFK(q)
        J=np.concatenate((Jp,Jo))
        Jinv=np.linalg.pinv(J)
        ep=np.reshape(pd,[-1,1])-p
        eo=np.reshape(.5*np.cross(np.ndarray.flatten(R[:,2]),np.array([0,0,-1])),[-1,1])
        L=-.5*np.matmul(np.array([[0,1,0],[-1,0,0],[0,0,0]]),np.array([[0,-R[2,2],R[1,2]],[R[2,2],0,-R[0,2]],[-R[1,2],R[0,2],0]]))
        A=np.matmul(Kp,ep)
        B=np.matmul(np.linalg.pinv(L),np.matmul(Ko,eo))
        u=np.concatenate((A,B),axis=0)
        qdot=np.matmul(np.diag([1,1,1,1,1,1]),np.matmul(Jinv,u))
        if dt*np.linalg.norm(qdot)>.1:#clamp max joint angular dispalcement
            #q = q + qdot /np.linalg.norm(qdot)
            qdot=.1/dt*(qdot /np.linalg.norm(qdot))
        q=q+dt*qdot#/np.linalg.norm(qdot)

    return q,p,np.linalg.norm(ep),np.linalg.norm(eo),qdot
