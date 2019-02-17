from __future__ import print_function
'''
Coordinate frame library
'''
import numpy
from numpy import linalg, sin, cos, dot, cross, pi, array, arctan2, sqrt, abs
import ikr
from ikr import fminbound, format, bounds
import time

DEG = pi/180
I3 = numpy.eye(3)
I6 = numpy.eye(6)
C_s = (1.) ** 2 # mm^2 ### spatial var
C_a = (1. * DEG) ** 2  ### angular var
W_s = 1. / C_s         ### spatial weight
W_a = 1. / C_a         ### angular weight
W = numpy.diag([W_s] * 3 +  [W_a] * 3) ### weighting matrix

def R_roll(theta):
    return array([[1, 0, 0],
                 [0, cos(theta), -sin(theta)],
                 [0, sin(theta),  cos(theta)]])
def R_pitch(theta):
    return array([[cos(theta), 0, -sin(theta)],
                  [0, 1., 0],
                  [sin(theta), 0, cos(theta)]])
def R_yaw(theta):
    return array([[cos(theta), -sin(theta), 0],
                  [sin(theta),  cos(theta), 0],
                  [0, 0, 1]])

def Null(theta):
    return I3

class CoordFrame:
    def __init__(self, name, ref, offset, R, angle=0):
        '''
        represent a new coord from offset from ref_point by
        rotation R
        ref -- CoordFrame (or none for standard basis reference)
        offset -- 3-vec
        R -- function (R_roll, R_pitch or R_yaw)
        '''
        self.name = name
        self.ref = ref
        self.offset = offset
        self.R = R
        self.angle = angle

    def getBasis(self):
        out = self.R(self.angle)
        ref_basis = self.ref.getBasis()
        out = dot(ref_basis, out)
        return out
    
    def getPoint(self):
        '''
        compute offset point location by rotating offset about axis by
        theta and adding ref_point
        '''
        p = dot(self.ref.getBasis(), self.offset) + self.ref.getPoint()
        return p
    def setAngle(self, angle):
        self.angle = angle
    def __repr__(self):
        return '\nCoordFrame:%s\n%s,\n%s' % (self.name,
                                           self.getBasis(), self.getPoint())
    
class Origin(CoordFrame):
    angle = 0.
    name = 'ORIGIN'
    def __init__(self):
        pass
    def getBasis(self):
        return I3
    def getPoint(self):
        return array([0, 0, 0.])
    def setAngle(self, angle):
        raise ValueError('Cannot set angle of Origin!')
    def __repr__(self):
        return 'ORIGIN'

def getRPY(R):
    roll = arctan2(R[2, 1], R[2, 2])
    pitch = arctan2(R[2, 0], sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
    yaw = arctan2(R[1, 0], R[0, 0])
    return array([roll, pitch, yaw])

def getR(rpy):
    roll, pitch, yaw = rpy
    return dot(R_yaw(yaw), dot(R_pitch(pitch), R_roll(roll)))

rpy = .1, .2, -.3
R = getR(rpy)
assert abs(linalg.norm(getRPY(R) - rpy)) < 1e-8

class Robot:
    def __init__(self, frames):
        self.frames = frames
    def move_joints(self, angles):
        for i in range(1, len(self.frames) - 1): ## skip origin and tool
            frame = self.frames[i]
            frame.setAngle(angles[i-1])
        return numpy.hstack(self.get_arm_pose())
    
    def getPoints(self):
        return array([f.getPoint() for f in self.frames])

    def __repr__(self):
        return '%s' % self.getPoints()

    def get_arm_pose(self):
        p = self.frames[-1].getPoint()
        rpy = getRPY(self.frames[-1].getBasis())
        return (p, rpy)
    def get_theta(self):
        return array([frame.angle for frame in self.frames])[1:-1] ## skip first and last

def forward_kinematics(theta):
    return robot.move_joints(theta)
fk = forward_kinematics
def inverse_kinematics_grad_descent(theta0, goal, angle_tol=1 * DEG, position_tol=1):
    '''
    find angles to put hand at position goal
    '''
    theta0 = robot.get_theta() 

    p0 = fk(theta0)
    J = numpy.zeros((6, 6))
    theta = theta0
    p = p0.copy()
    delta_p = goal - p
    fmt = '%10.4f'
    if (linalg.norm(delta_p[:3]) < angle_tol and
        linalg.norm(delta_p[3:]) < position_tol):
        out = theta0
    else:
        for i in range(40):
            delta_p = goal - p
            fmt = '%10.2f'
            for j in range(6):
                h = 1 * DEG
                J[:,j] = ((fk(theta + I6[j] * h) - fk(theta - I6[j] * h))/
                          (2 * h))
            #print()
            JTW = dot(J.T, W)
            delta_theta = dot(linalg.pinv(dot(JTW, J)), dot(JTW, delta_p))
            # print ('max(abs(delta_theta)) / DEG', max(abs(delta_theta)) / DEG)
            def minme(alpha):
                t = theta + alpha * delta_theta
                p = robot.move_joints(t)
                dp = p - goal
                n = dot(dp, dot(W, dp))
                assert n >= 0
                return n

            x = numpy.arange(0, 1, .01)
            y = [minme(a) for a in x]
            alpha = fminbound(minme, -2, 2, xtol=.01)
            # import pylab
            # pylab.clf()
            # pylab.plot(x, y)
            # pylab.plot(alpha, minme(alpha), 'ro')
            # pylab.show()
            theta = theta + alpha * delta_theta 
            theta = (theta + pi) % (2 * pi) - pi
            p = fk(theta)
            position_diff = linalg.norm(p[:3] - goal[:3])
            angle_diff    = linalg.norm(p[3:] - goal[3:])
            if position_diff < position_tol and angle_diff < angle_tol:
                break
        else:
            raise ValueError('Did not converge to \n%s\nfrom\n%s' % (goal, format('p0', p0, fmt)))
    print ('p:!', format('   p', p, fmt))
    print ('g:!', format('goal', goal, fmt))
    print ('r:!', format(' p-g', p - goal, fmt))
    print ('OK!', format('theta', theta / DEG, fmt), 'DEG')
    print ()
    return theta
    
def inverse_kinematics_ccd(theta0, goal, angle_tol=1 * DEG, position_tol=1, n_loop=20):
    '''
    find angles to put hand at position goal
    '''
    theta = theta0.copy()
    
    ### move joints one at a time to minimize wssr
    def cost(posrpy):
        xx = posrpy
        return dot(xx - goal, dot(W, xx - goal))
    print('initial cost:', sqrt(cost(theta)))
    for loops in range(n_loop):
        for i in range(6):
            def minme(theta_i):
                t = theta.copy()
                t[i] = theta_i
                posrpy = robot.move_joints(t)
                return cost(posrpy)
            theta[i] = fminbound(minme, bounds[i][0], bounds[i, 1])
        posrpy = robot.move_joints(theta)
        pos = posrpy[:3]
        rpy = posrpy[3:]
        if (linalg.norm(goal[:3] - pos) < position_tol and
            linalg.norm(goal[3:] - rpy) < angle_tol):
            # print ('CCD n_cycle:', loops)
            break
    print(' final cost:', sqrt(cost(theta)))
    return theta

def inverse_kinematics(goal, angle_tol=1 * DEG, position_tol=1):
    pos = goal[0:3]
    r, p, y = goal[3:6]
    R = dot(R_yaw(y), dot(R_pitch(p), R_roll(r)))
    nhat = R[:,0]
    L0 = ikr.SHOULDER_HEIGHT
    L1 = ikr.L1
    L2 = ikr.L2
    RHO = ikr.RHO
    start = time.time()
    theta_adhoc = ikr.ikr(L0, L1, L2, RHO, pos, nhat, 0)
    tick1 = time.time()
    adhoc = robot.move_joints(theta_adhoc)
    if (linalg.norm(adhoc[:3] - goal[:3]) < position_tol and
        linalg.norm(adhoc[3:] - goal[3:]) < angle_tol):
        out = theta_adhoc
    else:
        tick2 = time.time()
        theta_ccd = inverse_kinematics_ccd(theta_adhoc, goal)
        tick3 = time.time()
        got_ccd = robot.move_joints(theta_ccd)
        out = theta_ccd
        # got_gd = robot.move_joints(theta_gd)
        print(format('  adhoc', adhoc), tick1 - start)
        # print(format(' gd got', got_gd), tick3 - tick2)
        print(format('ccd got', got_ccd), tick3 - tick2)
        print(format('  goal', goal))
        print(format('theta_adhoc', theta_adhoc))
        print(format(' out', out))
    return out
    
ORIGIN = Origin()
WAIST = CoordFrame('Waist', ORIGIN, [0, 0, 103], R_yaw)
SHOULDER = CoordFrame('Shoulder', WAIST, [0, 0, 80], R_pitch)
ELBOW = CoordFrame('Elbow', SHOULDER, [0, 0, 210], R_pitch)
FORARM = CoordFrame('Forarm', ELBOW, [41.5, 0, 30], R_roll)
WRIST = CoordFrame('Wrist', FORARM, [180, 0, 0], R_pitch)
# HAND = CoordFrame('Hand', WRIST, [23.7, 0, -5.5], R_roll)
HAND = CoordFrame('Hand', WRIST, [16, 0, -5.5], R_roll)
TOOL = CoordFrame('Tool', HAND, [0, 0, 0], Null)

robot = Robot([ORIGIN, WAIST, SHOULDER, ELBOW, FORARM, WRIST, HAND, TOOL])
HOME = robot.move_joints([0, 0, 0, 0, 0, 0])

if __name__ == '__main__':
    theta = [pi/2-pi/10, 0, -pi/10, 0, 0, 0]
    print (theta)
    print(format('robot', robot.move_joints(theta)))
    
    from niryo_one_python_api.niryo_one_api import *
    import rospy
    rospy.init_node('niryo_one_example_python_api')
    niryoone = NiryoOne()
    #niryoone.move_joints(theta)
    #pose = niryoone.get_arm_pose()
    #pose = array([pose.position.x, pose.position.y, pose.position.z, pose.rpy.roll, pose.rpy.pitch, pose.rpy.yaw])
    #pose[:3] *= 1000
    #print(format(' pose', pose))

    home = robot.move_joints([0] * 6)
    N = 10
    r = numpy.linspace(linalg.norm(home[:2]) * 1.5, 50, N)
    goals = numpy.zeros((N, 6)) + HOME
    goals[:,0] = r * cos(numpy.arange(N) / float(N) * pi / 2)
    goals[:,1] = r * sin(numpy.arange(N) / float(N) * pi / 2)
    goals[:,2] = numpy.linspace(0, 550, N)
    fmt = '%10.4f'
    #import pylab
    #pylab.plot(goals[:,0], goals[:,1])
    #pylab.show(); here
    
    for g in goals:
        theta = inverse_kinematics(g)
        theta[5] = 0
        print(niryoone.move_joints(theta))
        print(niryoone.get_arm_pose())
        print(g)
        print
    print(niryoone.move_joints([0] * 6))
        
    
