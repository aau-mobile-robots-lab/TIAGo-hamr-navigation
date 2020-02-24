"""

ACADO --  controls: acc, delta

Path tracking simulation with iterative linear model predictive control for speed and steer control

author: Atsushi Sakai (@Atsushi_twi)

"""
import math
import acado
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MultipleLocator

#sys.path.append("../../PathPlanning/CubicSpline/")

try:
    import cubic_spline_planner
except:
    raise


NX = 4  # [x, y, v, yaw]
NY = 4  # reference state variables
NYN = 4  # reference terminal state variables
NU = 2  # [accel, delta]
T = 5  # horizon length

# mpc parameters
R = np.diag([0.01, 0.01])  # input cost matrix
Rd = np.diag([0.01, 1.0])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
Qf = Q  # state final matrix
GOAL_DIS = 1.5  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 1000.0  # max simulation time

# iterative paramter
MAX_ITER = 3  # Max iteration
DU_TH = 0.1  # iteration finish param

TARGET_SPEED = 10.0 / 3.6  # [m/s] target speed
N_IND_SEARCH = 10  # Search index number

DT = 0.1  # [s] time tick

# Vehicle parameters
LENGTH = 4.5  # [m]
WIDTH = 2.0  # [m]
BACKTOWHEEL = 1.0  # [m]
WHEEL_LEN = 0.3  # [m]
WHEEL_WIDTH = 0.2  # [m]
TREAD = 0.7  # [m]
WB = 1.32  # [m]

MAX_STEER = np.deg2rad(45.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(45.0)  # maximum steering speed [rad/s]
MAX_SPEED = 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]

show_animation = True


class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, d=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.delta = d
        self.predelta = None


def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def get_linear_model_matrix(v, phi, delta):

    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = DT * math.cos(phi)
    A[0, 3] = - DT * v * math.sin(phi)
    A[1, 2] = DT * math.sin(phi)
    A[1, 3] = DT * v * math.cos(phi)
    A[3, 2] = DT * math.tan(delta) / WB

    B = np.zeros((NX, NU))
    B[2, 0] = DT
    B[3, 1] = DT * v / (WB * math.cos(delta) ** 2)

    C = np.zeros(NX)
    C[0] = DT * v * math.sin(phi) * phi
    C[1] = - DT * v * math.cos(phi) * phi
    C[3] = - DT * v * delta / (WB * math.cos(delta) ** 2)

    return A, B, C


def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")




def update_state(state, a, delta, dt, predict):

    # input check
    if delta >= MAX_STEER:
        delta = MAX_STEER
    elif delta <= -MAX_STEER:
        delta = -MAX_STEER

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / WB * math.tan(delta) * dt
    state.v = state.v + a * dt

    if state.v > MAX_SPEED:
        state.v = MAX_SPEED
    elif state.v < MIN_SPEED:
        state.v = MIN_SPEED

    return state


def get_nparray_from_matrix(x):
    return np.array(x).flatten()


def calc_nearest_index(state, cx, cy, cyaw, pind):

    dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def predict_motion(x0, oa, od, xref):
    #xbar = xref * 0.0
    xbar = np.zeros((NX, T + 1))    
    for i, _ in enumerate(x0):
        xbar[i, 0] = x0[i]

    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
    for (ai, di, i) in zip(oa, od, range(1, T + 1)):       
        state = update_state(state, ai, di, DT, True)        
        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.v
        xbar[3, i] = state.yaw        

    return xbar


def iterative_linear_mpc_control(xref, x0, dref, oa, od):
    """
    MPC contorl with updating operational point iteraitvely
    """

    if oa is None or od is None:
        oa = [0.0] * T
        od = [0.0] * T

    for i in range(MAX_ITER):
        xbar = predict_motion(x0, oa, od, xref)
        poa, pod = oa[:], od[:]
        oa, od, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref)
        du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
        if du <= DU_TH:
            break
    else:
        print("Iterative is max iter")

    return oa, od, ox, oy, oyaw, ov

'''
def linear_mpc_control(xref, xbar, x0, dref):
    """
    linear mpc control

    xref: reference point
    xbar: operational point
    x0: initial state
    dref: reference steer angle
    """

    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))

    cost = 0.0
    constraints = []

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t], R)

        if t != 0:
            cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)

        A, B, C = get_linear_model_matrix(
            xbar[2, t], xbar[3, t], dref[0, t])
        constraints += [x[:, t + 1] == A * x[:, t] + B * u[:, t] + C]

        if t < (T - 1):
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
            constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <=
                            MAX_DSTEER * DT]

    cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)

    constraints += [x[:, 0] == x0]
    constraints += [x[2, :] <= MAX_SPEED]
    constraints += [x[2, :] >= MIN_SPEED]
    constraints += [cvxpy.abs(u[0, :]) <= MAX_ACCEL]
    constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.ECOS, verbose=False)

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = get_nparray_from_matrix(x.value[0, :])
        oy = get_nparray_from_matrix(x.value[1, :])
        ov = get_nparray_from_matrix(x.value[2, :])
        oyaw = get_nparray_from_matrix(x.value[3, :])
        oa = get_nparray_from_matrix(u.value[0, :])
        odelta = get_nparray_from_matrix(u.value[1, :])

    else:
        print("Error: Cannot solve mpc..")
        oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

    return oa, odelta, ox, oy, oyaw, ov
'''

# MPC using ACADO
def linear_mpc_control(xref, xbar, x0, dref):
    # see acado.c for parameter details
    _x0=np.zeros((1,NX))  
    X=np.zeros((T+1,NX))
    U=np.zeros((T,NU))    
    Y=np.zeros((T,NY))    
    yN=np.zeros((1,NYN))    
    _x0[0,:]=np.transpose(x0)  # initial state    
    for t in range(T):
      Y[t,:] = np.transpose(xref[:,t])  # reference state
      X[t,:] = np.transpose(xbar[:,t])  # predicted state
    X[-1,:] = X[-2,:]    
    yN[0,:]=Y[-1,:NYN]         # reference terminal state
    X, U = acado.mpc(0, 1, _x0, X,U,Y,yN, np.transpose(np.tile(Q,T)), Qf, 0)    
    ox = get_nparray_from_matrix(X[:,0])
    oy = get_nparray_from_matrix(X[:,1])
    ov = get_nparray_from_matrix(X[:,2])
    oyaw = get_nparray_from_matrix(X[:,3])
    oa = get_nparray_from_matrix(U[:,0])
    odelta = get_nparray_from_matrix(U[:,1])
    return oa, odelta, ox, oy, oyaw, ov


def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, pind):
    xref = np.zeros((NY, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)

    if pind >= ind:
        ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0  # steer operational point should be 0

    travel = 0.0

    for i in range(T + 1):
        travel += abs(state.v) * DT
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.420954595818
        y: -9.2302649916
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.674696521425
        w: 0.738095254
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.423198400784
        y: -9.20536569924
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.6756203116
        w: 0.73724975046
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.425308309873
        y: -9.18045496278
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676548396947
        w: 0.736398171229
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.427426611296
        y: -9.15554498927
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676477941696
        w: 0.736462894109
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.429546057128
        y: -9.13063501575
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676410518508
        w: 0.736524819983
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.431673513825
        y: -9.1057254237
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676340051904
        w: 0.736589528971
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.433802496401
        y: -9.08081621313
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676274939224
        w: 0.736649310444
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.435938726901
        y: -9.05590776549
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676207260424
        w: 0.73671143669
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.43807686475
        y: -9.03099931785
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.67614517644
        w: 0.73676841706
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.440221487585
        y: -9.00609163316
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676085887953
        w: 0.73682282274
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.442367636299
        y: -8.98118394846
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.67602403604
        w: 0.736879571365
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.444520651467
        y: -8.95627664523
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.675967539476
        w: 0.736931398147
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.446674429576
        y: -8.93136972347
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.675913839587
        w: 0.736980652022
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.448834692669
        y: -8.90646318319
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.675860378811
        w: 0.737029679426
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.450995718703
        y: -8.8815566429
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.675815075091
        w: 0.737071220629
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.453162085312
        y: -8.85665086555
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.675769768398
        w: 0.737112759433
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.455329214861
        y: -8.8317450882
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.675730549635
        w: 0.737148712466
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.457500922047
        y: -8.80683931085
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.675691085362
        w: 0.737184886689
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.459673392172
        y: -8.78193391497
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.675656977869
        w: 0.737216147583
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.461849676994
        y: -8.75702890056
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.675628716691
        w: 0.737242047894
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.464025961817
        y: -8.73212388615
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.675603501254
        w: 0.737265155214
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.466205679867
        y: -8.70721887175
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.675580841926
        w: 0.737285918775
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.468385016447
        y: -8.68231423881
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.675563785609
        w: 0.737301547247
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.470567023316
        y: -8.65740960587
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.675552578078
        w: 0.737311816162
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.472747885775
        y: -8.63250497293
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.675544172366
        w: 0.7373195177
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.474931037052
        y: -8.60760033999
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676246700421
        w: 0.736675233851
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.477016913548
        y: -8.58268769618
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676940223754
        w: 0.736037997296
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.479106223272
        y: -8.55777505238
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.67691783987
        w: 0.736058583311
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.481195151526
        y: -8.53286240858
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676895220775
        w: 0.736079384368
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.483287513007
        y: -8.50795014624
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676875399079
        w: 0.736097611817
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.485379111549
        y: -8.48303788391
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676858610082
        w: 0.736113049713
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.487473761849
        y: -8.45812562157
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676838787378
        w: 0.736131276268
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.489568030679
        y: -8.43321374071
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676821997766
        w: 0.73614671319
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.491664969797
        y: -8.40830147838
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676810804567
        w: 0.736157004192
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.493760764506
        y: -8.38338959751
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.6767993757
        w: 0.736167511544
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.495859229503
        y: -8.35847771665
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676785148145
        w: 0.736180591465
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.497956931561
        y: -8.33356621725
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676776752882
        w: 0.736188309306
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.500056540967
        y: -8.30865433639
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676768593377
        w: 0.736195810242
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.502155387434
        y: -8.28374245552
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676762760653
        w: 0.736201172095
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.504255759779
        y: -8.25883095613
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676759726304
        w: 0.736203961449
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.506354987716
        y: -8.23391945673
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676759962188
        w: 0.736203744611
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.508455360062
        y: -8.20900757587
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676759962188
        w: 0.736203744611
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.510554587998
        y: -8.18409607648
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676762760653
        w: 0.736201172095
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.512654578874
        y: -8.15918419561
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676771156009
        w: 0.736193454463
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.514752662402
        y: -8.13427269622
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676779551309
        w: 0.736185736706
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.516851508869
        y: -8.10936081535
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676788182215
        w: 0.736177802174
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.518948447987
        y: -8.08444893449
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.6767993757
        w: 0.736167511544
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.521045768575
        y: -8.05953705362
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.676816401179
        w: 0.736151858719
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.523140418874
        y: -8.03462479129
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.677379632907
        w: 0.73563362683
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.525161445516
        y: -8.00970680691
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.67793129187
        w: 0.73512527062
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.527181327748
        y: -7.98478844106
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.67793129187
        w: 0.73512527062
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.529202354389
        y: -7.95987045668
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.677934086979
        w: 0.735122692965
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.531221855152
        y: -7.93495209083
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.677934313593
        w: 0.73512248398
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.533242881793
        y: -7.91003372498
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.67793129187
        w: 0.73512527062
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.535262764026
        y: -7.8851157406
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.677934086979
        w: 0.735122692965
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.537283409197
        y: -7.86019737475
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.677937108674
        w: 0.73511990633
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.53930290996
        y: -7.8352790089
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.677939903748
        w: 0.735117328666
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.541323173662
        y: -7.81036064305
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.677942472268
        w: 0.735114959918
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.543342292955
        y: -7.78544265867
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.677948062429
        w: 0.735109804484
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.545361793717
        y: -7.76052429282
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.677956674062
        w: 0.735101862394
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.547379768601
        y: -7.73560592697
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.677965059133
        w: 0.735094129071
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.549398124954
        y: -7.71068756112
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.677973670451
        w: 0.735086186903
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.551414955428
        y: -7.6857688138
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.677982055342
        w: 0.735078453387
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.553432167373
        y: -7.66085044795
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.677993235107
        w: 0.73506814184
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.555447471968
        y: -7.63593170063
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.67801023058
        w: 0.73505246563
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.557462395094
        y: -7.61101295331
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.678024204843
        w: 0.735039575565
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.559475792341
        y: -7.58609420599
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.678038178947
        w: 0.735026685155
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.561488808118
        y: -7.56117545867
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.678057968065
        w: 0.735008429845
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.563499535076
        y: -7.53625632988
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.678077531078
        w: 0.734990382146
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.565509880565
        y: -7.51133758256
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.678097319108
        w: 0.734972125879
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.567517937236
        y: -7.4864180723
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.678119901072
        w: 0.734951290746
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.569525230967
        y: -7.46149894352
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.678142257303
        w: 0.734930662621
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.57153023588
        y: -7.43657943326
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.678164837921
        w: 0.734909826174
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.573534477853
        y: -7.411659923
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.678184623417
        w: 0.734891567894
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.575536812477
        y: -7.38674003127
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.678199851034
        w: 0.734877515003
  -
    header:
      seq: 0
      stamp:
        secs: 27
        nsecs: 532000000
      frame_id: "map"
    pose:
      position:
        x: 0.575536727905
        y: -7.38674020767
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.643188664848
        w: 0.7657077389
^C(venv) martinpc@martinpc-ThinkPad-X250:~/PycharmProjects/TIAGo-hamr-navigation/MPC_Differential_Drive$ python casadiMPC.py

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit http://projects.coin-or.org/Ipopt
******************************************************************************

      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  | 313.00us ( 15.65us) 303.00us ( 15.15us)        20
       nlp_g  |  11.02ms (551.20us)  26.53ms (  1.33ms)        20
    nlp_grad  |   1.29ms (  1.29ms)   1.29ms (  1.29ms)         1
  nlp_grad_f  | 623.00us ( 29.67us)   2.66ms (126.88us)        21
  nlp_hess_l  | 119.33ms (  6.28ms) 433.92ms ( 22.84ms)        19
   nlp_jac_g  |  71.96ms (  3.43ms) 219.73ms ( 10.46ms)        21
       total  | 323.63ms (323.63ms)   1.10 s (  1.10 s)         1
[ERROR] [1582286731.235624, 35.386000]: bad callback: <function pathcb at 0x7f700f01ad70>
Traceback (most recent call last):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "casadiMPC.py", line 18, in pathcb
    print(path_data.pose.pose.position.x)
AttributeError: 'Path' object has no attribute 'pose'

[ERROR] [1582286736.740469, 35.883000]: bad callback: <function pathcb at 0x7f700f01ad70>
Traceback (most recent call last):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "casadiMPC.py", line 18, in pathcb
    print(path_data.pose.pose.position.x)
AttributeError: 'Path' object has no attribute 'pose'

[ERROR] [1582286742.028016, 36.383000]: bad callback: <function pathcb at 0x7f700f01ad70>
Traceback (most recent call last):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "casadiMPC.py", line 18, in pathcb
    print(path_data.pose.pose.position.x)
AttributeError: 'Path' object has no attribute 'pose'

^C(venv) martinpc@martinpc-ThinkPad-X250:~/PycharmProjects/TIAGo-hamr-navigation/MPC_Differential_Drive$ ^C
(venv) martinpc@martinpc-ThinkPad-X250:~/PycharmProjects/TIAGo-hamr-navigation/MPC_Differential_Drive$ python casadiMPC.py

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit http://projects.coin-or.org/Ipopt
******************************************************************************

      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  | 616.00us ( 30.80us)   1.20ms ( 60.00us)        20
       nlp_g  |  10.84ms (542.10us)  51.74ms (  2.59ms)        20
    nlp_grad  |   1.78ms (  1.78ms)   8.23ms (  8.23ms)         1
  nlp_grad_f  | 630.00us ( 30.00us)   2.69ms (128.21us)        21
  nlp_hess_l  | 127.40ms (  6.71ms) 392.56ms ( 20.66ms)        19
   nlp_jac_g  |  72.75ms (  3.46ms) 226.78ms ( 10.80ms)        21
       total  | 339.24ms (339.24ms)   1.13 s (  1.13 s)         1
[ERROR] [1582286940.703258, 54.409000]: bad callback: <function pathcb at 0x7f0727659d70>
Traceback (most recent call last):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "casadiMPC.py", line 18, in pathcb
    print(path_data.poses.pose.position.x)
AttributeError: 'list' object has no attribute 'pose'

[ERROR] [1582286945.953780, 54.911000]: bad callback: <function pathcb at 0x7f0727659d70>
Traceback (most recent call last):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "casadiMPC.py", line 18, in pathcb
    print(path_data.poses.pose.position.x)
AttributeError: 'list' object has no attribute 'pose'

[ERROR] [1582286951.494430, 55.410000]: bad callback: <function pathcb at 0x7f0727659d70>
Traceback (most recent call last):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "casadiMPC.py", line 18, in pathcb
    print(path_data.poses.pose.position.x)
AttributeError: 'list' object has no attribute 'pose'

^C(venv) martinpc@martinpc-ThinkPad-X250:~/PycharmProjects/TIAGo-hamr-navigation/MPC_Differential_Drive$ python casadiMPC.py

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit http://projects.coin-or.org/Ipopt
******************************************************************************

      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  | 330.00us ( 16.50us) 364.26us ( 18.21us)        20
       nlp_g  |   9.41ms (470.45us)  23.14ms (  1.16ms)        20
    nlp_grad  |   1.44ms (  1.44ms)   1.82ms (  1.82ms)         1
  nlp_grad_f  | 607.00us ( 28.90us)   1.82ms ( 86.55us)        21
  nlp_hess_l  | 134.42ms (  7.07ms) 410.42ms ( 21.60ms)        19
   nlp_jac_g  |  76.97ms (  3.67ms) 239.74ms ( 11.42ms)        21
       total  | 346.52ms (346.52ms)   1.12 s (  1.12 s)         1
[ERROR] [1582286990.840740, 58.909000]: bad callback: <function pathcb at 0x7f574a723d70>
Traceback (most recent call last):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "casadiMPC.py", line 18, in pathcb
    print(path_data.poses.position.x)
AttributeError: 'list' object has no attribute 'position'

[ERROR] [1582286995.683845, 59.409000]: bad callback: <function pathcb at 0x7f574a723d70>
Traceback (most recent call last):
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "casadiMPC.py", line 18, in pathcb
    print(path_data.poses.position.x)
AttributeError: 'list' object has no attribute 'position'

^CException in thread /move_base/GlobalPlanner/plan (most likely raised during interpreter shutdown):
Traceback (most recent call last):
  File "/usr/lib/python2.7/threading.py", line 801, in __bootstrap_inner
  File "/usr/lib/python2.7/threading.py", line 754, in run
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/impl/tcpros_pubsub.py", line 185, in robust_connect_subscriber
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/impl/tcpros_base.py", line 823, in receive_loop
<type 'exceptions.TypeError'>: 'NoneType' object is not callable
(venv) martinpc@martinpc-ThinkPad-X250:~/PycharmProjects/TIAGo-hamr-navigation/MPC_Differential_Drive$ python casadiMPC.py

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit http://projects.coin-or.org/Ipopt
******************************************************************************

      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  | 299.00us ( 14.95us) 294.92us ( 14.75us)        20
       nlp_g  |   9.53ms (476.70us)  23.44ms (  1.17ms)        20
    nlp_grad  |   1.08ms (  1.08ms)   1.13ms (  1.13ms)         1
  nlp_grad_f  | 435.00us ( 20.71us) 426.03us ( 20.29us)        21
  nlp_hess_l  | 107.97ms (  5.68ms) 218.14ms ( 11.48ms)        19
   nlp_jac_g  |  71.49ms (  3.40ms) 162.05ms (  7.72ms)        21
       total  | 295.16ms (295.16ms) 660.39ms (660.39ms)         1
position:
  x: 1.57043695853
  y: -2.71625656324
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: -0.813814988783
  w: 0.581124052189
position:
  x: 1.68840265853
  y: -2.79974884353
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: -0.847837360978
  w: 0.530256361895
position:
  x: 1.76271830295
  y: -2.98439621567
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: -0.926488343563
  w: 0.376323463582
position:
  x: 1.77744761225
  y: -3.214777994
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: -0.950069119985
  w: 0.312039528348
^C(venv) martinpc@martinpc-ThinkPad-X250:~/PycharmProjects/TIAGo-hamr-navigation/MPC_Differential_Drive$ python casadiMPC.py

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit http://projects.coin-or.org/Ipopt
******************************************************************************

      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  | 400.00us ( 20.00us) 895.44us ( 44.77us)        20
       nlp_g  |  10.92ms (545.85us)  30.09ms (  1.50ms)        20
    nlp_grad  |   1.83ms (  1.83ms)  16.53ms ( 16.53ms)         1
  nlp_grad_f  | 917.00us ( 43.67us)   4.40ms (209.72us)        21
  nlp_hess_l  | 123.67ms (  6.51ms) 358.67ms ( 18.88ms)        19
   nlp_jac_g  |  85.56ms (  4.07ms) 232.34ms ( 11.06ms)        21
       total  | 336.89ms (336.89ms) 963.00ms (963.00ms)         1
0.903831189928
0.798787014168
0.753041545347
0.711904993831
^C(venv) martinpc@martinpc-ThinkPad-X250:~/PycharmProjects/TIAGo-hamr-navigation/MPC_Differential_Drive$ python casadiMPC.py

******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit http://projects.coin-or.org/Ipopt
******************************************************************************

      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  | 363.00us ( 18.15us) 389.47us ( 19.47us)        20
       nlp_g  |  11.31ms (565.65us)  34.80ms (  1.74ms)        20
    nlp_grad  |   1.98ms (  1.98ms)   1.98ms (  1.98ms)         1
  nlp_grad_f  | 548.00us ( 26.10us)   5.50ms (261.77us)        21
  nlp_hess_l  | 115.34ms (  6.07ms) 294.91ms ( 15.52ms)        19
   nlp_jac_g  |  75.02ms (  3.57ms) 142.95ms (  6.81ms)        21
       total  | 327.81ms (327.81ms) 798.10ms (798.10ms)         1
position:
  x: 0.409433446965
  y: -6.96118880117
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: -0.823847358932
  w: 0.566811722869
position:
  x: 0.330727467399
  y: -7.19204055022
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: -0.85863541737
  w: 0.512586792688
position:
  x: 0.23893173544
  y: -7.40890952623
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: -0.782714789155
  w: 0.622380557889
position:
  x: 0.151144105958
  y: -7.5755427532
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: -0.993901480033
  w: 0.110271700762
position:
  x: 0.0984452068574
  y: -7.69130355815
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: -0.99902617678
  w: 0.044121401937
position:
  x: 0.0437329105099
  y: -7.78336097834
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: -0.738769438001
  w: 0.673958246092
position:
  x: -0.00174896272643
  y: -7.81792289902
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.999289773228
  w: 0.0376822122735
position:
  x: -0.0482238016758
  y: -7.84749786589
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: -0.999651567419
  w: 0.0263959041621
position:
  x: -0.246674274604
  y: -7.89672615337
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: -0.999253306306
  w: 0.0386371561599
position:
  x: -0.49460709385
  y: -7.87088577518
  z: 0.0
orientation:

            xref[2, i] = sp[ind + dind]
            xref[3, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp[ncourse - 1]
            xref[3, i] = cyaw[ncourse - 1]
            dref[0, i] = 0.0

    return xref, ind, dref


def check_goal(state, goal, tind, nind):

    # check goal
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.sqrt(dx ** 2 + dy ** 2)

    isgoal = (d <= GOAL_DIS)

    if abs(tind - nind) >= 5:
        isgoal = False

    isstop = (abs(state.v) <= STOP_SPEED)

    if isgoal and isstop:
        return True

    return False


def do_simulation(cx, cy, cyaw, ck, sp, dl, initial_state):
    """
    Simulation

    cx: course x position list
    cy: course y position list
    cy: course yaw position list
    ck: course curvature list
    sp: speed profile
    dl: course tick [m]

    """

    goal = [cx[-1], cy[-1]]

    state = initial_state

    # initial yaw compensation
    if state.yaw - cyaw[0] >= math.pi:
        state.yaw -= math.pi * 2.0
    elif state.yaw - cyaw[0] <= -math.pi:
        state.yaw += math.pi * 2.0

    time = 0.0
    nextPlotTime = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    d = [0.0]
    a = [0.0]
    
    vref = [0.0]
    yawref = [0.0]
    delta = [0.0]
    ex = [0.0]
    ey = [0.0]
    ldu = [0.0]    
    
    target_ind, _ = calc_nearest_index(state, cx, cy, cyaw, 0)

    odelta, oa = None, None

    cyaw = smooth_yaw(cyaw)

    while MAX_TIME >= time:
        xref, target_ind, dref = calc_ref_trajectory(
            state, cx, cy, cyaw, ck, sp, dl, target_ind)

        x0 = [state.x, state.y, state.v, state.yaw]  # current state

        oa, odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(
            xref, x0, dref, oa, odelta)

        if odelta is not None:
            di, ai = odelta[0], oa[0]
            
        # warm-up solver
        if True: #target_ind < 10:
          if abs(state.v) < 0.05:
            if sp[target_ind]<0:
             ai = -0.1
            else:
             ai =  0.1
                    
        #for i in range(100):
        #  state = update_state(state, ai, di, 0.001, False)
        state = update_state(state, ai, di, DT, False)
        
        time = time + DT

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        d.append(di)
        a.append(ai)
        
        yawref.append(pi_2_pi(cyaw[target_ind]))        
        #vref.append(sp[len(v)-1])
        vref.append(sp[target_ind])                
        #delta.append(state.delta)
        #ldu.append(du*100)        
        ex.append(state.x-cx[target_ind])
        ey.append(state.y-cy[target_ind])        


        isgoal = check_goal(state, goal, target_ind, len(cx))

        if show_animation and (isgoal or time > nextPlotTime):  # pragma: no cover            
            nextPlotTime = time + 10.0
            fig = plt.figure(1, figsize=(10,5))            
            plt.subplot(121)
            plt.cla()
            if ox is not None:
                plt.plot(ox, oy, "xr", label="MPC")
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(xref[0, :], xref[1, :], "xk", label="xref")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plot_car(state.x, state.y, state.yaw, steer=di)
            plt.axis("equal")
            plt.grid(True)
            plt.title("Time[s]:" + str(round(time, 2))
                      + ", speed[km/h]:" + str(round(state.v * 3.6, 2)))
            ax = plt.subplot(122)
            ax.cla()            
            ax.plot(t, vref, "-c", label="vref")
            ax.plot(t, v, "-m", label="speed")                                    
            ax.plot(t, ex, "-g", label="err_x")
            ax.plot(t, ey, "-b", label="err_y")                                    
            ax.plot(t, yawref, '-y', label='yawref')
            ax.plot(t, yaw, '-r', label='yaw')            
            spacing = 0.1
            minorLocator = MultipleLocator(spacing)            
            ax.yaxis.set_minor_locator(minorLocator)
            #ax.xaxis.set_minor_locator(minorLocator)
            ax.grid(which = 'minor')            
            plt.legend()            
            plt.xlabel("Time [s]")
            plt.ylabel("Speed [m/s], Error x/y [m]")                                               
            plt.pause(0.0001)
            
        if isgoal:
            print("Goal")
            break
     
    print('time over')
    plt.show()        

    return t, x, y, yaw, v, d, a


def calc_speed_profile(cx, cy, cyaw, target_speed):

    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-1] = 0.0

    return speed_profile


def smooth_yaw(yaw):

    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw


def get_straight_course(dl):
    ax = [0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    ay = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_straight_course2(dl):
    ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_straight_course3(dl):
    ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    cyaw = [i - math.pi for i in cyaw]

    return cx, cy, cyaw, ck


def get_forward_course(dl):
    ax = [0.0, 60.0, 125.0, 50.0, 75.0, 30.0, -10.0]
    ay = [0.0, 0.0, 50.0, 65.0, 30.0, 50.0, -20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_switch_back_course(dl):
    ax = [0.0, 30.0, 6.0, 20.0, 35.0]
    ay = [0.0, 0.0, 20.0, 35.0, 20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    ax = [35.0, 10.0, 0.0, 0.0]
    ay = [20.0, 30.0, 5.0, 0.0]
    cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    cyaw2 = [i - math.pi for i in cyaw2]
    cx.extend(cx2)
    cy.extend(cy2)
    cyaw.extend(cyaw2)
    ck.extend(ck2)

    return cx, cy, cyaw, ck


def main():
    print(__file__ + " start!!")

    dl = 1.0  # course tick
    # cx, cy, cyaw, ck = get_straight_course(dl)
    # cx, cy, cyaw, ck = get_straight_course2(dl)
    # cx, cy, cyaw, ck = get_straight_course3(dl)
    # cx, cy, cyaw, ck = get_forward_course(dl)
    cx, cy, cyaw, ck = get_switch_back_course(dl)

    sp = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)

    initial_state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)

    t, x, y, yaw, v, d, a = do_simulation(
        cx, cy, cyaw, ck, sp, dl, initial_state)

    '''if show_animation:  # pragma: no cover
        plt.close("all")
        plt.subplots()
        plt.plot(cx, cy, "-r", label="spline")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        plt.subplots()
        plt.plot(t, v, "-r", label="speed")
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [kmh]")

        plt.show()'''


def main2():
    print(__file__ + " start!!")

    dl = 1.0  # course tick
    cx, cy, cyaw, ck = get_straight_course3(dl)

    sp = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)

    initial_state = State(x=cx[0], y=cy[0], yaw=0.0, v=0.0)

    t, x, y, yaw, v, d, a = do_simulation(
        cx, cy, cyaw, ck, sp, dl, initial_state)

    if show_animation:  # pragma: no cover
        plt.close("all")
        plt.subplots()
        plt.plot(cx, cy, "-r", label="spline")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        plt.subplots()
        plt.plot(t, v, "-r", label="speed")
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [kmh]")

        plt.show()


if __name__ == '__main__':
    main()
    # main2()
