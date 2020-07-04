from sympy import *
from code_gen import *

# q: quaternion describing rotation from frame 1 to frame 2
# returns a rotation matrix derived form q which describes the same
# rotation
def quat2Rot(q):
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    Rot = Matrix([[q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
                  [2*(q1*q2 + q0*q3), q0**2 - q1**2 + q2**2 - q3**2, 2*(q2*q3 - q0*q1)],
                   [2*(q1*q3-q0*q2), 2*(q2*q3 + q0*q1), q0**2 - q1**2 - q2**2 + q3**2]])

    return Rot

def create_cov_matrix(i, j):
    if j >= i:
        return Symbol("P(" + str(i) + "," + str(j) + ")", real=True)
    else:
        return 0

def create_Tbs_matrix(i, j):
    return Symbol("Tbs(" + str(i) + "," + str(j) + ")", real=True)

def quat_mult(p,q):
    r = Matrix([p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3],
                p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2],
                p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1],
                p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0]])

    return r

def create_symmetric_cov_matrix():
    # define a symbolic covariance matrix
    P = Matrix(24,24,create_cov_matrix)

    for index in range(24):
        for j in range(24):
            if index > j:
                P[index,j] = P[j,index]

    return P

def create_symbol(name, real=True):
    symbol_name_list.append(name)
    return Symbol(name, real=True)

symbol_name_list = []

dt = create_symbol("dt", real=True)  # dt
g = create_symbol("g", real=True) # gravity constant

r_hor_vel = create_symbol("R_hor_vel", real=True) # horizontal velocity noise variance
r_ver_vel = create_symbol("R_vert_vel", real=True) # vertical velocity noise variance
r_hor_pos = create_symbol("R_hor_pos", real=True) # horizontal position noise variance

# inputs, integrated gyro measurements
d_ang_x = create_symbol("dax", real=True)  # delta angle x
d_ang_y = create_symbol("day", real=True)  # delta angle y
d_ang_z = create_symbol("daz", real=True)  # delta angle z

d_ang = Matrix([d_ang_x, d_ang_y, d_ang_z])

# inputs, integrated accelerometer measurements
d_v_x = create_symbol("dvx", real=True)  # delta velocity x
d_v_y = create_symbol("dvy", real=True)  # delta velocity y
d_v_z = create_symbol("dvz", real=True)  # delta velocity z

d_v = Matrix([d_v_x, d_v_y,d_v_z])

u = Matrix([d_ang, d_v])

# input noise
d_ang_x_var = create_symbol("daxVar", real=True)
d_ang_y_var = create_symbol("dayVar", real=True)
d_ang_z_var = create_symbol("dazVar", real=True)

d_v_x_var = create_symbol("dvxVar", real=True)
d_v_y_var = create_symbol("dvyVar", real=True)
d_v_z_var = create_symbol("dvzVar", real=True)

var_u = Matrix.diag(d_ang_x_var, d_ang_y_var, d_ang_z_var, d_v_x_var, d_v_y_var, d_v_z_var)

# define state vector

# attitude quaternion
qw = create_symbol("q0", real=True)  # quaternion real part
qx = create_symbol("q1", real=True)  # quaternion x component
qy = create_symbol("q2", real=True)  # quaternion y component
qz = create_symbol("q3", real=True)  # quaternion z component

q = Matrix([qw,qx,qy,qz])
R_to_earth = quat2Rot(q)
R_to_body = R_to_earth.T

# velocity in NED local frame
vx = create_symbol("vn", real=True)  # north velocity
vy = create_symbol("ve", real=True)  # east velocity
vz = create_symbol("vd", real=True)  # down velocity

v = Matrix([vx,vy,vz])

# position in NED local frame
px = create_symbol("pn", real=True)  # north position
py = create_symbol("pe", real=True)  # east position
pz = create_symbol("pd", real=True)  # down position

p = Matrix([px,py,pz])

# delta angle bias
d_ang_bx = create_symbol("dax_b", real=True)  # delta angle bias x
d_ang_by = create_symbol("day_b", real=True)  # delta angle bias y
d_ang_bz = create_symbol("daz_b", real=True)  # delta angle bias z

d_ang_b = Matrix([d_ang_bx, d_ang_by, d_ang_bz])
d_ang_true = d_ang - d_ang_b

# delta velocity bias
d_vel_bx = create_symbol("dvx_b", real=True)  # delta velocity bias x
d_vel_by = create_symbol("dvy_b", real=True)  # delta velocity bias y
d_vel_bz = create_symbol("dvz_b", real=True)  # delta velocity bias z

d_vel_b = Matrix([d_vel_bx, d_vel_by, d_vel_bz])

d_vel_true = d_v - d_vel_b

# earth magnetic field vector
ix = create_symbol("magN", real=True)  # earth magnetic field x component
iy = create_symbol("magE", real=True)  # earth magnetic field y component
iz = create_symbol("magD", real=True)  # earth magnetic field z component

i = Matrix([ix,iy,iz])

# magnetometer bias in body frame
ibx = create_symbol("ibx", real=True)  # earth magnetic field bias in body x
iby = create_symbol("iby", real=True)  # earth magnetic field bias in body y
ibz = create_symbol("ibz", real=True)  # earth magnetic field bias in body z

ib = Matrix([ibx,iby,ibz])

# wind in local NE frame
wx = create_symbol("vwn", real=True)  # wind in north direction
wy = create_symbol("vwe", real=True)  # wind in east direction

w = Matrix([wx,wy])

# state vector at arbitrary time t
state = Matrix([q,v,p,d_ang_b,d_vel_b,i,ib,w])

# define state propagation
q_new = quat_mult(q, Matrix([1, 0.5 * d_ang_true[0],  0.5 * d_ang_true[1],  0.5 * d_ang_true[2]]))

v_new = v + R_to_earth * d_vel_true + Matrix([0,0,g]) * dt

p_new = p + v * dt

d_ang_b_new = d_ang_b
d_vel_b_new = d_vel_b
i_new = i
ib_new = ib
w_new = w

# predicted state vector at time t + dt
state_new = Matrix([q_new, v_new, p_new, d_ang_b_new, d_vel_b_new, i_new, ib_new, w_new])

# state transition matrix
A = state_new.jacobian(state)

# B
G = state_new.jacobian(u)

P = create_symmetric_cov_matrix()

# propagate covariance matrix
P_new = A * P * A.T + G * var_u * G.T

for index in range(24):
    for j in range(24):
        if index > j:
            P_new[index,j] = 0


P_new_simple = cse(P_new, symbols("PS0:400"), optimizations='basic')

cov_code_generator = CodeGenerator("./covariance_generated.cpp")
cov_code_generator.print_string("Equations for covariance matrix prediction, without process noise!")
cov_code_generator.write_subexpressions(P_new_simple[0])
cov_code_generator.write_matrix(Matrix(P_new_simple[1]), "nextP", True)

cov_code_generator.close()

# 3D magnetometer fusion
r_mag = create_symbol("R_MAG", real=True)  # magnetometer measurement noise variance

m_mag = R_to_body * i + ib

H_x_mag = Matrix([m_mag[0]]).jacobian(state)
H_y_mag = Matrix([m_mag[1]]).jacobian(state)
H_z_mag = Matrix([m_mag[2]]).jacobian(state)

K_x_mag = P * H_x_mag.T / (H_x_mag * P * H_x_mag.T + Matrix([r_mag]))
K_y_mag = P * H_y_mag.T / (H_y_mag * P * H_y_mag.T + Matrix([r_mag]))
K_z_mag = P * H_z_mag.T / (H_z_mag * P * H_z_mag.T + Matrix([r_mag]))

mag_x_innov_var = H_x_mag * P * H_x_mag.T + Matrix([r_mag])
mag_y_innov_var = H_y_mag * P * H_y_mag.T + Matrix([r_mag])
mag_z_innov_var = H_z_mag * P * H_z_mag.T + Matrix([r_mag])

mag_x_innov_var_simple = cse(mag_x_innov_var, symbols("SX0:100"))
mag_y_innov_var_simple = cse(mag_y_innov_var, symbols("SY0:100"))
mag_z_innov_var_simple = cse(mag_z_innov_var, symbols("SZ0:100"))

HK_x_simple = cse(Matrix([H_x_mag.transpose(), K_x_mag]), symbols("HKX0:200"))
HK_y_simple = cse(Matrix([H_y_mag.transpose(), K_y_mag]), symbols("HKY0:200"))
HK_z_simple = cse(Matrix([H_z_mag.transpose(), K_z_mag]), symbols("HKZ0:200"))

mag_code_generator = CodeGenerator("./3Dmag_generated.cpp")
mag_code_generator.print_string("Equations for 3D mag fusion")

# x axis
mag_code_generator.print_string("X axis innovation variance")
mag_code_generator.write_subexpressions(mag_x_innov_var_simple[0])
mag_code_generator.write_matrix(Matrix(mag_x_innov_var_simple[1]), "innov_var_x")

mag_code_generator.print_string("X axis observation matrix and kalman gain")
mag_code_generator.write_subexpressions(HK_x_simple[0])
mag_code_generator.write_matrix(Matrix(HK_x_simple[1][0][0:24]), "H_MAG")
mag_code_generator.write_matrix(Matrix(HK_x_simple[1][0][24:]), "Kfusion")

# y axis
mag_code_generator.print_string("Y axis innovation variance")
mag_code_generator.write_subexpressions(mag_y_innov_var_simple[0])
mag_code_generator.write_matrix(Matrix(mag_y_innov_var_simple[1]), "innov_var_y")

mag_code_generator.print_string("Y axis observation matrix and kalman gain")
mag_code_generator.write_subexpressions(HK_y_simple[0])
mag_code_generator.write_matrix(Matrix(HK_y_simple[1][0][0:24]), "H_MAG")
mag_code_generator.write_matrix(Matrix(HK_y_simple[1][0][24:]), "Kfusion")

# z axis
mag_code_generator.print_string("Z axis innovation variance")
mag_code_generator.write_subexpressions(mag_z_innov_var_simple[0])
mag_code_generator.write_matrix(Matrix(mag_z_innov_var_simple[1]), "innov_var_z")

mag_code_generator.print_string("Z axis observation matrix and kalman gain")
mag_code_generator.write_subexpressions(HK_z_simple[0])
mag_code_generator.write_matrix(Matrix(HK_z_simple[1][0][0:24]), "H_MAG")
mag_code_generator.write_matrix(Matrix(HK_z_simple[1][0][24:]), "Kfusion")

mag_code_generator.close()

# airspeed fusion
r_tas = create_symbol("R_TAS", real=True) # true airspeed measurement noise variance

tas = sqrt((vx-wx)*(vx-wx)+(vy-wy)*(vy-wy)+vz*vz)
H_tas = Matrix([tas]).jacobian(state)
K_tas = P * H_tas.T / (H_tas * P * H_tas.T + Matrix([r_tas]))
tas_innov_var = H_tas * P * H_tas.T + Matrix([r_tas])
tas_innov_var_simple = cse(tas_innov_var, symbols("S0:100"))
HK_tas_simple = cse(Matrix([H_tas.transpose(), K_tas]), symbols("HK0:200"))

tas_code_generator = CodeGenerator("./tas_generated.cpp")
tas_code_generator.print_string("Equations for TAS fusion")

tas_code_generator.print_string("TAS innovation variance")
tas_code_generator.write_subexpressions(tas_innov_var_simple[0])
tas_code_generator.write_matrix(Matrix(tas_innov_var_simple[1]), "innov_var")

tas_code_generator.print_string("TAS observation matrix and kalman gain")
tas_code_generator.write_subexpressions(HK_tas_simple[0])
tas_code_generator.write_matrix(Matrix(HK_tas_simple[1][0][0:24]), "H_TAS")
tas_code_generator.write_matrix(Matrix(HK_tas_simple[1][0][24:]), "Kfusion")

# sideslip fusion
r_beta = create_symbol("R_BETA", real=True) # sideslip measurement noise variance

v_rel_ef = Matrix([vx-wx,vy-wy,vz])
v_rel_bf = R_to_body * v_rel_ef
beta = v_rel_bf[1]/v_rel_bf[0]
H_beta = Matrix([beta]).jacobian(state)
K_beta = P * H_beta.T / (H_beta * P * H_beta.T + Matrix([r_beta]))
beta_innov_var = H_beta * P * H_beta.T + Matrix([r_beta])
beta_innov_var_simple = cse(beta_innov_var, symbols("S0:100"))
HK_beta_simple = cse(Matrix([H_beta.transpose(), K_beta]), symbols("HK0:200"))

beta_code_generator = CodeGenerator("./beta_generated.cpp")
beta_code_generator.print_string("Equations for sideslip fusion")

beta_code_generator.print_string("sideslip innovation variance")
beta_code_generator.write_subexpressions(beta_innov_var_simple[0])
beta_code_generator.write_matrix(Matrix(beta_innov_var_simple[1]), "innov_var")

beta_code_generator.print_string("sideslip observation matrix and kalman gain")
beta_code_generator.write_subexpressions(HK_beta_simple[0])
beta_code_generator.write_matrix(Matrix(HK_beta_simple[1][0][0:24]), "H_BETA")
beta_code_generator.write_matrix(Matrix(HK_beta_simple[1][0][24:]), "Kfusion")

# yaw fusion
yaw_code_generator = CodeGenerator("./yaw_generated.cpp")

# Derive observation Jacobian for fusion of 321 sequence yaw measurement
# Calculate the yaw (first rotation) angle from the 321 rotation sequence
# Provide alternative angle that avoids singularity at +-pi/2 yaw
angMeasA = atan(R_to_earth[1,0]/R_to_earth[0,0])
H_YAW321_A = Matrix([angMeasA]).jacobian(state)
H_YAW321_A_simple = cse(H_YAW321_A, symbols('SA0:200'))

angMeasB = pi/2 - atan(R_to_earth[0,0]/R_to_earth[1,0])
H_YAW321_B = Matrix([angMeasB]).jacobian(state)
H_YAW321_B_simple = cse(H_YAW321_B, symbols('SB0:200'))

yaw_code_generator.print_string("calculate 321 yaw observation matrix - option A")
yaw_code_generator.write_subexpressions(H_YAW321_A_simple[0])
yaw_code_generator.write_matrix(Matrix(H_YAW321_A_simple[1]).T, "H_YAW")

yaw_code_generator.print_string("calculate 321 yaw observation matrix - option B")
yaw_code_generator.write_subexpressions(H_YAW321_B_simple[0])
yaw_code_generator.write_matrix(Matrix(H_YAW321_B_simple[1]).T, "H_YAW")

# Derive observation Jacobian for fusion of 312 sequence yaw measurement
# Calculate the yaw (first rotation) angle from an Euler 312 sequence
# Provide alternative angle that avoids singularity at +-pi/2 yaw
angMeasA = atan(-R_to_earth[0,1]/R_to_earth[1,1])
H_YAW312_A = Matrix([angMeasA]).jacobian(state)
H_YAW312_A_simple = cse(H_YAW312_A, symbols('SA0:200'))

angMeasB = pi/2 - atan(-R_to_earth[1,1]/R_to_earth[0,1])
H_YAW312_B = Matrix([angMeasB]).jacobian(state)
H_YAW312_B_simple = cse(H_YAW312_B, symbols('SB0:200'))

yaw_code_generator.print_string("calculate 312 yaw observation matrix - option A")
yaw_code_generator.write_subexpressions(H_YAW312_A_simple[0])
yaw_code_generator.write_matrix(Matrix(H_YAW312_A_simple[1]).T, "H_YAW")

yaw_code_generator.print_string("calculate 312 yaw observation matrix - option B")
yaw_code_generator.write_subexpressions(H_YAW312_B_simple[0])
yaw_code_generator.write_matrix(Matrix(H_YAW312_B_simple[1]).T, "H_YAW")

# derive equations for sequential fusion of optical flow measurements
flow_code_generator = CodeGenerator("./flow_generated.cpp")
range = create_symbol("range", real=True) # range from camera focal point to ground along sensor Z axis
obs_var = create_symbol("R_LOS", real=True) # optical flow line of sight rate measurement noise variance

# Define rotation matrix from body to sensor frame
Tbs = Matrix(3,3,create_Tbs_matrix)

# Calculate earth relative velocity in a non-rotating sensor frame
relVelSensor = Tbs * R_to_body * Matrix([vx,vy,vz])

# Divide by range to get predicted angular LOS rates relative to X and Y
# axes. Note these are rates in a non-rotating sensor frame
losRateSensorX = +relVelSensor[1]/range
losRateSensorY = -relVelSensor[0]/range

# calculate the observation Jacobian and Kalman gains for the X axis
H_LOSX = Matrix([losRateSensorX]).jacobian(state)
flow_innov_var_X = H_LOSX * P * H_LOSX.T + Matrix([obs_var])
K_LOSX = (P * H_LOSX.T) / flow_innov_var_X
HK_flow_x_simple = cse(Matrix([H_LOSX.transpose(),K_LOSX]), symbols("SX0:1000"), optimizations='basic')

flow_code_generator.print_string("X axis")
flow_code_generator.write_subexpressions(HK_flow_x_simple[0])
flow_code_generator.write_matrix(Matrix(HK_flow_x_simple[1][0][0:24]), "H_LOS")
flow_code_generator.write_matrix(Matrix(HK_flow_x_simple[1][0][24:]), "Kfusion")

# calculate the observation Jacobian and Kalman gains for the Y axis
H_LOSY = Matrix([losRateSensorY]).jacobian(state)
flow_innov_var_y = H_LOSY * P * H_LOSY.T + Matrix([obs_var])
K_LOSY = (P * H_LOSY.T) / flow_innov_var_y
HK_flow_y_simple = cse(Matrix([H_LOSY.transpose(),K_LOSY]), symbols("SY0:1000"), optimizations='basic')

flow_code_generator.print_string("Y axis")
flow_code_generator.write_subexpressions(HK_flow_y_simple[0])
flow_code_generator.write_matrix(Matrix(HK_flow_y_simple[1][0][0:24]), "H_LOS")
flow_code_generator.write_matrix(Matrix(HK_flow_y_simple[1][0][24:]), "Kfusion")

# Derive equations for sequential fusion of body frame velocity measurements
obs_var = create_symbol("R_VEL", real=True) # measurement noise variance

# Calculate earth relative velocity in a non-rotating sensor frame
vel_bf = R_to_body * Matrix([vx,vy,vz])

vel_bf_code_generator = CodeGenerator("./vel_bf_generated.cpp")
axes = [0,1,2]
H_obs = vel_bf.jacobian(state) # observation Jacobians
K_gain = zeros(24,3)
for index in axes:
    innov_var = H_obs[index,:] * P * H_obs[index,:].T + Matrix([obs_var])
    K_gain[:,index] = (P * H_obs[index,:].T) / innov_var
    HK_simple = cse(Matrix([H_obs[index,:].transpose(),K_gain[:,index]]), symbols("S0:1000"), optimizations='basic')

    vel_bf_code_generator.print_string("axis %i" % index)
    vel_bf_code_generator.write_subexpressions(HK_simple[0])
    vel_bf_code_generator.write_matrix(Matrix(HK_simple[1][0][0:24]), "H_VEL")
    vel_bf_code_generator.write_matrix(Matrix(HK_simple[1][0][24:]), "Kfusion")

# attempt to calculate a combined result for efficiency
vel_bf_code_generator_alt = CodeGenerator("./vel_bf_generated_alt.cpp")
HK_simple = cse(Matrix([H_obs[0,:].transpose(),K_gain[:,0],H_obs[1,:].transpose(),K_gain[:,1],H_obs[2,:].transpose(),K_gain[:,2]]), symbols("S0:1000"), optimizations='basic')
vel_bf_code_generator_alt.print_string("Sub Expressions")
vel_bf_code_generator_alt.write_subexpressions(HK_simple[0])
vel_bf_code_generator_alt.print_string("X axis observation Jacobians and Kalman gains")
vel_bf_code_generator_alt.write_matrix(Matrix(HK_simple[1][0][0:24]), "H_VEL")
vel_bf_code_generator_alt.write_matrix(Matrix(HK_simple[1][0][24:48]), "Kfusion")
vel_bf_code_generator_alt.print_string("Y axis observation Jacobians and Kalman gains")
vel_bf_code_generator_alt.write_matrix(Matrix(HK_simple[1][0][48:72]), "H_VEL")
vel_bf_code_generator_alt.write_matrix(Matrix(HK_simple[1][0][72:96]), "Kfusion")
vel_bf_code_generator_alt.print_string("Z axis observation Jacobians and Kalman gains")
vel_bf_code_generator_alt.write_matrix(Matrix(HK_simple[1][0][96:120]), "H_VEL")
vel_bf_code_generator_alt.write_matrix(Matrix(HK_simple[1][0][120:144]), "Kfusion")

# derive equations for fusion of dual antenna yaw measurement
obs_var = create_symbol("R_YAW", real=True) # measurement noise variance
ant_yaw = create_symbol("ant_yaw", real=True) # yaw angle of antenna array axis wrt X body axis

# define antenna vector in body frame
ant_vec_bf = Matrix([cos(ant_yaw),sin(ant_yaw),0])

# rotate into earth frame
ant_vec_ef = R_to_body.T * ant_vec_bf

# Calculate the yaw angle from the projection
angMeas = atan(ant_vec_ef[1]/ant_vec_ef[0])

H_obs = Matrix([angMeas]).jacobian(state) # measurement Jacobians
innov_var = H_obs * P * H_obs.T + Matrix([obs_var])
K_gain = (P * H_obs.T) / innov_var # Kalman gains
HK_simple = cse(Matrix([H_obs.transpose(),K_gain]), symbols("S0:1000"), optimizations='basic')

gps_yaw_code_generator = CodeGenerator("./gps_yaw_generated.cpp")
gps_yaw_code_generator.print_string("Sub Expressions")
gps_yaw_code_generator.write_subexpressions(HK_simple[0])
gps_yaw_code_generator.print_string("Observation Jacobians")
gps_yaw_code_generator.write_matrix(Matrix(HK_simple[1][0][0:24]), "H_YAW")
gps_yaw_code_generator.print_string("Kalman gains")
gps_yaw_code_generator.write_matrix(Matrix(HK_simple[1][0][24:]), "Kfusion")

# derive equations for fusion of declination
obs_var = create_symbol("R_DECL", real=True) # measurement noise variance

# the predicted measurement is the angle wrt magnetic north of the horizontal
# component of the measured field
observation = atan(iy/ix)

H_obs = Matrix([observation]).jacobian(state) # measurement Jacobians
innov_var = H_obs * P * H_obs.T + Matrix([obs_var])
K_gain = (P * H_obs.T) / innov_var # Kalman gains
HK_simple = cse(Matrix([H_obs.transpose(),K_gain]), symbols("S0:1000"), optimizations='basic')

mag_decl_code_generator = CodeGenerator("./mag_decl_generated.cpp")
mag_decl_code_generator.print_string("Sub Expressions")
mag_decl_code_generator.write_subexpressions(HK_simple[0])
mag_decl_code_generator.print_string("Observation Jacobians")
mag_decl_code_generator.write_matrix(Matrix(HK_simple[1][0][0:24]), "H_DECL")
mag_decl_code_generator.print_string("Kalman gains")
mag_decl_code_generator.write_matrix(Matrix(HK_simple[1][0][24:]), "Kfusion")

# derive equations for fusion of lateral body acceleration (multirotors only)
obs_var = create_symbol("R_ACC", real=True) # measurement noise variance
Kaccx = create_symbol("Kaccx", real=True) # measurement noise variance
Kaccy = create_symbol("Kaccy", real=True) # measurement noise variance

# use relationship between airspeed along the X and Y body axis and the
# drag to predict the lateral acceleration for a multirotor vehicle type
# where propulsion forces are generated primarily along the Z body axis

vrel = R_to_body*Matrix([vx-wx,vy-wy,vz]) # predicted wind relative velocity

# Use this nonlinear model for the prediction in the implementation only
# It uses a ballistic coefficient for each axis
# accXpred = -0.5*rho*vrel[0]*vrel[0]*BCXinv # predicted acceleration measured along X body axis
# accYpred = -0.5*rho*vrel[1]*vrel[1]*BCYinv # predicted acceleration measured along Y body axis

# Use a simple viscous drag model for the linear estimator equations
# Use the the derivative from speed to acceleration averaged across the
# speed range. This avoids the generation of a dirac function in the derivation
# The nonlinear equation will be used to calculate the predicted measurement in implementation
observation = Matrix([-Kaccx*vrel[0],-Kaccy*vrel[1]])

axes = [0,1]
H_obs = observation.jacobian(state)
K_gain = zeros(24,2)
for index in axes:
    innov_var = H_obs[index,:] * P * H_obs[index,:].T + Matrix([obs_var])
    K_gain[:,index] = (P * H_obs[index,:].T) / innov_var

# calculate a combined result for efficiency
acc_bf_code_generator = CodeGenerator("./acc_bf_generated.cpp")
HK_simple = cse(Matrix([H_obs[0,:].transpose(),K_gain[:,0],H_obs[1,:].transpose(),K_gain[:,1]]), symbols("S0:1000"), optimizations='basic')
acc_bf_code_generator.print_string("Sub Expressions")
acc_bf_code_generator.write_subexpressions(HK_simple[0])

acc_bf_code_generator.print_string("X axis observation Jacobians")
acc_bf_code_generator.write_matrix(Matrix(HK_simple[1][0][0:24]), "H_VEL")

acc_bf_code_generator.print_string("X axis Kalman gains")
acc_bf_code_generator.write_matrix(Matrix(HK_simple[1][0][24:48]), "Kfusion")

acc_bf_code_generator.print_string("Y axis observation Jacobians")
acc_bf_code_generator.write_matrix(Matrix(HK_simple[1][0][48:72]), "H_VEL")

acc_bf_code_generator.print_string("XY axis Kalman gains")
acc_bf_code_generator.write_matrix(Matrix(HK_simple[1][0][72:96]), "Kfusion")
