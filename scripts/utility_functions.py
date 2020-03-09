
from labjack import ljm
import numpy as np

def pwm_val(value):
    return int(value/20000.0*1600000.0)
    

def bound_arm(value):
    maxvalue = 6.00
    minvalue = -6.00
    if(value>maxvalue):
        value = maxvalue
    elif(value<minvalue):
        value = minvalue
    return value

def bound(value, maxv):
    if(value>maxv):
        value = maxv
    elif(value< -maxv):
        value = -maxv
    return value


def update_control_inputs(handle, vector):
  ljm.eWriteName(handle, "DIO0_EF_CONFIG_A", pwm_val(vector[0]))
  ljm.eWriteName(handle, "DIO2_EF_CONFIG_A", pwm_val(vector[1]))
  ljm.eWriteName(handle, "DIO3_EF_CONFIG_A", pwm_val(vector[2]))
  ljm.eWriteName(handle, "DIO4_EF_CONFIG_A", pwm_val(vector[3]))

C17_ENAC = np.array([
        [-0.00913,  -0.08197,   0.00814,  -3.33902,  -0.06631,   3.24349 ],
        [ 0.15977,   4.06983,  -0.04982,  -1.98943,   0.02155,  -1.81224 ],
        [ 3.75783,   0.01159,   3.81047,  -0.05917,   3.73985,   0.01544 ],
        [ 0.59718,  24.80018,  20.62192, -12.26305, -21.14349, -11.18867 ],
        [-23.74593, 0.36636,   12.62541,  20.26775,  12.67510, -19.73277 ],
        [ 0.31759,  15.10976,   0.20531,  14.88887,   0.04100,  14.32650 ] ])

C17_ITU = np.array([
        [-0.03539,   0.06181,  -0.13151,  -3.17527,   0.11935,   3.46727 ],
        [ 0.16873,   3.87708,  -0.15694,  -1.82572,   0.01400,  -2.03634 ],
        [ 3.73105,  -0.06372,   3.63707,  -0.21033,   3.90777,  -0.08136 ],
        [ 1.59897,  44.95240,  19.47450, -22.11457, -21.50988, -23.46836 ],
        [24.25347,  -0.65355,  12.89910,  36.24407,  11.35994, -40.09825 ],
        [ 0.61677,  14.55758,   0.63916,  13.63150,   0.55855,  15.04069 ] ])


C40_ENAC = np.array([
        [ 0.10573,  -0.04946,  -0.03339,   6.31374,  -0.04072,  -6.35659 ],
        [ 0.24983,  -7.21789,   0.12016,   3.62807,  -0.08762,   3.70953 ],
        [10.12504,   0.21548,  10.10169,   0.36926,  10.57529,   0.28278 ],
        [-0.00249,  -0.03763,   0.14501,   0.02394,  -0.15264,   0.01486 ],
        [-0.17046,  -0.00215,   0.08486,  -0.03049,   0.08772,   0.03596 ],
        [ 0.00226,  -0.08472,  -0.00019,  -0.08519,   0.00105,  -0.08698 ] ])


B40 = np.array([[0.1476432979106903, 0.14606346189975739, -0.11490651965141296, 0.1375323235988617, 0.09234895557165146, -0.29624149203300476]])
B17 = np.array([[-0.08521055430173874, 0.22347553074359894, -0.10100627690553665, 0.45128822326660156, 0.1454315185546875, 0.5179573893547058]])
B17 = np.array([[ -0.282657,  0.159650,  0.453816,  0.306891,  -0.054251,  0.506583]])
def ttm(tx,ty,tz,rx,ry,rz):
    sx = np.sin(np.pi/180.0*rx)
    cx = np.cos(np.pi/180.0*rx)
    sy = np.sin(np.pi/180.0*ry)
    cy = np.cos(np.pi/180.0*ry)
    sz = np.sin(np.pi/180.0*rz)
    cz = np.cos(np.pi/180.0*rz)

    dx = tx
    dy = ty
    dz = tz

    R11 =  cy *cz
    R12 =  sx * sy * cz + cx * sz
    R13 =  sx * sz - cx * sy * cz
    R14 = 0.0
    R15 = 0.0
    R16 = 0.0
    R21 = -cy * sz
    R22 = -sx * sy * sz + cx * cz
    R23 =  sx * cz + cx * sy * sz
    R24 = 0.0
    R25 = 0.0
    R26 = 0.0
    R31 =  sy
    R32 = -sx * cy
    R33 =  cx * cy
    R34 = 0.0
    R35 = 0.0
    R36 = 0.0

    R41 = R13 * dy - R12 * dz
    R42 = R11 * dz - R13 * dx
    R43 = R12 * dx - R11 * dy
    R44 = R11
    R45 = R12
    R46 = R13
    
    R51 = R23 * dy - R22 * dz
    R52 = R21 * dz - R23 * dx
    R53 = R22 * dx - R21 * dy
    R54 = R21
    R55 = R22
    R56 = R23
    
    R61 = R33 * dy - R32 * dz 
    R62 = R31 * dz - R33 * dx
    R63 = R32 * dx - R31 * dy
    R64 = R31
    R65 = R32
    R66 = R33
    
    Result = np.array([
            [R11,  R12, R13, R14, R15, R16],
            [R21,  R22, R23, R24, R25, R26],
            [R31,  R32, R33, R34, R35, R36],
            [R41,  R42, R43, R44, R45, R46],
            [R51,  R52, R53, R54, R55, R56],
            [R61,  R62, R63, R64, R65, R66], ])
    return Result



def signal_to_force(signal, calibration_matrix, bias=np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])):
    A = np.array([[0.0,0.0,0.0,0.0,0.0,0.0]])
    A[0][0] = signal[0]
    A[0][1] = signal[1]
    A[0][2] = signal[2]
    A[0][3] = signal[3]
    A[0][4] = signal[4]
    A[0][5] = signal[5]
    AB = A - bias
    F = calibration_matrix.dot(AB.T)
    return F

def get_hebi_from_aircraft_angle(angle):
    # return ((angle - 30)*0.0186+0.503)
    return ((angle - 30)*0.01745329+0.4715)


