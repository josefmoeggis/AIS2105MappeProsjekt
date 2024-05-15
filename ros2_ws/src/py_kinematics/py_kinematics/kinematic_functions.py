import numpy as np

def oldUpdateMatrix(U_p, U_r, L): # Not being used
#    U_p = np.deg2rad(U_p)
#    U_r = np.deg2rad(U_r)
    newmatrix = [[(np.cos(U_r) * (-L/(2*np.square(3)))) + 0 * (-L/2*np.cos(U_p)) + (-np.sin(U_r) * (-L/2*np.sin(U_p))), (np.cos(U_r) * L/np.square(3)) + 0 * 0 + (-np.sin(U_r) * 0), (np.cos(U_r) * (-L/(2*np.square(3)))) + 0 * (L/2 * np.cos(U_p)) + (-np.sin(U_r) * L/2 * np.sin(U_p))],
              [(0 * (-L/(2*np.square(3))) + 1 * (-L/2 * np.cos(U_p)) + 0 * (-L/2 * np.sin(U_p))), (0 * L/np.square(3) + 1 * 0 + 0 * 0 ), (0 * (-L/((2*np.square(3)))) + (1 * L/2 * np.cos(U_p)) + (0 * L/2 * np.sin(U_p)))],
              [(np.sin(U_r) * (-L/(2*np.square(3))) + (0 * (-L/2 * np.cos(U_p))) + (np.cos(U_r) * (-L/2 * np.sin(U_p)))), (np.sin(U_r) * L/np.square(3) + (0 * 0) + (np.cos(U_r) * 0)), (np.sin(U_r) * (-L/(2*np.square(3))) + (0 * (L/2 * np.cos(U_p))) + (np.cos(U_p) * (L/2 * np.sin(U_p))))]]
    print(newmatrix)
    return newmatrix


def zposToAngle(z, r):
    return np.arctan(z/r)

def setServos(newMatrix, r):
    servoPos = []
    servoPos.append(zposToAngle(newMatrix[2][0], r))
    servoPos.append(zposToAngle(newMatrix[2][1], r))
    servoPos.append(zposToAngle(newMatrix[2][2], r))
    return servoPos

def pitchMatrix(U_p):
    matrix = [
        [1, 0, 0],
        [0, np.cos(U_p), -np.sin(U_p)],
        [0, np.sin(U_p), np.cos(U_p)]
    ]
    return matrix

def rollMatrix(U_r):
    matrix = [
        [np.cos(U_r), 0, np.sin(U_r)],
        [0, 1, 0],
        [-np.sin(U_r), 0, np.cos(U_r)]
    ]
    return matrix

def heaveMatrix(U_z):
    matrix = [
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, U_z,
        0, 0, 0, 1
    ]
    return matrix

def translationMatrix(L):
    matrix = [
        [-L/(2*np.sqrt(3)), L/np.sqrt(3), -L/(2*np.sqrt(3))],
        [-L/2, 0, L/2],
        [0, 0, 0]
    ]
    return matrix

def updateMatrix(U_r, U_p, L):
    #U_z = 20
    pitch_matrix = pitchMatrix(U_p)
    roll_matrix = rollMatrix(U_r)
    #heave_matrix = heaveMatrix(U_z)
    translation_matrix = translationMatrix(L)

    # Combine the matrices to get the total transformation matrix
    #total_matrix = np.array(pitch_matrix) @ np.array(roll_matrix) @ np.array(heave_matrix) @ np.array(translation_matrix)
    total_matrix = np.dot(np.dot(roll_matrix, pitch_matrix), translation_matrix)
 #   print('Tot matrix', total_matrix)
    return total_matrix

def limitMatrix(matrix, lowerLimit, upperLimit):
    # Finding minimum value
    min = np.min(matrix[2])
    max = np.max(matrix[2])
    # Center around axis
    if(min <= lowerLimit):
        scale = lowerLimit/min
        for i in range(len(matrix[2])):
            matrix[2][i] *= scale
    if(max >= upperLimit):
        scale = upperLimit/max
        for i in range(len(matrix[2])):
            matrix[2][i] *= scale
    return matrix

def totalAngle(servo1, servo2, servo3, servosPos): # not in use
    servo1 += servosPos[0]
    servo2 += servosPos[1]
    servo3 += servosPos[2]
    return 0

def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def smoothenOut(prevVal, inputVal, coefficient):
    newVal = 0
    if(0.8*inputVal < prevVal < 1.2*inputVal):
        toAdd = (prevVal - inputVal) * coefficient
        if(inputVal < prevVal):
            newVal = prevVal - np.abs(toAdd)
        elif(inputVal > prevVal):
            newVal = prevVal + np.abs(toAdd)
    else:
        newVal = inputVal
    prevVal = newVal
    return newVal

def timeOut(pos1, pos2, lastTimeVal, currentTime, limit):
    if((currentTime - lastTimeVal) > limit):
        return 0, 0
    else:
        return pos1, pos2