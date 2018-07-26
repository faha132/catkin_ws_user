import cv2
import numpy as np

def foo():
    cv_image = cv2.imread('transferm.png', flags=cv2.IMREAD_GRAYSCALE)

    amount, markers = cv2.connectedComponents(cv_image)

    acc = []
    component_num=2
    for y in range(len(markers)):
        for x in range(len(markers[1])):
            if(markers[y,x]==component_num):
                acc.append([x,y])
                component_num+=1

    print('found dots')
    print(acc)
        
    camera_vision_dots = np.zeros((6,2,1))
    camera_vision_dots[:,:,0]=np.array(acc)

    camera_formula = np.zeros((3, 3, 1), dtype = "float")
    #schema https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
    camera_formula[:,:,0] = np.array([  [614.1699,0,329.9491],
                                    [0, 614.9002, 237.2788],
                                    [0,0,1]])


    distortion_coefficients = np.zeros((4, 1))
    distortion_coefficients = np.array([[0.1115,-0.1089,0,0]])

    measured_dots = np.zeros((6, 3, 1))
    measured_dots[:,:,0] = np.array([  [0,0,0],
                            [0,40,0],
                            [31.5,0,0],
                            [31.5,40,0],
                            [59,0,0],
                            [59,40,0]])

    retval, rotation_vector, trans_vec = cv2.solvePnP(measured_dots,camera_vision_dots,camera_formula,distortion_coefficients)

    print("rotvec")
    print(rotation_vector)
    
    print("transvec")
    print(trans_vec)

    rotation_matrix = np.zeros((3,3))

    cv2.Rodrigues(rotation_vector, rotation_matrix, jacobian=0)

    print('rotmat')
    print(rotation_matrix)

foo()
# Now calculate the inverse of the Homogeneous transform using the rotation matrix and
# the translation vector.

# AP = rotation_matrix * BP + APBorg

# Calculate the angles (roll pitch and yaw or euler angles) from the rotation matrix.
# Print in terminal your results and check the if the location and orientation of the camera
# is correct related to the defined “World Origin”.