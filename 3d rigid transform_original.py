import numpy as np
from numpy import *
from math import sqrt
import cv2

# Input: expects Nx3 matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector
# Input matrix format: [x1,x2,x3;y1,y2,y3;z1,z2,z3] --->row vector matrix
#Transpose of matrix: [x1,y1,z1;x2,y2,z2;x3,y3,z3]

ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
end_header
'''
def write_ply(fn, verts):
    verts = verts.reshape(-1, 3)
    
    verts = np.hstack([verts])
    with open(fn, 'wb') as f:
        f.write((ply_header%dict(vert_num=len(verts))).encode('utf-8'))
        
        np.savetxt(f, verts, fmt="%f %f %f")
        
def rigid_transform_3D(after_rotation, before_rotation):
    assert len(after_rotation) == len(before_rotation)

    N = after_rotation.shape[0]; # total points

    centroid_A = mean(after_rotation, axis=0)
    centroid_B = mean(before_rotation, axis=0)
    
    # centre the points
    AA = after_rotation - tile(centroid_A, (N, 1))
    BB = before_rotation - tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    H = transpose(AA) * BB
    print("H matrix:",H.shape)

    U, S, Vt = linalg.svd(H)
    print("original scaling",S.shape)
    print("U value",U.shape)
    print("Vt value",Vt.shape)

    R = Vt.T * U.T
    S=S.T

    H1=after_rotation*before_rotation
    U1,S1,V1=linalg.svd(H1)
    print("U1: ",U1," S1: ",S1," V1: ",V1)

    # special reflection case
    if linalg.det(R) < 0:
       print ("Reflection detected")
       Vt[2,:] *= -1
       R = Vt.T * U.T

    t = -R*centroid_A.T + centroid_B.T

    print("t:\n")
    print (t)

    return R, t, S

# number of points
n = 3

#Input two matrices 
after_rotation = np.mat('1.5,2.5,3;1,3.5,4;0,0,0')
print("after_rotation matrix size:",after_rotation.shape)

after_rotation=after_rotation.T
before_rotation = np.mat('2,7,8;3,5,6;0,0,0')

before_rotation = before_rotation.T;

# recover the transformation
ret_R, ret_t, ret_S = rigid_transform_3D(after_rotation, before_rotation)

A2 = (ret_R*after_rotation.T) + tile(ret_t, (1, n))
A2 = A2.T

# Find the error
err = A2 - before_rotation

err = multiply(err, err)
err = sum(err)
rmse = sqrt(err/n);

print ("Points A")
print (after_rotation)
print ("")

print ("Points B")
print (before_rotation)
print ("")

print ("Rotation")
print (ret_R)
print ("")

print ("Translation")
print (ret_t)
print ("")

print("Scaling matrix S")
print (ret_S)
print ("")

print ("Converting A to B by applying transformation matrix:")
print (A2)
print("")
print ("RMSE:", rmse)
print ("If RMSE is near zero, the function is correct!")

C=np.mat('5,6,4;7,8,10;0,0,0')
C=C.T

answer=(ret_R*C.T)+tile(ret_t,(1,n))
answer=answer.T

print("C:")
print(answer)


# write files 
write_ply('A.ply', after_rotation)
write_ply('B.ply',before_rotation)
write_ply('transformed_matrix.ply',A2)
print ('finished writing')
