#!/usr/bin/env python  

from numpy import *
from math import sqrt

# Input: expects Nx3 matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector

def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    N = A.shape[0]; # total points

    centroid_A = mean(A, axis=0)
    centroid_B = mean(B, axis=0)
    
    # centre the points
    AA = A - tile(centroid_A, (N, 1))
    BB = B - tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    H = transpose(AA) * BB

    U, S, Vt = linalg.svd(H)

    R = Vt.T * U.T

    # special reflection case
    if linalg.det(R) < 0:
       print "Reflection detected"
       Vt[2,:] *= -1
       R = Vt.T * U.T

    t = -R*centroid_A.T + centroid_B.T

    return R, t

# Generate random data - for testin purposes.
def generate_random_data():
	# Random rotation and translation
	R = mat(random.rand(3,3));
	t = mat(random.rand(3,1));

	print "Random rotation"
	print R
	print ""

	print "Random translation"
	print t
	print ""

	# make R a proper rotation matrix, force orthonormal
	U, S, Vt = linalg.svd(R)
	R = U*Vt

	# remove reflection
	if linalg.det(R) < 0:
	   Vt[2,:] *= -1
	   R = U*Vt

	# number of points
	n = 10

	A = mat(random.rand(n,3));
	B = R*A.T + tile(t, (1, n));
	B = B.T;

	return (A,B,n)
	

# Calibration data.
def calibration_data():
	wide_camera_points = [[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
	left_camera_points = [
  [-0.25044866060156884, -0.9546185621785377, -0.018138013520308777],
  [-0.24596613638511766, -0.5327159784677284, -0.011456858189483443],
  [0.13391378309697685, -0.538782060922628, -0.010241241006622825],
  [0.1329034831779668, -0.9629397324626949, -0.013050991573856863]]

	return (wide_camera_points,left_camera_points,4)

# MAIN
#(A,B,n) = generate_random_data()
(A,B,n) = calibration_data()


# recover the transformation
ret_R, ret_t = rigid_transform_3D(A, B);

A2 = (ret_R*A.T) + tile(ret_t, (1, n))
A2 = A2.T

# Find the error
err = A2 - B

err = multiply(err, err)
err = sum(err)
rmse = sqrt(err/n);

print "Points A"
print A
print ""

print "Points B"
print B
print ""

print "Computed rotation"
print ret_R
print ""

print "Computed translation"
print ret_t
print ""

print "RMSE:", rmse
print "If RMSE is near zero, the function is correct!"

