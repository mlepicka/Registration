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
	left_camera_points = mat(random.rand(4,3));
	left_camera_points[:,:] = [
[ 0.064664, 0.0278218, 0.403826],
[ 0.0352655, 0.0346349, 0.392449],
[ 0.0526255, -0.029333, 0.479768],
[ 0.0827106, 0.014077, 0.423877]]

#INFO: HomogMatrix:
#0.921238  -0.384639  -0.0580754  0.064664
#0.275862  0.540722  0.794682  0.0278218 
#-0.274263  -0.748112  0.604241  0.403826  
#INFO: HomogMatrix:
#0.614021  -0.786903  -0.0613407  0.0352655
#0.506171  0.332949  0.795573  0.0346349  
#-0.605615  -0.519547  0.602745  0.392449  
#INFO: HomogMatrix:
#-0.0346679  0.997356  -0.0638747  0.0526255
#-0.605056  0.0299231  0.79562  -0.029333
#0.795428  0.0662302  0.602419  0.479768  
#INFO: HomogMatrix:
#0.995367  0.0783413  -0.055736  0.0827106
#-0.00366462  0.6102  0.792239  0.014077
#0.0960751  -0.788365  0.60766  0.423877

	wide_camera_points = mat(random.rand(4,3));
	wide_camera_points[:,:] = [
[-0.0833481, -0.00252866, 0.408507],
[-0.0527038, -0.0113427, 0.403288],
[ -0.0808477, 0.0751885, 0.463953],
[ -0.103744, 0.015809, 0.421157]]

#INFO: HomogMatrix:
#-0.879495  0.475852  -0.00725115  -0.0833481
#-0.387125  -0.724201  -0.570672  -0.00252866
#-0.276807  -0.499096  0.821146  0.408507  
#INFO: HomogMatrix:
#-0.53277  0.846254  -0.0030918  -0.0527038
#-0.694363  -0.439228  -0.570034  -0.0113427
#-0.483751  -0.30155  0.821616  0.403288 
#INFO: HomogMatrix:
#-0.0661192  -0.997809  -0.00237442  -0.0808477
#0.820647  -0.0530257  -0.568969  0.0751885
#0.567597  -0.0395683  0.822355  0.463953
#INFO: HomogMatrix:
#-0.999685  0.0218899  -0.01228  -0.103744
#-0.0110601  -0.823387  -0.567372  0.015809
#-0.0225309  -0.567057  0.82337  0.421157 

	return (wide_camera_points,left_camera_points,4)

# MAIN
#(A,B,n) = generate_random_data()
(A,B,n) = calibration_data()

print "Points A"
print A
print ""

print "Points B"
print B
print ""

# recover the transformation
ret_R, ret_t = rigid_transform_3D(A, B);

A2 = (ret_R*A.T) + tile(ret_t, (1, n))
A2 = A2.T

# Find the error
err = A2 - B

err = multiply(err, err)
err = sum(err)
rmse = sqrt(err/n);

print "Computed rotation"
print ret_R
print ""

print "Computed translation"
print ret_t
print ""

print "RMSE:", rmse
print "If RMSE is near zero, the function is correct!"

