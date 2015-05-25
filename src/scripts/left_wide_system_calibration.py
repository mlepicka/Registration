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
	

# Compute Euler angles from rotation matrix
def euler_from_matrix(matrix):
        """Return Euler angles (syxz) from rotation matrix for specified axis sequence.
        :Author:
          `Christoph Gohlke <http://www.lfd.uci.edu/~gohlke/>`_

        full library with coplete set of euler triplets (combinations of  s/r x-y-z) at
            http://www.lfd.uci.edu/~gohlke/code/transformations.py.html

        Note that many Euler angle triplets can describe one matrix.
        """
        # epsilon for testing whether a number is close to zero
        _EPS = finfo(float).eps * 5.0

        # axis sequences for Euler angles
        _NEXT_AXIS = [1, 2, 0, 1]
        firstaxis, parity, repetition, frame = (1, 1, 0, 0) # ''

        i = firstaxis
        j = _NEXT_AXIS[i+parity]
        k = _NEXT_AXIS[i-parity+1]

        M = array(matrix, dtype='float', copy=False)[:3, :3]
        if repetition:
            sy = sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
            if sy > _EPS:
                ax = arctan2( M[i, j],  M[i, k])
                ay = arctan2( sy,       M[i, i])
                az = arctan2( M[j, i], -M[k, i])
            else:
                ax = arctan2(-M[j, k],  M[j, j])
                ay = arctan2( sy,       M[i, i])
                az = 0.0
        else:
            cy = sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
            if cy > _EPS:
                ax = arctan2( M[k, j],  M[k, k])
                ay = arctan2(-M[k, i],  cy)
                az = arctan2( M[j, i],  M[i, i])
            else:
                ax = arctan2(-M[j, k],  M[j, j])
                ay = arctan2(-M[k, i],  cy)
                az = 0.0

        if parity:
            ax, ay, az = -ax, -ay, -az
        if frame:
            ax, az = az, ax
        return ax, ay, az

# Calibration data.
def calibration_data():
	left_camera_points = mat(random.rand(4,3));
	left_camera_points[:,:] = [
[ 0.1240659043011733, 0.02111554433142932, 0.6358149650660975],
[ 0.09699606347538577, 0.04173273787501437, 0.6057394422679113],
[ 0.05289831494795517, 0.05195238560131506, 0.5886741486384437],
[ 0.07893827757776968, -0.04399942621728885, 0.7196514343912929]]

	wide_camera_points = mat(random.rand(4,3));
	wide_camera_points[:,:] = [
[ -0.1556163314972513, 0.0237134379287295, 0.6317350663954315],
[ -0.1250221670344269, -0.003792988368899017, 0.6127610644345503],
[ -0.07905572416563864, -0.01701407271361409, 0.604931709023467],
[ -0.1212715119184913, 0.1127827749742727, 0.6959291865394635]]

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

# TO NIE RPY!!
# (ax, ay, az) = euler_from_matrix(ret_R)
#
#print "Euler angles"
#print ax, ay, az
#print ""

