#!/usr/bin/env python
import numpy as np

def lab3(thetas):
    q = np.ndarray((3,8))
    w = np.ndarray((3,7))
    
    q[0:3,0] = [0.0635, 0.2598, 0.1188]
    q[0:3,1] = [0.1106, 0.3116, 0.3885]
    q[0:3,2] = [0.1827, 0.3838, 0.3881]
    q[0:3,3] = [0.3682, 0.5684, 0.3181]
    q[0:3,4] = [0.4417, 0.6420, 0.3177]
    q[0:3,5] = [0.6332, 0.8337, 0.3067]
    q[0:3,6] = [0.7152, 0.9158, 0.3063]
    q[0:3,7] = [0.7957, 0.9965, 0.3058]

    w[0:3,0] = [-0.0059,  0.0113,  0.9999]
    w[0:3,1] = [-0.7077,  0.7065, -0.0122]
    w[0:3,2] = [ 0.7065,  0.7077, -0.0038]
    w[0:3,3] = [-0.7077,  0.7065, -0.0122]
    w[0:3,4] = [ 0.7065,  0.7077, -0.0038]
    w[0:3,5] = [-0.7077,  0.7065, -0.0122]
    w[0:3,6] = [ 0.7065,  0.7077, -0.0038]

    R = np.array([[0.0076, 0.0001, -1.0000],
                          [-0.7040, 0.7102, -0.0053],
                          [0.7102, 0.7040, 0.0055]]).T

    # YOUR CODE HERE
    g0 = np.hstack((R, q[0:3,7].reshape(3,1)))
    g0 = np.vstack((g0, np.zeros(4)))
    g0[3,3] = 1

    
    # write the twist xi_i for each joint in the manipulator
    xi = np.ndarray((6, 7))
    for i in range(7): 
        xi[:,i] = get_xi(w[:,i], q[:,i])
    
    g = prod_exp(xi, thetas)

    return g.dot(g0)


def get_xi(w, q):
    v = -np.cross(w, q)
    return np.hstack((v, w))


def skew_3d(omega):
    """
    Prelab Part (a)

    Converts a rotation vector in 3D to its corresponding skew-symmetric matrix.
    
    Args:
    omega - (3,) ndarray: the rotation vector
    
    Returns:
    omega_hat - (3,3) ndarray: the corresponding skew symmetric matrix
    """
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')
    
    w1 = omega[0]
    w2 = omega[1]
    w3 = omega[2]
    return np.array([[0, -w3, w2],
                     [w3, 0, -w1],
                     [-w2, w1, 0]])

def rotation_3d(omega, theta):
    """
    Prelab Part (b)

    Computes a 3D rotation matrix given a rotation axis and angle of rotation.
    
    Args:
    omega - (3,) ndarray: the axis of rotation
    theta: the angle of rotation
    
    Returns:
    rot - (3,3) ndarray: the resulting rotation matrix
    """
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')
    
    norm = np.linalg.norm(omega)
    omega_hat = skew_3d(omega) / norm
    I = np.identity(3)
    theta *= norm
    return I + omega_hat*np.sin(theta) + omega_hat.dot(omega_hat) * (1-np.cos(theta))

def hat_3d(xi):
    """
    Prelab Part (c)

    Converts a 3D twist to its corresponding 4x4 matrix representation
    
    Args:
    xi - (6,) ndarray: the 3D twist
    
    Returns:
    xi_hat - (4,4) ndarray: the corresponding 4x4 matrix
    """
    if not xi.shape == (6,):
        raise TypeError('xi must be a 6-vector')
    
    v = xi[:3]
    omega = xi[3:]
    omega_hat = skew_3d(omega)
    return np.vstack((np.hstack((omega_hat, v.reshape(3, 1))), np.zeros(4)))

def homog_3d(xi, theta):
    """
    Prelab Part (d)

    Computes a 4x4 homogeneous transformation matrix given a 3D twist and a 
    joint displacement.
    
    Args:
    xi - (6,) ndarray: the 3D twist
    theta: the joint displacement
    Returns:
    g - (4,4) ndarary: the resulting homogeneous transformation matrix
    """
    if not xi.shape == (6,):
        raise TypeError('xi must be a 6-vector')

    v = xi[:3]
    w = xi[3:]
    I = np.eye(3)
    if np.allclose(w, 0):
        # Pure translation
        R = np.eye(3)
        p = v*theta
    else:
        # Translation and rotation
        R = rotation_3d(w, theta)

        norm = np.linalg.norm(w)
        I = np.identity(3)
        w_v = skew_3d(w).dot(v)
        p = 1/norm/norm * ((I-R).dot(w_v) + w * w.dot(v) * theta)
    g = np.eye(4)
    g[:3, :3] = R
    g[:3, 3] = p
    return g


def prod_exp(xi, theta):
    """
    Prelab Part (e)
    
    Computes the product of exponentials for a kinematic chain, given 
    the twists and displacements for each joint.
    
    Args:
    xi - (6, N) ndarray: the twists for each joint
    theta - (N,) ndarray: the displacement of each joint
    
    Returns:
    g - (4,4) ndarray: the resulting homogeneous transformation matrix
    """
    if not xi.shape[0] == 6:
        raise TypeError('xi must be a 6xN')
    if not xi.shape[1] == theta.shape[0]:
        raise TypeError('there must be the same number of twists as joint angles.')

    g = np.eye(4)
    for i in range(xi.shape[1]):
        x = xi[:, i]
        t = theta[i]
        g = g.dot(homog_3d(x, t))

    return g

# Deleted the node stuff from lab3 because it's not needed anymore
