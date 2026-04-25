import numpy as np

from numpy.testing import assert_almost_equal

# ========================================================================================
# ====================================== PART 1 ==========================================
# ========================================================================================

def is_valid_rotation_matrix(matrix: np.ndarray) -> bool:
    """
    Check if a matrix is a valid 2D or 3D rotation matrix.

    A valid rotation matrix must be:
    - Square (2x2 or 3x3)
    - Orthogonal (R @ R.T = I)
    - Have a determinant of 1

    Parameters
    ----------
    matrix : np.ndarray
        The matrix to check.

    Returns
    -------
    bool
        True if the matrix is a valid rotation matrix, False otherwise.

    """
    is_valid = True

    # ================================== YOUR CODE HERE ==================================

    (shape_0, shape_1) = matrix.shape
    if (not shape_0 == 3) and (not shape_0 == 2):
        return False

    if not shape_0 == shape_1:
        return False

    if not np.allclose(matrix @ matrix.T, np.eye(shape_0)):
        return False
    
    if not np.allclose(np.linalg.det(matrix),1):
        print('Det', np.linalg.det(matrix))
        return False

    
    # ====================================================================================

    return True


# ========================================================================================
# ====================================== PART 2 ==========================================
# ========================================================================================


def euler_angles_to_rotation_matrix(euler_angles: np.ndarray) -> np.ndarray:
    """
    Convert Euler angles to a 3x3 rotation matrix using an intrinsic z-y'-x" convention.

    Parameters
    ----------
    euler_angles : np.ndarray
        Array of shape (3,) containing [yaw, pitch, roll] in degrees.

    Returns
    -------
    np.ndarray
        3x3 rotation matrix.

    """
    rotation_matrix = None

    # ================================== YOUR CODE HERE ==================================
    c = np.cos(np.deg2rad(euler_angles[2]))
    s = np.sin(np.deg2rad(euler_angles[2]))
    R_X = np.array([[1, 0, 0],
                    [0, c, -s],
                    [0, s, c]])

    c = np.cos(np.deg2rad(euler_angles[1]))
    s = np.sin(np.deg2rad(euler_angles[1]))
    R_Y = np.array([[c, 0, s],
                    [0, 1, 0],
                    [-s, 0, c]])

    c = np.cos(np.deg2rad(euler_angles[0]))
    s = np.sin(np.deg2rad(euler_angles[0]))
    R_Z = np.array([[c, -s, 0],
                    [s, c, 0],
                    [0, 0, 1]])
    
    rotation_matrix = R_Z @ R_Y @ R_X

    # ====================================================================================

    return rotation_matrix


def rotation_matrix_to_euler_angles(matrix: np.ndarray) -> np.ndarray:
    """
    Convert a 3x3 rotation matrix to Euler angles using an intrinsic z-y'-x" convention.

    Parameters
    ----------
    matrix : np.ndarray
        3x3 rotation matrix.

    Returns
    -------
    np.ndarray
        Array of shape (3,) containing [yaw, pitch, roll] in degrees.

    """
    euler_angles = None

    # ================================== YOUR CODE HERE ==================================

    euler_angles = np.zeros(3)
    euler_angles[0] = np.rad2deg(np.arctan2(matrix[1,0], matrix[0,0]))
    euler_angles[1] = np.rad2deg(np.arcsin(-matrix[2,0]))
    euler_angles[2] = np.rad2deg(np.arctan2(matrix[2,1], matrix[2,2]))


    # ====================================================================================

    return euler_angles


# ========================================================================================
# ====================================== PART 3 ==========================================
# ========================================================================================


def quaternion_to_rotation_matrix(quaternion: np.ndarray) -> np.ndarray:
    """
    Convert a unit quaternion to a 3x3 rotation matrix.

    Parameters
    ----------
    quaternion : np.ndarray
        Array of shape (4,) containing [w, x, y, z] where w is the scalar part.

    Returns
    -------
    np.ndarray
        3x3 rotation matrix.

    """
    rotation_matrix = None

    # ================================== YOUR CODE HERE ==================================
    w = quaternion[0]
    x = quaternion[1]
    y = quaternion[2]
    z = quaternion[3]
    rotation_matrix = np.array([[w**2 + x**2 - y**2 - z**2, 2*(x*y - w*z), 2*(x*z + w*y)],
                                [2*(x*y + w*z), w**2 - x**2 + y**2 - z**2, 2*(y*z - w*x)],
                                [2*(x*z - w*y), 2*(y*z + w*x), w**2 - x**2 - y**2 + z**2]])

    # ====================================================================================

    return rotation_matrix


def rotation_matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    """
    Convert a 3x3 rotation matrix to a unit quaternion.

    Parameters
    ----------
    matrix : np.ndarray
        3x3 rotation matrix.

    Returns
    -------
    np.ndarray
        Array of shape (4,) containing [w, x, y, z] where w is the scalar part.

    """
    quaternion = None

    # ================================== YOUR CODE HERE ==================================

    quaternion = np.zeros(4)
    T = np.trace(matrix)
    i = np.argmax([T, matrix[0,0], matrix[1,1], matrix[2,2]])
    if i == 0:
        p_0 = np.sqrt(1 + T)

        quaternion[0] = p_0/2
        quaternion[1] = (matrix[2,1] - matrix[1,2])/(2*p_0)
        quaternion[2] = (matrix[0,2] - matrix[2,0])/(2*p_0)
        quaternion[3] = (matrix[1,0] - matrix[0,1])/(2*p_0)
    else:
        p_i = np.sqrt(1 + 2*matrix[i-1,i-1] - T)
        quaternion[i] = p_i/2
        for j in range(1,4):
            k = (i+j) % 4
            if k == 0:
                quaternion[k] = (matrix[(i+2)%3, (i+1)%3] - matrix[(i+1)%3, (i+2)%3])/(2*p_i)
            else:
                quaternion[k] = (matrix[i-1, k-1] + matrix[k-1, i-1])/(2*p_i)



    # ====================================================================================

    return quaternion