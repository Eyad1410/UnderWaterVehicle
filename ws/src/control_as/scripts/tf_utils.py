import numpy
import math

class TF_Utils():
    def __init__(self):
        
        self._EPS = numpy.finfo(float).eps * 4.0

    def euler_from_quaternion(quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def quaternion_from_euler(roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[3] = cy * cp * cr + sy * sp * sr
        q[0] = cy * cp * sr - sy * sp * cr
        q[1] = sy * cp * sr + cy * sp * cr
        q[2] = sy * cp * cr - cy * sp * sr

        return q

    def pose_to_pq(self, msg):
        """Convert a C{geometry_msgs/Transform} into position/quaternion np arrays
        @param msg: ROS message to be converted
        @return:
        - p: position as a np.array
        - q: quaternion as a numpy array (order = [x,y,z,w])
        """
        p = numpy.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q = numpy.array([msg.pose.orientation.x, msg.pose.orientation.y,
                    msg.pose.orientation.z, msg.pose.orientation.w])
        return p, q

    def inverse_matrix(self, matrix):
        """Return inverse of square transformation matrix.

        >>> M0 = random_rotation_matrix()
        >>> M1 = inverse_matrix(M0.T)
        >>> numpy.allclose(M1, numpy.linalg.inv(M0.T))
        True
        >>> for size in range(1, 7):
        ...     M0 = numpy.random.rand(size, size)
        ...     M1 = inverse_matrix(M0)
        ...     if not numpy.allclose(M1, numpy.linalg.inv(M0)): print size

        """
        return numpy.linalg.inv(matrix)

    def concatenate_matrices(self, *matrices):
        """Return concatenation of series of transformation matrices.

        >>> M = numpy.random.rand(16).reshape((4, 4)) - 0.5
        >>> numpy.allclose(M, concatenate_matrices(M))
        True
        >>> numpy.allclose(numpy.dot(M, M.T), concatenate_matrices(M, M.T))
        True

        """
        M = numpy.identity(4)
        for i in matrices:
            M = numpy.dot(M, i)
        return M

    def translation_matrix(self, direction):
        """Return matrix to translate by direction vector.

        >>> v = numpy.random.random(3) - 0.5
        >>> numpy.allclose(v, translation_matrix(v)[:3, 3])
        True

        """
        M = numpy.identity(4)
        M[:3, 3] = direction[:3]
        return M

    def quaternion_matrix(self, quaternion):
        """Return homogeneous rotation matrix from quaternion.

        >>> R = quaternion_matrix([0.06146124, 0, 0, 0.99810947])
        >>> numpy.allclose(R, rotation_matrix(0.123, (1, 0, 0)))
        True

        """
        q = numpy.array(quaternion[:4], dtype=numpy.float64, copy=True)
        nq = numpy.dot(q, q)
        if nq < self._EPS:
            return numpy.identity(4)
        q *= math.sqrt(2.0 / nq)
        q = numpy.outer(q, q)
        return numpy.array((
            (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
            (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
            (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
            (                0.0,                 0.0,                 0.0, 1.0)
            ), dtype=numpy.float64)

    def translation_from_matrix(self, matrix):
        """Return translation vector from translation matrix.

        >>> v0 = numpy.random.random(3) - 0.5
        >>> v1 = translation_from_matrix(translation_matrix(v0))
        >>> numpy.allclose(v0, v1)
        True

        """
        return numpy.array(matrix, copy=False)[:3, 3].copy()

    def quaternion_from_matrix(self, matrix):
        """Return quaternion from rotation matrix.

        >>> R = rotation_matrix(0.123, (1, 2, 3))
        >>> q = quaternion_from_matrix(R)
        >>> numpy.allclose(q, [0.0164262, 0.0328524, 0.0492786, 0.9981095])
        True

        """
        q = numpy.empty((4, ), dtype=numpy.float64)
        M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:4, :4]
        t = numpy.trace(M)
        if t > M[3, 3]:
            q[3] = t
            q[2] = M[1, 0] - M[0, 1]
            q[1] = M[0, 2] - M[2, 0]
            q[0] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 0, 1, 2
            if M[1, 1] > M[0, 0]:
                i, j, k = 1, 2, 0
            if M[2, 2] > M[i, i]:
                i, j, k = 2, 0, 1
            t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
            q[i] = t
            q[j] = M[i, j] + M[j, i]
            q[k] = M[k, i] + M[i, k]
            q[3] = M[k, j] - M[j, k]
        q *= 0.5 / math.sqrt(t * M[3, 3])
        return q