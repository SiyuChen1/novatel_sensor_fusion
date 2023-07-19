def quaternion_multiply(q, r):
    """
    Perform quaternion multiply.

    The presence of the cross-product reveals that the quaternion product is not
    commutative in the general case. It means that q * r != r * q.
    https://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf page 6.
    Note: q is the new rotation and r is the previous rotation.

    :param q: q = q[0] + q[1] * i + q[2] * j + q[3] * k
    :param r: r = r[0] + r[1] * i + r[2] * j + r[3] * k
    :return: t = q * r
    """
    n0 = r[0] * q[0] - r[1] * q[1] - r[2] * q[2] - r[3] * q[3]
    n1 = r[0] * q[1] + r[1] * q[0] - r[2] * q[3] + r[3] * q[2]
    n2 = r[0] * q[2] + r[1] * q[3] + r[2] * q[0] - r[3] * q[1]
    n3 = r[0] * q[3] - r[1] * q[2] + r[2] * q[1] + r[3] * q[0]
    return [n0, n1, n2, n3]
