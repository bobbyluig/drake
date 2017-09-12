def cylinder_tensor(m, r, h):
    """
    Returns the non-zero inertial tensor values for a cylinder about the center
    of mass.
    :param m: The mass.
    :param r: The radius.
    :param h: The height.
    :return: (ixx, iyy, izz)
    """
    ixx = iyy = 1 / 12 * (3 * r ** 2 + h ** 2)
    izz = 1 / 2 * m * r ** 2
    return ixx, iyy, izz


m = 1
r = 0.1
h = 1
print(cylinder_tensor(m, r, h))
