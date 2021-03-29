import numpy as np


class NoInterceptionException(Exception):
    pass


def check_is_interception_exists(numerator, denominator):
    return (numerator > 0 and denominator < 0) or \
           (numerator < 0 and denominator > 0) or \
           (numerator > denominator and numerator > 0) or \
           (numerator < denominator and numerator < 0)


# https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection (Given two points on each line segment)
# generalized approach for vectors which calculates exact point
def calc_interception(line1, line2):
    v1, v2 = line1
    v3, v4 = line2
    t_denominator = np.linalg.det(np.matrix([v2 - v1, v4 - v3]))
    if t_denominator == 0:
        raise NoInterceptionException
    t_numerator = np.linalg.det(np.matrix([v3 - v1, v4 - v3]))
    if check_is_interception_exists(t_numerator, t_denominator):
        raise NoInterceptionException
    u_denominator = t_denominator
    u_numerator = np.linalg.det(np.matrix([v1 - v2, v3 - v1]))
    if check_is_interception_exists(u_numerator, u_denominator):
        raise NoInterceptionException
    t = t_numerator / t_denominator
    is_edge = t == 0 or t == 1 or u_numerator == 0 or u_numerator == u_denominator
    point = v1 + t * (v2 - v1)
    return point, is_edge


def check_is_point_on_line(point, line):
    v = np.array(point)
    v1 = np.array(line[0])
    v2 = np.array(line[1])
    n = v - v1
    d = v2 - v1
    if np.allclose(d[0], 0) or np.allclose(d[1], 0):
        return False
    if np.allclose(n[0], 0) and np.allclose(n[1], 0):
        return True
    t = n[0] / d[0]
    return 0 <= t <= 1 and np.allclose(t, n[1] / d[1])


if __name__ == '__main__':
    assert (not check_is_point_on_line((7, 4), [[2, 4], [3, 3]]))
    assert (check_is_point_on_line((4.666666666666668, 4.666666666666668), [[5, 4], [4, 6]]))
    assert (check_is_point_on_line((5, 4), [[5, 4], [4, 6]]))
    assert (check_is_point_on_line((4, 6), [[5, 4], [4, 6]]))
    assert (not check_is_point_on_line((3, 3), [[5, 4], [4, 6]]))
    assert (not check_is_point_on_line((3, 3), [[1, 1], [2, 2]]))

    try:
        calc_interception(
            np.array([[13, 3], [5, 10]]),
            np.array([[50, 5], [100, 5]]),
        )
        assert (False)
    except NoInterceptionException:
        assert (True)

    assert (np.allclose(calc_interception(
        np.array([[1, 1], [5, 5]]),
        np.array([[1, 5], [5, 1]]),
    )[0], np.array([3, 3])))

    try:
        calc_interception(
            np.array([[1, 1], [5, 5]]),
            np.array([[1, 5], [2, 4]]),
        )
        assert (False)
    except NoInterceptionException:
        assert (True)

    assert (calc_interception(
        np.array([[7, 4], [10, 10]]),
        np.array([[7, 4], [5, 4]]),
    )[1])
