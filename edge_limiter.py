def guess_edge_limiter(cnt):
    MAX_NO_LIMITER = 50
    # cnt = sum([len(obstacle) for obstacle in robot_data['obstacles']])
    factor = int(cnt / MAX_NO_LIMITER)
    if 0 <= factor < 2:
        return None
    if 2 <= factor < 4:
        return 'l1_norm_half'
    if 4 <= factor <= 8:
        return 'l1_norm_quarter'
    return 'l1_norm_half_quarter'


def l1_norm(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# Returns True if forbidden
def l1_norm_half_edge_limiter(edge, robot_data):
    max_dist_allowed = l1_norm(robot_data['finish'], robot_data['start']) / 2
    edge_l1_norm = l1_norm(edge[0], edge[1])
    return edge_l1_norm > max_dist_allowed


# Returns True if forbidden
def l1_norm_quarter_edge_limiter(edge, robot_data):
    max_dist_allowed = l1_norm(robot_data['finish'], robot_data['start']) / 4
    edge_l1_norm = l1_norm(edge[0], edge[1])
    return edge_l1_norm > max_dist_allowed


# Returns True if forbidden
def l1_norm_half_quarter_edge_limiter(edge, robot_data):
    max_dist_allowed = l1_norm(robot_data['finish'], robot_data['start']) / 8
    edge_l1_norm = l1_norm(edge[0], edge[1])
    return edge_l1_norm > max_dist_allowed


def always_forbid(edge, robot_data):
    return False


def edge_limiter_factory(name):
    if name == 'l1_norm_half':
        return l1_norm_half_edge_limiter
    if name == 'l1_norm_quarter':
        return l1_norm_quarter_edge_limiter
    if name == 'l1_norm_half_quarter':
        return l1_norm_half_quarter_edge_limiter
    return always_forbid


if __name__ == '__main__':
    import json
    def read_json(name):
        with open(name) as json_file:
            robot_data = json.load(json_file)
        return robot_data

    assert (guess_edge_limiter(read_json('tests/robot-test-5.json')) is None)
    assert (guess_edge_limiter(read_json('tests/robot-test-15.json')) is None)
    assert (guess_edge_limiter(read_json('tests/robot-test-18.json')) is None)
    assert (guess_edge_limiter(read_json('tests/robot-test-20.json')) is None)
    assert (guess_edge_limiter(read_json('tests/robot-test-30.json')) == 'l1_norm_half')
    assert(guess_edge_limiter(read_json('tests/robot-test-50.json')) == 'l1_norm_quarter')
    assert (guess_edge_limiter(read_json('tests/robot-test-100.json')) == 'l1_norm_quarter')
    assert(l1_norm_half_edge_limiter(
        ([12.913415113681303, 4.046380970957697], [43.25402915616585, 45.87877553738756]),
        {
            'start': (40, 30),
            'finish': (70, 120),
        }
    ))
    assert(l1_norm_half_edge_limiter is edge_limiter_factory('l1_norm_half'))
    assert(l1_norm_quarter_edge_limiter is edge_limiter_factory('l1_norm_quarter'))
    assert(l1_norm_half_quarter_edge_limiter is edge_limiter_factory('l1_norm_half_quarter'))