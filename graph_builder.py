import numpy as np
from graph import Graph, VertexNotFoundException
from interception import calc_interception, check_is_point_on_line, NoInterceptionException


def convert_points_polygon_to_lines(points):
    lines = []
    for pindex in range(len(points)):
        obstacle_line = np.array([points[pindex - 1], points[pindex]])
        lines.append(obstacle_line)
    return lines


def find_interceptions(line, obstacles, calc_interception_fn=calc_interception):
    interceptions = []
    for obstacle in obstacles:
        for obstacle_line in obstacle:
            try:
                new_point, is_edge = calc_interception_fn(line, obstacle_line)
                if not is_edge:
                    interceptions.append([tuple(new_point), obstacle_line])
            except NoInterceptionException:
                continue
    return interceptions


def check_is_intercept(line, obstacles):
    for obstacle in obstacles:
        for obstacle_line in obstacle:
            try:
                new_point, is_edge = calc_interception(line, obstacle_line)
                if not is_edge:
                    return True
            except NoInterceptionException:
                continue
    return False


def find_obstacle_index(location, obstacles):
    for obstacle_index, obstacle in enumerate(obstacles):
        for obstacle_line in obstacle:
            if check_is_point_on_line(location, obstacle_line):
                return obstacle_index
    return None


def find_closest_point(base, points):
    base_a = np.array(base)
    min_norm = float('inf')
    min_index = -1
    for index, point in enumerate(points):
        norm = np.linalg.norm(base_a - np.array(point))
        if norm < min_norm:
            min_norm = norm
            min_index = index
    return points[min_index]


def build_graph_for_robot_data(robot_data):
    finish_location = tuple(robot_data['finish'])
    iter_locations = [tuple(robot_data['start']),
                      *[tuple(item) for sublist in robot_data['obstacles'] for item in sublist]]
    obstacles = [convert_points_polygon_to_lines(obstacle) for obstacle in robot_data['obstacles']]
    locations = set(iter_locations)
    locations.add(finish_location)

    calc_interception_cache = dict()
    def cached_calc_interception(line1, line2):
        key = (line1.item(0), line1.item(1), line1.item(2), line1.item(3),
               line2.item(0), line2.item(1), line2.item(2), line2.item(3))
        if key in calc_interception_cache:
            return calc_interception_cache[key]
        calc_interception_cache[key] = calc_interception(line1, line2)
        return calc_interception_cache[key]

    def cached_find_interceptions(from_location, to_location):
        return find_interceptions(
            np.array([from_location, to_location]), obstacles,
            calc_interception_fn=cached_calc_interception,
        )
    # find_interceptions_cache = dict()
    #
    # def cached_find_interceptions(from_location, to_location):
    #     key = (from_location, to_location)
    #     if key in find_interceptions_cache:
    #         return find_interceptions_cache[key]
    #     find_interceptions_cache[key] = find_interceptions(
    #         np.array([from_location, to_location]), obstacles,
    #         calc_interception_fn=cached_calc_interception,
    #     )
    #     return find_interceptions_cache[key]

    find_obstacle_index_cache = dict()

    def cached_find_obstacle_index(location):
        if location in find_obstacle_index_cache:
            return find_obstacle_index_cache[location]
        find_obstacle_index_cache[location] = find_obstacle_index(location, obstacles)
        return find_obstacle_index_cache[location]

    while True:
        found_locations = []
        for v in iter_locations:
            interceptions = cached_find_interceptions(v, finish_location)
            if interceptions:
                new_point = find_closest_point(v, [interception[0] for interception in interceptions])
                if new_point not in locations:
                    found_locations.append(new_point)
                    locations.add(new_point)
        if not len(found_locations):
            break
        iter_locations = found_locations

    g = Graph()
    for location in locations:
        g.add_vertex(location)

    for obstacle in obstacles:
        for line in obstacle:
            g.add_edge(tuple(line[0]), tuple(line[1]))

    for location_from_index, location_from in enumerate(locations):
        for location_to_index, location_to in enumerate(locations):
            if location_to_index > location_from_index and not np.allclose(location_from, location_to):
                obstacle_from_index = cached_find_obstacle_index(location_from)
                obstacle_to_index = cached_find_obstacle_index(location_to)
                if obstacle_from_index is not None and obstacle_from_index == obstacle_to_index:
                    continue
                if not cached_find_interceptions(location_from, location_to):
                    g.add_edge(location_from, location_to)

    return g


if __name__ == '__main__':
    # g = build_graph_for_robot_data({
    #     'start': [11, 10],
    #     'finish': [3.14, 3.14],
    #     'obstacles': [
    #         # [[1, 8], [1, 11], [5, 11], [3, 7]],
    #         # [[6, 9], [5, 12], [7, 13]],
    #         [[8, 7], [7, 11], [9, 12]],
    #         # [[10, 10], [10, 12], [12, 12]],
    #         # [[2, 3], [1, 7], [3, 6], [3, 3]],
    #         # [[4, 6], [4, 8], [7, 8], [7, 6]],
    #         [[9, 4], [9, 10], [11, 4]],
    #         # [[12, 6], [10, 9], [13, 10]],
    #         # [[1, 0], [0, 1], [0, 2], [2, 2]],
    #         # [[2, 1], [6, 2], [7, -1]],
    #         [[4, 2], [4, 5], [5, 5], [5, 2]],
    #         [[6, 4], [6, 5], [8, 5], [8, 4]],
    #         [[6, 3], [11, 3], [11, -2]],
    #         # [[12, 2], [12, 5], [13, 6], [13, 1]]
    # ]})
    # print(g.neighbors((4, 2)))
    # exit()

    g = build_graph_for_robot_data({
        'start': [0, 0],
        'finish': [10, 10],
        'obstacles': [
            [[2, 2], [2, 4], [3, 3]],
            [[5, 4], [4, 6], [6, 5], [7, 4]],
            [[4, 4], [3.5, 4], [2, 6], [3, 7]],
            [[20, 1], [10, 2], [9, 4], [6, 10]],
        ]
    })

    assert (find_closest_point((0, 0), [[2, 2], [2, 4], [3, 3]]) == [2, 2])

    g = build_graph_for_robot_data({
        'start': [0, 0],
        'finish': [10, 10],
        'obstacles': [[[2, 2], [2, 4], [3, 3]], [[5, 4], [4, 6], [6, 5], [7, 4]]]
    })
    assert (len(g.vertices()) == 16)
    assert (sorted(g.neighbors((10, 10))) == [
        (4, 6),
        # (4.4, 5.800000000000001),
        # (5.333333333333334, 5.333333333333334),
        # (5.882352941176471, 5.0588235294117645),
        (6, 5)
    ])
    assert (g.cost((10, 10), (4, 6)) == np.sqrt((10 - 4) ** 2 + (10 - 6) ** 2))
    try:
        g.cost((10, 10), (5, 5))
        assert (False)
    except VertexNotFoundException:
        assert (True)
