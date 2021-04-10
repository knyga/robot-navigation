import numpy as np
from edge_limiter import edge_limiter_factory, guess_edge_limiter
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


def find_obstacle_index(location, obstacle_paths, obstacles):
    for obstacle_index, obstacle in enumerate(obstacles):
        if location in obstacle:
            return obstacle_index
    for obstacle_index, obstacle_path in enumerate(obstacle_paths):
        for obstacle_line in obstacle_path:
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


def build_graph_for_robot_data(robot_data, intermediate_discovery_mode=None, edge_limiter_name=None, is_report=False):
    finish_location = tuple(robot_data['finish'])
    iter_locations = [tuple(robot_data['start']),
                      *[tuple(item) for sublist in robot_data['obstacles'] for item in sublist]]
    obstacles_paths = [convert_points_polygon_to_lines(obstacle) for obstacle in robot_data['obstacles']]
    obstacles = [{tuple(point) for point in obstacle} for obstacle in robot_data['obstacles']]
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
            np.array([from_location, to_location]), obstacles_paths,
            calc_interception_fn=cached_calc_interception,
        )

    find_obstacle_index_cache = dict()

    def cached_find_obstacle_index(location):
        if location in find_obstacle_index_cache:
            return find_obstacle_index_cache[location]
        find_obstacle_index_cache[location] = find_obstacle_index(location, obstacles_paths, obstacles)
        return find_obstacle_index_cache[location]

    def check_is_line_on_obstacle(line):
        loc1, loc2 = line
        obstacle_1_index = cached_find_obstacle_index(loc1)
        obstacle_2_index = cached_find_obstacle_index(loc2)
        return obstacle_1_index is not None and obstacle_1_index == obstacle_2_index

    if intermediate_discovery_mode == 'ray':
        while True:
            found_locations = []
            for v in iter_locations:
                interceptions = cached_find_interceptions(v, finish_location)
                if interceptions:
                    new_point = find_closest_point(v, [interception[0] for interception in interceptions])
                    if new_point not in locations and not check_is_line_on_obstacle((v, new_point)):
                        found_locations.append(new_point)
                        locations.add(new_point)
            if not len(found_locations):
                break
            iter_locations = found_locations
    if intermediate_discovery_mode == 'full':
        is_continue = True
        while is_continue:
            MAX_CNT = 150
            edge_limiter = edge_limiter_factory(guess_edge_limiter(len(locations)))
            found_locations = []
            if is_report:
                print('edge limiter name', guess_edge_limiter(len(locations)))
                print('vertices count', len(locations))
            for location_from_index, location_from in enumerate(iter_locations):
                for location_to_index in range(location_from_index + 1, len(iter_locations)):
                    location_to = iter_locations[location_to_index]
                    if is_continue and not edge_limiter((location_from, location_to), robot_data):
                        interceptions = cached_find_interceptions(location_from, location_to)
                        if interceptions:
                            new_point = find_closest_point(location_from, [interception[0] for interception in interceptions])
                            if new_point not in locations and not check_is_line_on_obstacle((location_from, new_point)):
                                found_locations.append(new_point)
                                locations.add(new_point)
                                if len(locations) >= MAX_CNT:
                                    is_continue = False
            if not len(found_locations):
                break
            iter_locations = found_locations

    g = Graph()
    for location in locations:
        g.add_vertex(location)

    for obstacle in obstacles_paths:
        for line in obstacle:
            g.add_edge(tuple(line[0]), tuple(line[1]))

    real_edge_limiter_name = edge_limiter_name
    if edge_limiter_name == 'auto':
        real_edge_limiter_name = guess_edge_limiter(len(locations) ** 2)
    edge_limiter = edge_limiter_factory(real_edge_limiter_name)
    locations_list = list(locations)
    for location_from_index, location_from in enumerate(locations_list):
        if is_report:
            print('edging progress', 100 * location_from_index / len(locations_list))
        for location_to_index in range(location_from_index + 1, len(locations_list)):
            location_to = locations_list[location_to_index]
            if not g.exists(location_from, location_to) \
                    and not edge_limiter((location_from, location_to), robot_data) \
                    and not check_is_line_on_obstacle((location_from, location_to)) \
                    and not cached_find_interceptions(location_from, location_to):
                g.add_edge(location_from, location_to)

    return g


if __name__ == '__main__':
    import json

    set_cnt = 5
    with open(f'tests/robot-test-{set_cnt}.json') as json_file:
        robot_data = json.load(json_file)
    graph = build_graph_for_robot_data(robot_data, is_report=True,
                                                     intermediate_discovery_mode='full',
                                                     edge_limiter_name='auto')
    len(graph.vertices())
    exit()

    g = build_graph_for_robot_data({'start': [27, 13], 'finish': [5, 15], 'obstacles': [
        [[8.20928128223826, 13.045295685240944], [7.954678336770978, 16.661336833426862],
         [4.909876084555715, 13.338201317812267], [8.20928128223826, 13.045295685240944]],
        [[1.1902134867352288, 18.37892394968964], [4.948152349025261, 18.532755963114916],
         [1.7842546592576993, 21.47023586490136], [1.1902134867352288, 18.37892394968964]],
    ]})
    assert (g.neighbors((5, 15)) == [
        (4.948152349025261, 18.532755963114916),
        (4.909876084555715, 13.338201317812267),
        (1.1902134867352288, 18.37892394968964),
        (7.954678336770978, 16.661336833426862)])

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
    }, intermediate_discovery_mode='ray')
    assert (len(g.vertices()) == 12)
    assert (sorted(g.neighbors((10, 10))) == [
        (4, 6),
        (6, 5)
    ])
    assert (g.cost((10, 10), (4, 6)) == np.sqrt((10 - 4) ** 2 + (10 - 6) ** 2))
    try:
        g.cost((10, 10), (5, 5))
        assert (False)
    except VertexNotFoundException:
        assert (True)
