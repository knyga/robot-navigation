import numpy as np
from edge_limiter import guess_edge_limiter, edge_limiter_factory
from graph import Graph, VertexNotFoundException
from interception import calc_interception, check_is_point_on_line, NoInterceptionException


def convert_points_polygon_to_lines(points):
    lines = []
    for pindex in range(len(points)):
        obstacle_line = np.array([points[pindex - 1], points[pindex]])
        lines.append(obstacle_line)
    return lines


def find_interceptions_cached(cache, line, obstacle_paths, calc_interception_fn=calc_interception):
    interceptions = []
    for obstacle in obstacle_paths:
        for obstacle_line in obstacle:
            try:
                new_point, is_edge = calc_interception_fn(
                    cache=cache,
                    line1=line,
                    line2=obstacle_line
                )
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


def calc_interception_cached(cache, line1, line2):
    key = (line1.item(0), line1.item(1), line1.item(2), line1.item(3),
           line2.item(0), line2.item(1), line2.item(2), line2.item(3))
    if key in cache:
        return cache[key]
    cache[key] = calc_interception(line1, line2)
    return cache[key]


def find_obstacle_index_cached(cache, obstacles_paths, obstacles, location):
    if location in cache:
        return cache[location]
    cache[location] = find_obstacle_index(location, obstacles_paths, obstacles)
    return cache[location]


def check_is_line_on_obstacle_cached(cache, obstacles_paths, obstacles, line):
    loc1, loc2 = line
    obstacle_1_index = find_obstacle_index_cached(
        location=loc1,
        cache=cache,
        obstacles=obstacles,
        obstacles_paths=obstacles_paths,
    )
    obstacle_2_index = find_obstacle_index_cached(
        location=loc2,
        cache=cache,
        obstacles=obstacles,
        obstacles_paths=obstacles_paths,
    )
    return obstacle_1_index is not None and obstacle_1_index == obstacle_2_index


def get_obstacle_paths_from_robot_data(robot_data):
    return [convert_points_polygon_to_lines(obstacle) for obstacle in robot_data['obstacles']]


def get_obstacles_from_robot_data(robot_data):
    return [{tuple(point) for point in obstacle} for obstacle in robot_data['obstacles']]


def check_is_line_allowed_cached(graph, robot_data, line, cache, obstacles_paths, obstacles, edge_limiter):
    return not graph.exists(*line) \
           and not edge_limiter(line, robot_data) \
           and not check_is_line_on_obstacle_cached(line=line, cache=cache, obstacles_paths=obstacles_paths,
                                                    obstacles=obstacles) \
           and not find_interceptions_cached(line=np.array(line),
                                             cache=cache,
                                             obstacle_paths=obstacles_paths,
                                             calc_interception_fn=calc_interception_cached)


def build_graph_for_robot_data(robot_data, cache={}, edge_limiter_name=None, is_report=False):
    finish_location = tuple(robot_data['finish'])
    iter_locations = [tuple(robot_data['start']),
                      *[tuple(item) for sublist in robot_data['obstacles'] for item in sublist]]
    obstacles_paths = get_obstacle_paths_from_robot_data(robot_data)
    obstacles = get_obstacles_from_robot_data(robot_data)
    locations = set(iter_locations)
    locations.add(finish_location)

    # if intermediate_discovery_mode == 'ray':
    #     while True:
    #         found_locations = []
    #         for v in iter_locations:
    #             interceptions = cached_find_interceptions(v, finish_location)
    #             if interceptions:
    #                 new_point = find_closest_point(v, [interception[0] for interception in interceptions])
    #                 if new_point not in locations and not check_is_line_on_obstacle((v, new_point)):
    #                     found_locations.append(new_point)
    #                     locations.add(new_point)
    #         if not len(found_locations):
    #             break
    #         iter_locations = found_locations
    # if intermediate_discovery_mode == 'full':
    #     is_continue = True
    #     while is_continue:
    #         MAX_CNT = 150
    #         edge_limiter = edge_limiter_factory(guess_edge_limiter(len(locations)))
    #         found_locations = []
    #         if is_report:
    #             print('edge limiter name', guess_edge_limiter(len(locations)))
    #             print('vertices count', len(locations))
    #         for location_from_index, location_from in enumerate(iter_locations):
    #             for location_to_index in range(location_from_index + 1, len(iter_locations)):
    #                 location_to = iter_locations[location_to_index]
    #                 if is_continue and not edge_limiter((location_from, location_to), robot_data):
    #                     interceptions = cached_find_interceptions(location_from, location_to)
    #                     if interceptions:
    #                         new_point = find_closest_point(location_from, [interception[0] for interception in interceptions])
    #                         if new_point not in locations and not check_is_line_on_obstacle((location_from, new_point)):
    #                             found_locations.append(new_point)
    #                             locations.add(new_point)
    #                             if len(locations) >= MAX_CNT:
    #                                 is_continue = False
    #         if not len(found_locations):
    #             break
    #         iter_locations = found_locations

    g = Graph()
    for location in locations:
        g.add_vertex(location)

    for obstacle in obstacles_paths:
        for line in obstacle:
            g.add_edge(tuple(line[0]), tuple(line[1]))

    real_edge_limiter_name = edge_limiter_name
    if edge_limiter_name == 'auto':
        real_edge_limiter_name = guess_edge_limiter(len(locations))
        if is_report:
            print('selected edge limiter', real_edge_limiter_name)
    edge_limiter = edge_limiter_factory(real_edge_limiter_name)
    locations_list = list(locations)
    for location_from_index, location_from in enumerate(locations_list):
        if is_report:
            print('edging progress', 100 * location_from_index / len(locations_list))
        for location_to_index in range(location_from_index + 1, len(locations_list)):
            location_to = locations_list[location_to_index]
            line = (location_from, location_to)
            if check_is_line_allowed_cached(line=line,
                                            graph=g,
                                            robot_data=robot_data,
                                            cache=cache,
                                            obstacles=obstacles,
                                            edge_limiter=edge_limiter,
                                            obstacles_paths=obstacles_paths):
                g.add_edge(location_from, location_to)
    return g


if __name__ == '__main__':
    # import json
    #
    # set_cnt = 15
    # with open(f'tests/robot-test-{set_cnt}.json') as json_file:
    #     robot_data = json.load(json_file)
    robot_data = {"start": [0, 1], "finish": [15, 7], "obstacles": [
        [[10.451294060185973, 4.108310105473189], [9.17825411594646, 7.677252868658845],
         [8.666190306647069, 5.467417803070163], [10.451294060185973, 4.108310105473189]],
        [[5.734857386335971, 0.9622425837297927], [7.901643989551653, 3.8021372645774747],
         [5.644475016520235, 2.861229981173178], [5.734857386335971, 0.9622425837297927]],
        [[13.028385145630057, 4.690639800615818], [11.417616444457872, 7.371777409627574],
         [10.9180438351143, 5.236977800361233], [13.028385145630057, 4.690639800615818]],
        [[12.193947634566921, 6.789291366049799], [11.53313301366446, 9.61236490796634],
         [10.038381833649101, 8.992400831099332], [12.193947634566921, 6.789291366049799]],
        [[5.21513721757856, 1.020378185803844], [4.771657630381961, 3.777711104445885],
         [3.2713663870947607, 0.6998559922022322], [5.21513721757856, 1.020378185803844]],
        [[7.714768456954914, 1.0191868225518146], [8.387010201959246, 3.4411843797909483],
         [7.041383302296289, 2.433235172308025], [7.714768456954914, 1.0191868225518146]],
        [[8.9276447962133, 4.783005057708457], [7.37020041533083, 6.22730356485898],
         [7.224190415430602, 4.795257082380515], [8.9276447962133, 4.783005057708457]],
        [[2.3585790218901383, 1.1073141108127533], [2.9172034055791203, 4.464188136612809],
         [-1.1482363909217468, 2.4870293501635885], [2.3585790218901383, 1.1073141108127533]]]
                  }
    graph_cache = {}
    graph = build_graph_for_robot_data(robot_data,
                                       is_report=False,
                                       cache=graph_cache,
                                       edge_limiter_name='auto')
    p1 = (2.3585790218901383, 1.1073141108127533)
    p2 = (4.771657630381961, 3.777711104445885)
    assert (p2 in graph.neighbors(p1))

    g = build_graph_for_robot_data({'start': [27, 13], 'finish': [5, 15], 'obstacles': [
        [[8.20928128223826, 13.045295685240944], [7.954678336770978, 16.661336833426862],
         [4.909876084555715, 13.338201317812267], [8.20928128223826, 13.045295685240944]],
        [[1.1902134867352288, 18.37892394968964], [4.948152349025261, 18.532755963114916],
         [1.7842546592576993, 21.47023586490136], [1.1902134867352288, 18.37892394968964]],
    ]})
    assert (g.neighbors((5, 15)) == [(4.909876084555715, 13.338201317812267), (1.1902134867352288, 18.37892394968964),
                                     (4.948152349025261, 18.532755963114916), (7.954678336770978, 16.661336833426862)])

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
    assert (len(g.vertices()) == 9)
    print(sorted(g.neighbors((10, 10))))
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
