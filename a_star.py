from queue import PriorityQueue


# Manhattan distance
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def find_shortest_path(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = dict()
    cost_so_far = dict()
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()

        # if current == goal:
        #     break

        for neighbor in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, neighbor)
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(goal, neighbor)
                frontier.put(neighbor, priority)
                came_from[neighbor] = current

    path = [goal]

    while path[-1] != start:
        path.append(came_from[path[-1]])
    path.reverse()
    return path, cost_so_far[goal]


if __name__ == '__main__':
    import numpy as np
    from graph_explorer import GraphExplorer

    g = GraphExplorer({
        'start': [0, 0],
        'finish': [10, 10],
        'obstacles': [[[2, 2], [2, 4], [3, 3]], [[5, 4], [4, 6], [6, 5], [7, 4]]]
    }).graph


    def calc_cost_from_path(path):
        npath = [np.array(point) for point in path]
        total = 0
        for i in range(1, len(npath)):
            total += np.linalg.norm(npath[i] - npath[i - 1])
        return total


    path, cost = find_shortest_path(g, (0, 0), (10, 10))
    assert (path == [(0, 0), (2, 4), (4, 6), (10, 10)])
    assert (np.allclose(cost, calc_cost_from_path(path)))
