import numpy as np


class VertexNotFoundException(Exception):
    pass


class Vertex:
    def __init__(self, location):
        if type(location) != tuple:
            raise Exception(f'Location must be tuple, {type(location)} given')
        self.location = location
        self.adjacent = {}

    def calc_distance(self, neighbor_node_location):
        return np.linalg.norm(np.array(self.location) - np.array(neighbor_node_location))

    def add_neighbor(self, neighbor_node_location):
        self.adjacent[neighbor_node_location] = self.calc_distance(neighbor_node_location)

    def neighbors(self):
        return list(self.adjacent.keys())

    def get_id(self):
        return self.id

    def get_weight(self, neighbor_node_location):
        if neighbor_node_location in self.adjacent:
            return self.adjacent[neighbor_node_location]
        raise VertexNotFoundException

    def exists(self, neighbor_node_location):
        return neighbor_node_location in self.adjacent


class Graph:
    def __init__(self):
        self.vert_dict = {}
        self.num_vertices = 0

    def __iter__(self):
        return iter(self.vert_dict.values())

    def add_vertex(self, node_location):
        if node_location in self.vert_dict:
            return self.vertex(node_location)
        self.num_vertices = self.num_vertices + 1
        new_vertex = Vertex(node_location)
        self.vert_dict[node_location] = new_vertex
        return new_vertex

    def vertex(self, node_location):
        if node_location in self.vert_dict:
            return self.vert_dict[node_location]
        raise VertexNotFoundException

    def add_edge(self, from_location, to_location):
        if from_location not in self.vert_dict:
            self.add_vertex(from_location)
        if to_location not in self.vert_dict:
            self.add_vertex(to_location)

        self.vert_dict[from_location].add_neighbor(to_location)
        self.vert_dict[to_location].add_neighbor(from_location)

    def vertices(self):
        return list(self.vert_dict.keys())

    def cost(self, from_node_location, to_node_location):
        if from_node_location in self.vert_dict:
            return self.vert_dict[from_node_location].get_weight(to_node_location)
        raise VertexNotFoundException

    def exists(self, from_node_location, to_node_location):
        return from_node_location in self.vert_dict and self.vert_dict[from_node_location].exists(to_node_location)

    def neighbors(self, node_location):
        if node_location in self.vert_dict:
            return self.vert_dict[node_location].neighbors()
        raise VertexNotFoundException


if __name__ == '__main__':
    g = Graph()
    a = (0, 0)
    b = (2, 2)
    c = (1, 3)
    d = (2, 5)
    e = (7, 17)
    f = (3, 9)

    g.add_vertex(a)
    g.add_vertex(b)
    g.add_vertex(c)
    g.add_vertex(d)
    g.add_vertex(e)
    g.add_vertex(f)

    g.add_edge(a, b)
    g.add_edge(a, c)
    g.add_edge(a, f)

    g.add_edge(b, c)
    g.add_edge(b, d)

    g.add_edge(c, d)
    g.add_edge(c, f)
    g.add_edge(d, e)
    g.add_edge(e, f)

    assert (g.cost(a, b) == np.linalg.norm(np.array(a) - np.array(b)))
    assert (sorted(list(g.neighbors(a))) == sorted([b, c, f]))
    assert (sorted(g.vertices()) == sorted(list(set((a, b, c, d, e, f)))))
    try:
        g.cost(a, e)
    except VertexNotFoundException:
        assert (True)
    try:
        g.neighbors((100, 100))
    except VertexNotFoundException:
        assert (True)
