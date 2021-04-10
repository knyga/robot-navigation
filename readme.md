## Challenge
Develop a software function that takes as an input two vectors that represent the initial and the final position of the robot and the list of obstacles represented by init final convex polygons.
The function needs to return a polyline (the list of vectors) that starts with the initial vector and finishes with the final vector and must not intersect the given obstacles.

## Solution
The idea is to construct a graph and then execute some path finding algorithm.

### Graph
In 2d space the shortest distance between two points could be found with a straight line. But we have obstacle polygons. The claim is that the shortest path could be built on the set of points represented by the union of all vertices of obstacle polygons and all possible intersection points on the way the goal point.

#### Finding intersection
If we represent lines as:
```
l1=v1+t(v2-v1)
l2=v3+u(v4-v3)
``` 
Then t and u could be represented as:
```
t = det(matrix([v2 - v1, v4 - v3])) / det(matrix([v3 - v1, v4 - v3]))
u = det(matrix([v1 - v2, v3 - v1])) / det(matrix([v3 - v1, v4 - v3]))
```
The advantage of this method is that we can detect existence of intersection before calculating the exact point.

In the developed solution border's are not considered as an intersection, meaning that the line segment `[(0, 0), (1, 1)]` and `[(1, 1), (5, 5)]` do not intersect. 

#### Iterative vertices exploration (removed)
On the initial step we use all vertices of polygons we have.

During the iteration we find intersection's on our path to the goal point.

We repeat the iteration with found intersection points until we do not get new points.

In the current implementation this approach was removed due to high resulting computational complexity. So far results rely only on edges.

#### Connecting vertices
We can connect vertices if the connectivity edge (line) doesn't intersect with obstacle polygon's and doesn't go through them.   

#### Weights
Weight of the edge is l2 norm.

### Path finding algorithm
A* is the best solution for the context defined as we plan to build only one path.

### Issues and possible optimizations
Complexity of the algorithm grows fast with new polygons, especially if these polygons are positions on the direct path to the goal. It happens because so far algorithm tries to build the complete graph.

Solution already uses caching for intersection detection and has simple optimization for taking closest intersection point in case if multiple points exist.

#### Explore edges in runtime
We do not need all edges and with some heuristics, or even without, we could explore neighbors for the vertex by checking it's connectivity upon request.

#### Merge close vertices
Sometimes we get vertices really close one to another. Merging them with some epsilon would reduce the vertices number with a trade-off for growing path distance, which would really depend on epsilon we choose.

#### Limiting edge exploration
Algorithm tries to build full distance matrix by default. It becomes too costy for relatively high count of vertices. Edge limiter is created to reduce the number of edges and try to build edge only with vertices which are relatively close depending of the graph.  