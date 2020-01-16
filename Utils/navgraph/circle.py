from navgraph import NavGraph, Edge, Node
from math import sin, cos, pi


segments = 20
R = 10
width = 3
weight = 1


graph = NavGraph()

nodes = segments
edges = segments
da = 2 * pi / segments
for n in range(nodes):
    a = da * n
    graph.add_node(Node(n, sin(a) * R, cos(a) * R))

    graph.add_edge(Edge(n, n, (n + 1) % nodes, weight, width))
    graph.add_edge(Edge(n, (n + 1) % nodes, n, weight, width))

graph.write("circle.navgraph")
