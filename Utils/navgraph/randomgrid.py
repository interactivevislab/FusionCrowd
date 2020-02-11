from navgraph import NavGraph, Node, Edge

from random import random

grid_width = 6
grid_height = 6
cell_size = 10
weight = 1
width = 1


node_num = grid_width * grid_height
edge_num = 2 * grid_width * grid_height - grid_width - grid_height

graph = NavGraph()

for x in range(grid_width):
    for y in range(grid_height):
        graph.add_node(Node(
            id=x + y * grid_width,
            x=x * cell_size,
            y=y * cell_size
        ))

    edge_id = 0
    # horizontals
    for y in range(grid_height):
        for x in range(grid_width - 1):
            id1 = x + 0 + y * grid_width
            id2 = x + 1 + y * grid_width
            if random() < 0.5:
                id1, id2 = id2, id1

            graph.add_edge(Edge(
                id=edge_id,
                frm=id1,
                to=id2,
                weight=weight,
                width=width,
            ))
            edge_id += 1

    # verticals
    for x in range(grid_width):
        for y in range(grid_height - 1):        
            id1 = x + (y + 0) * grid_width
            id2 = x + (y + 1) * grid_width

            if random() < 0.5:
                id1, id2 = id2, id1

            graph.add_edge(Edge(
                id=edge_id,
                frm=id1,
                to=id2,
                weight=weight,
                width=width,
            ))
            edge_id += 1

graph.write("randomgrid.navgraph")
