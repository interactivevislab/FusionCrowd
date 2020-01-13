from math import sin, cos, pi
from random import random

grid_width = 20
grid_height = 20
cell_size = 3
weight = 1
width = 1
bidirectional = 0

node_num = grid_width * grid_height
edge_num = 2 * grid_width * grid_height - grid_width - grid_height

with open("unidirectionalgrid.navgraph", "w") as f:
    # nodes number
    f.write("{}\n".format(node_num))
    for x in range(grid_width):
        for y in range(grid_height):
            f.write("{:.2f} {:.2f}\n".format(x * cell_size, y * cell_size))            

    f.write("{}\n".format(edge_num))
    # horizontals
    for y in range(grid_height):
        for x in range(grid_width - 1):        
            id1 = x + 0 + y * grid_width
            id2 = x + 1 + y * grid_width
            
            if random() < 0.5:
                id1, id2 = id2, id1
                
            f.write("{} {} {} {} {}\n".format(id1, id2, weight, width, bidirectional))

    # verticals
    for x in range(grid_width):
        for y in range(grid_height - 1):        
            id1 = x + (y + 0) * grid_width
            id2 = x + (y + 1) * grid_width
            
            if random() < 0.5:
                id1, id2 = id2, id1
                
            f.write("{} {} {} {} {}\n".format(id1, id2, weight, width, bidirectional))
