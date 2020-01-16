from collections import namedtuple
from typing import Dict
from tkinter import *
from scanner import Scanner


Node = namedtuple("Node", ["id", "x", "y"])
Edge = namedtuple("Edge", ["id", "frm", "to", "width", "weight"])


class NavGraph:
    nodes: Dict[int, Node] = dict()
    edges: Dict[int, Edge] = dict()

    def __init__(self, filename=None):
        if filename is not None:
            self.read(filename)

    def add_node(self, node: Node):
        self.nodes[node.id] = node

    def add_edge(self, edge: Edge):
        self.edges[edge.id] = edge

    def write(self, filename):
        with open(filename, "w") as f:
            f.write("{}\n".format(len(self.nodes)))
            for node in self.nodes.values():
                f.write("{} {:.3f} {:.3f}\n".format(node.id, node.x, node.y))

            f.write("{}\n".format(len(self.edges)))
            for edge in self.edges.values():
                f.write("{} {} {} {} {}\n".format(
                    edge.id,
                    edge.frm, edge.to,
                    edge.weight, edge.width
                ))

    def read(self, filename):
        self.nodes.clear()
        self.edges.clear()

        with Scanner(filename) as scanner:
            node_count = scanner.next_int()
            for _ in range(node_count):
                node = Node(
                    id=scanner.next_int(),
                    x=scanner.next_float(),
                    y=scanner.next_float()
                )
                self.nodes[node.id] = node

            edge_count = scanner.next_int()
            for _ in range(edge_count):
                edge = Edge(
                    id=scanner.next_int(),
                    frm=scanner.next_int(),
                    to=scanner.next_int(),
                    width=scanner.next_float(),
                    weight=scanner.next_float()
                )

                self.edges[edge.id] = edge

    def draw(self, canvas, scale=1, shift=(0, 0)):
        r = 5
        shift_x, shift_y = shift
        for node in self.nodes.values():
            x = shift_x + node.x * scale
            y = shift_y + node.y * scale

            canvas.circle(x, y, r, outline="blue")
            canvas.create_text(
                x, y,
                anchor=NW,
                text=str(node.id),
                fill="#ccc"
            )

        for edge in self.edges.values():
            canvas.line(self.nodes[edge.frm], self.nodes[edge.to], "blue")
