import csv
import random
import argparse
from player import Player
from navgraph.navgraph import NavGraph

from itertools import zip_longest, tee
from collections import defaultdict, namedtuple


random.seed(1)
Trajectories = namedtuple("Trajectories", ["pos", "steps", "xmin", "ymin", "xmax", "ymax"])
NavMesh = namedtuple("NavMesh", ["vertices", "edges", "nodes", "obstacles", "xmin", "ymin", "xmax", "ymax"])
Node = namedtuple("Node", ["center", "vertices", "edges", "obstacles", "ABC"])
all_colors = ['AntiqueWhite1', 'AntiqueWhite2', 'AntiqueWhite3', 'AntiqueWhite4', 'CadetBlue1', 'CadetBlue2', 'CadetBlue3', 'CadetBlue4', 'DarkGoldenrod1', 'DarkGoldenrod2', 'DarkGoldenrod3', 'DarkGoldenrod4', 'DarkOliveGreen1', 'DarkOliveGreen2', 'DarkOliveGreen3', 'DarkOliveGreen4', 'DarkOrange1', 'DarkOrange2', 'DarkOrange3', 'DarkOrange4', 'DarkOrchid1', 'DarkOrchid2', 'DarkOrchid3', 'DarkOrchid4', 'DarkSeaGreen1', 'DarkSeaGreen2', 'DarkSeaGreen3', 'DarkSeaGreen4', 'DarkSlateGray1', 'DarkSlateGray2', 'DarkSlateGray3', 'DarkSlateGray4', 'DeepPink2', 'DeepPink3', 'DeepPink4', 'DeepSkyBlue2', 'DeepSkyBlue3', 'DeepSkyBlue4', 'DodgerBlue2', 'DodgerBlue3', 'DodgerBlue4', 'HotPink1', 'HotPink2', 'HotPink3', 'HotPink4', 'IndianRed1', 'IndianRed2', 'IndianRed3', 'IndianRed4', 'LavenderBlush2', 'LavenderBlush3', 'LavenderBlush4', 'LemonChiffon2', 'LemonChiffon3', 'LemonChiffon4', 'LightBlue1', 'LightBlue2', 'LightBlue3', 'LightBlue4', 'LightCyan2', 'LightCyan3', 'LightCyan4', 'LightGoldenrod1', 'LightGoldenrod2', 'LightGoldenrod3', 'LightGoldenrod4', 'LightPink1', 'LightPink2', 'LightPink3', 'LightPink4', 'LightSalmon2', 'LightSalmon3', 'LightSalmon4', 'LightSkyBlue1', 'LightSkyBlue2', 'LightSkyBlue3', 'LightSkyBlue4', 'LightSteelBlue1', 'LightSteelBlue2', 'LightSteelBlue3', 'LightSteelBlue4', 'LightYellow2', 'LightYellow3', 'LightYellow4', 'MediumOrchid1', 'MediumOrchid2', 'MediumOrchid3', 'MediumOrchid4', 'MediumPurple1', 'MediumPurple2', 'MediumPurple3', 'MediumPurple4', 'MistyRose2', 'MistyRose3', 'MistyRose4', 'NavajoWhite2', 'NavajoWhite3', 'NavajoWhite4', 'OliveDrab1', 'OliveDrab2', 'OliveDrab4', 'OrangeRed2', 'OrangeRed3', 'OrangeRed4', 'PaleGreen1', 'PaleGreen2', 'PaleGreen3', 'PaleGreen4', 'PaleTurquoise1', 'PaleTurquoise2', 'PaleTurquoise3', 'PaleTurquoise4', 'PaleVioletRed1', 'PaleVioletRed2', 'PaleVioletRed3', 'PaleVioletRed4', 'PeachPuff2', 'PeachPuff3', 'PeachPuff4', 'RosyBrown1', 'RosyBrown2', 'RosyBrown3', 'RosyBrown4', 'RoyalBlue1', 'RoyalBlue2', 'RoyalBlue3', 'RoyalBlue4', 'SeaGreen1', 'SeaGreen2', 'SeaGreen3', 'SkyBlue1', 'SkyBlue2', 'SkyBlue3', 'SkyBlue4', 'SlateBlue1', 'SlateBlue2', 'SlateBlue3', 'SlateBlue4', 'SlateGray1', 'SlateGray2', 'SlateGray3', 'SlateGray4', 'SpringGreen2', 'SpringGreen3', 'SpringGreen4', 'SteelBlue1', 'SteelBlue2', 'SteelBlue3', 'SteelBlue4', 'VioletRed1', 'VioletRed2', 'VioletRed3', 'VioletRed4', 'alice blue', 'antique white', 'aquamarine', 'aquamarine2', 'aquamarine4', 'azure', 'azure2', 'azure3', 'azure4', 'bisque', 'bisque2', 'bisque3', 'bisque4', 'blanched almond', 'blue', 'blue violet', 'blue2', 'blue4', 'brown1', 'brown2', 'brown3', 'brown4', 'burlywood1', 'burlywood2', 'burlywood3', 'burlywood4', 'cadet blue', 'chartreuse2', 'chartreuse3', 'chartreuse4', 'chocolate1', 'chocolate2', 'chocolate3', 'coral', 'coral1', 'coral2', 'coral3', 'coral4', 'cornflower blue', 'cornsilk2', 'cornsilk3', 'cornsilk4', 'cyan', 'cyan2', 'cyan3', 'cyan4', 'dark goldenrod', 'dark green', 'dark khaki', 'dark olive green', 'dark orange', 'dark orchid', 'dark salmon', 'dark sea green', 'dark slate blue', 'dark slate gray', 'dark turquoise', 'dark violet', 'deep pink', 'deep sky blue', 'dim gray', 'dodger blue', 'firebrick1', 'firebrick2', 'firebrick3', 'firebrick4', 'floral white', 'forest green', 'gainsboro', 'ghost white', 'gold', 'gold2', 'gold3', 'gold4', 'goldenrod', 'goldenrod1', 'goldenrod2', 'goldenrod3', 'goldenrod4', 'gray', 'gray1', 'gray10', 'gray11', 'gray12', 'gray13', 'gray14', 'gray15', 'gray16', 'gray17', 'gray18', 'gray19', 'gray2', 'gray20', 'gray21', 'gray22', 'gray23', 'gray24', 'gray25', 'gray26', 'gray27', 'gray28', 'gray29', 'gray3', 'gray30', 'gray31', 'gray32', 'gray33', 'gray34', 'gray35', 'gray36', 'gray37', 'gray38', 'gray39', 'gray4', 'gray40', 'gray42', 'gray43', 'gray44', 'gray45', 'gray46', 'gray47', 'gray48', 'gray49', 'gray5', 'gray50', 'gray51', 'gray52', 'gray53', 'gray54', 'gray55', 'gray56', 'gray57', 'gray58', 'gray59', 'gray6', 'gray60', 'gray61', 'gray62', 'gray63', 'gray64', 'gray65', 'gray66', 'gray67', 'gray68', 'gray69', 'gray7', 'gray70', 'gray71', 'gray72', 'gray73', 'gray74', 'gray75', 'gray76', 'gray77', 'gray78', 'gray79', 'gray8', 'gray80', 'gray81', 'gray82', 'gray83', 'gray84', 'gray85', 'gray86', 'gray87', 'gray88', 'gray89', 'gray9', 'gray90', 'gray91', 'gray92', 'gray93', 'gray94', 'gray95', 'gray97', 'gray98', 'gray99', 'green yellow', 'green2', 'green3', 'green4', 'honeydew2', 'honeydew3', 'honeydew4', 'hot pink', 'indian red', 'ivory2', 'ivory3', 'ivory4', 'khaki', 'khaki1', 'khaki2', 'khaki3', 'khaki4', 'lavender', 'lavender blush', 'lawn green', 'lemon chiffon', 'light blue', 'light coral', 'light cyan', 'light goldenrod', 'light goldenrod yellow', 'light grey', 'light pink', 'light salmon', 'light sea green', 'light sky blue', 'light slate blue', 'light slate gray', 'light steel blue', 'light yellow', 'lime green', 'linen', 'magenta2', 'magenta3', 'magenta4', 'maroon', 'maroon1', 'maroon2', 'maroon3', 'maroon4', 'medium aquamarine', 'medium blue', 'medium orchid', 'medium purple', 'medium sea green', 'medium slate blue', 'medium spring green', 'medium turquoise', 'medium violet red', 'midnight blue', 'mint cream', 'misty rose', 'navajo white', 'navy', 'old lace', 'olive drab', 'orange', 'orange red', 'orange2', 'orange3', 'orange4', 'orchid1', 'orchid2', 'orchid3', 'orchid4', 'pale goldenrod', 'pale green', 'pale turquoise', 'pale violet red', 'papaya whip', 'peach puff', 'pink', 'pink1', 'pink2', 'pink3', 'pink4', 'plum1', 'plum2', 'plum3', 'plum4', 'powder blue', 'purple', 'purple1', 'purple2', 'purple3', 'purple4', 'red', 'red2', 'red3', 'red4', 'rosy brown', 'royal blue', 'saddle brown', 'salmon', 'salmon1', 'salmon2', 'salmon3', 'salmon4', 'sandy brown', 'sea green', 'seashell2', 'seashell3', 'seashell4', 'sienna1', 'sienna2', 'sienna3', 'sienna4', 'sky blue', 'slate blue', 'slate gray', 'snow', 'snow2', 'snow3', 'snow4', 'spring green', 'steel blue', 'tan1', 'tan2', 'tan4', 'thistle', 'thistle1', 'thistle2', 'thistle3', 'thistle4', 'tomato', 'tomato2', 'tomato3', 'tomato4', 'turquoise', 'turquoise1', 'turquoise2', 'turquoise3', 'turquoise4', 'violet red', 'wheat1', 'wheat2', 'wheat3', 'wheat4', 'white smoke', 'yellow', 'yellow green', 'yellow2', 'yellow3', 'yellow4']
random.shuffle(all_colors)

AgentInfo = namedtuple("AgentInfo", ["agent_id", "pos", "orient", "R"])


def grouper(iterable, n, fillvalue=None):
    "Collect data into fixed-length chunks or blocks"
    # grouper('ABCDEFG', 3, 'x') --> ABC DEF Gxx"
    args = [iter(iterable)] * n
    return zip_longest(*args, fillvalue=fillvalue)


def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = tee(iterable)
    next(b, None)
    return zip(a, b)


def read_trajectories(filename):
    result = defaultdict(dict)

    minx, miny = 1000000, 1000000
    maxx, maxy = -1000000, -1000000
    steps = 0
    with open(filename) as csvfile:
        traj_file = csv.reader(csvfile, delimiter=',')

        for step, row in enumerate(traj_file):
            step_time = float(row[0])
            for agent_id, x, y, orientx, orienty, radius in grouper(row[1:], 6):
                info = AgentInfo(int(agent_id), (float(x), float(y)), (float(orientx), float(orienty)), float(radius))
                minx = min(info.pos[0], minx)
                miny = min(info.pos[1], miny)
                maxx = max(info.pos[0], maxx)
                maxy = max(info.pos[1], maxy)

                result[int(agent_id)][step] = info

        for vals in result.values():
            steps = max(steps, len(vals))

    return Trajectories(result, steps, xmin=minx, ymin=miny, xmax=maxx, ymax=maxy)


def draw_trajectories(canvas: Player, tr):
    for id, agent_infos in tr.pos.items():
        color = all_colors[id % len(all_colors)]
        for a1, a2 in pairwise(agent_infos.values()):
            canvas.line(a1.pos, a2.pos, color)


def redraw_positions(canvas: Player, tr, frame, size=1.0, ovals=None,orint=None):
    if ovals is None:
        ovals = dict()
    if orint is None:
        orint = dict()
    
    for agent_id, infos in tr.pos.items():
        agent_id = int(agent_id)

        if frame not in infos:
            if agent_id in ovals:
                canvas.hide_item(ovals[agent_id])
            continue

        if agent_id in ovals:
            canvas.show_item(ovals[agent_id])

            canvas.move_circle(ovals[agent_id], infos[frame].pos)

            if not hide_orientation:
                canvas.move_orientation(orint[agent_id], infos[frame])
            
        else:
            ovals[agent_id] = canvas.circle(
                infos[frame].pos,
                infos[frame].R,
                all_colors[agent_id % len(all_colors)]
            )

            if not hide_orientation:
                orint[agent_id] = canvas.orientation(infos[frame], "black")

    canvas.update_scroll()
    return ovals, orint


def read_mesh(filename):
    def read_next(f):
        line = None
        while not line:
            line = f.readline().strip()

        return line

    xmin, ymin = 10000000, 10000000
    xmax, ymax = -10000000, -10000000

    with open(filename) as f:
        vertex_count = int(f.readline().strip())
        vertices = dict()
        for id in range(vertex_count):
            line = f.readline().strip()
            if not line:
                continue

            x, y = map(float, line.split())
            xmin, ymin = min(xmin, x), min(ymin, y)
            xmax, ymax = max(xmax, x), max(ymax, y)

            vertices[id] = (x, y)

        edge_count = int(read_next(f))
        edges = dict()
        for id in range(edge_count):
            v0, v1, n0, n1 = map(int, f.readline().strip().split())
            edges[id] = (v0, v1)

        obst_count = int(read_next(f))
        obstacles = []
        # skip obstacles for now
        for _ in range(obst_count):
            line = f.readline().strip()
            if not line:
                continue
            v0, v1, n, nxt = map(int, line.split())
            obstacles.append((v0, v1))

        nodes = dict()
        read_next(f)  # node group name
        node_count = int(read_next(f))
        for id in range(node_count):
            line = read_next(f).split()
            cx, cy = map(float, line)
            node_vertex_cnt, *node_vertices = map(int, f.readline().strip().split())
            A, B, C = map(float, f.readline().strip().split())
            node_edge_cnt, *node_edges = map(int, f.readline().strip().split())
            node_obst_cnt, *node_obstacles = map(int, f.readline().strip().split())
            nodes[id] = Node(
                center=(cx, cy),
                vertices=list(node_vertices),
                edges=list(node_edges),
                obstacles=list(node_obstacles),
                ABC=(A, B, C)
            )

    return NavMesh(vertices=vertices, edges=edges, nodes=nodes, obstacles=obstacles, xmin=xmin, ymin=ymin, xmax=xmax, ymax=ymax)


def draw_mesh(canvas: Player, mesh: NavMesh, show_text=True):
    if show_text:
        for id, p in mesh.vertices.items():
            canvas.text(p, "{:0}, {:0}".format(p[0], p[1]), "#ccc")

    for id, node in mesh.nodes.items():
        for v0, v1 in pairwise(node.vertices):
            canvas.line(mesh.vertices[v0], mesh.vertices[v1], color="#ccc")

        p_last = mesh.vertices[node.vertices[-1]]
        p0     = mesh.vertices[node.vertices[0]]
        canvas.line(p_last, p0, color="#ccc")
        #print(p0)
        if show_text:
            canvas.text(node.center, str(id), color="#ccc")

    for v0, v1 in mesh.obstacles:
        canvas.vector(mesh.vertices[v0], mesh.vertices[v1], "red")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("source_file")
    parser.add_argument("--navmesh", default=None)
    parser.add_argument("--navgraph", default=None)
    parser.add_argument("--scale", default=1)
    parser.add_argument("--agent-size", default=1)
    parser.add_argument("--hide-trajectory", action='store_true')
    parser.add_argument("--hide-orientation", action='store_true')
    parser.add_argument("--show-mesh-text", action='store_true')

    args = parser.parse_args()
    scale = float(args.scale)
    agent_size = float(args.agent_size)
    hide_trajectory = args.hide_trajectory
    hide_orientation= args.hide_orientation

    tr = read_trajectories(args.source_file)

    canvas = Player(scale, tr.steps)

    if args.navmesh is not None:
        mesh = read_mesh(args.navmesh)
        draw_mesh(canvas, mesh, show_text=args.show_mesh_text)

    if args.navgraph is not None:
        graph = NavGraph(args.navgraph)
        graph.draw(canvas)

    if not hide_trajectory:
        draw_trajectories(canvas, tr)

    result_redraw= redraw_positions(canvas, tr=tr, size=agent_size, frame=0)
    ovals, orient = result_redraw
    canvas.set_redraw(lambda new_frame: redraw_positions(canvas, tr=tr, frame=int(new_frame), size=agent_size, ovals=ovals, orint=orient))

    canvas.main_loop()
