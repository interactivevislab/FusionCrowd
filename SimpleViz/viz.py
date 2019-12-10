import csv
import random
import argparse
from tkinter import *
from itertools import zip_longest, tee
from collections import defaultdict, namedtuple


random.seed(1)
Trajectories = namedtuple("Trajectories", ["pos", "steps", "xmin", "ymin", "xmax", "ymax"])
NavMesh = namedtuple("NavMesh", ["vertices", "edges", "nodes", "obstacles", "xmin", "ymin", "xmax", "ymax"])
Node = namedtuple("Node", ["center", "vertices", "edges", "obstacles", "ABC"])
all_colors = ['AntiqueWhite1', 'AntiqueWhite2', 'AntiqueWhite3', 'AntiqueWhite4', 'CadetBlue1', 'CadetBlue2', 'CadetBlue3', 'CadetBlue4', 'DarkGoldenrod1', 'DarkGoldenrod2', 'DarkGoldenrod3', 'DarkGoldenrod4', 'DarkOliveGreen1', 'DarkOliveGreen2', 'DarkOliveGreen3', 'DarkOliveGreen4', 'DarkOrange1', 'DarkOrange2', 'DarkOrange3', 'DarkOrange4', 'DarkOrchid1', 'DarkOrchid2', 'DarkOrchid3', 'DarkOrchid4', 'DarkSeaGreen1', 'DarkSeaGreen2', 'DarkSeaGreen3', 'DarkSeaGreen4', 'DarkSlateGray1', 'DarkSlateGray2', 'DarkSlateGray3', 'DarkSlateGray4', 'DeepPink2', 'DeepPink3', 'DeepPink4', 'DeepSkyBlue2', 'DeepSkyBlue3', 'DeepSkyBlue4', 'DodgerBlue2', 'DodgerBlue3', 'DodgerBlue4', 'HotPink1', 'HotPink2', 'HotPink3', 'HotPink4', 'IndianRed1', 'IndianRed2', 'IndianRed3', 'IndianRed4', 'LavenderBlush2', 'LavenderBlush3', 'LavenderBlush4', 'LemonChiffon2', 'LemonChiffon3', 'LemonChiffon4', 'LightBlue1', 'LightBlue2', 'LightBlue3', 'LightBlue4', 'LightCyan2', 'LightCyan3', 'LightCyan4', 'LightGoldenrod1', 'LightGoldenrod2', 'LightGoldenrod3', 'LightGoldenrod4', 'LightPink1', 'LightPink2', 'LightPink3', 'LightPink4', 'LightSalmon2', 'LightSalmon3', 'LightSalmon4', 'LightSkyBlue1', 'LightSkyBlue2', 'LightSkyBlue3', 'LightSkyBlue4', 'LightSteelBlue1', 'LightSteelBlue2', 'LightSteelBlue3', 'LightSteelBlue4', 'LightYellow2', 'LightYellow3', 'LightYellow4', 'MediumOrchid1', 'MediumOrchid2', 'MediumOrchid3', 'MediumOrchid4', 'MediumPurple1', 'MediumPurple2', 'MediumPurple3', 'MediumPurple4', 'MistyRose2', 'MistyRose3', 'MistyRose4', 'NavajoWhite2', 'NavajoWhite3', 'NavajoWhite4', 'OliveDrab1', 'OliveDrab2', 'OliveDrab4', 'OrangeRed2', 'OrangeRed3', 'OrangeRed4', 'PaleGreen1', 'PaleGreen2', 'PaleGreen3', 'PaleGreen4', 'PaleTurquoise1', 'PaleTurquoise2', 'PaleTurquoise3', 'PaleTurquoise4', 'PaleVioletRed1', 'PaleVioletRed2', 'PaleVioletRed3', 'PaleVioletRed4', 'PeachPuff2', 'PeachPuff3', 'PeachPuff4', 'RosyBrown1', 'RosyBrown2', 'RosyBrown3', 'RosyBrown4', 'RoyalBlue1', 'RoyalBlue2', 'RoyalBlue3', 'RoyalBlue4', 'SeaGreen1', 'SeaGreen2', 'SeaGreen3', 'SkyBlue1', 'SkyBlue2', 'SkyBlue3', 'SkyBlue4', 'SlateBlue1', 'SlateBlue2', 'SlateBlue3', 'SlateBlue4', 'SlateGray1', 'SlateGray2', 'SlateGray3', 'SlateGray4', 'SpringGreen2', 'SpringGreen3', 'SpringGreen4', 'SteelBlue1', 'SteelBlue2', 'SteelBlue3', 'SteelBlue4', 'VioletRed1', 'VioletRed2', 'VioletRed3', 'VioletRed4', 'alice blue', 'antique white', 'aquamarine', 'aquamarine2', 'aquamarine4', 'azure', 'azure2', 'azure3', 'azure4', 'bisque', 'bisque2', 'bisque3', 'bisque4', 'blanched almond', 'blue', 'blue violet', 'blue2', 'blue4', 'brown1', 'brown2', 'brown3', 'brown4', 'burlywood1', 'burlywood2', 'burlywood3', 'burlywood4', 'cadet blue', 'chartreuse2', 'chartreuse3', 'chartreuse4', 'chocolate1', 'chocolate2', 'chocolate3', 'coral', 'coral1', 'coral2', 'coral3', 'coral4', 'cornflower blue', 'cornsilk2', 'cornsilk3', 'cornsilk4', 'cyan', 'cyan2', 'cyan3', 'cyan4', 'dark goldenrod', 'dark green', 'dark khaki', 'dark olive green', 'dark orange', 'dark orchid', 'dark salmon', 'dark sea green', 'dark slate blue', 'dark slate gray', 'dark turquoise', 'dark violet', 'deep pink', 'deep sky blue', 'dim gray', 'dodger blue', 'firebrick1', 'firebrick2', 'firebrick3', 'firebrick4', 'floral white', 'forest green', 'gainsboro', 'ghost white', 'gold', 'gold2', 'gold3', 'gold4', 'goldenrod', 'goldenrod1', 'goldenrod2', 'goldenrod3', 'goldenrod4', 'gray', 'gray1', 'gray10', 'gray11', 'gray12', 'gray13', 'gray14', 'gray15', 'gray16', 'gray17', 'gray18', 'gray19', 'gray2', 'gray20', 'gray21', 'gray22', 'gray23', 'gray24', 'gray25', 'gray26', 'gray27', 'gray28', 'gray29', 'gray3', 'gray30', 'gray31', 'gray32', 'gray33', 'gray34', 'gray35', 'gray36', 'gray37', 'gray38', 'gray39', 'gray4', 'gray40', 'gray42', 'gray43', 'gray44', 'gray45', 'gray46', 'gray47', 'gray48', 'gray49', 'gray5', 'gray50', 'gray51', 'gray52', 'gray53', 'gray54', 'gray55', 'gray56', 'gray57', 'gray58', 'gray59', 'gray6', 'gray60', 'gray61', 'gray62', 'gray63', 'gray64', 'gray65', 'gray66', 'gray67', 'gray68', 'gray69', 'gray7', 'gray70', 'gray71', 'gray72', 'gray73', 'gray74', 'gray75', 'gray76', 'gray77', 'gray78', 'gray79', 'gray8', 'gray80', 'gray81', 'gray82', 'gray83', 'gray84', 'gray85', 'gray86', 'gray87', 'gray88', 'gray89', 'gray9', 'gray90', 'gray91', 'gray92', 'gray93', 'gray94', 'gray95', 'gray97', 'gray98', 'gray99', 'green yellow', 'green2', 'green3', 'green4', 'honeydew2', 'honeydew3', 'honeydew4', 'hot pink', 'indian red', 'ivory2', 'ivory3', 'ivory4', 'khaki', 'khaki1', 'khaki2', 'khaki3', 'khaki4', 'lavender', 'lavender blush', 'lawn green', 'lemon chiffon', 'light blue', 'light coral', 'light cyan', 'light goldenrod', 'light goldenrod yellow', 'light grey', 'light pink', 'light salmon', 'light sea green', 'light sky blue', 'light slate blue', 'light slate gray', 'light steel blue', 'light yellow', 'lime green', 'linen', 'magenta2', 'magenta3', 'magenta4', 'maroon', 'maroon1', 'maroon2', 'maroon3', 'maroon4', 'medium aquamarine', 'medium blue', 'medium orchid', 'medium purple', 'medium sea green', 'medium slate blue', 'medium spring green', 'medium turquoise', 'medium violet red', 'midnight blue', 'mint cream', 'misty rose', 'navajo white', 'navy', 'old lace', 'olive drab', 'orange', 'orange red', 'orange2', 'orange3', 'orange4', 'orchid1', 'orchid2', 'orchid3', 'orchid4', 'pale goldenrod', 'pale green', 'pale turquoise', 'pale violet red', 'papaya whip', 'peach puff', 'pink', 'pink1', 'pink2', 'pink3', 'pink4', 'plum1', 'plum2', 'plum3', 'plum4', 'powder blue', 'purple', 'purple1', 'purple2', 'purple3', 'purple4', 'red', 'red2', 'red3', 'red4', 'rosy brown', 'royal blue', 'saddle brown', 'salmon', 'salmon1', 'salmon2', 'salmon3', 'salmon4', 'sandy brown', 'sea green', 'seashell2', 'seashell3', 'seashell4', 'sienna1', 'sienna2', 'sienna3', 'sienna4', 'sky blue', 'slate blue', 'slate gray', 'snow', 'snow2', 'snow3', 'snow4', 'spring green', 'steel blue', 'tan1', 'tan2', 'tan4', 'thistle', 'thistle1', 'thistle2', 'thistle3', 'thistle4', 'tomato', 'tomato2', 'tomato3', 'tomato4', 'turquoise', 'turquoise1', 'turquoise2', 'turquoise3', 'turquoise4', 'violet red', 'wheat1', 'wheat2', 'wheat3', 'wheat4', 'white smoke', 'yellow', 'yellow green', 'yellow2', 'yellow3', 'yellow4']
random.shuffle(all_colors)


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
            for agent_id, x, y in grouper(row, 3):
                agent_id, x, y = int(agent_id), float(x), float(y)
                minx = min(x, minx)
                miny = min(y, miny)
                maxx = max(x, maxx)
                maxy = max(y, maxy)

                result[agent_id][step] = x, y

        for vals in result.values():
            steps = max(steps, len(vals))

    return Trajectories(result, steps, xmin=minx, ymin=miny, xmax=maxx, ymax=maxy)


def draw_trajectories(canvas, tr, xmin, ymin, scale=1.0):
    def flat(pos):
        for x, y in pos:
            yield x
            yield y

    for id, positions in tr.pos.items():
        scaled = flat((scale * (x - xmin), scale * (y - ymin)) for x, y in positions.values())
        canvas.create_line(*scaled, fill=all_colors[id % len(all_colors)])


def redraw_positions(canvas, tr, frame, scale, xmin, ymin, size=1.0, ovals=None):
    def coords(x, y):
        return ((x - xmin - size) * scale,
                (y - ymin - size) * scale,
                (x - xmin + size) * scale,
                (y - ymin + size) * scale)

    if ovals is None:
        ovals = dict()

    for agent_id, positions in tr.pos.items():
        if frame not in positions:
            if agent_id in ovals:
                canvas.itemconfigure(ovals[agent_id], state='hidden')
            continue

        x, y = positions[frame]
        if agent_id in ovals:
            canvas.itemconfigure(ovals[agent_id], state='normal')
            canvas.coords(ovals[agent_id], *coords(x, y))
            #canvas.coords(
            #    ovals[agent_id],
            #    (x - xmin) * scale,
            #    (y - ymin) * scale
            #)
        else:
            ovals[agent_id] = canvas.create_oval(*coords(x, y), fill=all_colors[agent_id % len(all_colors)], outline="white")
            #ovals[agent_id] = canvas.create_text(
            #    (x - xmin) * scale,
            #    (y - ymin) * scale,
            #    text=str(agent_id),
            #    fill=all_colors[agent_id % len(all_colors)],
            #    anchor=CENTER
            #)

    return ovals


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


def draw_mesh(canvas, mesh: NavMesh, scale, xmin, ymin, show_text=True):
    def v_adj(v):
        x, y = mesh.vertices[v]
        return (x - xmin) * scale, (y - ymin) * scale

    def draw_line(v0, v1, color="#ccc"):
        canvas.create_line(*v_adj(v0), *v_adj(v1), fill=color)

    def draw_obst_line(v0, v1):
        draw_line(v0, v1, "red")
        x0, y0 = v_adj(v0)
        x1, y1 = v_adj(v1)
        dx, dy = x1 - x0, y1 - y0
        x09 = x0 + dx * 0.9
        y09 = y0 + dy * 0.9
        r = 3
        canvas.create_oval(x09 - r/2, y09 - r/2, x09 + r/2, y09 + r/2, outline="red")

    if show_text:
        for id, (vx, vy) in mesh.vertices.items():
            canvas.create_text(
                (vx - xmin) * scale,
                (vy - ymin) * scale,
                anchor=NW,
                text="{:0}, {:0}".format(vx, vy),
                fill="#ccc"
            )

    for id, node in mesh.nodes.items():
        for v0, v1 in pairwise(node.vertices):
            draw_line(v0, v1)

        draw_line(node.vertices[-1], node.vertices[0])

        if show_text:
            canvas.create_text(
                (node.center[0] - xmin) * scale,
                (node.center[1] - ymin) * scale,
                text=str(id),
                fill="#ccc"
            )

    for v0, v1 in mesh.obstacles:
        draw_obst_line(v0, v1)


def draw_quadtree(canvas, x, y, w, h, level):
    if level == 0:
        return

    mx = x + w / 2.0
    my = y + h / 2.0

    canvas.create_line(mx,  y,    mx, y + h, fill="green", width=level)
    canvas.create_line( x, my, x + w,    my, fill="green", width=level)

    draw_quadtree(canvas,  x,  y, mx - x, my - y, level - 1)
    draw_quadtree(canvas, mx,  y, mx - x, my - y, level - 1)
    draw_quadtree(canvas,  x, my, mx - x, my - y, level - 1)
    draw_quadtree(canvas, mx, my, mx - x, my - y, level - 1)


PLAYING = 1
PAUSED = 0
PLAYER_STATE = PAUSED
PLAY_SPEED = 100

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("source_file")
    parser.add_argument("navmesh_file")
    parser.add_argument("--scale", default=1)
    parser.add_argument("--agent_size", default=1)
    parser.add_argument("--hide_trajectory", action='store_true')
    parser.add_argument("--hide_mesh_text", action='store_true')
    parser.add_argument("--show_quad_tree", action='store_true')

    args = parser.parse_args()
    scale = float(args.scale)
    agent_size = float(args.agent_size)
    hide_trajectory = args.hide_trajectory
    show_quad_tree = args.show_quad_tree

    tr = read_trajectories(args.source_file)
    mesh = read_mesh(args.navmesh_file)

    global_xmin, global_ymin = min(tr.xmin, mesh.xmin), min(tr.ymin, mesh.ymin)

    W, H = max(tr.xmax, mesh.xmax), max(tr.ymax, mesh.ymax)
    W, H = (W - global_xmin) * scale, (H - global_ymin) * scale,

    window = Tk()
    window.title("{}, {}x{}. Controls: left, right, space".format(args.source_file, W / scale, H / scale))
    canvas = Canvas(window, width=W, height=H, bg="black")
    canvas.pack()

    draw_mesh(canvas, mesh, scale=scale, xmin=global_xmin, ymin=global_ymin, show_text=not args.hide_mesh_text)

    if not hide_trajectory:
        draw_trajectories(canvas, tr, xmin=global_xmin, ymin=global_ymin, scale=scale)

    if show_quad_tree:
        draw_quadtree(canvas, 0, 0, W, H, 5)

    ovals = redraw_positions(canvas, tr=tr, scale=scale, size=agent_size, xmin=global_xmin, ymin=global_ymin, frame=0)

    redraw = lambda new_frame: redraw_positions(canvas, tr=tr, scale=scale, frame=int(new_frame), size=agent_size, xmin=global_xmin, ymin=global_ymin, ovals=ovals)
    timeline = Scale(window, length=W, from_=0, to=tr.steps, orient=HORIZONTAL, command=redraw)
    timeline.pack()

    def tick(omit=None):
        if PLAYER_STATE == PLAYING:
            timeline.set(timeline.get() + 1)

        timeline.after(PLAY_SPEED, tick)

    def toggle_player(omit=None):
        global PLAYER_STATE
        PLAYER_STATE = PAUSED if PLAYER_STATE == PLAYING else PLAYING

    def modify_speed(change):
        global PLAY_SPEED
        PLAY_SPEED = min(max(PLAY_SPEED, 20) + change, 1000)

    window.bind("<Right>", lambda key: timeline.set(timeline.get() + 1))
    window.bind("<Left>", lambda key: timeline.set(timeline.get() - 1))

    window.bind("<Up>", lambda key: modify_speed(25))
    window.bind("<Down>", lambda key: modify_speed(-25))

    window.bind("<space>", toggle_player)

    tick()
    window.mainloop()
