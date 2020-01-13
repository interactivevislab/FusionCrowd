from math import sin, cos, pi

segments = 20
R = 10
width = 3
weight = 1
bidirectional = 1

da = 2 * pi / segments
with open("circle.navgraph", "w") as f:
    # nodes number
    f.write("{}\n".format(segments))        
    for n in range(segments):
        a = da * n
        f.write("{:.2f} {:.2f}\n".format(sin(a) * R, cos(a) * R))
                    
    # edges number
    f.write("{}\n".format(segments))
    for n in range(segments):
        f.write("{} {} {} {} {}\n".format(n, (n + 1) % segments, weight, width, bidirectional))
