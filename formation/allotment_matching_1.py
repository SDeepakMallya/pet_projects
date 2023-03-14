#!/usr/bin/env python3

import networkx as nx
import numpy as np
import sys
import random as rn

def allotment_hungarian(source_pts, target_pts, cost = 'sum'):
    dist_list = []
    for s in source_pts:
        for t in target_pts:
            dist_list.append(np.linalg.norms(s - t))

    g = nx.Graph()
    source_nodes = []
    for s in range(len(source_pts)):
        a = 's_{}'.format(s)
        source_nodes.append(a)
        g.add_node(a)
        g.nodes[a]['x'] = float(source_pts[s][0])
        g.nodes[a]['y'] = float(source_pts[s][1])
        g.nodes[a]['l'] = 0

    target_nodes = []
    for t in range(len(target_pts)):
        b = 't_{}'.format(t)
        target_nodes.append(b)
        g.add_node(b)
        g.nodes[b]['x'] = float(target_pts[t][0])
        g.nodes[b]['y'] = float(target_pts[t][1])
        g.nodes[b]['l'] = 0

    for i, s in enumerate(source_nodes):
        for j, t in enumerate(target_nodes):
            g.add_edge(s, t)
            g[s][t]['length'] = dist_list(i * len(source_pts) + j)

def generate_source_pts(num_pts, mins, maxs):
    return [np.array([round(rn.uniform(mins[i], maxs[i])) for i in range(len(mins))]) for _ in range(num_pts)]


if __name__ == '__main__':
    num_pts = int(sys.argv[1])
    init_pts = generate_source_pts(num_pts, [0, 0], [1000, 1000])
    final_pts = generate_source_pts(num_pts, [0, 0], [1000, 1000])
    test = allotment_hungarian(init_pts, final_pts)
