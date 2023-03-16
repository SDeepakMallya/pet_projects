#!/usr/bin/env python3

import networkx as nx
import numpy as np
import sys
import random as rn

def bfs(gl, s):
    cur_paths = [[s]]
    final_paths = []
    while len(cur_paths):
        path = cur_paths.pop(0)
        node = path[-1]
        f = True
        # print(path, node)
        for n in gl.neighbors(node):
            f = False
            temp = path.copy()
            # print('in', temp)
            if n not in temp:
                temp.append(n)
                cur_paths.append(temp)
                # print(temp, cur_paths)
        if f:
            final_paths.append(path)
        # print('----')

    return final_paths


def allotment_hungarian(source_pts, target_pts, cost = 'sum'):
    dist_list = []
    for s in source_pts:
        for t in target_pts:
            dist_list.append(np.linalg.norm(s - t))

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
            g[s][t]['length'] = dist_list[i * len(source_pts) + j]

    gl = nx.DiGraph()
    gl.add_nodes_from(g)
    for s in source_nodes:
        for t in target_nodes:
            if abs(g.nodes[s]['l'] + g.nodes[t]['l'] - g[s][t]['length']) < 1e-05:
                gl.add_edge(s, t)

    while True:
        # print('oh yeah!')
        r_s = [s for s in source_nodes if gl.in_degree(s) == 0]
        r_t = [t for t in target_nodes if gl.out_degree(t) == 0]
        if not len(r_s):
            break 
        # zn_s = []
        # zn_t = []
        z_s = []
        z_t = []
        for s in r_s:
            paths = bfs(gl, s)
            for path in paths:
                if path[-1] in source_nodes:
                    z_s.append(path)
                    # if path[-1] in r_s: 
                        # zr_s.append(path)
                    # else:
                        # zn_s.append(path)
                else:
                    # if path[-1] in r_t:
                    z_t.append(path)
                    # else:
                        # zn_t.append(path)
        print('s', z_s, r_s)
        print('t', z_t, r_t)
        if len(z_s) + len(z_t) == 0:
            print(gl.edges())
            break
        if len(z_t):
            path = z_t[0]
            for i in range(len(path) - 1):
                gl.remove_edge(path[i], path[i + 1])
                gl.add_edge(path[i + 1], path[i])
        
        else:
            set_1 = set([p[-1] for p in z_s])
            set_2 = set(target_nodes)
            for p in z_t:
                if p[-1] in set_2:
                    set_2.remove(p[-1])

            delta = np.inf
            for i in set_1:
                for j in set_2:
                    delta = min(delta, g[i][j]['length'] - g.nodes[i]['l'] - g.nodes[j]['l'])
            
            set_3 = set([p[-1] for p in z_t])
            for i in set_1:
                g.nodes[i]['l'] += delta
            for i in set_3:
                g.nodes[i]['l'] -= delta

            for s in source_nodes:
                for t in target_nodes:
                    if not (s, t) in gl.edges() and not (t, s) in gl.edges():
                        if abs(g.nodes[s]['l'] + g.nodes[t]['l'] - g[s][t]['length']) < 1e-05:
                            gl.add_edge(s, t)


    matching = []
    for t in target_nodes:
        matching.extend(list(gl.out_edges(t)))
    return matching

def generate_source_pts(num_pts, mins, maxs):
    return [np.array([round(rn.uniform(mins[i], maxs[i])) for i in range(len(mins))]) for _ in range(num_pts)]


if __name__ == '__main__':
    num_pts = int(sys.argv[1])
    init_pts = generate_source_pts(num_pts, [0, 0], [1000, 1000])
    final_pts = generate_source_pts(num_pts, [0, 0], [1000, 1000])
    test = allotment_hungarian(init_pts, final_pts)
    print(len(test))
