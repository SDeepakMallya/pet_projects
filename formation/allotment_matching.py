#!/usr/bin/env python3

import pulp as pl
import numpy as np
import random as rn
import networkx as nx
import sys
import matplotlib.pyplot as plt

def min_lp(source_pts, target_pts, cost = 'sum'):
    '''
    Generates matching using LP
    Inputs:
    - source_pts: List of nD arrays with source coordinates
    - target_pts: List of nD arrays with destination coordinates
    
    '''

    if len(source_pts) != len(target_pts):
        return 'Unequal Points'
        
    dist_list = []
    for s in source_pts:
        for t in target_pts:
            dist_list.append(np.linalg.norm(s - t))

    if cost == 'sum':
        pow_list = dist_list

    elif cost == 'largest':
        ind_list = list(range(len(dist_list)))
        ind_list.sort(key = lambda x: dist_list[x])
        # dist_list.sort(key = lambda x: x[1])
        pow_list = [0 for s in source_pts for t in target_pts]
        i = 1
        for d in ind_list:
            pow_list[d] = i
            i *= 2
    else:
        return 'Cost either sum or largest'
    source_ids = list(range(len(source_pts)))
    target_ids = list(range(len(target_pts)))
    
    prob = pl.LpProblem('min_largest', pl.LpMinimize)
    # prob.setSolver(solver = pl.get_solver('GUROBI'))
    x_vars = pl.LpVariable.dicts('var', (source_ids, target_ids), lowBound = 0, upBound = 1)
    prob += pl.lpDot([x_vars[s][t] for s in source_ids for t in target_ids], pow_list)
    for s in source_ids:
        prob += pl.lpSum([x_vars[s][t] for t in target_ids]) == 1
    for t in target_ids:
        prob += pl.lpSum([x_vars[s][t] for s in source_ids]) == 1
    
    prob.solve()
    return x_vars


def min_hungarian(source_pts, target_pts, cost = 'sum'):
    if len(source_pts) != len(target_pts):
        return 'Unequal Points'
        
    dist_list = []
    for s in source_pts:
        for t in target_pts:
            dist_list.append(np.linalg.norm(s - t))
    
    if cost == 'sum':
        max_dist = max(dist_list) + 1
        pow_list = [max_dist - d for d in dist_list]

    elif cost == 'largest':
        ind_list = list(range(len(dist_list)))
        ind_list.sort(key = lambda x: dist_list[x], reverse = True)
        # dist_list.sort(key = lambda x: x[1])
        pow_list = [0 for s in source_pts for t in target_pts]
        i = 1
        for d in ind_list:
            pow_list[d] = i
            i *= 2

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
            # a = 's_{}'.format(s)
            # b = 't_{}'.format(t)
            g.add_edge(s, t)
            g[s][t]['length'] = pow_list[i * len(source_pts) + j]

    for s in source_nodes:
        # a = 's_{}'.format(s)
        nei = list(g.neighbors(s))
        for t in nei:
            g.nodes[s]['l'] = max(g.nodes[s]['l'], g[s][t]['length'])

    matching = nx.Graph()
    matching.add_nodes_from(g)
    gl = nx.Graph()
    gl.add_nodes_from(g)
        
    for s in source_nodes:
        for t in target_nodes:
            if abs(g.nodes[s]['l'] + g.nodes[t]['l'] - g[s][t]['length']) < 1e-5:
                gl.add_edge(s, t)

    for n in source_nodes:
        if matching.degree(n) == 0:
            for nei in gl.neighbors(n):
                if matching.degree(nei) == 0:
                    matching.add_edge(n, nei)
                    break

    while len(matching.edges()) < len(source_pts):

        # for s in source_nodes:
        #     for t in target_nodes:
        #         # print(s, t, abs(g.nodes[s]['l'] + g.nodes[t]['l'] - g[s][t]['length']))
        #         if not (s, t) in gl.edges() and abs(g.nodes[s]['l'] + g.nodes[t]['l'] - g[s][t]['length']) < 1e-5:
        #             gl.add_edge(s, t)
                    # if matching.degree(s) == 0:
                    #     for nei in gl.neighbors(s):
                    #         if matching.degree(nei) == 0:
                    #             matching.add_edge(s, nei)
                    #             break
                    # if matching.degree(t) == 0:
                    #     for nei in gl.neighbors(t):
                    #         if matching.degree(nei) == 0:
                    #             matching.add_edge(nei, t)
                    #             break

        # l_sum = 0
        # for n in g.nodes():
        #     l_sum += g.nodes[n]['l']
        print(len(matching.edges())) #, nx.is_matching(matching, set(matching.edges())))

        #find unmatched vertex
        unmatched_vertices = []
        for a in source_nodes:
            if len(list(matching.neighbors(a))) == 0:
                unmatched_vertices.append(a)
                # break
        if len(unmatched_vertices) == 0:
            break

        unmatched_targets = []
        for b in target_nodes:
            if matching.degree(b) == 0:
                unmatched_targets.append(b)

        found_augmenting_path = False
        found_alternating_path = False
        paths = []
        path = []
        # for cur_vertex in unmatched_vertices:
            # paths.append([cur_vertex])


        for p in unmatched_vertices:
            for n in gl.neighbors(p):
                paths.append([p, n])

        while len(paths):
            if found_augmenting_path or found_alternating_path:
                break
            new_paths = []
            for p in paths:
                cur_path = p.copy()
                if cur_path[-1] in unmatched_targets:
                    found_augmenting_path = True
                    path = cur_path
                    break
                cur_path.append(list(matching[cur_path[-1]].keys())[0])
                found_alternating_path = True
                for n in gl.neighbors(cur_path[-1]):
                    temp_path = cur_path.copy()
                    if not n in temp_path:
                        temp_path.append(n)
                        new_paths.append(temp_path)
                        found_alternating_path = False

                if found_alternating_path:
                    path = cur_path
                    break
            paths = new_paths.copy()


            # if found_augmenting_path:
            #     break
            # paths = [[cur_vertex]]
            # while len(paths):
            #     if found_augmenting_path:
            #         break
            #     cur_path = paths.pop(0)
            #     cur_vertex = cur_path[-1]
            #     for n in gl.neighbors(cur_vertex):
            #         if not n in cur_path:
            #             temp_path = cur_path.copy()
            #             temp_path.append(n)
            #             if matching.degree(n) == 0:
            #                 path = temp_path
            #                 found_augmenting_path = True
            #                 break
            #             temp_path.append(list(matching.neighbors(n))[0])
            #             if len(temp_path) > len(path):
            #                 path = temp_path
            #             paths.append(temp_path)
                        
                        
        
        
        
        # for unmatched_vertex in unmatched_vertices:
        #     if found_augmenting_path:
        #         break
        #     list1 = []
        #     dict1 = {}
        #     for n in gl.neighbors(unmatched_vertex):
        #         dict1[n] = [unmatched_vertex, n]
        #         list1.append(n)
        #     list2 = []
        #     while len(list1):
        #         cur_node = list1.pop(0)
        #         list2.append(cur_node)
        #         if matching.degree(cur_node) == 0:
        #             found_augmenting_path = True
        #             path = dict1[cur_node]
        #             break
        #         next_node = list(matching.neighbors(cur_node))[0]
        #         cur_path = dict1[cur_node]
        #         cur_path.append(next_node)
        #         if len(cur_path) > len(path):
        #             path = cur_path
        #         added = False
        #         for n in gl.neighbors(next_node):
        #             temp_path = cur_path.copy()
        #             if not n in temp_path:
        #                 temp_path.append(n)                    
        #                 if not n in list1:
        #                     dict1[n] = temp_path
        #                     list1.append(n)
        #                     added = True
        #                 elif n in list1 and len(temp_path) > len(dict1[n]):
        #                     dict1[n] = temp_path
        #                     added = True
                # if not added:
                    # path = cur_path
                    # break

        # print('before', matching.edges())   
        if found_augmenting_path:
            remove = False
            for i in range(len(path) - 1):
                if remove:
                    matching.remove_edge(path[i], path[i + 1])
                    remove = False
                else:
                    matching.add_edge(path[i], path[i + 1])
                    remove = True

        if not found_augmenting_path:
            nodes_s = []
            nodes_t = []
            for i in range(len(path)):
                if i % 2:
                    nodes_t.append(path[i])
                else:
                    nodes_s.append(path[i])

            if nodes_t[0] in source_nodes:
                temp = nodes_t.copy()
                nodes_t = nodes_s.copy()
                nodes_s = temp.copy()
            
            nl_s = []
            for s in nodes_s:
                print(s, list(gl.neighbors(s)))
                nl_s.extend(list(gl.neighbors(s)))
            nl_s = set(nl_s)

            nodes_not_t = [b for b in target_nodes if not b in nodes_t]

            del_l = np.inf
            min_edge = None
            for a in nodes_s:
                for b in nodes_not_t:
                    val = g.nodes[a]['l'] + g.nodes[b]['l'] - g[a][b]['length']
                    if val < del_l:
                        del_l = val
                        min_edge = (a, b)


            print('sad', nl_s, nodes_t)
            
            print('del', del_l, len(gl.edges()), len(nl_s) == len(set(nodes_t)))
            print(path)
            print(matching.edges())
            print(gl.edges())
            gl.add_edge(min_edge[0], min_edge[1])

            for a in nodes_s:
                g.nodes[a]['l'] -= del_l

            for b in nodes_t:
                g.nodes[b]['l'] += del_l

            # if len(nl_s) != len(set(nodes_t)):
            #     break
            if min_edge[1] in unmatched_targets:
                id = path.index(min_edge[0])
                temp = path[:id + 1].copy()
                temp.append(min_edge[1])
                path = temp
                remove = False
                for i in range(len(path) - 1):
                    if remove:
                        matching.remove_edge(path[i], path[i + 1])
                        remove = False
                    else:
                        matching.add_edge(path[i], path[i + 1])
                        remove = True

            for n in source_nodes:
                if matching.degree(n) == 0:
                    for nei in gl.neighbors(n):
                        if matching.degree(nei) == 0:
                            matching.add_edge(n, nei)
                            break

    return list(matching.edges())      

def generate_source_pts(num_pts, mins, maxs):
    '''
    Generates random points in the defined area
    '''
    return [np.array([round(rn.uniform(mins[i], maxs[i])) for i in range(len(mins))]) for _ in range(num_pts)]


def run(num_pts):
    print(num_pts)
    init_pts = generate_source_pts(num_pts, [0, 0], [1000, 1000])
    final_pts = generate_source_pts(num_pts, [0, 0], [1000, 1000])
    # test = min_lp(init_pts, final_pts)
    test = min_hungarian(init_pts, final_pts, 'sum')
    # print_out(num_pts, test)
    print(len(test))

def print_out(num_pts, test):
    for s in range(num_pts):
        for t in range(num_pts):
            if abs(pl.value(test[s][t]) - 1) <= 0.001:
                print(s, t, pl.value(test[s][t]))

if __name__ == '__main__':
    run(int(sys.argv[1]))
