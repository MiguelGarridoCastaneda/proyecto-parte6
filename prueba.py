from numpy import save
from algorithms import *
import random
from graph import *
import math


# g = gridGraph(3)
# g = erdosRenyiGraph(200, 8500)
# g = dorogovtsevMendesGraph(10)
# g.distances_btw_v()
g = gilbertGraph(30, p=0.4)
g.distances_btw_v()
# print([e.dist for e in list(g.edges.values())])
# tree = DFS_R(g, 5)
# saveGraph(tree, g.typee)
# tree = DFS_I(g, 5)
# tree.typee = 8
# saveGraph(tree, g.typee)

# obtener longitude iniciales de aristas deacuerdo a la posición de vertices
# saveGraph(tree, g.typee)
# edges_in = []
# edges_graph = list(g.edges.keys())
# print(g.edges.keys())
# for k, v in g.nodes.items():
#     print(f"nodo {k}")
#     # print(v.attr['POS_CART'])
#     for v_n in v.attr['NEIGHBORS']:
#         print(v_n.id, end=' ')
#         e = f"{k}->{v_n.id}"
#         e_inv = f"{v_n.id}->{k}"
#         print(e, e_inv)
#         if e in edges_graph and e not in edges_in:
#             g.edges[e].dist = round(np.linalg.norm(
#                 np.array(g.nodes[k].attr['POS_CART']) - np.array(g.nodes[v_n.id].attr['POS_CART'])), 2)
#             edges_in.append(e)
#             edges_in.append(e_inv)
#     print('')

# print([e.dist for e in list(g.edges.values())])

# algoritmo spring
# print(list(g.nodes.keys()))

nodos_graph = list(g.nodes.keys())
edges_in_graph = list(g.edges.keys())
# print(edges_in_graph)
c1 = 2
c2 = 1
c3 = 1
c4 = 0.1
for n in nodos_graph:
    fx = 0
    fy = 0
    vecinos = g.nodes[n].attr['NEIGHBORS']
    # print(f"nodo {n} con vecinos:")
    # for vecino in vecinos:
    #     print(vecino.id, end=" ")
    # print("")
    # print("Nodos que no son sus vecinos:")
    no_vecinos = list((set(range(0, len(nodos_graph))) -
                      set([vecino.id for vecino in vecinos])) - {n})
    # print(no_vecinos)
    for vecino in vecinos:
        e = f"{n}->{vecino.id}"
        e_inv = f"{vecino.id}->{n}"
        if e in edges_in_graph:
            # print(e)
            d = g.edges[e].dist
            # print(d)
        elif e_inv in edges_in_graph:
            # print(e_inv)
            d = g.edges[e_inv].dist
        force = c1 * math.log(d/c2)
        x1 = g.nodes[n].attr['POS_INI'][0]
        y1 = g.nodes[n].attr['POS_INI'][1]
        x2 = g.nodes[vecino.id].attr['POS_INI'][0]
        y2 = g.nodes[vecino.id].attr['POS_INI'][1]
        radians = math.atan2(y2 - y1, x2 - x1)
        fx += force * math.cos(radians)
        fy += force * math.sin(radians)

    for no_vecino in no_vecinos:
        x1 = g.nodes[n].attr['POS_INI'][0]
        y1 = g.nodes[n].attr['POS_INI'][1]
        x2 = g.nodes[no_vecino].attr['POS_INI'][0]
        y2 = g.nodes[no_vecino].attr['POS_INI'][1]

        d = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

        force = c3 / math.log(d)

        radians = math.atan2(y2 - y1, x2 - x1)
        fx -= force * math.cos(radians)
        fy -= force * math.sin(radians)

    g.nodes[n].attr['POS_FIN'][0] += fx
    g.nodes[n].attr['POS_FIN'][1] += fy

print("Posiciones iniciales")
print([g.nodes[n].attr['POS_INI'] for n in g.nodes])
print("Posiciones finales")
print([g.nodes[n].attr['POS_FIN'] for n in g.nodes])
# print("")
# for v in vecinos:
#     print(g.edges[f"{n}->{v.id}"].dist)


# g = barasiAlbertGraph(30, 4)
# g = dorogovtsevMendesGraph(300)
# g = erdosRenyiGraph(30, 330)
# print(f"Número de aristas: {len(g.edges)}")
# m = g.createAdjMat()
# print(f"Matriz de adyacencia de dimensión: {m.shape}")
# print(m)
# dg = g.Dijkstra(5)
# saveGraph(dg, g.typee)
# print(g.edges.keys())
# kg = g.KruskalD()
# # print(kg.nodes)
# # print(kg.edges.keys())
# print(f"MST value: {kg.mst}")
# saveGraph(kg, g.typee)
# print(set(g.getNodes()))
# kg = g.KruskalI()# pendiente Kruskal inverso
# pg = g.KruskalI()
# print()
# saveGraph(pg, g.typee)
