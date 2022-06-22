from numpy import save
from algorithms import *

if __name__ == "__main__":
    # ################################## PARTE 4 ######################################
    # Kruskal Direct
    # g = gridGraph(6)
    # kg = g.KruskalD()
    # print("Nodes: 30")
    # print(f"MST value: {kg.mst}")
    # saveGraph(kg, g.typee)
    # g = gridGraph(15)
    # kg = g.KruskalD()
    # print("Nodes: 225")
    # print(f"MST value: {kg.mst}")
    # saveGraph(kg, g.typee)
    # n = 200
    # g = barasiAlbertGraph(n, 10)
    # kg = g.KruskalD()
    # print(f"Nodes: {n}")
    # print(f"MST value: {kg.mst}")
    # saveGraph(kg, g.typee)
    # n = 200
    # g = geographicGraph(n, r=0.4)
    # kg = g.KruskalD()
    # print(f"Nodes: {n}")
    # print(f"MST value: {kg.mst}")
    # saveGraph(kg, g.typee)
    # n = 200
    # g = gilbertGraph(n, p=0.4)
    # kg = g.KruskalD()
    # print(f"Nodes: {n}")
    # print(f"MST value: {kg.mst}")
    # saveGraph(kg, g.typee)
    # n = 200
    # g = dorogovtsevMendesGraph(n)
    # kg = g.KruskalD()
    # print(f"Nodes: {n}")
    # print(f"MST value: {kg.mst}")
    # saveGraph(kg, g.typee)
    # n = 200
    # g = erdosRenyiGraph(n, 8500)
    # kg = g.KruskalD()
    # print(f"Nodes: {n}")
    # print(f"MST value: {kg.mst}")
    # saveGraph(kg, g.typee)
    ############Kruskal I#################
    # n = 200
    # g = dorogovtsevMendesGraph(n)
    # kg = g.KruskalI()
    # print(f"Nodes: {n}")
    # print(f"MST value: {kg.mst}")
    # saveGraph(kg, g.typee)
    # n = 200
    # g = erdosRenyiGraph(n, 8500)
    # kg = g.KruskalI()
    # print(f"Nodes: {n}")
    # print(f"MST value: {kg.mst}")
    # saveGraph(kg, g.typee)
    # n = 200
    # g = gilbertGraph(n, p=0.4)
    # kg = g.KruskalI()
    # print(f"Nodes: {n}")
    # print(f"MST value: {kg.mst}")
    # saveGraph(kg, g.typee)
    # n = 200
    # g = geographicGraph(n, r=0.4)
    # kg = g.KruskalI()
    # print(f"Nodes: {n}")
    # print(f"MST value: {kg.mst}")
    # saveGraph(kg, g.typee)
    # n = 200
    # g = barasiAlbertGraph(n, d=4)
    # kg = g.KruskalI()
    # print(f"Nodes: {n}")
    # print(f"MST value: {kg.mst}")
    # saveGraph(kg, g.typee)
    n = 15
    g = gridGraph(n)
    kg = g.KruskalI()
    print(f"Nodes: {n}")
    print(f"MST value: {kg.mst}")
    saveGraph(kg, g.typee)
    ############3PRIM#################
    # n = 200
    # g = dorogovtsevMendesGraph(n)
    # kg = g.Prim()
    # print(f"Nodes: {n}")
    # print(f"MST value: {kg.mst}")
    # saveGraph(kg, g.typee)
    # n = 200
    # g = erdosRenyiGraph(n, 8500)
    # kg = g.Prim()
    # print(f"Nodes: {n}")
    # print(f"MST value: {kg.mst}")
    # saveGraph(kg, g.typee)
    # n = 200
    # g = gilbertGraph(n, p=0.4)
    # kg = g.Prim()
    # print(f"Nodes: {n}")
    # print(f"MST value: {kg.mst}")
    # saveGraph(kg, g.typee)
    # n = 200
    # g = geographicGraph(n, r=0.4)
    # kg = g.Prim()
    # print(f"Nodes: {n}")
    # print(f"MST value: {kg.mst}")
    # saveGraph(kg, g.typee)
    # n = 200
    # g = barasiAlbertGraph(n, d=13)
    # kg = g.Prim()
    # print(f"Nodes: {n}")
    # print(f"MST value: {kg.mst}")
    # saveGraph(kg, g.typee)
    # n = 15
    # g = gridGraph(n)
    # kg = g.Prim()
    # print(f"Nodes: {n}")
    # print(f"MST value: {kg.mst}")
    # saveGraph(kg, g.typee)
