from collections import defaultdict, OrderedDict
from random import choice, sample, shuffle
import threading
import sys


class Graph:

    def __init__(self, connections=None, directed=False):

        self._graph = defaultdict(set)
        self._graphinc = defaultdict(set)
        self.exploredlist = dict()
        self.edges = set()
        self.shortestpath = dict()
        self.topoorder = dict()
        self.SCC = []
        self.top5SCC = [0, 0, 0, 0, 0]
        self.orderedlist = []
        self.directed = directed
        self.edgelist(connections)
        self.addconnections(connections)

    def addconnections(self, connections):
        # add nodes in adjacency list and also call edgelist function to add edges to edge list

        if connections:
            node1, node2 = connections.split(" ")[0], connections.split(" ")[1]

            for node in (node1, node2):
                if not node in self._graph.keys():
                    self._graph[node] = set()
                    self.explored(node, False)

            self._graph[node1].add(node2)
            if not self.directed:
                self._graph[node2].add(node1)

        self.edgelist(connections)
        self.addincoming(connections)

    def addincoming(self, connections):

        if self.directed:

            if connections:

                node1, node2 = connections.split(" ")[0], connections.split(" ")[1]
                for node in (node1, node2):
                    if node not in self._graphinc.keys():
                        self._graphinc[node] = set()

                self._graphinc[node2].add(node1)

    def edgelist(self, connections):
        # add an edge. If a graph is undirected add only one pair of nodes to the graph.

        if connections:
            nodes = [connections.split(" ")[0], connections.split(" ")[1]]

            if not self.directed:
                nodes = sorted(nodes)

            self.edges.add("-".join(nodes))

    def explored(self, node, explored=False):
        # explored == True/not explored == False for future implementation of BFS, DFS
        self.exploredlist[node] = explored

    def unexploreall(self):
        # used to reset all nodes to unexplored after running any method of class Graph
        for node in list(self._graph.keys()):
            self.explored(node, False)

    def BFS(self):
        vertices = [x for x in self._graph.keys()]
        startv = choice(vertices)
        self.explored(startv, True)
        Q = [startv]
        while Q:

            for node in list(self._graph[Q[0]]):
                if self.exploredlist[node] == False:
                    Q.append(node)
                    self.explored(node, True)
            Q.pop(0)

        self.unexploreall()

    def shortestpaths(self):

        # calculate shortest path by using BFS for undirected connected graph
        vertices = [x for x in self._graph.keys()]

        # intitialize shortpath dictionary where all nodes have None as a shortest path
        for node in vertices:
            self.shortestpath[node] = None
        shuffle(vertices)
        for vertex in vertices:
            if self.exploredlist[vertex] == False:
                # assigned 0 to the starting node and run BFS
                self.shortestpath[vertex] = 0
                Q = [vertex]
                self.explored(vertex, True)
                while Q:
                    for node in list(self._graph[Q[0]]):

                        if self.exploredlist[node] == False:
                            Q.append(node)
                            self.explored(node, True)
                            self.shortestpath[node] = self.shortestpath[Q[0]] + 1

                    Q.pop(0)

        self.unexploreall()

    def SCCs(self):
        # find all SCC in an undirected graph via BFS

        vertices = [x for x in self._graph.keys()]
        SCCcounter = 0

        # go through all vertices of the graph
        for vertex in vertices:
            # if a vertex is unexplored then:

            if not self.exploredlist[vertex]:
                Q = [vertex]
                self.explored(vertex, True)
                SCClist = [vertex]
                while Q:
                    for node in list(self._graph[Q[0]]):

                        if self.exploredlist[node] == False:
                            self.exploredlist[node] = True
                            Q.append(node)
                            SCClist.append(node)
                    Q.pop(0)
                self.SCC.append(SCClist)
                SCCcounter += 1

        self.unexploreall()

    def DFSiter(self):

        vertices = [x for x in self._graph.keys()]
        startv = choice(vertices)
        S = [startv]
        while S:
            # we need to "pop" the first node to make sure that we do not enter infinite loop if a vertex to be popped has been explored already
            v = S.pop(-1)
            if not self.exploredlist[v]:
                self.exploredlist[v] = True

                for node in list(self._graph[v]):
                    S.append(node)
        self.unexploreall()

    def DFSrecursion(self, vertex=None):

        if not vertex:
            vertex = choice([x for x in self._graph.keys()])

        if not self.exploredlist[vertex]:
            self.exploredlist[vertex] = True
            for node in list(self._graph[vertex]):
                self.DFSrecursion(node)

        self.unexploreall()

    def TopoSort(self, inverse = False):

        self.graphsize = len(list(self._graph.keys()))
        vertices = [x for x in self._graph.keys()]
        vertices = sorted(vertices)
        #shuffle(vertices)
        for node in vertices:
            if not self.exploredlist[node]:
                if not inverse:
                    self.DFS_Topo(node)
                else:
                    self.DFS_Topo_inverse_iter(node)

        self.unexploreall()

    def directed_SCC(self):

        self.TopoSort(True)
        SCCCounter = 0

        for node in reversed(self.orderedlist):
            if not self.exploredlist[node]:
                SCCCounter += 1
                S = [node]
                SCClist = []
                while S:
                    v = S.pop(-1)
                    if not self.exploredlist[v]:
                        self.exploredlist[v] = True
                        SCClist.append(v)
                        for vertex in list(self._graph[v]):
                            S.append(vertex)

                #self.SCC.append(SCClist)
                self.largestSCCs(SCClist)

        self.unexploreall()

    def DFS_Topo(self, node):

        self.exploredlist[node] = True
        for vertex in list(self._graph[node]):
            if not self.exploredlist[vertex]:
                self.DFS_Topo(vertex)
        self.topoorder[node] = self.graphsize
        self.graphsize -= 1

    def DFS_Topo_inverse(self, node):

        self.exploredlist[node] = True
        for vertex in list(self._graphinc[node]):
            if not self.exploredlist[vertex]:
                self.DFS_Topo_inverse(vertex)
        self.topoorder[node] = self.graphsize
        self.orderedlist.append(node)
        self.graphsize -= 1

    # Needs to be implemented
    def DFS_Topo_inverse_iter(self, node):

        self.exploredlist[node] = True
        S = []

        # add a node to a stack. If second parameter is True, this is an "assigment" node - or the node to be used to assign the order
        # if the second parameter is False it is a node used to iterate and discover the node's children

        S.append([node, True])
        S.append([node, False])
        while S:
            v = S.pop(-1)
            if v[1]:
                for node in list(self._graphinc[v[0]]):
                    if not self.exploredlist[node]:
                        print("This node was not explored:", node)
                self.topoorder[v[0]] = self.graphsize
                self.orderedlist.append(v[0])
                self.graphsize -= 1
            else:
                for vertex in list(self._graphinc[v[0]]):
                    if not self.exploredlist[vertex]:
                        self.exploredlist[vertex] = True
                        S.append([vertex, True])
                        S.append([vertex, False])

    def largestSCCs(self, SCC):

        length_new = len(SCC)
        sortedlist = sorted(self.top5SCC)
        if length_new > sortedlist[0]:
            sortedlist[0] = length_new
            self.top5SCC = sortedlist

if __name__ == 'main':
    graph = Graph(directed=True)


    #with open('C:\\Users\\golub\\Downloads\\SCCTest2.txt') as f:
    with open('C:\\Users\\golub\\Downloads\\SCCTest2.txt') as f:
        for line in f:
            graph.addconnections(str(line.strip()))

    #print(graph._graph)
    # print(graph.edges)
    # print(graph.exploredlist)

    # graph.SCCs()
    # print(graph.SCC)
    #
    # graph.shortestpaths()
    # print(graph.shortestpath)
    # graph.DFSiter()
    # graph.DFSrecursion()

    for i in range(10):
        graph.directed_SCC()
        print(sorted(graph.top5SCC))
        graph.top5SCC = [0,0,0,0,0]

    # graph.directed_SCC()
    # print(sorted(graph.top5SCC))
