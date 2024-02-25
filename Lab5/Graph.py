class Graph:
    def __init__(self):
        self.adjacency_matrix = []
        self.labels = []
        self.num_vertices = 0

    def setAMatrix(self, b, m):
        self.num_vertices = m
        self.adjacency_matrix = b

    def setLabel(self, c):
        self.labels = c

    def breadthFirstTraverse(self, start):
        visited = [False] * self.num_vertices
        queue = []
        queue.append(start)
        visited[start] = True

        while queue:
            start = queue.pop(0)
            print(self.labels[start], end=" ")

            for i in range(self.num_vertices):
                if self.adjacency_matrix[start][i] == 1 and (not visited[i]):
                    queue.append(i)
                    visited[i] = True

    def depthFirstTraverse(self, start, visited):
        visited[start] = True
        print(self.labels[start], end=" ")

        for i in range(self.num_vertices):
            if self.adjacency_matrix[start][i] == 1 and (not visited[i]):
                self.depthFirstTraverse(i, visited)


class WGraph:
    def __init__(self):
        self.weighted_matrix = []
        self.num_vertices = 0

    def dijkstraShortestPath(self, graph, src):
        self.num_vertices = graph.num_vertices
        self.weighted_matrix = graph.adjacency_matrix
        dist = [float('inf')] * self.num_vertices
        dist[src] = 0
        sptSet = [False] * self.num_vertices

        for _ in range(self.num_vertices):
            u = self.minDistance(dist, sptSet)
            sptSet[u] = True
            for v in range(self.num_vertices):
                if (self.weighted_matrix[u][v] > 0 and not sptSet[v] and
                        dist[v] > dist[u] + self.weighted_matrix[u][v]):
                    dist[v] = dist[u] + self.weighted_matrix[u][v]
        self.printSolution(dist)

    def minDistance(self, dist, sptSet):
        min_val = float('inf')
        min_index = -1
        for v in range(self.num_vertices):
            if dist[v] < min_val and not sptSet[v]:
                min_val = dist[v]
                min_index = v
        return min_index

    def printSolution(self, dist):
        print("Vertex \tDistance from Source")
        for i in range(self.num_vertices):
            print(i, "\t", dist[i])

    def primMST(self, graph):
        self.num_vertices = graph.num_vertices
        self.weighted_matrix = graph.adjacency_matrix
        parent = [-1] * self.num_vertices
        key = [float('inf')] * self.num_vertices
        key[0] = 0
        mstSet = [False] * self.num_vertices

        for _ in range(self.num_vertices):
            u = self.minKey(key, mstSet)
            mstSet[u] = True
            for v in range(self.num_vertices):
                if (self.weighted_matrix[u][v] > 0 and not mstSet[v] and
                        key[v] > self.weighted_matrix[u][v]):
                    key[v] = self.weighted_matrix[u][v]
                    parent[v] = u

        self.printMST(parent)

    def minKey(self, key, mstSet):
        min_val = float('inf')
        min_index = -1
        for v in range(self.num_vertices):
            if key[v] < min_val and not mstSet[v]:
                min_val = key[v]
                min_index = v
        return min_index

    def printMST(self, parent):
        print("Edge \tWeight")
        for i in range(1, self.num_vertices):
            print(parent[i], "-", i, "\t", self.weighted_matrix[i][parent[i]])

def read_graph(filename):
    graph = Graph()
    with open(filename, 'r') as file:
        lines = file.readlines()
        graph_data = [line.strip().split() for line in lines]
        graph.n = len(graph_data) - 1
        graph.setLabel(graph_data[0][1:])
        graph.setAMatrix([[int(x) for x in row[1:]] for row in graph_data[1:]], graph.n)
    return graph


if __name__ == "__main__":

    graph = read_graph("input.txt")

    print("Breadth-First Traversal:")
    graph.breadthFirstTraverse(0)  
    print("\n")

    print("Depth-First Traversal:")
    graph.depthFirstTraverse(0, [False] * 9)  
    print("\n")

    print("Shortest Paths from vertex 'a' using Dijkstra's algorithm:")
    w_graph = WGraph()
    w_graph.dijkstraShortestPath(graph, 0)
    print("\n")

    print("Minimum Spanning Tree using Prim's algorithm:")
    w_graph.primMST(graph)
