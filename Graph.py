import sys
from MinHeap import MinHeap


class Vertex:
    def __init__(self, name, data):
        """
        Vertex class for graph data structure.
        :param name: Name of the vertex.
        :param data: Additional associated data.
        :return: None
        """
        self.data = data
        self.name = name
        # self.adj_vertices = list()
        self.adj_vertices = dict()
        self.d = sys.maxsize
        self.pi = None
        self.pq_index = -1

    def set_d(self, d):
        self.d = d

    def remove_edge(self, destination):
        """
        Removes an edge. raise key error if the object is not found.
        This O(1) operation since the self.adj_vertices is a dictionary.
        :param destination: vertex object
        :return: None.
        """
        self.adj_vertices.pop(destination.name)

    def __lt__(self, other):
        return self.d < other.d

    def __repr__(self):
        return self.data


class Edge:
    def __init__(self, source, destination, weight):
        """
        Edge class to store edge information.
        :param source: Source vertex object.
        :param destination: Destination vertex object.
        :param weight: Edge weight which is a number.
        :return: None
        """
        self.source = source
        self.destination = destination
        self.weight = weight

    def __repr__(self):
        return "Source: %r Destination: %r Weight: %d" %(self.source.name, self.destination.name, self.weight)


class ObjectAlreadyPresentException(Exception):
    pass


class ObjectNotFound(Exception):
    pass


class Graph:
    def __init__(self):
        """
        Graph data structure.
        self.vertex_map holds all the vertex mapped by its name. Hence the name needs to hashable.
        :return:
        """
        self.vertex_map = dict()

    def add_vertex(self, node_name, data=None):
        """
        Add a new vertex.
        :param node_name: Name of the vertex.
        :param data: Associated data object.
        :return:
        """
        if node_name not in self.vertex_map:
            self.vertex_map[node_name] = Vertex(node_name, data)

    def has_key(self, k):
        """
        To check is a vertex k exists of not.
        :param k: Vertex name.
        :return: boolean. True if present, False if not.
        """
        return k in self.vertex_map

    def add_edge(self, source, destination, weight, source_data=None, destination_data=None, directed=False,
                 ignore_previous_destination_data_validation=True, set_destination_parent=False):
        """
        Method to add a new edge.
        :param source: Source object.
        :param destination: Destination object.
        :param weight: Edge weight.
        :param source_data: Optional source_data if adding the source vertex for the first time.
        :param destination_data: Optional destination_data if adding the destination vertex for the first time.
        :param directed: True for undirected and False for directed.
        :param ignore_previous_destination_data_validation: If set to True, raise ObjectAlreadyPresentException
        if destination is present already. If set to False, the method ignore this check.
        :param set_destination_parent: If directed it set to True and set_destination_parent set to True, then
        each child will have it parent stored in variable pi in vertex object.
        :return: None
        """

        if source not in self.vertex_map:
            self.vertex_map[source] = Vertex(source, source_data)
        if destination not in self.vertex_map:
            self.vertex_map[destination] = Vertex(destination, destination_data)
        else:
            if not ignore_previous_destination_data_validation:
                raise ObjectAlreadyPresentException

        e1 = Edge(self.vertex_map[source], self.vertex_map[destination], weight)

        # self.vertex_map[source].adj_vertices.append(e1)
        self.vertex_map[source].adj_vertices[e1.destination.name] = e1
        if directed and set_destination_parent:
            self.vertex_map[destination].pi = self.vertex_map[source]
        if not directed:
            e2 = Edge(self.vertex_map[destination], self.vertex_map[source], weight)
            # self.vertex_map[destination].adj_vertices.append(e2)
            self.vertex_map[destination].adj_vertices[e2.destination.name] = e2

    def min_path(self, source, goal, return_as_list=False):
        """
        Dijkstra algorithm.
        :param source: Source name
        :param goal: Destination name
        :param return_as_list: Boolean variable which determines the return output.
        :return: If return_as_list is set to True, return a list of vertex names as a list else, None.
        """
        # Dijkstra algorithm
        for u in self.vertex_map.values():
            u.d = sys.maxsize
            u.pi = None
        self.vertex_map[source].d = 0

        heap = MinHeap(list(self.vertex_map.values()), key=lambda x: x.d)
        while len(heap) > 0:
            mn = heap.extract_min()
            # for ed in mn.adj_vertices:
            for ed in mn.adj_vertices.values():
                if ed.destination.d > ed.source.d + ed.weight:
                    ed.destination.pi = ed.source
                    heap.decrease_key(ed.destination, ed.source.d + ed.weight, ed.destination.set_d)
                    # heapq.heapify(heap)
        if return_as_list:
            out_list = list()
            t_vertex = self.vertex_map[goal]
            while t_vertex is not None:
                out_list.append(t_vertex)
                t_vertex = t_vertex.pi
            out_list.reverse()
            return out_list

    def traverse_to(self, source, destination):
        """
        To traverse from child to parent until the parent's pi variable is None.
        :param source: Source object name.
        :param destination: Destination object name.
        :return: A list of traversal path.
        """
        if source not in self.vertex_map or destination not in self.vertex_map:
            raise ObjectNotFound
        out_list = list()
        t_vertex = self.vertex_map[destination]
        while t_vertex is not None:
            out_list.append(t_vertex)
            t_vertex = t_vertex.pi
        out_list.reverse()
        return out_list

    def __repr__(self):
        return str(self.vertex_map.keys())
