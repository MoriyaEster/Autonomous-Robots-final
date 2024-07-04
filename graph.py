import networkx as nx
import pygame

class Graph:
    def __init__(self):
        self.graph = nx.Graph()
        self.node_counter = 0

    def add_node(self, x, y):
        node_id = self.node_counter
        self.graph.add_node(node_id, pos=(x, y))
        self.node_counter += 1
        return node_id

    def add_edge(self, node1, node2):
        self.graph.add_edge(node1, node2)

    def draw(self, screen):
        pos = nx.get_node_attributes(self.graph, 'pos')
        for node, position in pos.items():
            pygame.draw.circle(screen, (0, 255, 0), position, 5)
        for edge in self.graph.edges:
            start_pos = pos[edge[0]]
            end_pos = pos[edge[1]]
            pygame.draw.line(screen, (0, 255, 0), start_pos, end_pos, 1)
