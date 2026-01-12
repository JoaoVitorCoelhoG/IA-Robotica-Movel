from grid import Node, NodeGrid
from math import inf
import heapq


class PathPlanner(object):
    """
    Represents a path planner, which may use Dijkstra, Greedy Search or A* to plan a path.
    """
    def __init__(self, cost_map):
        """
        Creates a new path planner for a given cost map.

        :param cost_map: cost used in this path planner.
        :type cost_map: CostMap.
        """
        self.cost_map = cost_map
        self.node_grid = NodeGrid(cost_map)

    @staticmethod
    def construct_path(goal_node):
        """
        Extracts the path after a planning was executed.

        :param goal_node: node of the grid where the goal was found.
        :type goal_node: Node.
        :return: the path as a sequence of (x, y) positions: [(x1,y1),(x2,y2),(x3,y3),...,(xn,yn)].
        :rtype: list of tuples.
        """
        node = goal_node
        # Since we are going from the goal node to the start node following the parents, we
        # are transversing the path in reverse
        reversed_path = []
        while node is not None:
            reversed_path.append(node.get_position())
            node = node.parent
        return reversed_path[::-1]  # This syntax creates the reverse list


    def dijkstra(self, start_position, goal_position):
        """
        Plans a path using the Dijkstra algorithm.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
		# Todo: implement the Dijkstra algorithm
		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path
        path = []
        #actual_position = Node(Node.get_position(Node.set_position(start_position)))
        actual_position = self.node_grid.get_node(start_position[0],start_position[1])
        actual_position.f = 0 #PODE SER QUE ESTEJA ERRADO   
        actual_position.parent = None

        #end_position = Node(Node.get_position(Node.set_position(goal_position)))
        end_position = self.node_grid.get_node(goal_position[0],goal_position[1])

        heapq.heappush(path, (actual_position.f, actual_position))

        while path:
            
            lixo, actual_position = heapq.heappop(path)

            if actual_position.closed:
                continue

            actual_position.closed = True

            if actual_position == end_position:
                break

            for node in self.node_grid.get_successors(actual_position.i,actual_position.j):
                #node é uma coordenada
                neighborh_position = self.node_grid.get_node(node[0], node[1])

                if neighborh_position.closed:
                    continue

                #ERA PARA TER USADO get_edge_cost
                # self.cost_map.get_edge_cost(actual_position.get_position,node)
                custo = actual_position.f + self.cost_map.get_cell_cost(node[0],node[1]) + actual_position.distance_to(node[0],node[1])
                if neighborh_position.f > custo:
                    neighborh_position.f = custo
                    neighborh_position.parent = actual_position
                    heapq.heappush(path,(neighborh_position.f,neighborh_position))  
            var = (self.construct_path(end_position), end_position.f)

        self.node_grid.reset()

        return var[0],var[1]

    def greedy(self, start_position, goal_position):
        
        """
        Plans a path using Greedy.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
		# Todo: implement the A* algorithm
		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path
        path = []
        #actual_position = Node(Node.get_position(Node.set_position(start_position)))
        actual_position = self.node_grid.get_node(start_position[0],start_position[1])
        actual_position.f = 0 #PODE SER QUE ESTEJA ERRADO 
        actual_position.g = actual_position.distance_to(goal_position[0],goal_position[1])  
        actual_position.parent = None

        #end_position = Node(Node.get_position(Node.set_position(goal_position)))
        end_position = self.node_grid.get_node(goal_position[0],goal_position[1])

        heapq.heappush(path, (actual_position.g, actual_position))

        while path:
            
            lixo, actual_position = heapq.heappop(path)

            if actual_position.closed:
                continue

            actual_position.closed = True

            if actual_position == end_position:
                break

            for node in self.node_grid.get_successors(actual_position.i,actual_position.j):
                #node é uma coordenada
                neighborh_position = self.node_grid.get_node(node[0], node[1])

                if neighborh_position.closed:
                    continue
                
                custo = actual_position.f + self.cost_map.get_cell_cost(node[0],node[1]) + actual_position.distance_to(node[0],node[1])
                custo_star = neighborh_position.distance_to(goal_position[0],goal_position[1])
                
                if neighborh_position.g > custo_star:
                    neighborh_position.f = custo
                    neighborh_position.g = custo_star
                    neighborh_position.parent = actual_position
                    heapq.heappush(path,(custo_star,neighborh_position))  
            var = (self.construct_path(end_position), end_position.f)

        self.node_grid.reset()

        return var[0],var[1]
    
    def a_star(self, start_position, goal_position):
        """
        Plans a path using A*.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
		# Todo: implement the A* algorithm
		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path
        path = []
        #actual_position = Node(Node.get_position(Node.set_position(start_position)))
        actual_position = self.node_grid.get_node(start_position[0],start_position[1])
        actual_position.f = 0 #PODE SER QUE ESTEJA ERRADO 
        actual_position.g = actual_position.distance_to(goal_position[0],goal_position[1])  
        actual_position.parent = None

        #end_position = Node(Node.get_position(Node.set_position(goal_position)))
        end_position = self.node_grid.get_node(goal_position[0],goal_position[1])

        heapq.heappush(path, (actual_position.f + actual_position.g, actual_position))

        while path:
            
            lixo, actual_position = heapq.heappop(path)

            if actual_position.closed:
                continue

            actual_position.closed = True

            if actual_position == end_position:
                break

            for node in self.node_grid.get_successors(actual_position.i,actual_position.j):
                #node é uma coordenada
                neighborh_position = self.node_grid.get_node(node[0], node[1])

                if neighborh_position.closed:
                    continue
                
                custo = actual_position.f + self.cost_map.get_cell_cost(node[0],node[1]) + actual_position.distance_to(node[0],node[1])
                custo_star = neighborh_position.distance_to(goal_position[0],goal_position[1])
                
                if neighborh_position.f + neighborh_position.g > custo_star + custo:
                    neighborh_position.f = custo
                    neighborh_position.g = custo_star
                    neighborh_position.parent = actual_position
                    heapq.heappush(path,(custo + custo_star,neighborh_position))  
            var = (self.construct_path(end_position), end_position.f)

        self.node_grid.reset()

        return var[0],var[1]
