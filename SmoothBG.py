import numpy as np 
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.animation

from IPython.display import HTML, display

from IPVISLazyPRM import visibilityPRMVisualizeWspace

from SmootherBase import SmootherBase

class SmoothBG(SmootherBase):
    def __init__(self):
        super().__init__()
    
    def smooth_path(self, path, planer, config, clean_up = False):
        '''        
        :param path: Collison free path of path planer
        :param planer: Used path planer algorithem for access to the graph and collisionchecker
        :param config: Configuration of smoother
        :return: smoothed path 
        '''
        
        if path == []:
            return []
        
        collision_free_path:list = list(path)
        self.path_planer = planer
        self.config:dict = config
        
        corner_threshold:float = self.config["corner_threshold"]
        epoches = self.config["epoches"]
        epoche_counter = 0
        
        '''
        Run the skip process. Starting with first node till the last one
        '''
        id = 0
        self.added_nodes = 0
        new_added_nodes_in_a_cascade = 0
      
        start_node_name = collision_free_path[id]
        while start_node_name != collision_free_path[-1] and not id + 2 >= len(collision_free_path) and epoche_counter != epoches:
            
            self.path_per_epoche.append(list(collision_free_path))
            epoche_counter += 1
            
            goal_node_name = collision_free_path[id + 2]
            skip_node_name = collision_free_path[id + 1]
            
            start_node = np.array(self.path_planer.graph.nodes[start_node_name]["pos"])
            skip_node = np.array(self.path_planer.graph.nodes[skip_node_name]["pos"])
            goal_node = np.array(self.path_planer.graph.nodes[goal_node_name]["pos"])
            
            
            # check id it's a edge which is worth to get skipped
            unshorted_length = np.linalg.norm(skip_node-start_node) + np.linalg.norm(goal_node - skip_node)
            shorted_length = np.linalg.norm(goal_node - start_node)
            
            new_nodes_generated = False
            
            if unshorted_length / shorted_length >= corner_threshold:

                if self.try_direct_skip(start_node, goal_node):
                    collision_free_path.remove(skip_node_name)
                else:                    
                    success, new_start, new_goal = self.try_deltree_skip(start_node, goal_node, skip_node)
                    if success:
                        collision_free_path.insert(id + 1, "new_start_node_" + str(self.added_nodes))
                        collision_free_path.insert(id + 2, "new_goal_node_" + str(self.added_nodes))
                        # add to graph
                        self.path_planer.graph.add_node("new_start_node_" + str(self.added_nodes), pos = list(new_start))
                        self.path_planer.graph.add_node("new_goal_node_" + str(self.added_nodes), pos = list(new_goal))
                        
                        self.path_planer.graph.add_edge(start_node_name, "new_start_node_" + str(self.added_nodes))
                        self.path_planer.graph.add_edge("new_start_node_" + str(self.added_nodes), "new_goal_node_" + str(self.added_nodes))
                        self.path_planer.graph.add_edge("new_goal_node_" + str(self.added_nodes), goal_node_name)
                        
                        
                        # remove skipped node
                        collision_free_path.remove(skip_node_name)

                        
                        
                        self.added_nodes += 1  
                        id += 1
                    else:
                        id += 1 # no skip possible
            else: 
                id += 1 # no relevant edge to skip
                
            
            # if new_nodes_generated:
            #     new_added_nodes_in_a_cascade += 1
            #     # skip next node in path (what is in that case a "new_start_node") and jump directly to the "new_goal_node"
            #     if new_added_nodes_in_a_cascade == max_new_node_in_cascade:
            #         new_added_nodes_in_a_cascade = 0
            #         id += 1
                    
            # else:
            #     new_added_nodes_in_a_cascade = 0
            
            start_node_name = collision_free_path[id]
            
            
            
        
        if clean_up:
            nodes = self.path_planer.graph.nodes()
            to_remove = []
            for node in nodes:
                if not node in collision_free_path and type(node) == type(""):
                    to_remove.append(node)
        
            for node in to_remove:
                self.path_planer.graph.remove_node(node)

        # # create edges
        # for i in range(1, len(collision_free_path)):
        #     self.path_planer.graph.add_edge(collision_free_path[i-1], collision_free_path[i])


        self.smoothed_path = collision_free_path
        return collision_free_path
                
            
                    
                
            
    
    def try_direct_skip(self, start, goal) -> bool:
        collision_intervals = self.config["collision_intervals"]
        
        unit_vector = (goal - start) / np.linalg.norm((goal - start))
        
        for i in range(1, collision_intervals + 1):
            if self.path_planer._collisionChecker.pointInCollision(start + unit_vector * i):
                return False
        
        return True
    
    def try_deltree_skip(self, start, goal, skip_node):
        max_deltree_depth:int = self.config["max_deltree_depth"]
        min_deltree_delta:float = self.config["min_deltree_delta"]
        
        '''
        From step to step the new_start and new_goal node are getting closer to the skipping node
        '''
        for k in range(1, max_deltree_depth + 1):
            new_start_node = start + (skip_node - start) / 2**k
            new_goal_node = skip_node + (goal - skip_node) / 2**k
            
            delta_1 = np.linalg.norm(skip_node - new_start_node)
            delta_2 = np.linalg.norm(new_goal_node - skip_node)
            
            if delta_1 <= min_deltree_delta or delta_2 <= min_deltree_delta:
                return False, None, None
            
            if self.try_direct_skip(new_start_node, new_goal_node):
                return True, new_start_node, new_goal_node
            
            start = new_start_node
            goal = new_goal_node

    # def visualize_smoothing(self, environment):
    #     figure = plt.figure(figsize=(7, 7))
    
    #     ax = figure.add_subplot(1, 1, 1)
        
        
    #     workSpaceLimits = environment.robot.getLimits()
                
    #     def animation(frame):
    #         ## clear taks space figure
    #         ax.cla()
    #         ## fix figure size
    #         ax.set_title("Path smoothing BG", fontsize=14)
    #         ax.set_xlim(workSpaceLimits[0])
    #         ax.set_ylim(workSpaceLimits[1])
    #         ## draw obstacles
    #         environment.drawObstacles(ax)
    #         ## update robot position

    #         graph = nx.Graph()
    #         for node in self.path_per_epoche[frame]:
    #             graph.add_node(node, pos = self.path_planer.graph.nodes[node]["pos"])
            
            
    #         for i in range(len(self.path_per_epoche[frame]) - 1):
    #             graph.add_edge(self.path_per_epoche[frame][i], self.path_per_epoche[frame][i + 1])

    #         pos = nx.get_node_attributes(graph,'pos')
    #         # todo extract from pos the first two dimensions only for drawing in workspace
    #         pos2D = dict()
    #         for key in pos.keys():
    #             pos2D[key] = (pos[key][0], pos[key][1])
                
    #         pos = pos2D

    #         nx.draw_networkx_nodes(graph, pos,  cmap=plt.cm.Blues, ax = ax, node_size=100)
    #         nx.draw_networkx_edges(graph,pos, ax = ax)
        
    #     ani = matplotlib.animation.FuncAnimation(figure, animation, frames=len(self.path_per_epoche))
    #     html = HTML(ani.to_jshtml())
    #     display(html)
    #     plt.close()
                    