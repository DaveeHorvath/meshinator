#!/usr/bin/env python3

from copy import Error
import subprocess

# the class containing a group of neightboring drones.
# @id: the unique id of the cluster of int type
class Cluster:
    def __init__(self, id):
        self.id = id
        self.nodes = [] # list of all nodes in the cluster. each cluster has between 3 and 5 nodes


    def get_cluster_by_id(self, id):
        if self.id == id:
            return self
        else:
            print("cluster not found")

    def add_node(self, node):
        if node in self.nodes:
            print("already a node in the cluster")
            return
        
        if self.nodes.__len__() >= 5:
            print("maximum number of nodes already reached")
            return
        
        self.nodes.append(node)
        
        node.cluster_id = self.id
        
        for n in self.nodes:
            if n != node:
                n.add_neighbor(node)
                node.add_neighbor(n)

    
    def remove_node(self, node):
        if node not in self.nodes:
            print("node is not a current member of the cluster")
            return

        self.nodes.remove(node)
        node.cluster_id = None
        for n in self.nodes:
            n.remove_neighbor(node)
            node.remove_neighbor(n)

        if self.nodes.__len__() < 3:
            print("WARNING: not enough drones in the cluster.")

# each node represents one drone in the mesh structure.
# @id: the unique id of the drone in the mesh of int type
# @position: initial position of the drone in 3D space represented by the array of 3 floates
class Node:
    def __init__ (self, node_id, initial_position):
        self.node_id = node_id
        self.cluster_id = None # added when node joins the cluster
        self.position = initial_position
        self.neightbors = {} # node_id -> Node
        self.neightbors_position = {} # node_id -> last recorded position
        self.neightbors_last_seen = {} # node_id -> last seen time 

        self.time = 0 # for storing when neightbors have been seen for the last time. will be measured by threads that run in the backgroud (global)


    def add_neighbor(self, neighbor: Node):

        if neighbor in self.neightbors:
            print("already a neightbord")
            return
        
        self.neighbors[neighbor.node_id] = 0
        self.neighbor_positions[neighbor.node_id] = [0.0, 0.0, 0.0]
        self.neightbors_last_seen[neighbor.node_id] = 0.0
    
    def remove_neighbor(self, neighbor):
        if neighbor not in self.neightbors:
            print("node is not a current neighbor")
            return

        self.neighbors.pop(neighbor.node_id, None)
        self.neighbor_positions.pop(neighbor.node_id, None)
        self.neightbors_last_seen.pop(neighbor.node_id, None)

    def read_message(self, message: str):

        if message.split()[0] != "RECEIVE":
            print("invalid message format")
            return
        
        message_type = message.split()[1]
        sender_id = message.split()[2]

        if sender_id not in self.neightbors:
            return

        if message_type == "ALIVE_PING":
            sender_position = message.split()[3:6]
            print("received alive ping from node " + sender_id + " at position " + str(sender_position))
        elif message_type == "DRONE_MISSING":
            missing_uuid = message.split()[3]
            claster_id = message.split()[4]
            print("received drone missing message from node " + sender_id + " about node " + missing_uuid + " in cluster " + claster_id)
        elif message_type == "LAST_SEEN":
            missing_uuid = message.split()[3]
            timestamp = message.split()[4]
            print("received drone last seen message from node " + sender_id + " about node " + missing_uuid + " last seen at " + timestamp)
        else:
            print("message type not found")
            return


if __name__ == "__main__":

    clusters = []
    nodes = []
    move = 0

    for i in range(1, 10):
        node = Node(i, [0.0, 0.0, 0.0])
        nodes.append(node)

    for i in range(1, 4):
        cluster = Cluster(i)
        clusters.append(cluster)

        for j in range (3):
            cluster.add_node(nodes[move])
            move += 1

    print(clusters)
    print(nodes)

    proc = subprocess.Popen(["cat", "/etc/services"], stdout=subprocess.PIPE, shell=True) # change the name to proper c file

    for line in proc.stdout:
        try:
            message = line.decode("utf-8").strip()
            for node in nodes:
                node.read_message(message)
        except Error as e:
            print("failed to read the line: " + str(e))
            continue

