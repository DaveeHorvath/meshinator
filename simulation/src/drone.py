#!/usr/bin/env python3

from copy import Error
import subprocess
import numpy as np

# the class containing a group of neightboring drones.
# @id: the unique id of the cluster of int type
class Cluster:
    def __init__(self, id: int):
        self.id: str = str(id)
        self.nodes: dict[str, Node] = {} # dict of all nodes in the cluster assigned by id

    def add_node(self, node: Node):
        if node.node_id in self.nodes:
            print("already a node in the cluster")
            return
        
        if self.nodes.__len__() >= 5:
            print("maximum number of nodes already reached")
            return
        
        self.nodes[node.node_id] = node
        node.cluster_id = self.id
        
        for n in self.nodes:
            if n != node:
                self.nodes[n].add_neighbor(node)
                node.add_neighbor(self.nodes[n])

    
    def remove_node(self, node: Node):
        if node.node_id not in self.nodes:
            print("node is not a current member of the cluster")
            return

        del self.nodes[node.node_id]
        node.cluster_id = None
        for n in self.nodes:
            self.nodes[n].remove_neighbor(node)
            node.remove_neighbor(self.nodes[n])

        if self.nodes.__len__() < 3:
            print("WARNING: not enough drones in the cluster.")

# each node represents one drone in the mesh structure.
# @id: the unique id of the drone in the mesh of int type
# @position: initial position of the drone in 3D space represented by the array of 3 floates
class Node:
    def __init__ (self, node_id: int, initial_position: list[int, int, int]):
        self.node_id: str = str(node_id)
        self.cluster_id: str = None # added when node joins the cluster
        self.position: list[int, int, int] = initial_position
        self.neightbors: dict[str, Node] = {} # node_id -> Node
        self.neightbors_position: dict[str, tuple[int, int, int]] = {} # node_id -> tuple of twolast recorded position for linear position prediction, oldest first
        self.neightbors_last_seen: dict[str, tuple[float]] = {} # node_id -> two last seen times, older first

        self.time: float = 0 # for storing when neightbors have been seen for the last time. will be measured by threads that run in the backgroud (global)

        self.lost_distances: dict[str, float] = {}

## Handling drones association and disassociation within one cluster

    def add_neighbor(self, neighbor: Node):

        if neighbor.node_id in self.neightbors:
            print("already a neightbord")
            return
        
        self.neighbors[neighbor.node_id] = neighbor
        self.neighbor_positions[neighbor.node_id] = tuple([0, 0, 0], [0, 0,0 ])
        self.neightbors_last_seen[neighbor.node_id] = [0.0, 0.0]
    
    def remove_neighbor(self, neighbor: Node):
        if neighbor.node_id not in self.neightbors:
            print("node is not a current neighbor")
            return

        self.neighbors.pop(neighbor.node_id, None)
        self.neighbor_positions.pop(neighbor.node_id, None)
        self.neightbors_last_seen.pop(neighbor.node_id, None)

##Handling message receiving and updating neightbor information

    def read_message(self, message: str):

        if message.split()[0] != "RECEIVE":
            print("invalid message format")
            return
        
        message_type: str = message.split()[1]
        sender_id: str = message.split()[2]

        if sender_id in self.neightbors or sender_id == self.node_id:
            if message_type == "LOST_DRONE":
                lost_uuid: str = message.split()[3]
                if lost_uuid == self.node_id:
                    return
                print("received lost drone message from node " + sender_id + " about node " + lost_uuid)
                self.handle_lost(lost_uuid, self.time) # update time to the received time of the message

        if message_type == "ALIVE_PING" and sender_id == self.node_id:
            sender_position = [float(x) for x in message.split()[3].split(",")]
            print("received alive ping from node " + sender_id + " at position " + str(sender_position))
            self.position = sender_position
            return

        if sender_id not in self.neightbors:
            return

        if message_type == "ALIVE_PING":
            sender_position: tuple[int, int, int] = [float(x) for x in message.split()[3].split(",")]
            print("received alive ping from node " + sender_id + " at position " + str(sender_position))
            self.neightbors_position[sender_id].drop(0)
            self.neightbors_position[sender_id].append(sender_position)
            self.neightbors_last_seen[sender_id].drop(0)
            self.neightbors_last_seen[sender_id].append(self.time)
            return
        elif message_type == "DRONE_MISSING":
            missing_uuid: str = message.split()[3]
            claster_id: str = message.split()[4]
            print("received drone missing message from node " + sender_id + " about node " + missing_uuid + " in cluster " + claster_id)
            return
        elif message_type == "LAST_SEEN":
            missing_uuid: str = message.split()[3]
            timestamp: float = float(message.split()[4])
            print("received drone last seen message from node " + sender_id + " about node " + missing_uuid + " last seen at " + str(timestamp))
            return
        else:
            print("message type not found")
            return

## Handling neightboring node lost

    def handle_lost(self, lost_id: str, current_time: float):
        v_lost: np.ndarray = (np.array(self.neightbors_position[lost_id][0]) - np.array(self.neightbors_position[lost_id][1])) / (self.self.neightbors_last_seen[0] - self.self.neightbors_last_seen[1])
        dt: float = current_time - self.self.neightbors_last_seen[1]
        predicted_position: np.ndarray = np.array(self.neightbors_position[lost_id][1]) + v_lost * dt
        print(self.node_id +": predicted position of the lost drone " + lost_id + " is " + str(predicted_position))

    # calculate the average distance from the drone to the missing neigthbour based on the array of distances provided by other neightbors
    def find_closest(self, predicted_positions: dict[str, np.ndarray]):

        distances_from_nodes: dict[str, float] = {}

        av_x: float = sum(x[0] for x in predicted_positions.values()) / len(predicted_positions)
        av_y: float = sum(x[1] for x in predicted_positions.values()) / len(predicted_positions)
        av_z: float = sum(x[2] for x in predicted_positions.values()) / len(predicted_positions)

        average_position = [av_x, av_y, av_z]

        distances_from_nodes[self.node_id] = np.linalg.norm(np.array(self.position) - np.array(average_position))

        for node in self.neightbors:
            difference: np.ndarray = np.array(self.neightbors_position[node][1]) - np.array(average_position)
            distance = np.linalg.norm(difference)
            distances_from_nodes[node] = distance

        closest_node: str = min(distances_from_nodes, key=distances_from_nodes.get)
        if closest_node == self.node_id:
            print(self.node_id + ": I am the closest node. I'll move to the avergage position to heal the mesh")
            self.position = np.array(average_position) / 2


if __name__ == "__main__":

    clusters: list[Cluster] = []
    nodes: list[Node] = []
    move: int = 0

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

