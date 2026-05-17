#!/usr/bin/env python3
from copy import Error
import subprocess
import numpy as np
import select
import time

# the class containing a group of neightboring drones.
# @id: the unique id of the cluster of int type
class Cluster:
    def __init__(self, id: int):
        self.id: str = str(id)
        self.nodes: dict[str, Node] = {}

    def get_cluster_by_id(self, id: str):
        if id == self.id:
            return self
        else:
            print("cluster not found")
            return None

    def add_node(self, node: Node):
        if node.node_id in self.nodes:
            print("already a node in the cluster")
            return
        if self.nodes.__len__() >= 5:
            print("maximum number of nodes already reached")
            return
        for n in self.nodes:
            if n != node:
                self.nodes[n].add_neighbor(node)
                node.add_neighbor(self.nodes[n])
        self.nodes[node.node_id] = node
        node.cluster_id = self.id

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
# @position: initial position of the drone in 3D space represented by the array of 3 floats
class Node:
    def __init__(self, node_id: int, initial_position: list):
        self.node_id: str = str(node_id)
        self.cluster_id: str = None
        self.position: list = initial_position
        self.neightbors: dict[str, Node] = {}
        self.neightbors_position: dict[str, list] = {}  # node_id -> [prev_pos, last_pos]
        self.neightbors_last_seen: dict[str, list] = {}  # node_id -> [prev_time, last_time]
        self.can_die: bool = False
        self.time: float = 0.0
        self.lost_distances: dict[str, float] = {}
        self.range = 1.81

    def get_cluster(self):
        return Cluster.get_cluster_by_id(self.cluster_id)

    def update(self, d: dict, i, new_content):
        if i not in d:
            return None
        content = d[i]
        content_list = list(content)
        if content_list:
            del content_list[0]
        content_list.append(new_content)
        updated = tuple(content_list) if isinstance(content, tuple) else content_list
        d[i] = updated
        return updated

    ## Handling death
    def die(self):
        cluster = self.get_cluster()
        if cluster is not None:
            cluster.remove_node(self)
        return

    ## Handling drones association and disassociation within one cluster
    def add_neighbor(self, neighbor: Node):
        if neighbor.node_id in self.neightbors:
            print("already a neightbor")
            return
        self.neightbors[neighbor.node_id] = neighbor
        self.neightbors_position[neighbor.node_id] = [None, None]
        self.neightbors_last_seen[neighbor.node_id] = [0.0, 0.0]

    def remove_neighbor(self, neighbor: Node):
        if neighbor.node_id not in self.neightbors:
            print("node is not a current neighbor")
            return
        self.neightbors.pop(neighbor.node_id, None)
        self.neightbors_position.pop(neighbor.node_id, None)
        self.neightbors_last_seen.pop(neighbor.node_id, None)

    ## Handling message receiving and updating neighbor information
    def read_message(self, message: str):
        if not message:
            return
        parts = message.split()
        if len(parts) < 3 or parts[0] != "RECEIVING":
            return

        message_type: str = parts[1]
        sender_id: str = parts[2]

        if sender_id in self.neightbors or sender_id == self.node_id:
            if message_type == "LOST_DRONE":
                if len(parts) < 4:
                    return
                lost_uuid: str = parts[3]
                if lost_uuid == self.node_id:
                    return
                current_time: float = float(parts[4]) if len(parts) > 4 else self.time
                print("received lost drone message from node " + sender_id + " about node " + lost_uuid + "\n")
                self.handle_lost(lost_uuid, current_time)
                return

        if message_type == "ALIVE_PING" and sender_id == self.node_id:
            if len(parts) < 5:
                return
            sender_position = [float(x) for x in parts[3].split(",")]
            current_time: float = float(parts[4])
            print("current time: " + str(current_time) + "\n")
            self.position = sender_position
            self.time = current_time
            return

        # All further handling requires sender to be a known neighbor
        if sender_id not in self.neightbors:
            return

        if message_type == "ALIVE_PING":
            if len(parts) < 5:
                print("invalid ALIVE_PING message")
                return
            sender_position: list = [float(x) for x in parts[3].split(",")]
            current_time: float = float(parts[4])
            distance = self.find_distance(self.position, sender_position)
            
            if distance > self.range:
                print("received drone missing message from node " + self.node_id + " about node " + sender_id + " in cluster " + self.cluster_id + " at time: " + str(current_time) + "\n")
                print("received drone last seen message from node " + self.node_id + " about node " + sender_id + " ; current time: " + str(current_time) + "\n")
                print("received lost drone message from node " + self.node_id + " about node " + sender_id + "\n")
                self.handle_lost(sender_id, current_time)
            else:
                print("received alive ping from node " + sender_id + " at position " + str(sender_position) + " at time: " + str(current_time) + "\n")
                self.update(self.neightbors_position, sender_id, sender_position)
                self.update(self.neightbors_last_seen, sender_id, current_time)
            return

        elif message_type == "DRONE_MISSING":
            if len(parts) < 6:
                return
            missing_uuid: str = parts[3]
            cluster_id: str = parts[4]
            current_time: float = float(parts[5])
            print("received drone missing message from node " + sender_id + " about node " + missing_uuid + " in cluster " + cluster_id + " at time: " + str(current_time) + "\n")
            return

        elif message_type == "LAST_SEEN":
            if len(parts) < 5:
                return
            missing_uuid: str = parts[3]
            timestamp: float = float(parts[4])
            print("received drone last seen message from node " + sender_id + " about node " + missing_uuid + " last seen at " + str(timestamp) + "\n")
            return

    def find_distance(self, p1: list, p2: list):
        dif = np.array(p2) - np.array(p1)
        return np.linalg.norm(dif)

    ## Handling neighboring node lost
    def handle_lost(self, lost_id: str, current_time: float):
        if lost_id not in self.neightbors_position or lost_id not in self.neightbors_last_seen:
            print(self.node_id + ": no history for lost drone " + lost_id + "\n")
            return

        prev_pos, last_pos = self.neightbors_position[lost_id]
        prev_time, last_time = self.neightbors_last_seen[lost_id]

        if last_pos is None:
            print(self.node_id + ": no real position history yet for " + lost_id + "\n")
            return

        predicted_positions = self.find_predicted_positions(lost_id, current_time)

        if not predicted_positions:
            if last_time == prev_time:
                predicted_positions = {self.node_id: np.array(last_pos)}
            else:
                v_lost = (np.array(last_pos) - np.array(prev_pos)) / (last_time - prev_time)
                dt = current_time - last_time
                fallback_position = np.array(last_pos) + v_lost * dt
                predicted_positions = {self.node_id: fallback_position}
        else:
            if prev_pos is not None and last_time != prev_time:
                own_velocity = (np.array(last_pos) - np.array(prev_pos)) / (last_time - prev_time)
                own_predicted = np.array(last_pos) + own_velocity * (current_time - last_time)
                predicted_positions[self.node_id] = own_predicted

        self.find_closest(predicted_positions, lost_id)

    ## For simulation: neighbors share position data directly instead of via messages
    def find_predicted_positions(self, lost_id: str, current_time: float):
        predicted_positions: dict[str, np.ndarray] = {}
        for neighbor_id, neighbor in self.neightbors.items():
            if lost_id not in neighbor.neightbors_position or lost_id not in neighbor.neightbors_last_seen:
                continue
            prev_pos, last_pos = neighbor.neightbors_position[lost_id]
            prev_time, last_time = neighbor.neightbors_last_seen[lost_id]
            if last_pos is None or prev_pos is None:
                continue
            time_delta = last_time - prev_time
            if time_delta == 0:
                continue
            v_lost: np.ndarray = (np.array(last_pos) - np.array(prev_pos)) / time_delta
            dt: float = current_time - last_time
            predicted_positions[neighbor_id] = np.array(last_pos) + v_lost * dt
        return predicted_positions

    def find_closest(self, predicted_positions: dict[str, np.ndarray], lost_id: str):
        if not predicted_positions:
            print(self.node_id + ": no predicted positions to work with" + "\n")
            return

        values_list = list(predicted_positions.values())
        av_x: float = sum(x[0] for x in values_list) / len(values_list)
        av_y: float = sum(x[1] for x in values_list) / len(values_list)
        av_z: float = sum(x[2] for x in values_list) / len(values_list)
        average_position = np.array([av_x, av_y, av_z])

        distances_from_nodes: dict[str, float] = {}
        distances_from_nodes[self.node_id] = np.linalg.norm(np.array(self.position) - average_position)

        for node_id, neighbor in self.neightbors.items():
            if node_id == lost_id:
                continue
            distance = np.linalg.norm(np.array(neighbor.position) - average_position)
            distances_from_nodes[node_id] = distance

        closest_node: str = min(distances_from_nodes, key=distances_from_nodes.get)
        new_pos = average_position / 2
        print(f"{closest_node}: I am the closest node. Moving to predicted position: {new_pos} to heal the mesh and find node {lost_id}" + "\n")


        if closest_node == self.node_id:
            self.position = new_pos.tolist()

