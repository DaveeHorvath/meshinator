#!/usr/bin/env python3
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from drone import Cluster, Node
from copy import Error
import subprocess
import numpy as np
import select
import time 

if __name__ == "__main__":

    clusters: list[Cluster] = []
    nodes: list[Node] = []
    move: int = 0

    for i in range(3):
        node = Node(i, [0.0, 0.0, 0.0])
        nodes.append(node)

    for i in range(1):
        cluster = Cluster(i)
        clusters.append(cluster)

        for j in range (3):
            cluster.add_node(nodes[move])
            move += 1

    print(clusters)
    print(nodes)

    # with subprocess.Popen(["ifconfig"], stdout= subprocess.PIPE) as proc:
    #     print(proc.stdout.read())
    path = "/home/dhorvath/Desktop/hack_defence/meshinator_davids_abomination/examples_c"
    # pon = subprocess.Popen(["./run_drone", "--logging", "./0.toml"], stdout=subprocess.PIPE,)
    # while True:
    #     print(pon.stdout.readline())
    procs = [
        subprocess.Popen([f"./run_drone", "--logging", f"./0.toml"], stdout=subprocess.PIPE, shell=False),
        subprocess.Popen([f"./run_drone", "--logging", f"./1.toml"], stdout=subprocess.PIPE, shell=False),
        subprocess.Popen([f"./run_drone", "--logging", f"./2.toml"], stdout=subprocess.PIPE, shell=False)
    ]

    # for i, p in enumerate(procs):
    #     print(f"proc {i} pid={p.pid}, returncode={p.returncode}")

    while True:
        for p in procs:
            try:
                out = p.stdout.readline().decode("utf-8")
                if len(out) == 0:
                    continue
                #print(out)
                for node in nodes:
                    node.read_message(out)
                    
            except Error as e:
                print("failed to read the line: " + str(e))
                continue



