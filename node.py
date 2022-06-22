import numpy as np
import random


class Node:
    def __init__(self, id):
        self.id = id
        self.attr = {
            "NEIGHBORS": [],
            "EDGES": [],
            "N_POS": np.array([random.random(), random.random()]),
            "POS_INI": [random.randint(5, 70), random.randint(5, 70)]
        }
        self.attr["DEGREE"] = len(self.attr["NEIGHBORS"])
        self.attr["POS_FIN"] = self.attr["POS_INI"]
