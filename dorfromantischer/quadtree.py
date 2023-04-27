from typing import Any
import numpy as np
from collections import deque, namedtuple

Pos = namedtuple("Pos", "s t")

class Quadtree:
    root_s: int
    root_t: int
    root_width: int

    # 4 elements, iterating row-wise from small to large.
    nodes: list[int]

    def _gen_quadtree(self, leafs: list[tuple[Pos, Any]], index_step_size: int) -> list[tuple[Pos, Any]]:
        nodes = {}
        for (s, t), leaf in leafs:
            # Position of the quad_tree.
            node_s = s & (~index_step_size)
            node_t = t & (~index_step_size)
            node_pos = (node_s, node_t)

            node = nodes.get(node_pos)
            if not node:
                node = [None] * 4

            leaf_pos = 2 * int(t > node_t) + int(s > node_s)
            node[leaf_pos] = leaf
            nodes[node_pos] = node

        return list(nodes.items())

    def __init__(self, items: list[tuple[Pos, Any]]):
        """Incoming indices MUST be positive!"""
        index_step_size = 1
        while len(items) > 1:
            items = self._gen_quadtree(items, index_step_size)
            index_step_size <<= 1

        # `index_step_size` is now the width of the root rect.
        self.root_width = index_step_size
        [((self.root_s, self.root_t), root_node)] = items

        leaf_bit = np.int32(1) << np.int32(31)
        nodes: list[int] = []
        queue = deque([root_node])
        next_free_index = 1 # 0 is root node.
        while queue:
            node = queue.popleft()
            for branch in node:
                if branch is None:
                    nodes.append(-1)
                elif isinstance(branch, int):
                    nodes.append(int(branch | leaf_bit))
                else:
                    nodes.append(next_free_index)
                    next_free_index += 1
                    queue.append(branch)

        self.nodes = nodes
