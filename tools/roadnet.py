"""Road network graph for routing + NPC traffic, built from web/data/roads.json.

Welds road-polyline vertices into a graph (nodes = welded vertices, edges = road
segments with length), so we can (a) compute a shortest navigable route between two
scene points for SPL / route-following, and (b) let NPC vehicles follow real roads.
Pure stdlib + the baked roads.json.
"""
import heapq
import json
import math
import os

from tools.twin_server import DATA

_CELL = 4.0   # weld vertices to this grid (roads are resampled ~4-5 m; shares junctions)


class RoadGraph:
    def __init__(self, data_dir=None):
        self.nodes = []      # idx -> (x, z)
        self.adj = {}        # idx -> [(neighbor_idx, length), ...]
        self.ok = False
        path = os.path.join(data_dir or DATA, "roads.json")
        if os.path.exists(path):
            try:
                self._build(json.load(open(path)))
                self.ok = len(self.nodes) > 1
            except Exception:  # noqa: BLE001
                pass

    def _node(self, index, x, z):
        k = (round(x / _CELL), round(z / _CELL))
        i = index.get(k)
        if i is None:
            i = index[k] = len(self.nodes)
            self.nodes.append((x, z)); self.adj[i] = []
        return i

    def _build(self, r):
        index = {}
        for road in r.get("roads", []):
            prev = None
            for p in road["pts"]:
                n = self._node(index, p[0], p[2])
                if prev is not None and prev != n:
                    w = math.dist(self.nodes[prev], self.nodes[n])
                    self.adj[prev].append((n, w)); self.adj[n].append((prev, w))
                prev = n

    def nearest(self, x, z):
        best, bd = -1, 1e18
        for i, (nx, nz) in enumerate(self.nodes):
            d = (nx - x) ** 2 + (nz - z) ** 2
            if d < bd:
                bd, best = d, i
        return best

    def route(self, start, goal):
        """Shortest path (list of (x,z) waypoints) + its length, between two scene
        points. Falls back to the straight line if the graph can't connect them."""
        straight = math.dist(start, goal)
        if not self.ok:
            return [start, goal], straight
        s, g = self.nearest(*start), self.nearest(*goal)
        if s < 0 or g < 0:
            return [start, goal], straight
        dist = {s: 0.0}; prev = {}; pq = [(0.0, s)]
        while pq:
            d, u = heapq.heappop(pq)
            if u == g:
                break
            if d > dist.get(u, 1e18):
                continue
            for v, w in self.adj[u]:
                nd = d + w
                if nd < dist.get(v, 1e18):
                    dist[v] = nd; prev[v] = u; heapq.heappush(pq, (nd, v))
        if g not in dist:
            return [start, goal], straight
        path, u = [], g
        while u != s:
            path.append(self.nodes[u]); u = prev[u]
        path.append(self.nodes[s]); path.reverse()
        total = dist[g] + math.dist(start, self.nodes[s]) + math.dist(self.nodes[g], goal)
        return [start] + path + [goal], total

    def random_route(self, rng, start_node=None, hops=None):
        """A wandering route of node waypoints (for NPC roaming). Avoids immediate
        U-turns. Returns a list of (x,z)."""
        if not self.ok:
            return []
        u = start_node if start_node is not None else int(rng.integers(len(self.nodes)))
        hops = hops or int(rng.integers(8, 20))
        out = [self.nodes[u]]; prev = -1
        for _ in range(hops):
            nbrs = [v for v, _ in self.adj[u] if v != prev]
            if not nbrs:
                nbrs = [v for v, _ in self.adj[u]]
            if not nbrs:
                break
            prev, u = u, int(nbrs[int(rng.integers(len(nbrs)))])
            out.append(self.nodes[u])
        return out


_GRAPH = None


def shared_graph():
    global _GRAPH
    if _GRAPH is None:
        _GRAPH = RoadGraph()
    return _GRAPH
