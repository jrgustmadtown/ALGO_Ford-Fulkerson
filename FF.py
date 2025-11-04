import sys
from collections import deque

data = list(map(int, sys.stdin.buffer.read().split()))
k = data[0]
idx = 1

def update_flow_by_mapping(flows, path_res_indices, res_meta, q):
    """
    flows: list of original-edge flows
    path_res_indices: list of indices (into residual edge list) forming the augmenting path
    res_meta: list of tuples (orig_edge_index, direction) for each residual edge
              direction = +1 for forward residual of original edge, -1 for backward residual
    q: bottleneck
    """
    for ridx in path_res_indices:
        orig_idx, direction = res_meta[ridx]
        if direction == 1:
            flows[orig_idx] += q
        else:
            flows[orig_idx] -= q

def build_residual(edges, capacities, flows):
    """
    Build residual-edge lists and metadata mapping to original edges.
    Returns:
      Gf_edges: list of (u,v)
      Gf_caps: list of residual capacities (non-negative)
      res_meta: list of (orig_index, direction) for each residual edge
                direction = +1 forward (original u->v), -1 backward (v->u)
    """
    Gf_edges = []
    Gf_caps = []
    res_meta = []
    for i, (u, v) in enumerate(edges):
        fwd = capacities[i] - flows[i]
        back = flows[i]
        # forward residual edge (u->v)
        Gf_edges.append((u, v))
        Gf_caps.append(fwd if fwd > 0 else 0)
        res_meta.append((i, 1))
        # backward residual edge (v->u)
        Gf_edges.append((v, u))
        Gf_caps.append(back if back > 0 else 0)
        res_meta.append((i, -1))
    return Gf_edges, Gf_caps, res_meta

def bfs_on_residual(Gf_edges, Gf_caps, s, t, n):
    """
    BFS on residual graph:
    - Returns (path_res_indices, bottleneck) where path_res_indices is list of
      residual-edge indices (into Gf_edges/Gf_caps) in order from s->t.
    - If no path, returns (None, 0)
    """
    # build adjacency lists for residual graph using residual-edge indices
    adj = [[] for _ in range(n + 1)]      # adjacency stores neighbor node
    adj_ridx = [[] for _ in range(n + 1)] # adjacency stores residual-edge indices
    for ridx, ((u, v), cap) in enumerate(zip(Gf_edges, Gf_caps)):
        if cap > 0:
            adj[u].append(v)
            adj_ridx[u].append(ridx)

    parent_node = [-1] * (n + 1)
    parent_ridx = [-1] * (n + 1)  # which residual-edge index was used to reach this node
    visited = [False] * (n + 1)

    dq = deque([s])
    visited[s] = True
    parent_node[s] = -1

    while dq:
        u = dq.popleft()
        if u == t:
            break
        for k_idx in range(len(adj[u])):
            v = adj[u][k_idx]
            ridx = adj_ridx[u][k_idx]
            if not visited[v]:
                visited[v] = True
                parent_node[v] = u
                parent_ridx[v] = ridx
                if v == t:
                    # reconstruct path (as residual-edge indices) and compute bottleneck
                    path_res = []
                    bottleneck = float('inf')
                    cur = t
                    while cur != s:
                        rid = parent_ridx[cur]
                        path_res.append(rid)
                        # use Gf_caps[rid] to update bottleneck
                        if Gf_caps[rid] < bottleneck:
                            bottleneck = Gf_caps[rid]
                        cur = parent_node[cur]
                    path_res.reverse()
                    return path_res, int(bottleneck)
                dq.append(v)

    return None, 0

def edmonds_karp(edges, capacities, s, t, n):
    m = len(edges)
    flows = [0] * m
    maxflow = 0

    while True:
        Gf_edges, Gf_caps, res_meta = build_residual(edges, capacities, flows)
        path_res_indices, bottleneck = bfs_on_residual(Gf_edges, Gf_caps, s, t, n)
        if path_res_indices is None or bottleneck == 0:
            break
        # update original flows using mapping
        update_flow_by_mapping(flows, path_res_indices, res_meta, bottleneck)
        maxflow += bottleneck

    return maxflow

out_lines = []
for _ in range(k):
    n = data[idx]; idx += 1
    m = data[idx]; idx += 1
    edges = []
    capacities = []
    for _e in range(m):
        u = data[idx]; v = data[idx+1]; idx += 2
        c = data[idx]; idx += 1
        edges.append((u, v))
        capacities.append(c)
    s, t = 1, n
    ans = edmonds_karp(edges, capacities, s, t, n)
    out_lines.append(str(ans))

# print results: one per line
print("\n".join(out_lines))
