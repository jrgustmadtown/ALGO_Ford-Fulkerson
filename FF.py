import sys
from collections import deque

inputy = list(map(int, sys.stdin.buffer.read().split()))
k = inputy[0]
index = 1

def update_flow(edges, flows, P, q):
    for i in range(len(P) - 1):
        u = P[i]
        v = P[i + 1]
        fixed = False
        for j in range(len(edges)):
            if edges[j] == (u, v):
                flows[j] += q
                fixed = True
                break
        if not fixed:
            for j in range(len(edges)):
                if edges[j] == (v, u):
                    flows[j] -= q
                    fixed = True
                    break

def P_finder_bfs(edges, capacities, s, t, n):
    adj = [[] for _ in range(n)]
    adj_c = [[] for _ in range(n)]

    for (u, v), c in zip(edges, capacities):
        if c > 0:
            adj[u].append(v)
            adj_c[u].append(c)

    visited = [False] * n
    parent = [-1] * n

    q = deque([s])
    visited[s] = True

    while q:
        u = q.popleft()
        if u == t:
            break
        for i in range(len(adj[u])):
            v = adj[u][i]
            c = adj_c[u][i]
            if not visited[v] and c > 0:
                visited[v] = True
                parent[v] = u
                q.append(v)

    if not visited[t]:
        return None, -1

    # reconstruct path
    P = []
    v = t
    while v != -1:
        P.append(v)
        v = parent[v]
    P.reverse()

    # bottleneck
    bottleneck = float('inf')
    for i in range(len(P) - 1):
        u = P[i]
        v = P[i + 1]
        for (x, y), c in zip(edges, capacities):
            if (x, y) == (u, v):
                bottleneck = min(bottleneck, c)
    return P, bottleneck

def residualmaker(edges, capacities, flows):
    Gf_edges = []
    Gf_capacities = []
    for i, edge in enumerate(edges):
        # forward
        Gf_edges.append(edge)
        Gf_capacities.append(capacities[i] - flows[i])
        # backward
        Gf_edges.append((edge[1], edge[0]))
        Gf_capacities.append(flows[i])
    return Gf_edges, Gf_capacities

def FF(edges, capacities, s, t, n):
    flows = [0 for _ in capacities]
    max_flow = 0

    while True:
        Gf_e, Gf_c = residualmaker(edges, capacities, flows)
        P, q = P_finder_bfs(Gf_e, Gf_c, s, t, n)
        if q == -1:
            break
        update_flow(edges, flows, P, q)
        max_flow += q

    return max_flow, flows


for _ in range(k):
    n = inputy[index]; index += 1
    m = inputy[index]; index += 1
    s = 0
    t = n - 1

    edges = []
    capacities = []
    for _ in range(m):
        edges.append((inputy[index], inputy[index + 1]))
        index += 2
        capacities.append(inputy[index])
        index += 1

    maxflow, flows = FF(edges, capacities, s, t, n)
    print(maxflow)
