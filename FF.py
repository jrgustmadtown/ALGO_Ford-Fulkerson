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
    adj = [[] for _ in range(n + 1)]
    cap = [[] for _ in range(n + 1)]
    edge_index = [[] for _ in range(n + 1)]

    # build adjacency structure
    for i, ((u, v), c) in enumerate(zip(edges, capacities)):
        if c > 0:
            adj[u].append(v)
            cap[u].append(c)
            edge_index[u].append(i)

    visited = [False] * (n + 1)
    parent = [-1] * (n + 1)
    parent_edge = [-1] * (n + 1)

    q = deque([s])
    visited[s] = True

    while q:
        u = q.popleft()
        for i in range(len(adj[u])):
            v = adj[u][i]
            c = cap[u][i]
            if not visited[v] and c > 0:
                visited[v] = True
                parent[v] = u
                parent_edge[v] = edge_index[u][i]
                if v == t:
                    # reconstruct path
                    P = []
                    bottleneck = float('inf')
                    x = t
                    while x != s:
                        P.append(x)
                        eidx = parent_edge[x]
                        bottleneck = min(bottleneck, capacities[eidx])
                        x = parent[x]
                    P.append(s)
                    P.reverse()
                    return P, bottleneck
                q.append(v)

    return None, -1


def residualmaker(edges, capacities, flows):
    Gf_edges = []
    Gf_capacities = []
    for i, (u, v) in enumerate(edges):
        # forward
        Gf_edges.append((u, v))
        Gf_capacities.append(capacities[i] - flows[i])
        # backward
        Gf_edges.append((v, u))
        Gf_capacities.append(flows[i])
    return Gf_edges, Gf_capacities


def FF(edges, capacities, s, t, n):
    flows = [0 for _ in capacities]
    maxflow = 0

    while True:
        Gf_edges, Gf_caps = residualmaker(edges, capacities, flows)
        P, q = P_finder_bfs(Gf_edges, Gf_caps, s, t, n)
        if q == -1:
            break
        update_flow(edges, flows, P, q)
        maxflow += q

    return maxflow


for _ in range(k):
    n = inputy[index]; index += 1
    m = inputy[index]; index += 1

    edges = []
    capacities = []
    for _ in range(m):
        edges.append((inputy[index], inputy[index + 1]))
        index += 2
        capacities.append(inputy[index])
        index += 1

    s, t = 1, n
    sol = FF(edges, capacities, s, t, n)
    print(sol)
