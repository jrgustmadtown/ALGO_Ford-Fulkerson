import sys
from collections import deque

inputy = list(map(int, sys.stdin.buffer.read().split()))
k = inputy[0]
index = 1


def update_flow(edges, flows, P, q):
    for i in range(len(P) - 1):
        u = P[i]
        v = P[i + 1]
        found = False
        for j in range(len(edges)):
            if edges[j] == (u, v):
                flows[j] += q
                found = True
                break
        if not found:
            for j in range(len(edges)):
                if edges[j] == (v, u):
                    flows[j] -= q
                    found = True
                    break


def P_finder_bfs(edges, capacities, s, t, n):
    adj = [[] for _ in range(n + 1)]
    cap = [[] for _ in range(n + 1)]

    # Build adjacency lists only for positive capacity edges
    for (u, v), c in zip(edges, capacities):
        if c > 0:
            adj[u].append(v)
            cap[u].append(c)

    parent = [-1] * (n + 1)
    visited = [False] * (n + 1)
    q = deque([s])
    visited[s] = True

    while q:
        u = q.popleft()
        for i in range(len(adj[u])):
            v = adj[u][i]
            c = cap[u][i]
            if not visited[v] and c > 0:
                parent[v] = u
                visited[v] = True
                if v == t:
                    # Reconstruct path and find bottleneck
                    P = []
                    bottleneck = float('inf')
                    x = t
                    while x != s:
                        P.append(x)
                        prev = parent[x]
                        # find capacity on this edge in residual graph
                        for (a, b), cc in zip(edges, capacities):
                            if a == prev and b == x and cc > 0:
                                bottleneck = min(bottleneck, cc)
                                break
                        x = prev
                    P.append(s)
                    P.reverse()
                    return P, bottleneck
                q.append(v)

    return None, -1


def residualmaker(edges, capacities, flows):
    Gf_edges = []
    Gf_capacities = []
    for i, (u, v) in enumerate(edges):
        # Forward residual
        fwd = capacities[i] - flows[i]
        # Backward residual
        back = flows[i]
        Gf_edges.append((u, v))
        Gf_capacities.append(max(fwd, 0))  # no negative residuals
        Gf_edges.append((v, u))
        Gf_capacities.append(max(back, 0))
    return Gf_edges, Gf_capacities


def FF(edges, capacities, s, t, n):
    flows = [0 for _ in capacities]
    maxflow = 0
    iteration = 0

    while True:
        iteration += 1
        if iteration > 10000:  # safety break (shouldn't ever hit)
            break
        Gf_edges, Gf_caps = residualmaker(edges, capacities, flows)
        P, q = P_finder_bfs(Gf_edges, Gf_caps, s, t, n)
        if q == -1 or P is None:
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
