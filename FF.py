import sys
sys.setrecursionlimit(10**7)

inputy = list(map(int, sys.stdin.buffer.read().split()))
k = inputy[0]
index = 1


def dfs(u, t, flow, edges, capacities, flows, visited):
    if u == t:
        return flow
    visited[u] = True

    for i, (a, b) in enumerate(edges):
        if a == u and not visited[b] and capacities[i] - flows[i] > 0:
            pushed = dfs(b, t, min(flow, capacities[i] - flows[i]),
                         edges, capacities, flows, visited)
            if pushed > 0:
                flows[i] += pushed
                return pushed

        if b == u and not visited[a] and flows[i] > 0:
            pushed = dfs(a, t, min(flow, flows[i]),
                         edges, capacities, flows, visited)
            if pushed > 0:
                flows[i] -= pushed
                return pushed
    return 0


def ford_fulkerson(edges, capacities, s, t, n):
    flows = [0 for _ in capacities]
    maxflow = 0

    while True:
        visited = [False] * (n + 1)
        pushed = dfs(s, t, float('inf'), edges, capacities, flows, visited)
        if pushed == 0:
            break
        maxflow += pushed

    return maxflow


for _ in range(k):
    n = inputy[index]; index += 1
    m = inputy[index]; index += 1

    edges = []
    capacities = []
    for _ in range(m):
        u = inputy[index]; v = inputy[index + 1]; c = inputy[index + 2]
        index += 3
        if c > 0:
            edges.append((u, v))
            capacities.append(c)

    s, t = 1, n
    sol = ford_fulkerson(edges, capacities, s, t, n)
    print(sol)
