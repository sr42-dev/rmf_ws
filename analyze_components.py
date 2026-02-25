#!/usr/bin/env python3
"""Analyze nav graph connectivity and find connected components."""
import yaml
from collections import defaultdict, deque

with open("/tmp/rmf_nav_graph.yaml") as f:
    g = yaml.safe_load(f)

lanes = g["levels"]["L1"]["lanes"]
verts = g["levels"]["L1"]["vertices"]

# Build adjacency list (bidirectional)
adj = defaultdict(set)
for l in lanes:
    src, dst = l[0], l[1]
    bidir = True
    if len(l) > 2 and isinstance(l[2], dict):
        bidir = l[2].get("bidirectional", True)
    adj[src].add(dst)
    if bidir:
        adj[dst].add(src)

print(f"Total vertices: {len(verts)}")
print(f"Total lanes: {len(lanes)}")
print(f"Vertices with lanes: {len(adj)}")

# Find connected components via BFS
visited = set()
components = []

for start in adj:
    if start in visited:
        continue
    component = set()
    queue = deque([start])
    while queue:
        v = queue.popleft()
        if v in visited:
            continue
        visited.add(v)
        component.add(v)
        for n in adj[v]:
            if n not in visited:
                queue.append(n)
    components.append(component)

components.sort(key=len, reverse=True)
print(f"\nConnected components: {len(components)}")
for i, comp in enumerate(components):
    sample = sorted(comp)[:10]
    coords = [(round(verts[v][0], 2), round(verts[v][1], 2)) for v in sample]
    print(f"  Component {i}: {len(comp)} vertices, sample: {list(zip(sample[:5], coords[:5]))}")

# Check which component each checkpoint lane vertex maps to
checkpoints = [4860, 4873, 5373, 5402, 5697, 5762, 7049, 7058, 9468, 9363, 6474, 6476, 5148, 5136, 4861]
lane_verts = set()
for l in lanes:
    lane_verts.add(l[0])
    lane_verts.add(l[1])

print("\nCheckpoint analysis:")
for cp in checkpoints:
    cpx, cpy = round(verts[cp][0], 2), round(verts[cp][1], 2)
    
    # Find nearest lane vertex
    best_dist = float('inf')
    best_lv = None
    for lv in lane_verts:
        dx = verts[cp][0] - verts[lv][0]
        dy = verts[cp][1] - verts[lv][1]
        d = (dx*dx + dy*dy)**0.5
        if d < best_dist:
            best_dist = d
            best_lv = lv
    
    lvx, lvy = round(verts[best_lv][0], 2), round(verts[best_lv][1], 2)
    
    # Find which component
    comp_idx = -1
    for ci, comp in enumerate(components):
        if best_lv in comp:
            comp_idx = ci
            break
    
    print(f"  CP {cp} ({cpx},{cpy}) → LV {best_lv} ({lvx},{lvy}) dist={best_dist:.3f} → component {comp_idx}")

# Check specific: 4859 and 4873
print(f"\nVertex 4859 neighbors: {sorted(adj[4859])}")
print(f"Vertex 4873 neighbors: {sorted(adj[4873])}")

# Check if 4859→4880 is the lane from the sample
for l in lanes:
    if l[0] == 4859 or l[1] == 4859 or l[0] == 4873 or l[1] == 4873:
        s, d = l[0], l[1]
        print(f"  Lane: {s}({round(verts[s][0],2)},{round(verts[s][1],2)}) → {d}({round(verts[d][0],2)},{round(verts[d][1],2)})")
