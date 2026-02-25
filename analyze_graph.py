#!/usr/bin/env python3
import yaml

with open("/tmp/rmf_nav_graph.yaml") as f:
    g = yaml.safe_load(f)

lanes = g["levels"]["L1"]["lanes"]
verts = g["levels"]["L1"]["vertices"]

vertices_in_lanes = set()
for l in lanes:
    vertices_in_lanes.add(l[0])
    vertices_in_lanes.add(l[1])

print(f"Total vertices: {len(verts)}")
print(f"Total lanes: {len(lanes)}")
print(f"Unique vertices in lanes: {len(vertices_in_lanes)}")
print(f"Sample vertices in lanes: {sorted(vertices_in_lanes)[:30]}...")

checkpoints = [4860, 4873, 5373, 5402, 5697, 5762, 7049, 7058, 9468, 9363, 6474, 6476, 5148, 5136, 4861]
for cp in checkpoints:
    connected = cp in vertices_in_lanes
    neighbors = []
    for l in lanes:
        if l[0] == cp:
            neighbors.append(l[1])
        if l[1] == cp:
            neighbors.append(l[0])
    v = verts[cp]
    print(f"  CP {cp} ({v[0]:.2f},{v[1]:.2f}): in_lanes={connected}, neighbors={neighbors}")

# Show a few sample lanes  
print("\nSample lanes:")
for l in lanes[:10]:
    s, d = l[0], l[1]
    sv, dv = verts[s], verts[d]
    print(f"  {s}({sv[0]:.2f},{sv[1]:.2f}) -> {d}({dv[0]:.2f},{dv[1]:.2f})")
