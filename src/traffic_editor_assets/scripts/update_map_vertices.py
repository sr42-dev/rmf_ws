import json
import yaml
import math
import os
import sys

'''
Requirements definition:

write a script that accepts png image map.png.
use opencv to filter out the white part of the image.
I want to fill this white space with as many points as possible such that no two points are less than 10 units away (units being the displacement between their coordinates) from each other.
collect these points and scale them such that the coordinate of the top left most part of the white region is 250.9980, 62.9970. the origins of the image and the resultant are the same.

now with the resultant list of points such that each point is x,y, append
- [x, y, 0, ""] to vertices in map.building.yaml 

and

append
- [n1, n2, {bidirectional: [4, true], demo_mock_floor_name: [1, ""], demo_mock_lift_name: [1, ""], graph_idx: [2, 0], mutex: [1, ""], orientation: [1, ""], speed_limit: [3, 0]}] to lanes in map.building.yaml

where n1 and n2 are the indices of each successive point in vertices.

when appending n1, n2 to lanes, I want it to show up as

lanes:
    - [n1, n2, {bidirectional: [4, true], demo_mock_floor_name: [1, ""], demo_mock_lift_name: [1, ""], graph_idx: [2, 0], mutex: [1, ""], orientation: [1, ""], speed_limit: [3, 0]}]

and not 

lanes: 
-  n1
      - n2
      - bidirectional: [4, true]
        demo_mock_floor_name: [1, '']
        demo_mock_lift_name: [1, '']
        graph_idx: [2, 0]
        mutex: [1, '']
        orientation: [1, '']
        speed_limit: [3, 0]


and optimize the algorithm such that

1. the minimum distance is checked against vertices already in map.building.yaml and vertices are not added when they are too close to an existing vertex.

2. if multiple vertices lie along the same axis, then instead of connecting each one with its own lane, connect only the ones on the edge.

what this means is that if i have points x,y1 x,y2 x,y3, then only connect the indexes of the point with the lowest and highest y.

likewise when you have points x1,y x2,y x3,y.

3. when you append to vertices or lanes, leave two spaces before you start. dont append at the same level as vertices/lanes
'''

# Two points in the original map and two points in the transformed rmf map
# Don't choose points that lie along the same axis
# Zoom in as much as possible when sampling these points
ORIG_P1 = [2.55, 21.68]
ORIG_P2 = [61.503, 17.38]
TRANS_P1 = [302.5024, 545.9836]
TRANS_P2 = [1481.6013, 632.0320]

def calculate_transform_without_rotation(orig_p1, orig_p2, trans_p1, trans_p2):
    """
    Calculate scale and shift transformation parameters from two point pairs.
    
    Args:
        orig_p1: Original point 1 as [x, y]
        orig_p2: Original point 2 as [x, y]
        trans_p1: Transformed point 1 as [x, y]
        trans_p2: Transformed point 2 as [x, y]
    
    Returns:
        [scale_x, scale_y, shift_x, shift_y]
    """
    # Calculate scale factors
    dx_orig = orig_p2[0] - orig_p1[0]
    dy_orig = orig_p2[1] - orig_p1[1]
    dx_trans = trans_p2[0] - trans_p1[0]
    dy_trans = trans_p2[1] - trans_p1[1]
    
    # Avoid division by zero
    if abs(dx_orig) < 1e-10 or abs(dy_orig) < 1e-10:
        raise ValueError("Original points must have non-zero distance in both x and y")
    
    scale_x = dx_trans / dx_orig
    scale_y = dy_trans / dy_orig
    
    # Calculate shift using first point
    # trans = orig * scale + shift
    # shift = trans - orig * scale
    shift_x = trans_p1[0] - (orig_p1[0] * scale_x)
    shift_y = trans_p1[1] - (orig_p1[1] * scale_y)
    
    return [scale_x, scale_y, shift_x, shift_y]

def truncate(n):
    return math.floor(n * 100) / 100.0

def main():
    # Determine base directory based on script location
    # Assumes script is in src/rmf/traffic_editor_assets/scripts/
    script_dir = os.path.dirname(os.path.abspath(__file__))
    assets_dir = os.path.dirname(script_dir) # Parent directory: src/rmf/traffic_editor_assets/
    
    maps_json_path = os.path.join(assets_dir, 'maps.json')
    map_building_yaml_path = os.path.join(assets_dir, 'map.building.yaml')

    if not os.path.exists(maps_json_path):
        print(f"Error: {maps_json_path} not found.")
        return
    
    if not os.path.exists(map_building_yaml_path):
        print(f"Error: {map_building_yaml_path} not found.")
        return

    print(f"Loading {maps_json_path}...")
    with open(maps_json_path, 'r') as f:
        maps_data = json.load(f)

    if not maps_data.get('maps'):
        print("No maps found in maps.json")
        return

    # Process the first map found
    map_data = maps_data['maps'][0]
    spots = map_data.get('spots', [])
    
    print(f"Found {len(spots)} spots.")

    SCALE_X, SCALE_Y, SHIFT_X, SHIFT_Y = calculate_transform_without_rotation(ORIG_P1, ORIG_P2, TRANS_P1, TRANS_P2)

    new_vertices = []
    for spot in spots:
        coords = None
        # Try to find coordinates in 'pos' (preferred) or 'nav_pos'
        if 'pos' in spot and 'coordinates' in spot['pos']:
            coords = spot['pos']['coordinates']
        elif 'nav_pos' in spot and 'coordinates' in spot['nav_pos']:
            coords = spot['nav_pos']['coordinates']
        
        if coords and len(coords) >= 2:
            # Apply scale and shift
            x = truncate((coords[0] * SCALE_X) + SHIFT_X)
            y = truncate((coords[1] * SCALE_Y) + SHIFT_Y)
            # Create vertex element: [x, y, 0, ""]
            new_vertices.append([x, y, 0, ""])

    print(f"Extracted {len(new_vertices)} vertices.")

    print(f"Loading {map_building_yaml_path}...")
    with open(map_building_yaml_path, 'r') as f:
        building_data = yaml.safe_load(f)

    # Append to Level L1
    if 'levels' in building_data and 'L1' in building_data['levels']:
        if 'vertices' not in building_data['levels']['L1'] or building_data['levels']['L1']['vertices'] is None:
            building_data['levels']['L1']['vertices'] = []
        
        initial_count = len(building_data['levels']['L1']['vertices'])
        
        # Append new vertices
        building_data['levels']['L1']['vertices'].extend(new_vertices)
        
        print(f"Appending {len(new_vertices)} vertices to existing {initial_count}...")
        
        print(f"Writing updated {map_building_yaml_path}...")
        with open(map_building_yaml_path, 'w') as f:
            # default_flow_style=None allows the dumper to choose flow style for simple lists like the vertices
            yaml.dump(building_data, f, default_flow_style=None)
        
        print("Done.")
    else:
        print("Error: Could not find levels.L1 in map.building.yaml")

if __name__ == "__main__":
    main()
