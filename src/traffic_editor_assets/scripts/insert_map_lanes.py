import cv2
import numpy as np
import yaml
import math
import os

'''
Requirements definition:

write a script to get all spot coordinate values from maps.json maps.spots.[].coordinates and truncate them to two decimal points.

then take these points as x,y pairs and append them into map.building.yaml as elements of the form [x, y, 0, ""].

'''

# Configuration
IMAGE_FILENAME = 'map.png'
YAML_FILENAME = 'map.building.yaml'
MIN_DISTANCE = 10.0
TARGET_TOP_LEFT = (250.9980, 62.9970)

# ----------------- YAML Customization -----------------
class FlowList(list):
    """A list that will be dumped in flow style (inline) in YAML."""
    pass

def flow_list_representer(dumper, data):
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)

class IndentDumper(yaml.SafeDumper):
    def increase_indent(self, flow=False, indentless=False):
        # Force indent for lists (indentless=False)
        return super(IndentDumper, self).increase_indent(flow, False)

# Register for the custom Dumper
IndentDumper.add_representer(FlowList, flow_list_representer)
# ------------------------------------------------------

def truncate(n):
    return math.floor(n * 10000) / 10000.0

def load_image(path):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Could not load image at {path}")
    return img

def get_white_pixel_coords(img):
    y_coords, x_coords = np.where(img > 250)
    # Return as list of (x, y) tuples
    return list(zip(x_coords, y_coords))

def _wrap_props(props):
    new_props = {}
    for k, val in props.items():
        if isinstance(val, list):
            new_props[k] = FlowList(val)
        else:
            new_props[k] = val
    return new_props

def load_existing_vertices(level_l1):
    """Parse existing vertices into a list of [x, y] coordinates."""
    pts = []
    if 'vertices' in level_l1 and level_l1['vertices']:
        for v in level_l1['vertices']:
            if len(v) >= 2:
                pts.append((v[0], v[1]))
    return pts

def build_spatial_map(points, cell_size):
    """Build a simple grid-based spatial map for querying."""
    s_map = {}
    for i, (x, y) in enumerate(points):
        gx = int(x / cell_size)
        gy = int(y / cell_size)
        key = (gx, gy)
        if key not in s_map:
            s_map[key] = []
        s_map[key].append((x, y))
    return s_map

def is_too_close(x, y, s_map, cell_size, min_dist):
    """Check if (x,y) is within min_dist of any point in s_map."""
    gx = int(x / cell_size)
    gy = int(y / cell_size)
    # Check 3x3 grid neighborhood
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            key = (gx + dx, gy + dy)
            if key in s_map:
                for ex, ey in s_map[key]:
                    dist = math.hypot(x - ex, y - ey)
                    if dist < min_dist:
                        return True
    return False

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    assets_dir = os.path.dirname(script_dir)
    image_path = os.path.join(assets_dir, IMAGE_FILENAME)
    yaml_path = os.path.join(assets_dir, YAML_FILENAME)

    if not os.path.exists(yaml_path):
        print(f"Error: YAML file not found at {yaml_path}")
        return

    # 1. Load YAML and Setup
    print(f"Loading {yaml_path}...")
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    if 'levels' not in data or 'L1' not in data['levels']:
        print("Error: levels.L1 structure not found in YAML.")
        return
    
    level_l1 = data['levels']['L1']
    if 'vertices' not in level_l1 or level_l1['vertices'] is None:
        level_l1['vertices'] = []
    if 'lanes' not in level_l1 or level_l1['lanes'] is None:
        level_l1['lanes'] = []
    
    initial_vertices_count = len(level_l1['vertices'])
    existing_vertices_list = load_existing_vertices(level_l1)
    print(f"Found {len(existing_vertices_list)} existing vertices.")

    # Build spatial map of existing vertices for distance checking
    # Use min_dist as cell magnitude
    s_map = build_spatial_map(existing_vertices_list, MIN_DISTANCE)

    # 2. Image Processing
    print(f"Loading image from {image_path}...")
    img = load_image(image_path)
    height, width = img.shape
    
    # Filter white parts
    print("Finding white pixels...")
    # This is a list of (x,y)
    white_pixels = get_white_pixel_coords(img)
    if len(white_pixels) == 0:
        print("Error: No white pixels found in map.")
        return

    # Sort for consistent top-left finding
    # min y, then min x
    sorted_pixels = sorted(white_pixels, key=lambda p: (p[0], p[1])) # Sort by x then y? No, finding top-left usually means Min Y (top) then Min X (left) or vice-versa.
    # Previous script used: sort by Y (col 1), then X (col 0)
    # "top left most part"
    # Let's align with previous script logic: lexsort((white_pixels[:, 0], white_pixels[:, 1])) which is sort by Y then X.
    # But wait, lexsort is (keys...) where last key is primary.
    # np.lexsort((white_pixels[:, 0], white_pixels[:, 1])) -> Sort by X (secondary), then Y (primary).
    # This finds smallest Y, and for ties, smallest X.
    # Consistent with standard image origin (0,0 is top-left).
    
    # We will compute scale based on the absolute top-left white pixel (Min Y, then Min X)
    # Finding min-y
    bg_y, bg_x = np.where(img > 250)
    # Sort by y, then x
    # Combine
    combined = list(zip(bg_x, bg_y))
    combined.sort(key=lambda p: (p[1], p[0]))
    
    img_tl_pixel = combined[0]
    img_tl_x, img_tl_y = img_tl_pixel
    print(f"Top-left white pixel at: ({img_tl_x}, {img_tl_y})")

    target_tl_x, target_tl_y = TARGET_TOP_LEFT
    scale_x = target_tl_x / img_tl_x if img_tl_x != 0 else 1.0
    scale_y = target_tl_y / img_tl_y if img_tl_y != 0 else 1.0
    print(f"Calculated Scale: X={scale_x}, Y={scale_y}")

    # 3. Generate Candidate Points (Grid)
    print("Generating points...")
    step = int(MIN_DISTANCE)
    
    # Map: (grid_x, grid_y) -> new_vertex_index (if added) or None
    # grid_x, grid_y are coordinates in the image domain
    grid_point_status = {} 
    
    new_vertices = []
    
    # Generate grid points
    # We work on the image grid `range(0, width, step)` etc.
    # Note: image shape is (height, width)
    # This creates a sparse set of 'potential' vertices.
    
    candidate_indices = [] # Stores (x, y) of grid valid points
    
    for y in range(0, height, step):
        for x in range(0, width, step):
            if img[y, x] > 250:
                # Candidate found. Check close-ness.
                # Transform to world coords
                tx = x * scale_x
                ty = y * scale_y
                
                if is_too_close(tx, ty, s_map, MIN_DISTANCE, MIN_DISTANCE):
                    grid_point_status[(x, y)] = None
                else:
                    # Not close, add it
                    idx = initial_vertices_count + len(new_vertices)
                    new_vertices.append([truncate(tx), truncate(ty), 0, ""])
                    grid_point_status[(x, y)] = idx
                    candidate_indices.append((x, y))
                    
                    # Also update s_map so we don't add dense clumps of NEW points?
                    # The grid step is 10, min_dist is 10.
                    # Grid points are by definition >= 10 units apart (Manhattan/Euclidean approx).
                    # Actually diagonal is 14.
                    # So grid points generally won't trigger "too close" against each other unless we check >10.
                    # Optimization: We assume grid provides sufficient separation between new points.
                    # So we only check against *existing* vertices as requested.
    
    print(f"Generated {len(new_vertices)} new valid vertices (filtered from grid).")
    
    level_l1['vertices'].extend(new_vertices)

    # 4. Generate Lanes (Sparse)
    new_lanes = []
    
    # Helper to add lane
    def add_lane(idx1, idx2):
        if idx1 == idx2: return
        props = {
            'bidirectional': [4, True],
            'demo_mock_floor_name': [1, ""],
            'demo_mock_lift_name': [1, ""],
            'graph_idx': [2, 0],
            'mutex': [1, ""],
            'orientation': [1, ""],
            'speed_limit': [3, 0]
        }
        new_lanes.append([idx1, idx2, props])

    # Sort candidates for Row pass
    # y primary, x secondary
    sorted_by_y = sorted(candidate_indices, key=lambda p: (p[1], p[0]))
    
    # Group by row
    current_y = -1
    row_points = []
    
    # We need to process "segments" -- contiguous blocks of white pixels
    # grid_point_status has keys for all valid NEW points, but we filtered some.
    # What about points that were 'skipped'? They are holes.
    # The requirement: "connect only the ones on the edge".
    # Interpretation: For a contiguous run of potential grid points (white space),
    # identify the Valid Start and Valid End and connect them.
    # If a run has only 1 valid point, no lane.
    
    # To do this, we need to traverse the *grid* not just the candidates.
    # Iterate grid rows.
    for y in range(0, height, step):
        # Find runs in this row [0...width]
        x = 0
        while x < width:
            # Check if this grid point is white
            if img[y, x] > 250:
                # Start of a white segment
                segment_valid_indices = []
                
                # Consume standard grid points until non-white
                curr_x = x
                while curr_x < width and img[y, curr_x] > 250:
                    if (curr_x, y) in grid_point_status:
                        idx = grid_point_status[(curr_x, y)]
                        if idx is not None:
                            segment_valid_indices.append(idx)
                    curr_x += step
                
                # End of segment. 
                # Connect first and last of valid indices if they differ
                if len(segment_valid_indices) > 1:
                    start_idx = segment_valid_indices[0]
                    end_idx = segment_valid_indices[-1]
                    add_lane(start_idx, end_idx)
                
                # Update loop x
                x = curr_x
            else:
                x += step

    # Col pass
    for x in range(0, width, step):
        y = 0
        while y < height:
            if img[y, x] > 250:
                segment_valid_indices = []
                curr_y = y
                while curr_y < height and img[curr_y, x] > 250:
                    if (x, curr_y) in grid_point_status:
                        idx = grid_point_status[(x, curr_y)]
                        if idx is not None:
                            segment_valid_indices.append(idx)
                    curr_y += step
                
                if len(segment_valid_indices) > 1:
                    start_idx = segment_valid_indices[0]
                    end_idx = segment_valid_indices[-1]
                    add_lane(start_idx, end_idx)
                y = curr_y
            else:
                y += step

    print(f"Generated {len(new_lanes)} sparse lanes.")
    level_l1['lanes'].extend(new_lanes)

    # 5. Format and Save
    # Apply FlowList to all vertices
    level_l1['vertices'] = [FlowList(v) if isinstance(v, list) else v for v in level_l1['vertices']]
    
    # Apply FlowList to lanes
    formatted_lanes = []
    for l in level_l1['lanes']:
        if len(l) >= 3 and isinstance(l[2], dict):
            n1, n2, props = l[0], l[1], l[2]
            new_props = _wrap_props(props)
            formatted_lanes.append(FlowList([n1, n2, new_props]))
        elif isinstance(l, list):
            formatted_lanes.append(FlowList(l))
        else:
            formatted_lanes.append(l)
    level_l1['lanes'] = formatted_lanes
    
    # Walls
    if 'walls' in level_l1 and level_l1['walls']:
        level_l1['walls'] = [FlowList(w) if isinstance(w, list) else w for w in level_l1['walls']]

    print(f"Writing {yaml_path}...")
    with open(yaml_path, 'w') as f:
        # Use IndentDumper
        yaml.dump(data, f, Dumper=IndentDumper, default_flow_style=None)
    
    print("Done.")

if __name__ == "__main__":
    main()
