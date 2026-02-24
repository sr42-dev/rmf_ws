import yaml
import os

def generate_config():
    map_path = '/home/dev/ros2_ws/src/rmf/traffic_editor_assets/map.building.yaml'
    output_path = '/home/dev/ros2_ws/src/robots_cv_sim/config/robots_vertices.yaml'
    
    with open(map_path, 'r') as f:
        data = yaml.safe_load(f)
        
    vertices = data['levels']['L1']['vertices']
    walls = data['levels']['L1']['walls']
    
    wall_indices = set()
    for w in walls:
        wall_indices.add(w[0])
        wall_indices.add(w[1])
        
    free_vertices = []
    for i, v in enumerate(vertices):
        if i not in wall_indices:
            # v is [x, y, z, name]
            free_vertices.append((v[0], v[1]))
            
    print(f"Total vertices: {len(vertices)}")
    print(f"Wall vertices: {len(wall_indices)}")
    print(f"Free vertices: {len(free_vertices)}")
    
    robots_config = {'robots': []}
    
    # We need 100 robots ideally, or as many as we have free vertices?
    # User said "load the locations ... as the initial locations"
    # User also said "simulating 100 robots" previously.
    # If we have > 100 free vertices, pick 100? Or all?
    # I'll use up to 100 from the free list. Ideally distributed.
    
    count = 0
    # Use first 100 or all if less
    import random
    # Shuffle to get random distribution or just take first? 
    # User said "locations of vertices... as initial locations"
    # Let's use all of them if possible or clamp to 100.
    
    # Actually, let's just create entries for all free vertices, up to 100.
    # If there are fewer than 100, we'll loop or just have fewer robots.
    # But the user requirement "config for the number of robots ... simulator should be able to simulate 100 robots".
    # And "load the locations... as initial locations".
    
    # Let's take 100 samples from free_vertices (with replacement if needed, or without if enough).
    
    target_robots = 100
    chosen_locs = []
    
    if len(free_vertices) == 0:
        print("Error: No free vertices found!")
        return

    if len(free_vertices) >= target_robots:
         chosen_locs = free_vertices[:target_robots]
    else:
        # Repeat
        chosen_locs = free_vertices * (target_robots // len(free_vertices)) + free_vertices[:(target_robots % len(free_vertices))]
        
    for i, (x, y) in enumerate(chosen_locs):
        robots_config['robots'].append({
            'id': i,
            'x': float(x),
            'y': float(y),
            'theta': -1.57 # pointing up
        })
        
    with open(output_path, 'w') as f:
        yaml.dump(robots_config, f)
        
    print(f"Generated config {output_path} with {len(robots_config['robots'])} robots")

if __name__ == '__main__':
    generate_config()
