import yaml
import math

def generate_config():
    yaml_path = '/home/dev/ros2_ws/src/rmf/traffic_editor_assets/map.building.yaml'
    out_path = '/home/dev/ros2_ws/src/robots_cv_sim/config/robots_vertices.yaml'
    
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
        
    vertices = data['levels']['L1']['vertices']
    walls = data['levels']['L1']['walls']
    
    # Collect indices used in walls
    wall_indices = set()
    for w in walls:
        wall_indices.add(w[0])
        wall_indices.add(w[1])
        
    # Find free vertices
    robots = []
    count = 1 # Start from 1
    for i, v in enumerate(vertices):
        if i not in wall_indices:
            # v is [x, y, z, name]
            robots.append({
                'id': count,
                'x': float(v[0]),
                'y': float(v[1]),
                'theta': -math.pi / 2.0 # Pointing Up (Negative Y)
            })
            count += 1
            
    print(f"Found {len(robots)} free vertices.")
    
    with open(out_path, 'w') as f:
        yaml.dump({'robots': robots}, f)

if __name__ == '__main__':
    generate_config()
