import pygame
import zmq
import json
import math
import os

is_headless = False

# --- Setup ZMQ ---
context = zmq.Context()
pub_socket = context.socket(zmq.PUB)
pub_socket.bind("tcp://*:5555")

sub_socket = context.socket(zmq.SUB)
sub_socket.connect("tcp://localhost:5556")
sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")

# --- Arena Constants (Matches Blueprint Exactly) ---
ARENA_L = 8.10  
ARENA_W = 4.57  
SCALE = 80      

# Colors
BG_COLOR = (20, 20, 20)
ROVER_FRONT_COLOR = (200, 255, 200) 
ROVER_BACK_COLOR = (0, 100, 0)      
ROCK_COLOR = (0, 150, 255)    
CRATER_COLOR = (139, 69, 19)  
ZONE_COLOR = (100, 100, 100)
BERM_COLOR = (255, 0, 0)      
ARUCO_COLOR = (255, 255, 0)
DETECTION_COLOR = (255, 100, 100)  # Red dots for detected points

# =====================================================================
# SENSOR CONSTRAINTS — Matched to real perception.py
# =====================================================================
FRONT_FOV_DEG = 80.0       # D435i horizontal FOV (~87 deg, using 80 for margin)
FRONT_MIN_DIST = 0.3       # Matches perception.py MIN_DISTANCE
FRONT_MAX_DIST = 3.0       # Matches perception.py MAX_DISTANCE

# Simulated 16-column perception bands
# perception.py splits the 640px image into 16 bands of 40px each.
# In the sim, we split the FOV into 16 angular bands and raycast each one.
NUM_BANDS = 16
VIEW_WIDTH = 2.4           # Matches perception.py VIEW_WIDTH

REAR_FOV_DEG = 20.0        # Narrow FOV to mimic tracking the marker (+/- 10 deg)
REAR_MIN_DIST = 0.5
REAR_MAX_DIST = 6.5

# ArUco Marker Global Position
ARUCO_X = 8.10
ARUCO_Z = 3.5

# =====================================================================
# ROVER DIMENSIONS (for accurate drawing)
# =====================================================================
ROVER_WIDTH_M = 0.889   # 2'11" side to side
ROVER_LENGTH_M = 0.686  # 2'3" front to back

# --- Obstacle Definitions ---
# Guidebook: rocks 30-40cm diameter, craters 40-50cm wide
obstacles = [
    {"x": 5.0, "z": 3.4, "r": 0.30, "type": "rock"},
    {"x": 3.0, "z": 3.4, "r": 0.1, "type": "rock"},    
    {"x": 5.0, "z": 1.2, "r": 0.30, "type": "rock"},   
    {"x": 3.5, "z": 1.2, "r": 0.1, "type": "rock"}, 
]

# Arena walls (the rover must detect these as obstacles per guidebook rules)
# Represented as line segments: (x1, z1, x2, z2)
arena_walls = [
    (0.0, 0.0, ARENA_L, 0.0),      # Bottom wall
    (0.0, ARENA_W, ARENA_L, ARENA_W),  # Top wall
    (0.0, 0.0, 0.0, ARENA_W),      # Left wall
    (ARENA_L, 0.0, ARENA_L, ARENA_W),  # Right wall
]

# --- Setup Pygame ---
if is_headless:
    os.environ["SDL_VIDEODRIVER"] = "dummy"

pygame.init()
WIDTH, HEIGHT = int(ARENA_L * SCALE), int(ARENA_W * SCALE)
screen = pygame.display.set_mode((WIDTH, HEIGHT))

if not is_headless:
    pygame.display.set_caption("Lunabotics Arena Sim - Perception-Accurate")

# Rover State - Starting in the Starting Zone facing the Berm
rover_x, rover_z = 7.0, 3.5 
rover_yaw = -math.pi / 2 # Facing Left (-X direction)

last_time = pygame.time.get_ticks()
running = True


def ray_circle_intersect(rx, rz, dx, dz, cx, cz, radius):
    """
    Find the closest intersection distance of a ray (rx,rz) + t*(dx,dz)
    with a circle centered at (cx,cz) with the given radius.
    Returns the distance t, or None if no intersection.
    """
    fx = rx - cx
    fz = rz - cz
    a = dx * dx + dz * dz
    b = 2.0 * (fx * dx + fz * dz)
    c = fx * fx + fz * fz - radius * radius
    
    discriminant = b * b - 4.0 * a * c
    if discriminant < 0:
        return None
    
    sqrt_disc = math.sqrt(discriminant)
    t1 = (-b - sqrt_disc) / (2.0 * a)
    t2 = (-b + sqrt_disc) / (2.0 * a)
    
    # We want the closest positive intersection (entering the circle)
    if t1 > 0:
        return t1
    elif t2 > 0:
        return t2
    return None


def ray_line_intersect(rx, rz, dx, dz, x1, z1, x2, z2):
    """
    Find intersection of ray (rx,rz)+t*(dx,dz) with line segment (x1,z1)-(x2,z2).
    Returns distance t, or None.
    """
    sx = x2 - x1
    sz = z2 - z1
    
    denom = dx * sz - dz * sx
    if abs(denom) < 1e-8:
        return None  # Parallel
    
    t = ((x1 - rx) * sz - (z1 - rz) * sx) / denom
    u = ((x1 - rx) * dz - (z1 - rz) * dx) / denom
    
    if t > 0 and 0 <= u <= 1:
        return t
    return None


def simulate_perception(rover_x, rover_z, rover_yaw):
    """
    Simulate what perception.py's 16-band floor scanner would see.
    
    For each of the 16 column bands, cast a ray from the rover and find
    what it hits (obstacle or wall). If the hit is within detection range,
    report it as an obstacle detection with the correct rel_x, rel_z.
    
    This produces multiple detections per obstacle (just like the real
    camera does across adjacent column bands), giving a realistic test
    of the clustering and inflation in main.rs.
    """
    detections = []  # list of (rel_x, rel_z, type, hit_x, hit_z)
    
    half_fov = math.radians(FRONT_FOV_DEG / 2.0)
    
    for band in range(NUM_BANDS):
        # Angle for this band: spread evenly across the FOV
        # Band 0 = left edge, band 15 = right edge
        band_frac = (band + 0.5) / NUM_BANDS  # 0.03125 to 0.96875
        band_angle = rover_yaw + half_fov - (band_frac * 2.0 * half_fov)
        
        # Ray direction
        ray_dx = math.sin(band_angle)
        ray_dz = math.cos(band_angle)
        
        closest_dist = None
        closest_type = None
        
        # Check all obstacles (rocks and craters)
        for obs in obstacles:
            t = ray_circle_intersect(rover_x, rover_z, ray_dx, ray_dz, 
                                      obs["x"], obs["z"], obs["r"])
            if t is not None and FRONT_MIN_DIST < t < FRONT_MAX_DIST:
                if closest_dist is None or t < closest_dist:
                    closest_dist = t
                    closest_type = obs["type"]
        
        # Check arena walls
        for wall in arena_walls:
            t = ray_line_intersect(rover_x, rover_z, ray_dx, ray_dz,
                                    wall[0], wall[1], wall[2], wall[3])
            if t is not None and FRONT_MIN_DIST < t < FRONT_MAX_DIST:
                if closest_dist is None or t < closest_dist:
                    closest_dist = t
                    closest_type = "wall"
        
        if closest_dist is not None:
            # Compute the hit point in global coords
            hit_x = rover_x + ray_dx * closest_dist
            hit_z = rover_z + ray_dz * closest_dist
            
            # Convert to rover-relative coords (same as perception.py output)
            dx_o = hit_x - rover_x
            dz_o = hit_z - rover_z
            rel_x = dx_o * math.cos(rover_yaw) - dz_o * math.sin(rover_yaw)
            rel_z = dx_o * math.sin(rover_yaw) + dz_o * math.cos(rover_yaw)
            
            detections.append({
                "rel_x": rel_x,
                "rel_z": rel_z,
                "type": closest_type,
                "hit_x": hit_x,
                "hit_z": hit_z,
            })
    
    return detections


while running:
    if not is_headless:
        screen.fill(BG_COLOR)
        
    dt = (pygame.time.get_ticks() - last_time) / 1000.0
    last_time = pygame.time.get_ticks()

    for event in pygame.event.get():
        if event.type == pygame.QUIT: running = False

    # 1. Listen for Rust Commands
    # 1. Listen for Rust Commands
    try:
        msg = sub_socket.recv_string(zmq.DONTWAIT)
        cmd = json.loads(msg)
        
        # The physical encoders in Rust now 100% dictate the position in the sim.
        rover_x = ARUCO_X - cmd['x']
        rover_z = ARUCO_Z - cmd['z']
        
        # FLIP THE YAW by 180 degrees (pi radians) to match the inverted X/Z axes
        rover_yaw = cmd['yaw'] + math.pi 

    except zmq.Again:
        pass

    # 2. Draw Arena
    if not is_headless:
        # Zone dividers
        pygame.draw.line(screen, ZONE_COLOR, (2.6*SCALE, 0), (2.6*SCALE, HEIGHT), 2)
        pygame.draw.line(screen, ZONE_COLOR, ((8.1-4.0)*SCALE, 0), ((8.1-4.0)*SCALE, HEIGHT), 2)

        # Berm target
        berm_x, berm_z, berm_w, berm_h = 0.5, 1.2, 1.5, 0.9
        berm_rect = pygame.Rect(berm_x*SCALE, HEIGHT - (berm_z + berm_h)*SCALE, berm_w*SCALE, berm_h*SCALE)
        pygame.draw.rect(screen, BERM_COLOR, berm_rect, 2)

        # Arena boundary walls
        pygame.draw.rect(screen, (80, 80, 80), (0, 0, WIDTH, HEIGHT), 2)

        # ArUco Marker
        aruco_px_x = int(ARUCO_X * SCALE)
        aruco_px_z = int(HEIGHT - (ARUCO_Z * SCALE))
        pygame.draw.line(screen, ARUCO_COLOR, (aruco_px_x-30, aruco_px_z), (aruco_px_x, aruco_px_z), 4)

    payload = {
        "localization_mode": "BLIND (ENC)",
        "aruco_pos": [],
        "vo_status": "OK",
        "obstacles": []
    }

    rx_px = int(rover_x * SCALE)
    rz_px = int(HEIGHT - (rover_z * SCALE))

    # --- 3A. Process REAR CAMERA (ArUco Detection) ---
    rear_yaw = rover_yaw + math.pi
    
    dx_a = ARUCO_X - rover_x
    dz_a = ARUCO_Z - rover_z
    dist_a = math.hypot(dx_a, dz_a)
    angle_to_aruco = math.atan2(dx_a, dz_a)
    
    diff_a = (angle_to_aruco - rear_yaw)
    diff_a = (diff_a + math.pi) % (2 * math.pi) - math.pi
    
    aruco_visible = abs(diff_a) < math.radians(REAR_FOV_DEG / 2.0) and REAR_MIN_DIST < dist_a < REAR_MAX_DIST
    
    if aruco_visible:
        payload["localization_mode"] = "ARUCO_LOCKED"
        payload["aruco_pos"] = [ARUCO_X - rover_x, ARUCO_Z - rover_z, rover_yaw]
        ray_color = (0, 255, 0)
    else:
        ray_color = (255, 50, 50)

    if not is_headless:
        ray_len = 3.0 * SCALE
        end_x = rx_px + int(ray_len * math.sin(rear_yaw))
        end_y = rz_px - int(ray_len * math.cos(rear_yaw))
        pygame.draw.line(screen, ray_color, (rx_px, rz_px), (end_x, end_y), 2)

    # --- 3B. Simulate Perception (16-band floor scanner) ---
    # Draw FOV cone
    if not is_headless:
        fov_surf = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        fov_pts = [(rx_px, rz_px)]
        half_fov = int(FRONT_FOV_DEG / 2)
        for angle_offset in range(-half_fov, half_fov + 1, 5):
            rad = rover_yaw + math.radians(angle_offset)
            ex = rx_px + int(FRONT_MAX_DIST * SCALE * math.sin(rad))
            ey = rz_px - int(FRONT_MAX_DIST * SCALE * math.cos(rad))
            fov_pts.append((ex, ey))
        pygame.draw.polygon(fov_surf, (50, 255, 50, 40), fov_pts)
        screen.blit(fov_surf, (0, 0))

    # Draw obstacles (always, even if not detected)
    for obs in obstacles:
        px = int(obs["x"] * SCALE)
        pz = int(HEIGHT - (obs["z"] * SCALE)) 
        if not is_headless:
            color = ROCK_COLOR if obs["type"] == "rock" else CRATER_COLOR
            pygame.draw.circle(screen, color, (px, pz), int(obs["r"] * SCALE))

    # Run the 16-band perception simulation
    detections = simulate_perception(rover_x, rover_z, rover_yaw)
    
    for det in detections:
        payload["obstacles"].append({
            "type": "point_cloud",
            "rel_x": det["rel_x"],
            "rel_z": det["rel_z"]
        })
        
        # Draw detection points on the arena view
        if not is_headless:
            det_px = int(det["hit_x"] * SCALE)
            det_pz = int(HEIGHT - (det["hit_z"] * SCALE))
            pygame.draw.circle(screen, DETECTION_COLOR, (det_px, det_pz), 4)
            # Draw ray from rover to detection
            pygame.draw.line(screen, (50, 50, 50), (rx_px, rz_px), (det_px, det_pz), 1)

    # 4. Draw Rover (accurate dimensions)
    if not is_headless:
        r_w = ROVER_WIDTH_M * SCALE
        r_h = ROVER_LENGTH_M * SCALE
        rover_surf = pygame.Surface((int(r_w), int(r_h)), pygame.SRCALPHA)
        pygame.draw.rect(rover_surf, ROVER_FRONT_COLOR, (0, 0, r_w, r_h // 2))
        pygame.draw.rect(rover_surf, ROVER_BACK_COLOR, (0, r_h // 2, r_w, r_h // 2))

        rotated_rover = pygame.transform.rotate(rover_surf, math.degrees(-rover_yaw))
        rect = rotated_rover.get_rect(center=(rx_px, rz_px))
        screen.blit(rotated_rover, rect)

    # 5. Send back to Rust
    pub_socket.send_string(json.dumps(payload))
    
    if not is_headless:
        pygame.display.flip()
        
    pygame.time.delay(20)

pygame.quit()