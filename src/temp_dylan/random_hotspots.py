import os
import random
import math

# -------------------------
# Configuration
# -------------------------

URDF_PATH = "/home/student/41068_ws/src/41068_ignition_bringup (1)/41068_ignition_bringup/urdf" # Change the file path here to match yours
HOTSPOT_RADIUS = 0.3
HOTSPOT_HEIGHT = 0.05
MIN_DISTANCE = 1.0
WORLD_BOUNDS = (-12, 12)  # x,y limits

# -------------------------
# Utility functions
# -------------------------

def distance(a, b):
    """Euclidean distance in 2D."""
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def is_valid_position(new_pos, existing_positions):
    """Check if new hotspot position is far enough from existing objects."""
    for pos in existing_positions:
        if distance(new_pos, pos) < MIN_DISTANCE:
            return False
    return True


def intensity_to_colour(intensity):
    """Convert intensity (1-10) to an RGBA shade of red."""
    red = 1.0
    green = (10 - intensity) * 0.05
    blue = (10 - intensity) * 0.05
    alpha = 0.3 + (intensity / 10) * 0.7
    return f"{red:.2f} {green:.2f} {blue:.2f} {alpha:.2f}"


# -------------------------
# User input
# -------------------------

while True:
    try:
        num_hotspots = int(input("Enter number of hotspots (1–10): "))
        if 1 <= num_hotspots <= 10:
            break
        else:
            print("Please enter a number between 1 and 10.")
    except ValueError:
        print("Invalid input. Please enter an integer between 1 and 10.")

# -------------------------
# Generate positions with unique intensities
# -------------------------

hotspots = []
attempts = 0
used_intensities = set()  # track used intensities

while len(hotspots) < num_hotspots and attempts < 5000:
    x = random.uniform(WORLD_BOUNDS[0], WORLD_BOUNDS[1])
    y = random.uniform(WORLD_BOUNDS[0], WORLD_BOUNDS[1])

    # pick an intensity not already used
    available_intensities = [i for i in range(1, 11) if i not in used_intensities]
    if not available_intensities:
        break  # all intensities used

    intensity = random.choice(available_intensities)
    new_pos = (x, y, intensity)

    if is_valid_position((x, y), [(h[0], h[1]) for h in hotspots]):
        hotspots.append(new_pos)
        used_intensities.add(intensity)

    attempts += 1

if len(hotspots) < num_hotspots:
    raise RuntimeError("Could not find enough valid positions for hotspots.")

# -------------------------
# Output hotspot info
# -------------------------

print("\nGenerated hotspot positions and intensities:")
for i, (x, y, intensity) in enumerate(hotspots, start=1):
    print(f"  hotspot{i}: position=({x:.2f}, {y:.2f}), intensity={intensity}/10")

# -------------------------
# Generate URDF.XACRO files
# -------------------------

for i, (x, y, intensity) in enumerate(hotspots, start=1):
    colour = intensity_to_colour(intensity)
    filename = os.path.join(URDF_PATH, f"hotspot_{i}.urdf.xacro")

    hotspot_urdf = f"""<?xml version="1.0"?>
<robot name="hotspot_{i}" xmlns:xacro="http://ros.org/wiki/xacro">
  <link name="hotspot_{i}_link">
    <visual>
      <geometry>
        <cylinder radius="{HOTSPOT_RADIUS}" length="{HOTSPOT_HEIGHT}"/>
      </geometry>
      <origin xyz="0 0 {HOTSPOT_HEIGHT/2:.2f}" rpy="0 0 0"/>
      <material name="hotspot_colour">
        <color rgba="{colour}"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="{HOTSPOT_RADIUS}" length="{HOTSPOT_HEIGHT}"/>
      </geometry>
      <origin xyz="0 0 {HOTSPOT_HEIGHT/2:.2f}" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="1e-6" iyy="1e-6" izz="1e-6" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <gazebo reference="hotspot_{i}_link">
    <material>Gazebo/Red</material>
    <transparency>{1.0 - float(colour.split()[-1]):.2f}</transparency>
  </gazebo>
</robot>
"""

    # Write to file
    with open(filename, "w") as f:
        f.write(hotspot_urdf)

    print(f"Generated {filename}")
