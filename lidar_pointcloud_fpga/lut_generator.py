import math

# File to generate
OUTPUT_FILE = "azimuth_sincos.mem"

# Velodyne VLP-16 standard elevation angles (in degrees)
# Mapped to Laser IDs 0 through 15 based on the VLP-16 manual.
# If your C++ software sorts these differently (e.g. from lowest to highest), 
# simply update this array to match your software's laser_id output.
ELEVATIONS_DEG = [
    -15.0,   1.0, -13.0,   3.0, 
    -11.0,   5.0,  -9.0,   7.0, 
     -7.0,   9.0,  -5.0,  11.0, 
     -3.0,  13.0,  -1.0,  15.0
]

def to_q1_15(float_val):
    """Converts a float between -1.0 and 1.0 to a 16-bit Q1.15 hex string."""
    # Multiply by 2^15 and round
    q_val = int(round(float_val * 32768.0))
    
    # Clamp to signed 16-bit limits to prevent overflow
    if q_val > 32767:
        q_val = 32767
    elif q_val < -32768:
        q_val = -32768
        
    # Apply two's complement mask for negative numbers and format as 4-char hex
    return f"{(q_val & 0xFFFF):04X}"

def main():
    print(f"Generating {OUTPUT_FILE} with 8192 entries...")
    
    with open(OUTPUT_FILE, "w") as f:
        # Loop through all 16 Laser IDs (Top 4 bits of the address)
        for laser_id in range(16):
            elevation_deg = ELEVATIONS_DEG[laser_id]
            elevation_rad = math.radians(elevation_deg)
            
            # Loop through all 512 possible azimuth slices (Bottom 9 bits of the address)
            for az_slice in range(512):
                
                # If we are in the valid 0-359 degree range
                if az_slice < 360:
                    azimuth_rad = math.radians(az_slice)
                    
                    # Calculate the 3 coefficients based on the documentation math
                    # X = cos(elev) * sin(azim)
                    # Y = cos(elev) * cos(azim)
                    # Z = sin(elev)
                    x_float = math.cos(elevation_rad) * math.sin(azimuth_rad)
                    y_float = math.cos(elevation_rad) * math.cos(azimuth_rad)
                    z_float = math.sin(elevation_rad)
                    
                    # Convert to Q1.15 Hex
                    hex_x = to_q1_15(x_float)
                    hex_y = to_q1_15(y_float)
                    hex_z = to_q1_15(z_float)
                    
                    # Concatenate to form the 48-bit word (XXXXYYYYZZZZ)
                    mem_line = f"{hex_x}{hex_y}{hex_z}\n"
                    
                # If we are in the "dead space" (360-511)
                else:
                    mem_line = "000000000000\n"
                    
                f.write(mem_line)
                
    print("Generation complete! Copy this file to your Quartus project directory.")

if __name__ == "__main__":
    main()
