import numpy as np
import os

# ---------------------------------------------------------
# Configuration and Constants
# ---------------------------------------------------------
Tp = 1  # 10 x clock period
StartW = Tp
ZeroW = Tp
OneW = 3 * Tp
StopW = 13 * Tp
SpaceW = Tp

# Output directory
output_dir = r'C:\TM\Master_TM_DataCollar\01-Report\mic-parameter'

# Ensure directory exists (optional, prevents errors if path is missing)
if not os.path.exists(output_dir):
    try:
        os.makedirs(output_dir)
    except OSError:
        pass # Handle cases where drive might not be mapped

# ---------------------------------------------------------
# Data Definitions
# ---------------------------------------------------------

# Device Write Pattern [1 0 1 0 0 1 1 0]
device_write = np.array([1, 0, 1, 0, 0, 1, 1, 0])

# Helper function to convert hex/int to 8-bit array (MSB First)
# Note: Standard register communication is usually MSB first. 
# If your MATLAB int2bit produced LSB first, change 'big' to 'little'.
def to_bits(value, num_bits=8):
    return np.array([int(x) for x in format(value, f'0{num_bits}b')])

# Register Address and Data pairs
# Format: (Address, Data)
reg_data_pairs = [
    (0x5C, 0x00), # AAD Unlock write sequence
    (0x3E, 0x00), # AAD Unlock write sequence
    (0x6F, 0x00), # AAD Unlock write sequence
    (0x3B, 0x00), # AAD Unlock write sequence
    (0x4C, 0x00), # AAD Unlock write sequence
    (0x35, 0x02), # AAD A_LPF = 0x2 (2 kHz) ADDA
    (0x36, 0x0A), # Write SPL value (85 dB SPL) ADDA
    # (0x29, 0x08), # Enable AAD A (Commented out)
    # (0x29, 0x00), # ADD enable register: Disable AAD (Commented out)
    (0x2A, 0x00), # FLOOR MSBs FLOOR = 0x00A0 (65dB SPL)
    (0x2B, 0xA0), # FLOOR LSBs FLOOR = 0x00A0) (65dB SPL)
    (0x2C, 0x32), # Reserved; must be written for AAD D
    (0x2D, 0x80), # ALGO_SEL bits [7:6], bit 6=relative, bit 7 absolute
    (0x2E, 0x00), # REL_PULSE_MIN = 0x0D5
    (0x2F, 0x00), # ABS_PULSE_MIN MSBs
    (0x30, 0x00), # ABS_PULSE_MIN = 0x0D5
    (0x31, 0x2C), # ABS_TH = 0x62C (85dB SPL)
    (0x32, 0x46), # ABS_TH MSBs
    (0x33, 0x24), # RE_TH = 0x36 (+6dB)
    (0x29, 0x09)  # Enable AAD D1 and ADDA 0x09
]

# ---------------------------------------------------------
# Processing Logic
# ---------------------------------------------------------

bit_pattern = np.array([])
file_counter = 1

# Loop through all register pairs
for addr, data in reg_data_pairs:
    
    # 1. Start Sequence: Space, Start(High), Space
    start_seq = np.concatenate([
        np.zeros(SpaceW), 
        np.ones(StartW), 
        np.zeros(SpaceW)
    ])
    bit_pattern = np.concatenate([bit_pattern, start_seq])

    # 2. Build Write Sequence: [DeviceWrite, Addr, Data]
    # Result is 24 bits long
    addr_bits = to_bits(addr, 8)
    data_bits = to_bits(data, 8)
    write_seq = np.concatenate([device_write, addr_bits, data_bits])

    # 3. Encode bits (Manually creating Pulse Width Modulation)
    for bit in write_seq:
        if bit == 1:
            # Logic 1: High for OneW, Low for SpaceW
            pulse = np.concatenate([np.ones(OneW), np.zeros(SpaceW)])
        else:
            # Logic 0: High for ZeroW, Low for SpaceW
            pulse = np.concatenate([np.ones(ZeroW), np.zeros(SpaceW)])
        
        bit_pattern = np.concatenate([bit_pattern, pulse])

    # 4. Stop Sequence
    bit_pattern = np.concatenate([bit_pattern, np.ones(StopW)])

    # 5. File Saving Logic (Chunking)
    if len(bit_pattern) > 900:
        filename = os.path.join(output_dir, f'AADpattern{file_counter}.csv')
        
        # Save as column vector (fmt='%.0f' saves as integers)
        np.savetxt(filename, bit_pattern, fmt='%.0f', delimiter=',')
        
        print(f"Saved: {filename}")
        file_counter += 1
        bit_pattern = np.array([]) # Reset buffer

# Save any remaining data after loop finishes
if len(bit_pattern) > 0:
    filename = os.path.join(output_dir, f'AADpattern{file_counter}.csv')
    np.savetxt(filename, bit_pattern, fmt='%.0f', delimiter=',')
    print(f"Saved: {filename}")