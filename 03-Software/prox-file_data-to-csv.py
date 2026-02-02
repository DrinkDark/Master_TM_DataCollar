import struct
import csv
from datetime import datetime

def read_proximity_65byte_format(file_path, csv_path):
    # Header: uint16_t + uint32_t + uint8_t + uint32_t = 11 bytes
    HEADER_FORMAT = "<H I B I" 
    # Device: uint32_t + 6*uint8_t + int16_t + int8_t + uint8_t + uint8_t = 15 bytes
    DEVICE_FORMAT = "<I 6s h b B B"
    
    with open(file_path, "rb") as f:
        header_raw = f.read(11)
        if len(header_raw) < 11: 
            print("File too small")
            return

        fid, ver, log_id, start_ts = struct.unpack(HEADER_FORMAT, header_raw)
        print(f"Header: ID={hex(fid)}, Ver={ver}, Log={log_id}, Start={datetime.fromtimestamp(start_ts)}")

        # Write to CSV
        with open(csv_path, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Timestamp", "DateTime", "MAC Address", "Device Number", "RSSI", "Days of Recording", "System Status"])
            
            cnt = 0
            while True:
                d_raw = f.read(15)
                if len(d_raw) < 15: 
                    break
                
                ts, addr, dev_num, rssi, dor, status = struct.unpack(DEVICE_FORMAT, d_raw)
                addr_hex = ":".join(f"{b:02X}" for b in addr)
                dt = datetime.fromtimestamp(ts)
                
                writer.writerow([ts, dt, addr_hex, dev_num, rssi, dor, status])
                cnt += 1
        
        print(f"Total records read: {cnt}")
        print(f"CSV saved to: {csv_path}")

# Prompt user for input
file_path = input("Enter the path and filename for the .dat file: ")
csv_path = input("Enter the path and filename for the .csv output file: ")

read_proximity_65byte_format(file_path, csv_path)