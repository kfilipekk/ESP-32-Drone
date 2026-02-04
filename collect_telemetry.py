import asyncio
import websockets
import json
import csv
import time
from datetime import datetime

##connection settings
URI = "wss://krystianfilipek.com/ws?role=controller"
FILENAME = f"imu_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

async def collect_data():
    print(f"Connecting to {URI}...")
    async with websockets.connect(URI) as websocket:
        print("Connected! Waiting for telemetry...")
        
        with open(FILENAME, 'w', newline='') as csvfile:
            fieldnames = ['timestamp', 'roll', 'pitch', 'yaw', 'voltage', 
                          'ax', 'ay', 'az', 'gx', 'gy', 'gz', 
                          'm1', 'm2', 'm3', 'm4',
                          'kp_p', 'kp_i', 'kp_d']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            
            print(f"Recording data to {FILENAME}. Press Ctrl+C to stop.")
            
            count = 0
            while True:
                try:
                    message = await websocket.recv()
                    data = json.loads(message)
                    
                    ##handle both old 'telem' and new 't=1' formats
                    if data.get('telem') or data.get('t') == 1:
                        row = {
                            'timestamp': time.time(),
                            'roll': data.get('r'),
                            'pitch': data.get('p'),
                            'yaw': data.get('y'),
                            'voltage': data.get('v'),
                            'ax': data.get('ax', 0),
                            'ay': data.get('ay', 0),
                            'az': data.get('az', 0),
                            'gx': data.get('gx', 0),
                            'gy': data.get('gy', 0),
                            'gz': data.get('gz', 0),
                            'm1': data.get('m1', 0),
                            'm2': data.get('m2', 0),
                            'm3': data.get('m3', 0),
                            'm4': data.get('m4', 0),
                            'kp_p': data.get('pi', 0),
                            'kp_i': data.get('ii', 0),
                            'kp_d': data.get('di', 0)
                        }
                        writer.writerow(row)
                        count += 1
                        if count % 20 == 0:
                            print(f"Recorded {count} samples... (Last: R={row['roll']:.1f} P={row['pitch']:.1f})")
                            
                except json.JSONDecodeError:
                    pass
                except Exception as e:
                    print(f"Error: {e}")
                    break

if __name__ == "__main__":
    try:
        asyncio.run(collect_data())
    except KeyboardInterrupt:
        print("\nStopped recording.")
