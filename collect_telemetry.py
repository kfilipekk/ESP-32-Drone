import asyncio
import websockets
import json
import time
import argparse
import csv
from datetime import datetime

DEFAULT_URI = "ws://krystianfilipek.com/ws"
OUTPUT_FILE = f"flight_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

async def telemetry_logger(uri, log_file):
    print(f"Connecting to {uri}...")
    try:
        async with websockets.connect(uri) as websocket:
            print(f"Connected! Logging to {log_file}...")
            
            with open(log_file, 'w', newline='') as csvfile:
                fieldnames = ['timestamp', 'roll', 'pitch', 'yaw', 'throttle', 'vbat', 'ax', 'ay', 'az', 'gx', 'gy', 'gz']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                
                start_time = time.time()
                
                while True:
                    try:
                        message = await websocket.recv()
                        data = json.loads(message)
                        
                        data['timestamp'] = time.time() - start_time
                        row = {
                            'timestamp': data.get('timestamp', 0),
                            'roll': data.get('roll', 0),
                            'pitch': data.get('pitch', 0),
                            'yaw': data.get('yaw', 0),
                            'throttle': data.get('thm', 0),
                            'vbat': data.get('vbat', 0),
                            'ax': data.get('ax', 0),
                            'ay': data.get('ay', 0),
                            'az': data.get('az', 0),
                            'gx': data.get('gx', 0),
                            'gy': data.get('gy', 0),
                            'gz': data.get('gz', 0)
                        }
                        
                        writer.writerow(row)
                        print(f"Time: {row['timestamp']:.2f}s | R: {row['roll']:.1f} P: {row['pitch']:.1f} Bat: {row['vbat']:.2f}V", end='\r')
                        
                    except websockets.exceptions.ConnectionClosed:
                        print("\nConnection closed by server.")
                        break
                    except json.JSONDecodeError:
                        print(f"\nInvalid JSON received: {message}")
                        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ESP32 Drone Telemetry Logger")
    parser.add_argument("--ip", default="krystianfilipek.com", help="Drone IP address")
    parser.add_argument("--port", default="80", help="WebSocket port (if not 80)")
    parser.add_argument("--output", default=OUTPUT_FILE, help="Output CSV filename")
    
    args = parser.parse_args()
    uri = f"ws://{args.ip}/ws"
    
    try:
        asyncio.run(telemetry_logger(uri, args.output))
    except KeyboardInterrupt:
        print("\nLogging stopped by user.")
