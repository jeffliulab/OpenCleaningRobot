#!/usr/bin/env python3
import os
import subprocess
import time
from pathlib import Path

"""
THIS IS THE (1) FIRST STEP SCRIPT IN ROUTE ANALYZE FUNCTIONS
THIS IS A UTILITY SCRIPT

FUNCTION:
1. Run ROS map_server node, load map file
2. Run AMCL to localize
3. Run RViz with config file (localization.rviz)
"""

## Update: Nov 25, 2024
## Modify the comments, use English

def find_latest_map():
    """Find the newest map file"""
    # Get cleaning_robot/maps directory's path
    current_dir = Path(os.path.dirname(os.path.abspath(__file__)))  # this is the path in current file
    maps_dir = current_dir.parents[2] / "maps"  # from sweep/src up_find to cleaning_robot，then get into /maps
    
    # print directories for debugging
    print(f"Current directory: {current_dir}")
    print(f"Looking for maps in: {maps_dir}")
    
    # find if the directory is exist
    if not maps_dir.exists():
        raise FileNotFoundError(f"Maps directory not found: {maps_dir}")
    
    # find all yaml files and print
    yaml_files = list(maps_dir.glob("map_*.yaml"))
    print(f"Found map files: {[f.name for f in yaml_files]}")
    
    if not yaml_files:
        raise FileNotFoundError(f"No map files found in {maps_dir}")
    
    # Sort by file names and choose the newest one
    latest_yaml = sorted(yaml_files)[-1]
    print(f"Selected latest map: {latest_yaml.name}")
    
    # Check file authorizations
    if not os.access(str(latest_yaml), os.R_OK):
        raise PermissionError(f"Cannot read map file: {latest_yaml}")
    
    return str(latest_yaml)

def main():
    try:
        # Get the path of newest map file
        yaml_path = find_latest_map()
        print(f"\nTrying to load map from: {yaml_path}")
        
        # check the content of the file
        with open(yaml_path, 'r') as f:
            print("Map YAML content:")
            print(f.read())
        
        # Get the path of RViz config file
        script_dir = os.path.dirname(os.path.abspath(__file__))
        rviz_config_path = os.path.join(script_dir, "config/localization.rviz")
        
        processes = []
        
        try:
            # 1. Run map_server
            print("\nStarting map_server...")
            map_server_process = subprocess.Popen(
                ["rosrun", "map_server", "map_server", yaml_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            processes.append(("map_server", map_server_process))
            
            # wait and check if the map_server is running successfully
            time.sleep(2)
            if map_server_process.poll() is not None:
                # get wrong output
                _, stderr = map_server_process.communicate()
                print(f"Error: map_server failed to start!")
                if stderr:
                    print(f"Map server error output: {stderr.decode()}")
                return
            
            print("Map server started successfully")
            
            # 2. Run AMCL
            print("\nStarting AMCL...")
            amcl_process = subprocess.Popen(
                ["roslaunch", "turtlebot3_navigation", "amcl.launch"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            processes.append(("amcl", amcl_process))
            print("AMCL started")
            
            time.sleep(2)
            
            # 3. Run rviz (use saved config)
            print("\nStarting RViz...")
            if os.path.exists(rviz_config_path):
                rviz_process = subprocess.Popen(
                    ["rosrun", "rviz", "rviz", "-d", rviz_config_path],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                print(f"RViz started with config: {rviz_config_path}")
            else:
                print(f"RViz config file doesn't exist: {rviz_config_path}")
                
                # if doesn't exist, create the directory
                os.makedirs(os.path.dirname(rviz_config_path), exist_ok=True)
                
                rviz_process = subprocess.Popen(
                    ["rosrun", "rviz", "rviz"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
            
            processes.append(("rviz", rviz_process))
            
            
            # Check process status on time
            while True:
                time.sleep(1)
                for name, process in processes:
                    if process.poll() is not None:
                        # The process is end, print wrong information log
                        _, stderr = process.communicate()
                        if stderr:
                            print(f"\nError in {name}:")
                            print(stderr.decode())
                        print(f"\n{name} has stopped unexpectedly!")
                        return
                    
        except subprocess.CalledProcessError as e:
            print(f"Process error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")
            import traceback
            traceback.print_exc()
            
    except KeyboardInterrupt:
        print("\nShutting down...")
        for name, process in processes:
            print(f"Terminating {name}...")
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
    except Exception as e:
        print(f"Error occurred: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()