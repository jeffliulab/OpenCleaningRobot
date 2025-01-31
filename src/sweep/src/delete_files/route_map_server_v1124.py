#!/usr/bin/env python3
import os
import subprocess
import time
from pathlib import Path

"""
THIS IS THE (0) ZERO SCRIPT IN ROUTE MODULE
THIS IS A UTILITY SCRIPT
功能：
1. 自动启动 ROS map_server 节点，加载指定的静态地图
2. 启动AMCL进行定位
3. 启动配置好的rviz (read localization.rviz)
"""

def find_latest_map():
    """查找最新的地图文件"""
    # 获取cleaning_robot/maps目录的路径
    current_dir = Path(os.path.dirname(os.path.abspath(__file__)))  # 当前脚本目录
    maps_dir = current_dir.parents[2] / "maps"  # 从sweep/src上溯到cleaning_robot，然后进入maps
    
    # 打印路径以便调试
    print(f"Current directory: {current_dir}")
    print(f"Looking for maps in: {maps_dir}")
    
    # 检查目录是否存在
    if not maps_dir.exists():
        raise FileNotFoundError(f"Maps directory not found: {maps_dir}")
    
    # 查找所有yaml文件并打印
    yaml_files = list(maps_dir.glob("map_*.yaml"))
    print(f"Found map files: {[f.name for f in yaml_files]}")
    
    if not yaml_files:
        raise FileNotFoundError(f"No map files found in {maps_dir}")
    
    # 按文件名排序并选择最新的
    latest_yaml = sorted(yaml_files)[-1]
    print(f"Selected latest map: {latest_yaml.name}")
    
    # 检查文件权限
    if not os.access(str(latest_yaml), os.R_OK):
        raise PermissionError(f"Cannot read map file: {latest_yaml}")
    
    return str(latest_yaml)

def main():
    try:
        # 获取最新地图文件的路径
        yaml_path = find_latest_map()
        print(f"\nTrying to load map from: {yaml_path}")
        
        # 检查文件内容
        with open(yaml_path, 'r') as f:
            print("Map YAML content:")
            print(f.read())
        
        # 获取RViz配置文件路径
        script_dir = os.path.dirname(os.path.abspath(__file__))
        rviz_config_path = os.path.join(script_dir, "config/localization.rviz")
        
        processes = []
        
        try:
            # 1. 启动 map_server
            print("\nStarting map_server...")
            map_server_process = subprocess.Popen(
                ["rosrun", "map_server", "map_server", yaml_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            processes.append(("map_server", map_server_process))
            
            # 等待并检查map_server是否成功启动
            time.sleep(2)
            if map_server_process.poll() is not None:
                # 获取错误输出
                _, stderr = map_server_process.communicate()
                print(f"Error: map_server failed to start!")
                if stderr:
                    print(f"Map server error output: {stderr.decode()}")
                return
            
            print("Map server started successfully")
            
            # 2. 启动 AMCL
            print("\nStarting AMCL...")
            amcl_process = subprocess.Popen(
                ["roslaunch", "turtlebot3_navigation", "amcl.launch"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            processes.append(("amcl", amcl_process))
            print("AMCL started")
            
            time.sleep(2)
            
            # 3. 启动 rviz（使用保存的配置）
            print("\nStarting RViz...")
            if os.path.exists(rviz_config_path):
                rviz_process = subprocess.Popen(
                    ["rosrun", "rviz", "rviz", "-d", rviz_config_path],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                print(f"RViz started with config: {rviz_config_path}")
            else:
                print(f"RViz配置文件不存在: {rviz_config_path}")
                print("启动默认RViz并请手动配置以下显示项：")
                print("1. 将 Fixed Frame 设置为 map")
                print("2. Add -> Map (设置topic为 /map)")
                print("3. Add -> PoseArray (设置topic为 /particlecloud)")
                print("4. Add -> LaserScan (设置topic为 /scan)")
                print("5. Add -> RobotModel")
                print("\n配置完成后，请保存配置到以下路径：")
                print(f"{rviz_config_path}")
                
                # 创建config目录（如果不存在）
                os.makedirs(os.path.dirname(rviz_config_path), exist_ok=True)
                
                rviz_process = subprocess.Popen(
                    ["rosrun", "rviz", "rviz"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
            
            processes.append(("rviz", rviz_process))
            
            print("\n所有节点已启动。使用Ctrl+C退出...\n")
            
            # 定期检查进程状态
            while True:
                time.sleep(1)
                for name, process in processes:
                    if process.poll() is not None:
                        # 进程已结束，打印错误信息
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