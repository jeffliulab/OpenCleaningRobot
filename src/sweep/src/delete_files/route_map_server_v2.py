#!/usr/bin/env python3
import os
import subprocess
import time

"""
THIS IS THE (0) ZERO SCRIPT IN ROUTE MODULE
THIS IS A UTILITY SCRIPT
功能：
1. 自动启动 ROS map_server 节点，加载指定的静态地图(CLEANING_ROBOT/maps/{newest_map})
2. 启动AMCL进行定位
3. 启动配置好的rviz (read localization.rviz)
"""

def main():
    # 动态设置地图路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    yaml_path = os.path.join(script_dir, "maps/map_20241121_164920.yaml")
    rviz_config_path = os.path.join(script_dir, "config/localization.rviz")  # RViz配置文件路径
    
    # 检查 yaml 文件是否存在
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"YAML file not found: {yaml_path}")
    
    print(f"Loading map from: {yaml_path}")
    processes = []
    
    try:
        # 1. 启动 map_server
        map_server_process = subprocess.Popen(
            ["rosrun", "map_server", "map_server", yaml_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        processes.append(("map_server", map_server_process))
        print("Started map_server")
        
        time.sleep(2)  # 等待地图加载
        
        # 2. 启动 AMCL
        amcl_process = subprocess.Popen(
            ["roslaunch", "turtlebot3_navigation", "amcl.launch"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        processes.append(("amcl", amcl_process))
        print("Started AMCL")
        
        time.sleep(2)  # 等待AMCL启动
        
        # 3. 启动 rviz（使用保存的配置）
        if os.path.exists(rviz_config_path):
            rviz_process = subprocess.Popen(
                ["rosrun", "rviz", "rviz", "-d", rviz_config_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
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
        print("Started rviz")
        
        print("\n所有节点已启动。使用Ctrl+C退出...")
        
        # 等待进程
        while True:
            time.sleep(1)
            
    except subprocess.CalledProcessError as e:
        print(f"Error occurred: {e}")
    except KeyboardInterrupt:
        print("\nShutting down...")
        for name, process in processes:
            print(f"Terminating {name}...")
            process.terminate()

if __name__ == "__main__":
    main()