#!/usr/bin/env python3
import rospy
import yaml
import os
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


"""
THIS IS THE (3) THIRD STEP SCRIPT IN ROUTE ANALYZE FUNCTIONS
THIS IS A UTILITY SCRIPT

FUNCTION:
This is a visualization utility script that displays the planned cleaning path
in ROS visualization tool (RViz). It provides real-time visualization of:

1. Path points with different status:
   - Unvisited points (red)
   - Visited points (green) 
   - Obstacle points (black)

2. Path connections:
   - Shows directional arrows between adjacent points
   - Visualizes the planned cleaning sequence
   - Indicates movement direction with arrow markers

3. Dynamic updates:
   - Loads the latest path plan from YAML files
   - Updates visualization at 1Hz
   - Uses persistent publishing (latch=True) for stable display

4. Visualization features:
   - Efficient obstacle point sampling for clearer view
   - Color-coded status representation
   - Configurable point sizes and connection styles
   - Frame-based visualization in ROS map coordinate system
"""

## Latest Update: Nov 25
## Modify comments and use English only

## Need updates:
## Need determine the relationship between route_plan, to think whether this script should modify to a simpler one


class MapData:
    OBSTACLE = -1    # Obstacle points (Black)
    UNVISITED = 0    # Unvisited points (Red)
    VISITED = 1      # Visited points (Green)

class PointDisplay:
    OBSTACLE_COLOR = ColorRGBA(0.0, 0.0, 0.0, 0.8)    # Black
    UNVISITED_COLOR = ColorRGBA(1.0, 0.0, 0.0, 0.8)   # Red
    VISITED_COLOR = ColorRGBA(0.0, 1.0, 0.0, 0.8)     # Green
    POINT_SIZE = 0.1  # 点的大小

class RouteVisualizer:
    def __init__(self):
        rospy.init_node('route_visualizer', anonymous=True)
        
        # store the data
        self.map_data = None
        self.path_points = []
        self.current_point_id = 0  # the ID now access
        
        # Publisher (use latch=True to ensure message persistence)
        self.points_pub = rospy.Publisher('/path_visualization', MarkerArray, queue_size=1, latch=True)
        self.connection_pub = rospy.Publisher('/path_connections', MarkerArray, queue_size=1, latch=True)
        
        # load
        self.path_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pathfiles")
        if not os.path.exists(self.path_dir):
            os.makedirs(self.path_dir)
        self.load_latest_path()
        
        # timer
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_visualizations)
        rospy.loginfo("Route visualizer initialized")

    def load_latest_path(self):
        """Load the latest path file"""
        try:
            path_files = [f for f in os.listdir(self.path_dir) 
                         if f.startswith('coverage_path_') and f.endswith('.yaml')]
            if not path_files:
                rospy.logwarn("No path files found")
                return
                
            latest_file = sorted(path_files)[-1]
            path_file = os.path.join(self.path_dir, latest_file)
            rospy.loginfo(f"Loading path from {path_file}")
            
            with open(path_file, 'r') as f:
                data = yaml.safe_load(f)
                self.map_data = data['map_data']
                self.path_points = data['path_points']
                rospy.loginfo(f"Loaded {len(self.path_points)} path points")
                
        except Exception as e:
            rospy.logerr(f"Error loading path file: {str(e)}")

    def are_adjacent(self, point1, point2):
        """Determine whether two points are adjacent"""
        sampling_interval = self.path_points[0].get('sampling_interval', 5)  # Default value is 5
        dx = abs(point1['grid_x'] - point2['grid_x'])
        dy = abs(point1['grid_y'] - point2['grid_y'])
        return (dx == sampling_interval and dy == 0) or (dx == 0 and dy == sampling_interval)

    def find_valid_connections(self, points):
        """
        Find All Valid Connections
        Same logic in route_plan
        """
        connections = []
        visited = set()
        
        if not points:
            return connections
            
        current_point = points[0]
        visited.add(0)
        
        while len(visited) < len(points):
            best_distance = float('inf')
            best_next_point = None
            best_next_idx = None
            
            # Find the nearest unvisited point
            for i, point in enumerate(points):
                if i in visited:
                    continue
                    
                if not self.are_adjacent(current_point, point):
                    continue
                
                distance = ((current_point['grid_x'] - point['grid_x']) ** 2 + 
                          (current_point['grid_y'] - point['grid_y']) ** 2) ** 0.5
                          
                if distance < best_distance:
                    best_distance = distance
                    best_next_point = point
                    best_next_idx = i
            
            if best_next_point is None:
                # If the next point cannot be found, choose a new starting point from the remaining unvisited points
                unvisited = set(range(len(points))) - visited
                if unvisited:
                    next_start_idx = min(unvisited)
                    current_point = points[next_start_idx]
                    visited.add(next_start_idx)
                    continue
                else:
                    break
            
            connections.append((current_point, best_next_point))
            visited.add(best_next_idx)
            current_point = best_next_point
        
        return connections

    def publish_visualizations(self, event):
        """Publishing visual information"""
        if not self.map_data or not self.path_points:
            return

        # Create point markers
        point_markers = MarkerArray()
        
        # Adding Waypoints
        for i, point in enumerate(self.path_points):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "path_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = point['world_x']
            marker.pose.position.y = point['world_y']
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = marker.scale.y = marker.scale.z = PointDisplay.POINT_SIZE
            
            # Set the color according to the state of the point
            if point.get('status', 0) == MapData.VISITED:
                marker.color = PointDisplay.VISITED_COLOR
            else:
                marker.color = PointDisplay.UNVISITED_COLOR
            
            point_markers.markers.append(marker)
        
        # Add obstacle points
        obstacle_id = len(self.path_points)
        map_array = self.map_data['array']
        resolution = self.map_data['resolution']
        origin_x = self.map_data['origin_x']
        origin_y = self.map_data['origin_y']
        
        # Sample obstacle points (one every few points)
        sampling = 5  # adjust this value to change the density of obstacle points.
        for y in range(0, len(map_array), sampling):
            for x in range(0, len(map_array[0]), sampling):
                if map_array[y][x] == MapData.OBSTACLE:
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = rospy.Time.now()
                    marker.ns = "obstacle_points"
                    marker.id = obstacle_id
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    
                    marker.pose.position.x = x * resolution + origin_x
                    marker.pose.position.y = y * resolution + origin_y
                    marker.pose.position.z = 0.0
                    marker.pose.orientation.w = 1.0
                    
                    marker.scale.x = marker.scale.y = marker.scale.z = PointDisplay.POINT_SIZE
                    marker.color = PointDisplay.OBSTACLE_COLOR
                    
                    point_markers.markers.append(marker)
                    obstacle_id += 1

        # Publishing point tags
        if point_markers.markers:
            self.points_pub.publish(point_markers)
            rospy.loginfo_throttle(10, f"Published {len(point_markers.markers)} markers")

        # Create and publish connection line markers
        connections = self.find_valid_connections(self.path_points)
        connection_markers = MarkerArray()
        
        for i, (start_point, end_point) in enumerate(connections):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "connections"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # Set start and end point
            start = Point()
            start.x = start_point['world_x']
            start.y = start_point['world_y']
            start.z = 0.0
            
            end = Point()
            end.x = end_point['world_x']
            end.y = end_point['world_y']
            end.z = 0.0
            
            marker.points = [start, end]
            marker.scale.x = 0.02  # Arrow width
            marker.scale.y = 0.04  # Arrow head width
            marker.color = ColorRGBA(0.3, 0.3, 1.0, 0.8)  # 蓝色
            
            connection_markers.markers.append(marker)
        
        if connection_markers.markers:
            self.connection_pub.publish(connection_markers)
            rospy.loginfo_throttle(10, f"Published {len(connection_markers.markers)} connections")

    def run(self):
        rospy.loginfo("Route visualizer running...")
        rospy.spin()

if __name__ == "__main__":
    try:
        visualizer = RouteVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass