
# cleaning_bot 中的 Explore 模块

`cleaning_bot` 包中的 **Explore** 模块用于在未知环境中实现自主探索。它帮助机器人识别未知区域（前沿，Frontier），导航到这些区域，并完成环境地图的构建。

---

## 1. 头文件（include/explore）

### 1.1 `costmap_client.h`

**功能**：

- 封装了对机器人代价地图（costmap）的管理和更新。
- 提供获取机器人位姿（`getRobotPose`）和地图信息的接口。
- 负责通过订阅 ROS 主题接收和更新全局代价地图。

**核心类与方法**：

- **类**：`Costmap2DClient`
  - **方法**：
    - `geometry_msgs::Pose getRobotPose()`：获取机器人全局位姿。
    - `costmap_2d::Costmap2D* getCostmap()`：返回代价地图的指针。

**数据成员**：

- 订阅以下 ROS 主题：
  - `costmap_topic`：完整代价地图。
  - `costmap_updates_topic`：增量更新地图。

---

### 1.2 `costmap_tools.h`

**功能**：

- 提供处理代价地图（costmap）的工具函数，用于地图搜索和路径规划。

**核心函数**：

- `std::vector<unsigned int> nhood4(...)`：计算 4 邻域。
- `std::vector<unsigned int> nhood8(...)`：计算 8 邻域。
- `bool nearestCell(...)`：寻找最近的特定值的格子。

**用途**：

- 被 `frontier_search.h` 调用，用于前沿搜索任务。

---

### 1.3 `explore.h`

**功能**：

- 定义 `Explore` 类，负责机器人探索逻辑的主要功能。
- 处理目标生成、路径规划、目标点可视化以及与导航栈（`move_base`）的交互。

**核心类与方法**：

- **类**：`Explore`
  - **方法**：
    - `void start()`：启动探索任务。
    - `void stop()`：停止探索任务。
    - `void makePlan()`：生成前沿目标点并发送到导航栈。
    - `void visualizeFrontiers(...)`：在 RViz 中可视化前沿点。
    - `bool goalOnBlacklist(...)`：判断目标是否被列入黑名单。

---

### 1.4 `frontier_search.h`

**功能**：

- 定义了前沿搜索（frontier search）的具体逻辑，用于识别未知区域与已知区域的边界。

**核心类与方法**：

- **类**：`FrontierSearch`
  - **构造方法**：初始化搜索任务，接收代价地图。
  - `std::vector<Frontier> searchFrom(...)`：从给定位置开始搜索前沿。
  - `Frontier buildNewFrontier(...)`：从初始格子构建新的前沿。
  - `bool isNewFrontierCell(...)`：判断某个格子是否是新的前沿。
  - `double frontierCost(...)`：计算前沿的成本。

---

## 2. 源文件（src）

### 2.1 `costmap_client.cpp`

**功能**：

- 实现 `costmap_client.h` 中的 `Costmap2DClient` 类。
- 管理代价地图的订阅与更新。
- 提供机器人全局位姿信息。

**核心逻辑**：

- **ROS 主题订阅**：
  - `costmap_topic`：完整代价地图。
  - `costmap_updates_topic`：增量更新地图。
- **代价地图的更新**：
  - `updateFullMap(...)`：接收并更新完整地图。
  - `updatePartialMap(...)`：接收并更新增量地图。

---

### 2.2 `explore.cpp`

**功能**：

- 实现 `explore.h` 中的 `Explore` 类。
- 控制机器人自主探索未知区域。

**核心逻辑**：

- **启动和停止探索**：
  - `start()`：启动探索任务。
  - `stop()`：停止探索任务。
- **路径规划与目标选择**：
  - `makePlan()`：生成前沿点的目标并发送到导航栈。
  - **黑名单机制**：防止重复选择无效目标。
- **可视化**：
  - 使用 RViz 的 `MarkerArray` 显示前沿点。
- **与 `move_base` 的交互**：
  - 使用 `move_base` 发送目标点。

---

### 2.3 `frontier_search.cpp`

**功能**：

- 实现 `frontier_search.h` 中的 `FrontierSearch` 类。
- 基于代价地图搜索前沿点。

**核心逻辑**：

- `searchFrom(...)`：
  - 使用广度优先搜索（BFS）从当前点找到所有前沿点。
- `buildNewFrontier(...)`：
  - 从一个未知格子开始构建前沿。
  - 计算前沿的中心点、大小和成本。
- `isNewFrontierCell(...)`：
  - 判断某个格子是否是一个新的前沿。
- `frontierCost(...)`：
  - 计算前沿的成本，用于目标点排序。

---

## 3. 启动文件（launch）

### 3.1 `explore_costmap.launch`

**功能**：

- 加载 `explore` 节点，订阅 `move_base/global_costmap` 作为代价地图。

**参数**：

- `robot_base_frame`：机器人基座坐标系。
- `costmap_topic`：代价地图主题。
- `planner_frequency`：规划器更新频率。
- `potential_scale`、`gain_scale`：前沿成本计算的比例参数。

---

### 3.2 `explore.launch`

**功能**：

- 类似于 `explore_costmap.launch`，但使用不同的主题名称（例如 `map`）。
- 可能用于不同的地图或机器人配置。

---

## 4. 配置文件

### 4.1 `CMakeLists.txt`

**功能**：

- 定义如何构建该包，包括头文件和源文件的路径。
- 声明该包依赖的 ROS 库（如 `costmap_2d`、`move_base_msgs` 等）。

---

### 4.2 `package.xml`

**功能**：

- 定义 ROS 包的依赖项、名称、版本号等元信息。
- 声明依赖的其他 ROS 包，例如：
  - `roscpp`：C++ 客户端库。
  - `geometry_msgs`：几何消息类型。
  - `move_base_msgs`：导航栈消息类型。

---

## 总结

- **保留 Explore 模块**：
  - 如果你的项目需要机器人在未知环境中实现自主探索，此模块是必要的。
- **移除 Explore 模块**：
  - 如果机器人在已知地图中运行，或者仅需手动指定目标点，则可以考虑移除此模块。

如果需要分析其他模块（如 `map_merge` 或 `robot_navigation`），请提供源代码或功能描述，我可以继续帮你分析。
