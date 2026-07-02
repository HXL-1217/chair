# import json
# import os

# class WaypointManager:
#     def __init__(self, logger, config_path):
#         self.logger = logger
#         self.config_path = config_path
        
#         # 出厂默认拓扑地图
#         self.topological_map = {
#             0x32: {
#                 "name": "客厅", "is_room": False,
#                 "default_goal": [-1.687, 4.518, -0.013],
#                 "polygon": [[0.361, 1.451], [2.571, 1.153], [2.948, 4.336], [0.659, 4.446]]
#             },
#             0x33: {
#                 "name": "厨房", "is_room": True,
#                 "default_goal": [4.910, 2.084, 0.031],
#                 "polygon": [[3.0, 1.0], [6.0, 1.0], [6.0, 4.0], [3.0, 4.0]],
#                 "door": {
#                     "outside_node": [3.500, 1.500], 
#                     "inside_node":  [4.000, 2.000], 
#                     "enter_yaw": 0.78               
#                 }
#             },
#             0x34: {
#                 "name": "主卧", "is_room": True,
#                 "default_goal": [2.500, 4.500, 1.57],
#                 "polygon": [[1.0, 3.0], [4.0, 3.0], [4.0, 6.0], [1.0, 6.0]],
#                 "door": {
#                     "outside_node": [1.500, 2.000], 
#                     "inside_node":  [1.500, 3.500], 
#                     "enter_yaw": 1.57                
#                 }
#             },
#             0x35: {
#                 "name": "厕所", "is_room": True,
#                     "default_goal": [0.856, -2.017, -0.022], 
#                     "polygon": [[0.361, 1.451], [2.571, 1.153], [2.0, -1.0], [0.5, -1.0]],
#                 "door": {
#                     "outside_node": [0.961, 2.167], 
#                     "inside_node":  [0.732, 0.388], 
#                     "enter_yaw": -1.625               
#                 }
#             },
#             0x36: {
#                 "name": "客卧", "is_room": True,
#                 "default_goal": [3.500, -3.500, -1.57],
#                 "polygon": [[2.0, -2.0], [5.0, -2.0], [5.0, -5.0], [2.0, -5.0]],
#                 "door": {
#                     "outside_node": [2.500, -1.000], 
#                     "inside_node":  [2.500, -2.500], 
#                     "enter_yaw": -1.57               
#                 }
#             }
#         }
        
#         # 独立的马桶泊车基准点
#         self.toilet_dock_target = [-0.354, -1.984, -0.027]
#         self.load_waypoints()

#     def load_waypoints(self):
#         """从硬盘读取历史标定坐标"""
#         if os.path.exists(self.config_path):
#             try:
#                 with open(self.config_path, 'r', encoding='utf-8') as f:
#                     data = json.load(f)
#                     if "toilet_dock_target" in data:
#                         self.toilet_dock_target = data["toilet_dock_target"]
#                     if "topological_map" in data:
#                         for k_str, room_data in data["topological_map"].items():
#                             k_int = int(k_str) 
#                             if k_int in self.topological_map:
#                                 self.topological_map[k_int]['default_goal'] = room_data.get('default_goal', self.topological_map[k_int]['default_goal'])
#                                 self.topological_map[k_int]['door'] = room_data.get('door', self.topological_map[k_int].get('door'))
#                 self.logger.info(f"[Data] 💾 成功恢复坐标配置: {self.config_path}")
#             except Exception as e:
#                 self.logger.error(f"[Data] ❌ 读取配置文件失败: {e}")
#         else:
#             self.logger.info("[Data] 🆕 未检测到历史配置文件，使用默认坐标。")

#     def save_waypoints(self):
#         """将内存坐标持久化到硬盘"""
#         try:
#             config_dir = os.path.dirname(self.config_path)
#             if not os.path.exists(config_dir):
#                 os.makedirs(config_dir)

#             save_map = {}
#             for k, v in self.topological_map.items():
#                 save_map[str(k)] = {
#                     "default_goal": v.get("default_goal"),
#                     "door": v.get("door")
#                 }
                
#             data = {
#                 "toilet_dock_target": self.toilet_dock_target,
#                 "topological_map": save_map
#             }
#             with open(self.config_path, 'w', encoding='utf-8') as f:
#                 json.dump(data, f, indent=4, ensure_ascii=False)
#             self.logger.info("[Data] 💾 坐标已成功保存至 JSON。")
#         except Exception as e:
#             self.logger.error(f"[Data] ❌ 保存硬盘失败: {e}")

#     def identify_room_by_polygon(self, click_x, click_y):
#         for room_hex, room_info in self.topological_map.items():
#             if 'polygon' in room_info:
#                 if self._is_point_in_polygon(click_x, click_y, room_info['polygon']):
#                     return room_hex
#         return None

#     def _is_point_in_polygon(self, x, y, polygon):
#         n = len(polygon)
#         inside = False
#         p1x, p1y = polygon[0]
#         for i in range(1, n + 1):
#             p2x, p2y = polygon[i % n]
#             if min(p1y, p2y) < y <= max(p1y, p2y):
#                 if x <= max(p1x, p2x):
#                     if p1y != p2y:
#                         xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
#                     if p1x == p2x or x <= xints:
#                         inside = not inside
#             p1x, p1y = p2x, p2y
#         return inside



import json
import os

class WaypointManager:
    def __init__(self, logger, config_path):
        self.logger = logger
        self.config_path = config_path
        
        # ==========================================
        # 静态拓扑结构与预设默认位姿
        # ==========================================
        self.topological_map = {
            0x32: { 
                "name": "客厅", "is_room": False,
                "default_goal": [1.531, 3.384, 1.382],
                "polygon": [[0.25, 1.395], [2.598, 1.129], [2.997, 4.382], [0.592, 4.61]]
            },
            0x33: { 
                "name": "厨房", "is_room": True,
                "default_goal": [-1.041, 2.295, -1.839],
                "polygon": [[3.0, 1.0], [6.0, 1.0], [6.0, 4.0], [3.0, 4.0]],
                "door": {
                    "outside_node": [3.500, 1.500], 
                    "inside_node":  [4.000, 2.000], 
                    "enter_yaw": 0.78               
                }
            },
            0x34: { 
                "name": "主卧", "is_room": True,
                "default_goal": [5.451, 2.114, -0.058],
                "polygon": [[1.0, 3.0], [4.0, 3.0], [4.0, 6.0], [1.0, 6.0]],
                "door": {
                    "outside_node": [1.500, 2.000], 
                    "inside_node":  [1.500, 3.500], 
                    "enter_yaw": 1.57                
                }
            },
            0x31: { # 厕所: 仅到达
                "name": "厕所(仅到达)", "is_room": True,
                "default_goal": [-0.515, 0.082, -0.061],
                "polygon": [[-2.021, 1.297], [2.581, 0.887], [2.389, -0.845], [-2.222, -0.41]],
                "door": {
                    "outside_node": [1.005, 2.220],
                    "inside_node":  [0.760, 0.401],
                    "enter_yaw": -1.705,
                    "exit_offset": 0.3,
                    "approach_offset": 0.3
                }
            },
            0x35: { # 厕所: 倒车模式
                "name": "厕所倒车模式", "is_room": True,
                "default_goal": [-0.515, 0.082, -0.061],
                "polygon": [[-2.021, 1.297], [2.581, 0.887], [2.389, -0.845], [-2.222, -0.41]],
                "door": {
                    "outside_node": [1.005, 2.220],
                    "inside_node":  [0.760, 0.401],
                    "enter_yaw": -1.705,
                    "exit_offset": 0.3,
                    "approach_offset": 0.3
                }
            },
            0x36: { 
                "name": "客卧", "is_room": True,
                "default_goal": [3.500, -3.500, -1.57],
                "polygon": [[2.0, -2.0], [5.0, -2.0], [5.0, -5.0], [2.0, -5.0]],
                "door": {
                    "outside_node": [2.500, -1.000], 
                    "inside_node":  [2.500, -2.500], 
                    "enter_yaw": -1.57               
                }
            }
        }
        
        # 独立的马桶泊车倒车基准点
        self.toilet_dock_target = [-1.715, 0.221, -0.05]
        
        # 启动时读取 JSON 覆盖默认位姿
        self.load_waypoints()

    # ==========================================
    # 动态数据读写逻辑 (仅处理位姿)
    # ==========================================
    def load_waypoints(self):
        """从硬盘读取历史标定位姿（仅覆盖 default_goal 和 dock_target）"""
        if os.path.exists(self.config_path):
            try:
                with open(self.config_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    
                    # 1. 恢复马桶倒车点
                    if "toilet_dock_target" in data:
                        self.toilet_dock_target = data["toilet_dock_target"]
                        
                    # 2. 恢复各房间目标位姿
                    if "topological_map" in data:
                        for k_str, room_data in data["topological_map"].items():
                            k_int = int(k_str) 
                            if k_int in self.topological_map and "default_goal" in room_data:
                                # 只提取 default_goal 进行覆盖，绝不触碰门禁和边界数据
                                self.topological_map[k_int]['default_goal'] = room_data["default_goal"]
                                
                self.logger.info(f"[Data] 💾 成功恢复用户标定位姿: {self.config_path}")
            except Exception as e:
                self.logger.error(f"[Data] ❌ 读取配置文件失败: {e}")
        else:
            self.logger.info("[Data] 🆕 未检测到历史配置文件，使用默认位姿。")

    def save_waypoints(self):
        """将内存中的用户位姿持久化到硬盘（剥离静态物理参数）"""
        try:
            config_dir = os.path.dirname(self.config_path)
            if not os.path.exists(config_dir):
                os.makedirs(config_dir)

            save_map = {}
            for k, v in self.topological_map.items():
                # 仅将 default_goal 存入 JSON，剥离 door 和 polygon
                save_map[str(k)] = {
                    "default_goal": v.get("default_goal")
                }
                
            data = {
                "toilet_dock_target": self.toilet_dock_target,
                "topological_map": save_map
            }
            
            with open(self.config_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=4, ensure_ascii=False)
                
            self.logger.info("[Data] 💾 动态位姿已成功保存至 JSON。")
        except Exception as e:
            self.logger.error(f"[Data] ❌ 保存硬盘失败: {e}")

    # ==========================================
    # 工具函数
    # ==========================================
    def identify_room_by_polygon(self, click_x, click_y):
        for room_hex, room_info in self.topological_map.items():
            if 'polygon' in room_info:
                if self._is_point_in_polygon(click_x, click_y, room_info['polygon']):
                    return room_hex
        return None

    def _is_point_in_polygon(self, x, y, polygon):
        n = len(polygon)
        inside = False
        p1x, p1y = polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n]
            if min(p1y, p2y) < y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xints:
                        inside = not inside
            p1x, p1y = p2x, p2y
        return inside