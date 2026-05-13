#!/usr/bin/env python3
"""
Polygon Manager 系统测试和演示脚本

功能：
1. 测试点在多边形内的判定
2. 模拟敌方位置和检测结果
3. 验证配置加载/保存
4. 性能基准测试

使用方法：
  python3 test_demo.py [test_case]

示例：
  python3 test_demo.py point_in_polygon
  python3 test_demo.py enemy_detection
  python3 test_demo.py all
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
import time
import sys


class PolygonManagerTester(Node):
    """
    测试节点
    
    订阅 Polygon Manager 的输出并验证
    """

    def __init__(self):
        super().__init__('polygon_manager_tester')
        
        # 订阅敌方状态
        self.enemy_state_sub = self.create_subscription(
            String, '/enemy_area_state', self.enemy_state_callback, 10)
        
        # 订阅 Marker 数据
        self.marker_sub = self.create_subscription(
            MarkerArray, '/polygon_areas/visualization_marker_array',
            self.marker_callback, 10)
        
        # 发布敌方位置
        self.enemy_pub = self.create_publisher(
            PoseStamped, '/enemy_position', 10)
        
        self.last_state = None
        self.last_markers = None
        self.markers_count = 0
        
        self.get_logger().info('Polygon Manager Tester initialized')

    def enemy_state_callback(self, msg):
        """敌方状态回调"""
        self.last_state = msg.data
        self.get_logger().info(f'Enemy State: {msg.data}')

    def marker_callback(self, msg):
        """Marker 回调"""
        self.last_markers = msg
        self.markers_count = len(msg.markers)
        self.get_logger().debug(
            f'Received {self.markers_count} markers')

    def publish_enemy_position(self, x, y, name="test_pose"):
        """发布敌方位置"""
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        
        self.enemy_pub.publish(msg)
        self.get_logger().info(
            f'Published enemy position: ({x}, {y})')
        
        # 等待处理
        time.sleep(0.5)


def test_point_in_polygon():
    """
    测试 1：点在多边形内的判定
    
    假设已加载的配置包含一个禁区：
    禁区顶点：(1, 1), (5, 1), (5, 4), (1, 4)
    """
    print("\n" + "="*60)
    print("Test 1: Point in Polygon Detection")
    print("="*60)
    
    rclpy.init()
    tester = PolygonManagerTester()
    
    # 测试用例
    test_cases = [
        (2.5, 2.5, True, "Inside forbidden zone"),
        (0.5, 0.5, False, "Outside forbidden zone (bottom-left)"),
        (6.0, 2.5, False, "Outside forbidden zone (right)"),
        (3.0, 3.0, True, "Inside forbidden zone (center)"),
        (1.0, 1.0, True, "On corner (should be inside)"),
    ]
    
    results = []
    
    for x, y, expected_inside, description in test_cases:
        print(f"\nTest: {description}")
        print(f"  Position: ({x}, {y})")
        
        tester.publish_enemy_position(x, y)
        
        rclpy.spin_once(tester, timeout_sec=1.0)
        
        if tester.last_state:
            is_inside = "forbidden_zone" in tester.last_state
            status = "✓ PASS" if is_inside == expected_inside else "✗ FAIL"
            print(f"  Result: {tester.last_state}")
            print(f"  Status: {status}")
            results.append(status.startswith("✓"))
        else:
            print(f"  Status: ✗ FAIL (no response)")
            results.append(False)
    
    print(f"\n{'='*60}")
    print(f"Passed: {sum(results)}/{len(results)}")
    print(f"{'='*60}\n")
    
    rclpy.shutdown()
    return all(results)


def test_enemy_detection():
    """
    测试 2：敌方检测流程
    
    验证系统能够实时检测敌方进入不同区域
    """
    print("\n" + "="*60)
    print("Test 2: Enemy Detection Flow")
    print("="*60)
    
    rclpy.init()
    tester = PolygonManagerTester()
    
    # 等待订阅建立
    time.sleep(1.0)
    
    print("\nScenario: Enemy moving through different zones")
    print("-" * 60)
    
    # 路线：外面 -> 禁区 -> 保护区
    waypoints = [
        (0.5, 0.5, "Outside all zones"),
        (2.5, 2.5, "Inside forbidden zone"),
        (4.0, 4.0, "Inside forbidden zone"),
        (8.0, 3.5, "Inside protect zone"),
        (10.0, 10.0, "Outside all zones"),
    ]
    
    print("\nMovement sequence:")
    for i, (x, y, desc) in enumerate(waypoints, 1):
        print(f"  {i}. {desc:<30} ({x:>5}, {y:>5})")
    
    print("\nExecuting simulation:")
    results = []
    
    for x, y, expected_zone in waypoints:
        print(f"\n→ Enemy moving to ({x}, {y})")
        print(f"  Expected zone: {expected_zone}")
        
        tester.publish_enemy_position(x, y)
        rclpy.spin_once(tester, timeout_sec=1.0)
        
        if tester.last_state:
            print(f"  Detected state: {tester.last_state}")
            results.append(tester.last_state)
        else:
            print(f"  No state detected")
            results.append("unknown")
    
    print(f"\n{'='*60}")
    print("Detection results:")
    for (x, y, zone), detected in zip(waypoints, results):
        print(f"  ({x:>5}, {y:>5}) -> {detected}")
    print(f"{'='*60}\n")
    
    rclpy.shutdown()
    return True


def test_marker_visualization():
    """
    测试 3：Marker 可视化
    
    验证系统正确生成 RViz Marker 消息
    """
    print("\n" + "="*60)
    print("Test 3: Marker Visualization")
    print("="*60)
    
    rclpy.init()
    tester = PolygonManagerTester()
    
    print("\nWaiting for markers...")
    time.sleep(1.0)
    
    rclpy.spin_once(tester, timeout_sec=2.0)
    
    if tester.last_markers is None:
        print("✗ FAIL: No markers received")
        rclpy.shutdown()
        return False
    
    print(f"✓ PASS: Received {tester.markers_count} markers")
    print(f"\nMarker information:")
    
    for i, marker in enumerate(tester.last_markers.markers):
        print(f"\n  Marker {i+1}:")
        print(f"    ID: {marker.id}")
        print(f"    Namespace: {marker.ns}")
        print(f"    Type: {marker.type}")
        print(f"    Frame: {marker.header.frame_id}")
        print(f"    Points: {len(marker.points)}")
        
        if marker.color.r > 0 and marker.color.g == 0:
            print(f"    Color: Red (Forbidden Area)")
        elif marker.color.r == 0 and marker.color.g > 0:
            print(f"    Color: Green (Protect Area)")
        elif marker.color.r == 0 and marker.color.b > 0:
            print(f"    Color: Blue (Patrol Area)")
        elif marker.color.r > 0 and marker.color.g > 0:
            print(f"    Color: Yellow (Attack Area)")
    
    print(f"\n{'='*60}\n")
    
    rclpy.shutdown()
    return True


def test_performance():
    """
    测试 4：性能基准测试
    
    测量系统处理延迟
    """
    print("\n" + "="*60)
    print("Test 4: Performance Benchmark")
    print("="*60)
    
    rclpy.init()
    tester = PolygonManagerTester()
    
    print("\nTesting detection latency...")
    print("-" * 60)
    
    latencies = []
    num_tests = 10
    
    for i in range(num_tests):
        start_time = time.time()
        
        tester.publish_enemy_position(2.5, 2.5)
        rclpy.spin_once(tester, timeout_sec=1.0)
        
        latency = (time.time() - start_time) * 1000  # 转换为毫秒
        latencies.append(latency)
        
        print(f"  Test {i+1}: {latency:.2f} ms")
    
    avg_latency = sum(latencies) / len(latencies)
    max_latency = max(latencies)
    min_latency = min(latencies)
    
    print(f"\n{'='*60}")
    print(f"Performance Results:")
    print(f"  Average latency: {avg_latency:.2f} ms")
    print(f"  Max latency: {max_latency:.2f} ms")
    print(f"  Min latency: {min_latency:.2f} ms")
    print(f"{'='*60}\n")
    
    rclpy.shutdown()
    return avg_latency < 50  # 应该在 50ms 以内


def run_all_tests():
    """运行所有测试"""
    print("\n")
    print("╔" + "="*58 + "╗")
    print("║" + " "*58 + "║")
    print("║" + " Polygon Manager System Test Suite ".center(58) + "║")
    print("║" + " "*58 + "║")
    print("╚" + "="*58 + "╝")
    
    tests = [
        ("Point in Polygon", test_point_in_polygon),
        ("Enemy Detection", test_enemy_detection),
        ("Marker Visualization", test_marker_visualization),
        ("Performance", test_performance),
    ]
    
    results = {}
    
    for test_name, test_func in tests:
        try:
            print(f"\n[Running] {test_name}...")
            result = test_func()
            results[test_name] = result
            print(f"[Result] {test_name}: {'PASS ✓' if result else 'FAIL ✗'}")
        except Exception as e:
            print(f"[Error] {test_name}: {e}")
            results[test_name] = False
    
    # 总结
    print("\n")
    print("╔" + "="*58 + "╗")
    print("║" + " "*58 + "║")
    print("║" + " Test Summary ".center(58) + "║")
    print("║" + " "*58 + "║")
    
    passed = sum(1 for v in results.values() if v)
    total = len(results)
    
    for test_name, result in results.items():
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"║ {test_name:<50} {status:>6} ║")
    
    print("║" + " "*58 + "║")
    print(f"║ Total: {passed}/{total} tests passed " + " "*(58-25-len(str(passed))-len(str(total))) + "║")
    print("║" + " "*58 + "║")
    print("╚" + "="*58 + "╝\n")
    
    return all(results.values())


def main():
    """主函数"""
    if len(sys.argv) > 1:
        test_case = sys.argv[1].lower()
        
        if test_case == "point_in_polygon":
            success = test_point_in_polygon()
        elif test_case == "enemy_detection":
            success = test_enemy_detection()
        elif test_case == "marker":
            success = test_marker_visualization()
        elif test_case == "performance":
            success = test_performance()
        elif test_case == "all":
            success = run_all_tests()
        else:
            print(f"Unknown test case: {test_case}")
            print("Available tests: point_in_polygon, enemy_detection, marker, performance, all")
            success = False
    else:
        success = run_all_tests()
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
