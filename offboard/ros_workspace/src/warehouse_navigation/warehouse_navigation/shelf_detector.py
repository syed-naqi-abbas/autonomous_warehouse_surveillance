#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math
import tf_transformations
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray
import yaml
import os

class ShelfDetector(Node):
    def __init__(self):
        super().__init__('shelf_detector')

        self.detection_duration = 80.0  # seconds
        self.first_shelf_detected = False
        self.get_logger().info(f"Shelf detection will stop 80 seconds after first detection.")

        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Parameters (tweak if needed) ---
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("laser_frame", "my_robot/base_link/gpu_lidar")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("leg_diameter", 0.05)
        self.declare_parameter("leg_min_points", 0)
        self.declare_parameter("gap_thresh", 0.05)         # debug value
        self.declare_parameter("max_cluster_length", 0.12) # debug value
        self.declare_parameter("max_detection_range", 7.0)

        # self.declare_parameter("shelf_width", 0.96)
        # self.declare_parameter("shelf_depth", 0.40)

        # self.declare_parameter("width_tol", 0.1)          # debug value
        # self.declare_parameter("depth_tol", 0.1)          # debug value
        # self.declare_parameter("range_tol_for_same_row", 0.1)
        self.declare_parameter("yaw_tol", 0.2)


        # Topics and publishers
        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)

        self.pose_pub = self.create_publisher(PoseArray, "/detected_shelves", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/shelf_markers", 10)
        self.shelf_info_pub = self.create_publisher(
            Float32MultiArray,
            "/detected_shelf_info",
            10
        )
        self.shelf_pub = self.create_publisher(Float32MultiArray, "/detected_shelf_info", 10)


        self.seq = 0
        self.get_logger().info("ShelfDetector node started. Listening to %s" % scan_topic)

        self.last_published = None
        self.accepted_shelves = []

    # --------------------- utilities ---------------------
    def transform_point(self, x, y, from_frame, to_frame):
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame,        # target frame
                from_frame,      # source frame
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )

            nx = t.transform.translation.x
            ny = t.transform.translation.y
            dz = t.transform.translation.z

            # rotation
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w*q.z + q.x*q.y),
                1.0 - 2.0 * (q.y*q.y + q.z*q.z)
            )

            # rotate point + translate
            x_map = math.cos(yaw)*x - math.sin(yaw)*y + nx
            y_map = math.sin(yaw)*x + math.cos(yaw)*y + ny

            return x_map, y_map

        except TransformException as ex:
            self.get_logger().warn(f"TF transform failed: {ex}")
            return None

    def polar_to_xy(self, ranges, angle_min, angle_inc, range_max):
        points = []
        angles = []
        for i, r in enumerate(ranges):
            if r is None or math.isinf(r) or math.isnan(r):
                continue
            if r <= 0.0 or r > range_max:
                continue
            ang = angle_min + i * angle_inc
            x = r * math.cos(ang)
            y = r * math.sin(ang)
            points.append((x, y, ang, r))
            angles.append(ang)
        return points

    def euclidean(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def cluster_scan(self, points, gap_thresh):
        if not points:
            return []

        clusters = []
        cur = [points[0]]

        # Step 1: split into clusters based on gap
        for p_prev, p in zip(points[:-1], points[1:]):
            d = self.euclidean((p_prev[0], p_prev[1]), (p[0], p[1]))
            if d <= gap_thresh:
                cur.append(p)
            else:
                clusters.append(cur)
                cur = [p]

        clusters.append(cur)

        # Step 2: print cluster widths (debug)
        for i, cluster in enumerate(clusters):
            xs = [p[0] for p in cluster]
            ys = [p[1] for p in cluster]
            width = max(xs) - min(xs)
            height = max(ys) - min(ys)
            size = max(width, height)
            # self.get_logger().info(f"Cluster {i}: size ≈ {size:.3f} m, points = {len(cluster)}")

        return clusters

    def inside_arena(self, x, y):
        # arena dimensions: 3.1m (x), 4.2m (y)
        if abs(x) > 1.6:   # 3.1 / 2
            return False
        if abs(y) > 1.9:   # 4.2 / 2
            return False
        return True

    def compute_aabb(self, cx, cy, yaw, w=0.96, d=0.40):
        """Return AABB (minx, maxx, miny, maxy) of a rotated shelf box."""
        # corners of box around origin
        corners = [
            (+w/2, +d/2),
            (+w/2, -d/2),
            (-w/2, +d/2),
            (-w/2, -d/2),
        ]

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        xs, ys = [], []
        for dx, dy in corners:
            x = cx + dx*cos_y - dy*sin_y
            y = cy + dx*sin_y + dy*cos_y
            xs.append(x)
            ys.append(y)

        return min(xs), max(xs), min(ys), max(ys)

    def aabb_overlap_ratio(self, box1, box2):
        """Return overlap area ratio relative to box1."""
        minx1, maxx1, miny1, maxy1 = box1
        minx2, maxx2, miny2, maxy2 = box2

        inter_x = max(0, min(maxx1, maxx2) - max(minx1, minx2))
        inter_y = max(0, min(maxy1, maxy2) - max(miny1, miny2))

        if inter_x == 0 or inter_y == 0:
            return 0.0

        intersection = inter_x * inter_y
        area1 = (maxx1 - minx1) * (maxy1 - miny1)

        return intersection / area1



    # --------------------- main callback ---------------------
    def scan_callback(self, msg: LaserScan):
        
        # DEBUG 1 ----------------------
        # self.get_logger().info(f"Scan callback: {len(msg.ranges)} points")

        laser_frame = self.get_parameter("laser_frame").get_parameter_value().string_value

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        marker_array = MarkerArray()
        marker_id = 0

        # Extract parameters
        leg_dia = self.get_parameter("leg_diameter").get_parameter_value().double_value
        min_points = int(self.get_parameter("leg_min_points").get_parameter_value().integer_value)
        gap_thresh = self.get_parameter("gap_thresh").get_parameter_value().double_value
        max_cluster_len = self.get_parameter("max_cluster_length").get_parameter_value().double_value
        max_range = self.get_parameter("max_detection_range").get_parameter_value().double_value

        # ONLY legs now
        points = self.polar_to_xy(msg.ranges, msg.angle_min, msg.angle_increment, max_range)
        if len(points) == 0:
            return

        clusters = self.cluster_scan(points, gap_thresh)

        # # walls
        # walls = []
        # wall_min_length = 0.2
        # wall_max_error = 1.0
        # for c in clusters:
        #     pts = np.array([[p[0], p[1]] for p in c])
        #     if len(pts) < 2:
        #         continue

        #     xs, ys = pts[:,0], pts[:,1]
        #     length = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        #     self.get_logger().info(f"[CLUSTER] size={len(pts)}, raw length={length:.3f} m")

        #     if length < wall_min_length:
        #         continue                                    
            
        #     mean = np.mean(pts, axis=0)
        #     centered = pts - mean
        #     u, s, vh = np.linalg.svd(centered)
        #     direction = vh[0]

        #     diffs = centered
        #     cross = np.abs(diffs[:,0]*direction[1] - diffs[:,1]*direction[0])
        #     rms_error = np.sqrt(np.mean(cross**2))

        #     if rms_error > wall_max_error:
        #         continue
            
        #     transformed = self.transform_point(mean[0], mean[1], laser_frame, "map")
        #     if transformed is None:
        #         continue
        #     mx = float(transformed[0])
        #     my = float(transformed[1])       

        #     walls.append((mx, my, float(direction[0]), float(direction[1]), float(length)))
            
        #     self.get_logger().info(
        #         f"[WALL] Detected wall at map=({mx:.2f}, {my:.2f}), "
        #         f"length={length:.2f} m, direction=({direction[0]:.2f}, {direction[1]:.2f})"
        #     )

        # # --- draw wall markers (cyan) ---
        # for (wx, wy, dx, dy, length) in walls:
        #     wx = float(wx)
        #     wy = float(wy)
        #     dx = float(dx)
        #     dy = float(dy)
        #     length = float(length)

        #     m = Marker()
        #     m.header = header
        #     m.header.frame_id = "map"
        #     m.ns = "walls"
        #     m.id = marker_id; marker_id += 1
        #     m.type = Marker.LINE_STRIP
        #     m.action = Marker.ADD
        #     lx1 = mean[0] - direction[0] * (length/2.0)
        #     ly1 = mean[1] - direction[1] * (length/2.0)

        #     lx2 = mean[0] + direction[0] * (length/2.0)
        #     ly2 = mean[1] + direction[1] * (length/2.0)
        #     p1_map = self.transform_point(lx1, ly1, laser_frame, "map")
        #     p2_map = self.transform_point(lx2, ly2, laser_frame, "map")

        #     if p1_map is None or p2_map is None:
        #         continue

        #     px1, py1 = float(p1_map[0]), float(p1_map[1])
        #     px2, py2 = float(p2_map[0]), float(p2_map[1])

        #     # Two points define the wall segment
        #     m.points.append(Point(x=px1, y=py1, z=0.0))
        #     m.points.append(Point(x=px2, y=py2, z=0.0))

        #     m.scale.x = 0.05    # shaft diameter
        #     m.scale.y = 0.1     # head diameter
        #     m.scale.z = 0.1     # head length
        #     m.color = ColorRGBA(r=0.2, g=0.8, b=1.0, a=0.9)

        #     marker_array.markers.append(m)

        leg_candidates = []

        for c in clusters:
            if len(c) < min_points:
                continue
            xs = [p[0] for p in c]
            ys = [p[1] for p in c]
            centroid = (float(np.mean(xs)), float(np.mean(ys)))

            # Compute max internal distance
            if len(c) < 2:
                max_len = 0.0
            else:
                max_len = max(
                    self.euclidean((c[i][0], c[i][1]), (c[j][0], c[j][1]))
                    for i in range(len(c)) for j in range(i + 1, len(c))
                )

            if max_len <= max_cluster_len:
                mean_range = float(np.mean([p[3] for p in c]))
                leg_candidates.append({'centroid': centroid, 'range': mean_range, 'points': c})

        # Draw only leg markers
        laser_frame = self.get_parameter("laser_frame").get_parameter_value().string_value
        map_frame = "map"

        map_legs = []
        for inx, leg in enumerate(leg_candidates):
            cx, cy = leg['centroid']

            # --- transform centroid from laser → map ---
            transformed = self.transform_point(cx, cy, laser_frame, map_frame)
            if transformed is None:
                # skip if tf missing
                continue

            mx, my = transformed
            # # ignore legs outside arena
            # if not self.inside_arena(mx, my):
            #     self.get_logger().info(f"Leg {inx} ignored (outside arena)")
            #     continue

            map_legs.append((mx, my))


            # Debug
            # self.get_logger().info(f"Leg {inx} (map frame): x={mx:.3f}, y={my:.3f}")

            # --- publish marker in map frame ---
            m_leg = Marker()
            m_leg.header = header
            m_leg.header.frame_id = "map"     # IMPORTANT
            m_leg.ns = "leg_clusters"
            m_leg.id = marker_id; marker_id += 1
            m_leg.type = Marker.SPHERE
            m_leg.action = Marker.ADD
            m_leg.pose.position.x = mx
            m_leg.pose.position.y = my
            m_leg.pose.position.z = 0.0
            m_leg.scale.x = 0.06
            m_leg.scale.y = 0.06
            m_leg.scale.z = 0.06
            m_leg.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
            marker_array.markers.append(m_leg)

        deduped_legs = []
        merge_thresh = 0.2 # determine how close the centroid needs to be to merge

        for leg in map_legs:
            lx, ly = leg
            too_close = False
            for (dx, dy) in deduped_legs:
                if math.hypot(lx - dx, ly - dy) < merge_thresh:
                    too_close = True
                    break
            if not too_close:
                deduped_legs.append((lx, ly))

        map_legs = deduped_legs   

        # ----------------------------
        # Shelf Detection (map frame)
        # ----------------------------

        shelf_min = 0.7
        shelf_max = 1.3

        from itertools import combinations

        for (i, j) in combinations(range(len(map_legs)), 2):
            p1 = map_legs[i]
            p2 = map_legs[j]

            dist = math.dist(p1, p2)
            if not (shelf_min <= dist <= shelf_max):
                self.get_logger().info(
                    f"Rejected leg pair (legs {i},{j}) due to distance {dist:.3f} m."
                )
                continue

            

            # ----- Midpoint (shelf center) -----
            cx = (p1[0] + p2[0]) / 2.0
            cy = (p1[1] + p2[1]) / 2.0
            
            # Ignore shelf centers outside arena
            if not self.inside_arena(cx, cy):
                self.get_logger().info(f"Ignored shelf at ({cx:.2f}, {cy:.2f}) outside arena.")
                continue

            # ----- shelf direction vector -----
            vx = p2[0] - p1[0]
            vy = p2[1] - p1[1]

            mag = math.hypot(vx, vy)

            ux = vx / mag
            uy = vy / mag

            # ----- normal vector -----
            nx = -uy
            ny = ux

            # ----- shelf yaw -----
            yaw = math.atan2(ny, nx)  # radians

            # If yaw is basically -pi/2, make it +pi/2 for consistency
            if abs(yaw + math.pi/2) < 0.05:   # tolerance = 0.05 rad
                yaw = -math.pi/2
            
            if abs(yaw - math.pi) < 0.05 or abs(yaw + math.pi) < 0.05:
                yaw = 0.0

            yaw_tol = 0.2  # radians
            yaw_norm = (yaw + 2*math.pi) % math.pi  # normalize to 0..pi

            # --- Yaw filter ---
            if not (abs(yaw_norm - 0) < yaw_tol or abs(yaw_norm - math.pi/2) < yaw_tol):
                self.get_logger().info(
                    f"Rejected shelf (legs {i},{j}) due to yaw={math.degrees(yaw_norm):.1f}°"
                )
                continue  # skip this pair

            # Adjust centroid based on yaw
            if abs(yaw_norm - 0) < yaw_tol:      # shelf along x-axis
                cx += 0.15   # move back along -x
            elif abs(yaw_norm - math.pi/2) < yaw_tol:  # shelf along y-axis
                cy -= 0.15   # move back along -y
                
            # self.get_logger().info(
            #     f"SHELF detected between legs {i} & {j}, dist={dist:.3f}, center=({cx:.2f}, {cy:.2f})"
            # )

            # ---------------------------------------------
            #  DEDUPLICATION: ignore shelf if overlapping
            # ---------------------------------------------
            new_aabb = self.compute_aabb(cx, cy, yaw)

            ignore = False
            for old_aabb in self.accepted_shelves:
                overlap = self.aabb_overlap_ratio(new_aabb, old_aabb)
                if overlap > 0.35:
                    ignore = True
                    break

            if ignore:
                continue  # skip this shelf, too overlapping

            self.accepted_shelves.append(new_aabb)

            # Start 5-second shutdown timer after first shelf detected
            if not self.first_shelf_detected:
                self.first_shelf_detected = True
                self.get_logger().info("First shelf detected, starting 40-second shutdown timer.")
                self.stop_timer = self.create_timer(self.detection_duration, self.shutdown_node)
            
            # Draw BLUE shelf-center sphere
            m_s = Marker()
            m_s.header = header
            m_s.header.frame_id = "map"
            m_s.ns = "shelf_centers"
            m_s.id = marker_id; marker_id += 1
            m_s.type = Marker.SPHERE
            m_s.action = Marker.ADD
            m_s.pose.position.x = cx
            m_s.pose.position.y = cy
            m_s.pose.position.z = 0.0
            m_s.scale.x = 0.12
            m_s.scale.y = 0.12
            m_s.scale.z = 0.12
            m_s.color = ColorRGBA(r=0.0, g=0.4, b=1.0, a=0.95)  # bright blue
            marker_array.markers.append(m_s)

            # Norml vector ARROW (red)
            m_arrow = Marker()
            m_arrow.header = header
            m_arrow.header.frame_id = "map"
            m_arrow.ns = "shelf_yaw"
            m_arrow.id = marker_id; marker_id += 1
            m_arrow.type = Marker.ARROW
            m_arrow.action = Marker.ADD

            start = Point(x=cx, y=cy, z=0.0)
            length = 0.3  # 30 cm long arrow
            end = Point(x=cx + nx*length, y=cy + ny*length, z=0.0)

            m_arrow.points = [start, end]

            m_arrow.scale.x = 0.05  # shaft diameter
            m_arrow.scale.y = 0.08  # head diameter
            m_arrow.scale.z = 0.08  # head length

            m_arrow.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)  # red arrow
            marker_array.markers.append(m_arrow)

            # ---------------------------------------
            # Publish shelf info on /detected_shelf_info
            # Format: [cx, cy, nx, ny]
            # ---------------------------------------
            data_tuple = (round(cx, 3), round(cy, 3), round(nx, 3), round(ny, 3))

            # if nothing changed, do not publish
            if self.last_published == data_tuple:
                return
            
            # otherwise publish
            msg = Float32MultiArray()
            msg.data = [float(cx), float(cy), float(nx), float(ny)]
            self.shelf_info_pub.publish(msg)

            # update memory
            self.last_published = data_tuple

            # Draw GREEN bounding box
            m_box = Marker()
            m_box.header = header
            m_box.header.frame_id = "map"
            m_box.ns = "shelf_bbox"
            m_box.id = marker_id; marker_id += 1
            m_box.type = Marker.CUBE
            m_box.action = Marker.ADD

            # Center of the box = shelf centroid
            m_box.pose.position.x = cx
            m_box.pose.position.y = cy
            m_box.pose.position.z = 0.0

            # Orientation — rotate the box by yaw
            q = tf_transformations.quaternion_from_euler(0, 0, yaw-math.pi/2)
            m_box.pose.orientation.x = q[0]
            m_box.pose.orientation.y = q[1]
            m_box.pose.orientation.z = q[2]
            m_box.pose.orientation.w = q[3]

            # Box dimensions (meters)
            m_box.scale.x = 0.96   # shelf width
            m_box.scale.y = 0.4   # shelf depth
            m_box.scale.z = 0.05   # thickness so it is visible in RViz

            # Color: semi-transparent green
            m_box.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.35)

            marker_array.markers.append(m_box)
        # Publish all markers   
        self.marker_pub.publish(marker_array)

    def shutdown_node(self):
        self.get_logger().info("Shelf detection time over. Shutting down node.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ShelfDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        # rclpy.shutdown()

if __name__ == "__main__":
    main()