#!/usr/bin/env python3
"""Camera HUD overlay node.

Subscribes to CameraSensor and DynamicsSensorOdom for a given agent, draws a
pilot-style HUD (artificial horizon, heading tape, depth/speed gauges, data
readout), and republishes the annotated image.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import numpy as np
import cv2
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation

# BGR color palette
_G = (0, 210, 55)        # green  – static HUD chrome
_A = (0, 165, 255)       # amber  – live data / needles
_W = (220, 220, 220)     # off-white – labels
_BG = (12, 12, 12)       # panel background

_FONT = cv2.FONT_HERSHEY_SIMPLEX


class CameraHUDNode(Node):
    def __init__(self):
        super().__init__('camera_hud')

        self.declare_parameter('agent_name', 'auv0')
        agent = self.get_parameter('agent_name').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.odom: Odometry | None = None
        self.agent_name = agent

        self.create_subscription(Image,    f'{agent}/CameraSensor',       self._image_cb, 10)
        self.create_subscription(Odometry, f'{agent}/DynamicsSensorOdom', self._odom_cb,  10)
        self.pub = self.create_publisher(Image, f'{agent}/CameraHUD', 10)

        self.get_logger().info(f'Camera HUD ready  [{agent}]')

    #  Callbacks

    def _odom_cb(self, msg: Odometry):
        self.odom = msg

    def _image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.odom is not None:
            frame = self._draw_hud(frame)
        out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out.header = msg.header
        self.pub.publish(out)

    #  Top-level draw

    def _draw_hud(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2

        pos = self.odom.pose.pose.position
        vel = self.odom.twist.twist.linear
        ang = self.odom.twist.twist.angular
        q   = self.odom.pose.pose.orientation

        rpy = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=True)
        roll, pitch, yaw = float(rpy[0]), float(rpy[1]), float(rpy[2])
        heading = yaw % 360.0
        depth   = -float(pos.z)          # UE Z-up: below surface → negative z
        spd_fwd = float(vel.x)

        # Artificial horizon is drawn directly onto the frame so the
        # semi-transparent panel blend does not wash it out.
        self._draw_horizon(frame, cx, cy, roll, pitch, h)

        # Everything else on an overlay for a uniform transparency pass.
        overlay = frame.copy()
        self._draw_crosshair(overlay, cx, cy)
        self._draw_roll_arc(overlay, cx, cy, roll)
        self._draw_heading_tape(overlay, heading, w, h)
        self._draw_depth_gauge(overlay, depth, h)
        self._draw_speed_gauge(overlay, spd_fwd, h, w)
        self._draw_data_panel(overlay, pos, vel, ang, roll, pitch, heading, h, w)

        cv2.addWeighted(overlay, 0.9, frame, 0.1, 0, frame)
        return frame

    #  Artificial horizon & pitch ladder

    def _draw_horizon(self, img, cx, cy, roll, pitch, h):
        roll_rad   = np.deg2rad(roll)
        cos_r, sin_r = np.cos(roll_rad), np.sin(roll_rad)

        # pixels per degree of pitch (60 deg fills half the frame height)
        px_per_deg = h / 60.0

        # horizon direction and "aircraft-up" direction in screen coords
        # screen: x-right, y-down
        # horizon direction: (cos_r, sin_r)
        # aircraft-up:      (sin_r, -cos_r)
        hdir  = np.array([ cos_r,  sin_r])
        up_dir = np.array([ sin_r, -cos_r])

        half = int(img.shape[1] * 0.65)

        # horizon centre (shifts down when pitch is positive / nose-up)
        hcx = cx
        hcy = cy + int(pitch * px_per_deg)
        p1 = (int(hcx - half * hdir[0]), int(hcy - half * hdir[1]))
        p2 = (int(hcx + half * hdir[0]), int(hcy + half * hdir[1]))
        cv2.line(img, p1, p2, _G, 2, cv2.LINE_AA)

        # Pitch ladder — marks live at:
        #   centre = (cx, cy) + deg*px_per_deg * up_dir
        # which simplifies to the horizon for deg==pitch
        for deg in range(-30, 35, 5):
            if deg == 0:
                continue
            mark_cx = int(cx + deg * px_per_deg * up_dir[0])
            mark_cy = int(cy + deg * px_per_deg * up_dir[1])

            # skip if off screen
            if not (0 < mark_cy < img.shape[0] and 0 < mark_cx < img.shape[1]):
                continue

            mark_len = 30 if deg % 10 == 0 else 14
            mp1 = (int(mark_cx - mark_len * hdir[0]), int(mark_cy - mark_len * hdir[1]))
            mp2 = (int(mark_cx + mark_len * hdir[0]), int(mark_cy + mark_len * hdir[1]))
            cv2.line(img, mp1, mp2, _G, 1, cv2.LINE_AA)

            if deg % 10 == 0:
                lx = int(mark_cx + mark_len * hdir[0]) + 5
                ly = int(mark_cy + mark_len * hdir[1]) + 4
                cv2.putText(img, f'{abs(deg)}', (lx, ly), _FONT, 0.32, _G, 1, cv2.LINE_AA)

    #  Crosshair

    def _draw_crosshair(self, img, cx, cy):
        arm, gap = 18, 7
        cv2.line(img, (cx - arm - gap, cy), (cx - gap, cy),       _G, 2, cv2.LINE_AA)
        cv2.line(img, (cx + gap, cy),       (cx + arm + gap, cy), _G, 2, cv2.LINE_AA)
        cv2.line(img, (cx, cy - arm - gap), (cx, cy - gap),       _G, 2, cv2.LINE_AA)
        cv2.line(img, (cx, cy + gap),       (cx, cy + arm + gap), _G, 2, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), 2, _G, -1, cv2.LINE_AA)

    #  Roll arc (bank angle indicator, centred above crosshair)

    def _draw_roll_arc(self, img, cx, cy, roll):
        r = 80
        # Static arc chrome: -60° to +60° (measured from 12 o'clock)
        cv2.ellipse(img, (cx, cy), (r, r), -90, -60, 60, _G, 1, cv2.LINE_AA)

        # Tick marks at ±10, ±20, ±30, ±45, ±60
        for deg in [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]:
            a = np.deg2rad(deg - 90)
            tick = 10 if deg % 30 == 0 else 5
            x1, y1 = int(cx + r * np.cos(a)),       int(cy + r * np.sin(a))
            x2, y2 = int(cx + (r - tick) * np.cos(a)), int(cy + (r - tick) * np.sin(a))
            cv2.line(img, (x1, y1), (x2, y2), _G, 1, cv2.LINE_AA)

        # Moving pointer (filled triangle) at current roll
        a = np.deg2rad(-roll - 90)
        tip_x = int(cx + (r - 14) * np.cos(a))
        tip_y = int(cy + (r - 14) * np.sin(a))
        ba1 = np.deg2rad(-roll - 90 + 7)
        ba2 = np.deg2rad(-roll - 90 - 7)
        pts = np.array([
            [tip_x, tip_y],
            [int(cx + r * np.cos(ba1)), int(cy + r * np.sin(ba1))],
            [int(cx + r * np.cos(ba2)), int(cy + r * np.sin(ba2))],
        ], np.int32)
        cv2.fillPoly(img, [pts], _A)

        # Numeric roll value
        self._text(img, f'{roll:+.1f}', cx - 18, cy - r - 6, 0.35, _A)

    #  Heading tape

    def _draw_heading_tape(self, img, heading, w, h):
        tape_h = 36
        x0, x1 = w // 4, 3 * w // 4
        y0, y1 = 8, 8 + tape_h
        self._panel(img, x0, y0, x1, y1)

        mx = (x0 + x1) // 2
        tape_w = x1 - x0
        px_per_deg = tape_w / 60.0

        for off in range(-35, 36):
            deg = int((heading + off) % 360)
            x = int(mx + off * px_per_deg)
            if x < x0 + 2 or x > x1 - 2:
                continue
            if off % 10 == 0:
                cv2.line(img, (x, y0), (x, y0 + tape_h // 2), _G, 1)
                label = {0: 'N', 90: 'E', 180: 'S', 270: 'W'}.get(deg, f'{deg:03d}')
                self._text(img, label, x - 8, y1 - 4, 0.32, _G)
            elif off % 5 == 0:
                cv2.line(img, (x, y0), (x, y0 + tape_h // 3), _G, 1)

        # Centre notch (amber)
        pts = np.array([[mx, y1], [mx - 5, y0 + tape_h // 2],
                        [mx + 5, y0 + tape_h // 2]], np.int32)
        cv2.fillPoly(img, [pts], _A)

        # Numeric heading box below tape
        bw = 58
        self._panel(img, mx - bw // 2, y1, mx + bw // 2, y1 + 20)
        self._text(img, f'{heading:05.1f}', mx - bw // 2 + 3, y1 + 14, 0.38, _A, 1)

    #  Depth gauge  (left side)

    def _draw_depth_gauge(self, img, depth, h):
        gx0, gx1 = 10, 62
        gy0, gy1 = h // 5, 4 * h // 5
        gh = gy1 - gy0
        self._panel(img, gx0, gy0, gx1, gy1)
        self._text(img, 'D', gx0 + 16, gy0 - 5, 0.38, _G)

        max_d = 50.0
        px_per_m = gh / max_d

        for d in range(0, int(max_d) + 1):
            y = int(gy0 + d * px_per_m)
            if y > gy1:
                break
            if d % 10 == 0:
                cv2.line(img, (gx0 + 2, y), (gx1 - 2, y), _G, 1)
                self._text(img, f'{d}', gx0 + 3, y + 4, 0.30, _G)
            elif d % 5 == 0:
                cv2.line(img, (gx0 + 18, y), (gx1 - 2, y), _G, 1)

        # Needle
        ny = int(gy0 + float(np.clip(depth, 0.0, max_d)) * px_per_m)
        cv2.line(img, (gx0, ny), (gx1, ny), _A, 2)
        self._panel(img, gx0, ny - 13, gx1, ny + 2)
        self._text(img, f'{depth:.1f}', gx0 + 2, ny - 1, 0.36, _A)
        self._text(img, 'm', gx0 + 18, gy1 + 14, 0.35, _G)

    #  Speed gauge  (right side, centred at zero, forward positive)

    def _draw_speed_gauge(self, img, spd_fwd, h, w):
        gx0, gx1 = w - 62, w - 10
        gy0, gy1 = h // 5, 4 * h // 5
        gh = gy1 - gy0
        self._panel(img, gx0, gy0, gx1, gy1)
        self._text(img, 'SPD', gx0 + 2, gy0 - 5, 0.35, _G)

        max_s = 10.0
        zero_y = (gy0 + gy1) // 2
        px_per_ms = gh / (2 * max_s)

        # Zero line
        cv2.line(img, (gx0 + 2, zero_y), (gx1 - 2, zero_y), _G, 1)

        for s in range(-int(max_s), int(max_s) + 1):
            y = int(zero_y - s * px_per_ms)
            if y < gy0 or y > gy1:
                continue
            if s % 5 == 0:
                cv2.line(img, (gx0 + 2, y), (gx1 - 2, y), _G, 1)
                self._text(img, f'{s}', gx0 + 3, y + 4, 0.30, _G)
            elif s % 2 == 0:
                cv2.line(img, (gx0 + 18, y), (gx1 - 2, y), _G, 1)

        # Needle
        ny = int(zero_y - float(np.clip(spd_fwd, -max_s, max_s)) * px_per_ms)
        cv2.line(img, (gx0, ny), (gx1, ny), _A, 2)
        self._panel(img, gx0, ny - 13, gx1, ny + 2)
        self._text(img, f'{spd_fwd:.2f}', gx0 + 2, ny - 1, 0.34, _A)
        self._text(img, 'm/s', gx0 + 2, gy1 + 14, 0.35, _G)

    #  Data panel  (bottom, right of depth gauge)

    def _draw_data_panel(self, img, pos, vel, ang, roll, pitch, heading, h, w):
        pw, ph = 180, 112
        x0, y0 = 72, h - ph - 10
        x1, y1 = x0 + pw, y0 + ph
        self._panel(img, x0, y0, x1, y1)

        spd = float(np.sqrt(vel.x**2 + vel.y**2 + vel.z**2))

        lines = [
            (f'X  {pos.x:+8.1f} m',         _G),
            (f'Y  {pos.y:+8.1f} m',         _G),
            (f'Z  {pos.z:+8.1f} m',         _G),
            (f'SPD {spd:6.2f} m/s',          _A),
            (f'R {roll:+5.1f}  P {pitch:+5.1f}', _G),
            (f'HDG     {heading:6.1f} deg',  _A),
        ]
        self._text(img, self.agent_name.upper(), x0 + 5, y0 - 3, 0.38, _W)
        for i, (line, color) in enumerate(lines):
            self._text(img, line, x0 + 5, y0 + 15 + i * 16, 0.36, color)

    #  Drawing helpers

    @staticmethod
    def _panel(img, x0, y0, x1, y1):
        """Semi-transparent dark panel with a green border."""
        x0, y0 = max(x0, 0), max(y0, 0)
        x1, y1 = min(x1, img.shape[1]), min(y1, img.shape[0])
        if x1 <= x0 or y1 <= y0:
            return
        roi = img[y0:y1, x0:x1]
        dark = np.full_like(roi, _BG[0])
        img[y0:y1, x0:x1] = cv2.addWeighted(dark, 0.60, roi, 0.40, 0)
        cv2.rectangle(img, (x0, y0), (x1, y1), _G, 1)

    @staticmethod
    def _text(img, txt, x, y, scale=0.45, color=_G, thickness=1):
        cv2.putText(img, txt, (x, y), _FONT, scale, color, thickness, cv2.LINE_AA)


def main(args=None):
    rclpy.init(args=args)
    node = CameraHUDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
