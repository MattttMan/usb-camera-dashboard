#!/usr/bin/env python3
"""
DECXIN Dual Stereo Camera + ICM-42688 IMU Viewer
Official decode protocol from DECXIN TimeStamp_Data_Decode_DemoCode_V010002

IMU chip : ICM-42688 (TDK InvenSense)
Rate     : ~600 Hz (11 samples per 30 fps frame)
Accel    : ±4 G  (default),  resolution = 4000/32768 mg/LSB
Gyro     : ±1000 °/s (default), resolution = 1000/32768 °/s per LSB
Strip    : top rows of left-eye image, LSB-first 8×8 pixel cells
Packet   : 16 bytes = 4-byte µs timestamp + 6×2-byte int16 (AX AY AZ GX GY GZ)
"""

import cv2
import numpy as np
import struct
import time
from collections import deque

# ── Config ────────────────────────────────────────────────────────────────────
CAM_INDICES = [0, 1]
# Each eye is 2000×1200 → aspect 5:3.  At eye_w=700px → correct height=420px.
W           = 1400
EYE_H       = 420          # correct aspect ratio (700 * 3/5 = 420)
VIDEO_H     = len(CAM_INDICES) * EYE_H   # 840
PANEL_H     = 240
H           = VIDEO_H + PANEL_H          # 1080
BOX_W       = 340

# ICM-42688 default scale (as used by official sample)
AFS_4G      = 4000.0 / 32768.0    # mg/LSB → divide by 1000 for g
GFS_1000DPS = 1000.0 / 32768.0    # dps/LSB

# Complementary filter
ALPHA       = 0.05      # accel weight
IMU_HISTORY = 300       # samples in history graphs (~0.5 s at 600 Hz)

CAM_COLORS = [(80, 220, 80), (80, 180, 255)]
AXIS_COLS  = [(255,80,80),(255,160,80),(255,255,80),(80,255,80),(80,255,200),(80,180,255)]
AXIS_NAMES = ['AX','AY','AZ','GX','GY','GZ']

# ── Official IMU strip decoder ────────────────────────────────────────────────

U_SZ = 8   # cell size (pixels)

def _decode_row(row_g):
    """
    Decode one image row (green channel, 1-D array) using the official algorithm.
    Returns list of bytes (may be 0 if row is not a data row).
    """
    # Preamble check: first few bytes must be > 220
    if row_g[0] <= 220 or row_g[1] <= 220 or row_g[2] <= 220 or row_g[3] <= 220:
        return []

    # Find first pixel below 220 to locate data start
    start = None
    for ind in range(len(row_g)):
        if row_g[ind] < 220:
            start = ind + (U_SZ >> 1)
            break
    if start is None:
        return []

    # Read cells (two adjacent pixels summed)
    bits = []
    ind = start
    while ind + 1 < len(row_g):
        val = int(row_g[ind]) + int(row_g[ind + 1])
        if val < 100:
            bits.append(0)
        elif val < 440:
            bits.append(1)
        else:
            break           # end marker
        ind += U_SZ

    # Pack bits LSB-first into bytes
    result = []
    for i in range(0, len(bits) - 7, 8):
        b = 0
        for j in range(8):
            b |= bits[i + j] << j
        result.append(b)
    return result


def decode_imu_frame(frame):
    """
    Decode all ICM-42688 IMU packets from a video frame.
    Returns (packets, exposure_start_us, exposure_end_us)
    packets: list of dicts {t_us, ax, ay, az, gx, gy, gz}
    exposure times are 0 if not decoded
    """
    H_f, W_f = frame.shape[:2]

    # The strip is in the left-eye half (cols 0..W_f//2).
    # BMP starts at row (height-3) from bottom → OpenCV row 2.
    # Each logical data row = 8 pixel rows.

    buf   = bytearray()
    line  = 0
    max_lines = 200

    # First pass: read until we have the 16-byte header group
    while len(buf) < 16 and line < max_lines:
        bmp_row = H_f - 3 - line * U_SZ
        ocv_row = H_f - 1 - bmp_row
        if 0 <= ocv_row < H_f:
            row_g = frame[ocv_row, :W_f // 2, 1]   # green channel, left eye only
            buf.extend(_decode_row(row_g))
        line += 1

    if len(buf) < 16:
        return [], 0, 0

    # Parse header — Group 1: [Fixed Header 8B][Exposure Start 4B][Exposure End 4B]
    hdr = bytes(buf[:16])
    if hdr[:8] == hdr[8:16]:                    # old protocol (header repeated)
        n_pkts = 11
        exp_start_us = 0
        exp_end_us   = 0
    else:                                        # new protocol
        n_pkts       = hdr[1] & 0x0F            # DeviceDtSize[0]
        exp_start_us = struct.unpack('>I', hdr[8:12])[0]
        exp_end_us   = struct.unpack('>I', hdr[12:16])[0]

    total_needed = 16 * (1 + n_pkts)

    # Second pass: collect IMU data groups
    while len(buf) < total_needed and line < max_lines:
        bmp_row = H_f - 3 - line * U_SZ
        ocv_row = H_f - 1 - bmp_row
        if 0 <= ocv_row < H_f:
            row_g = frame[ocv_row, :W_f // 2, 1]
            buf.extend(_decode_row(row_g))
        line += 1

    # Decode packets
    packets = []
    for i in range(n_pkts):
        off = 16 * (1 + i)
        if off + 16 > len(buf):
            break
        p = buf[off:off + 16]
        t    = struct.unpack('>I', p[0:4])[0]
        raw  = struct.unpack('>hhhhhh', p[4:16])
        ax_g  = raw[0] * AFS_4G   / 1000.0
        ay_g  = raw[1] * AFS_4G   / 1000.0
        az_g  = raw[2] * AFS_4G   / 1000.0
        gx    = raw[3] * GFS_1000DPS
        gy    = raw[4] * GFS_1000DPS
        gz    = raw[5] * GFS_1000DPS
        # Error check: all axes == -1 or gyro_x == -32768
        if (raw[0]==-1 and raw[1]==-1 and raw[2]==-1) or raw[3]==-32768:
            continue
        packets.append(dict(t_us=t, ax=ax_g, ay=ay_g, az=az_g,
                            gx=gx, gy=gy, gz=gz))
    return packets, exp_start_us, exp_end_us


# ── Complementary filter ──────────────────────────────────────────────────────

class OrientationFilter:
    def __init__(self):
        self.roll = self.pitch = self.yaw = 0.0
        self._last_t = None

    def update(self, packets):
        if not packets:
            return self.roll, self.pitch, self.yaw

        for p in packets:
            t_us = p['t_us']
            if self._last_t is None:
                self._last_t = t_us
                continue
            dt = (t_us - self._last_t) * 1e-6
            self._last_t = t_us
            if dt <= 0 or dt > 0.05:
                continue

            ax, ay, az = p['ax'], p['ay'], p['az']
            gx, gy, gz = p['gx'], p['gy'], p['gz']

            # Accel tilt estimate
            norm = np.sqrt(ax*ax + ay*ay + az*az) + 1e-9
            roll_acc  = np.arctan2(ay/norm, az/norm)
            pitch_acc = np.arctan2(-ax/norm, np.sqrt((ay/norm)**2 + (az/norm)**2))

            # Gyro integration (°/s → rad)
            self.roll  += np.radians(gx) * dt
            self.pitch += np.radians(gy) * dt
            self.yaw   += np.radians(gz) * dt

            # Fuse
            self.roll  = (1-ALPHA)*self.roll  + ALPHA*roll_acc
            self.pitch = (1-ALPHA)*self.pitch + ALPHA*pitch_acc

        return self.roll, self.pitch, self.yaw

    def reset(self):
        self.roll = self.pitch = self.yaw = 0.0
        self._last_t = None


def euler_to_R(roll, pitch, yaw):
    cr,sr = np.cos(roll), np.sin(roll)
    cp,sp = np.cos(pitch),np.sin(pitch)
    cy,sy = np.cos(yaw), np.sin(yaw)
    return (np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
          @ np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
          @ np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]]))


# ── 3D box ────────────────────────────────────────────────────────────────────

_VERTS = np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],
                   [-1,-1, 1],[1,-1, 1],[1,1, 1],[-1,1, 1]], dtype=float)
_FACE_VI  = [[0,1,2,3],[4,5,6,7],[0,1,5,4],[2,3,7,6],[0,3,7,4],[1,2,6,5]]
_FACE_COL = [(140,60,60),(60,140,60),(60,60,140),(140,140,60),(140,60,140),(60,140,140)]
_LIGHT    = np.array([0.4,0.7,1.0]); _LIGHT /= np.linalg.norm(_LIGHT)

def draw_box(canvas, R, cx, cy, size=80, focal=200, tint=(255,255,255)):
    v3 = _VERTS * size
    def proj(v):
        p = R @ v
        d = focal / (p[2] + focal*2.5 + 1e-6)
        return (int(cx+p[0]*d), int(cy-p[1]*d))
    pts = [proj(v) for v in v3]
    face_data = []
    for vi, bc in zip(_FACE_VI, _FACE_COL):
        vr = [R @ v3[i] for i in vi]
        z  = np.mean([v[2] for v in vr])
        n  = np.cross(vr[1]-vr[0], vr[2]-vr[0])
        n /= (np.linalg.norm(n)+1e-9)
        face_data.append((z, vi, bc, n))
    face_data.sort(key=lambda x: x[0])
    for _, vi, bc, n in face_data:
        if (R @ n)[2] < 0: continue
        sh  = max(0.3, float(np.dot(n, _LIGHT)))
        col = tuple(min(255,int(c*sh*t/255)) for c,t in zip(bc,tint))
        poly = np.array([pts[i] for i in vi], np.int32)
        cv2.fillPoly(canvas, [poly], col)
        cv2.polylines(canvas, [poly], True, (210,210,210), 1, cv2.LINE_AA)
    o = proj(np.zeros(3))
    for v,col,lbl in [(np.array([size*1.4,0,0]),(0,0,255),'X'),
                      (np.array([0,size*1.4,0]),(0,200,0),'Y'),
                      (np.array([0,0,size*1.4]),(200,0,0),'Z')]:
        end = proj(R @ v)
        cv2.arrowedLine(canvas, o, end, col, 2, tipLength=0.25, line_type=cv2.LINE_AA)
        cv2.putText(canvas, lbl, (end[0]+3,end[1]+4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)


# ── Graph helpers ─────────────────────────────────────────────────────────────

def draw_graphs(canvas, histories, x0, y0, gw, gh):
    cv2.rectangle(canvas, (x0,y0),(x0+gw,y0+gh),(22,22,32),-1)
    cv2.rectangle(canvas, (x0,y0),(x0+gw,y0+gh),(60,60,70),1)
    zy = y0 + gh//2
    cv2.line(canvas,(x0,zy),(x0+gw,zy),(60,60,70),1)
    n = len(histories[0][0])
    if n < 2: return
    for ci, (cam_hist, tint) in enumerate(histories):
        alpha = 1.0 if ci == 0 else 0.55
        for ch, (hist, col) in enumerate(zip(cam_hist, AXIS_COLS)):
            pts = [(x0+int(i*gw/IMU_HISTORY),
                    zy-int(np.clip(v,-2,2)*(gh//2-4)/2))
                   for i,v in enumerate(hist)]
            sc = tuple(int(c*alpha) for c in col)
            for i in range(1,len(pts)):
                cv2.line(canvas, pts[i-1], pts[i], sc, 1, cv2.LINE_AA)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    caps = []
    for idx in CAM_INDICES:
        c = cv2.VideoCapture(idx)
        if not c.isOpened():
            print(f'Cannot open camera {idx}'); return
        print(f'  Camera {idx}: {int(c.get(cv2.CAP_PROP_FRAME_WIDTH))}×'
              f'{int(c.get(cv2.CAP_PROP_FRAME_HEIGHT))}')
        caps.append(c)

    filters  = [OrientationFilter() for _ in caps]
    # histories[cam][axis] = deque of float values
    histories = [[deque([0.]*IMU_HISTORY, maxlen=IMU_HISTORY)
                  for _ in range(6)] for _ in caps]
    fps_hist    = deque(maxlen=30)
    t_last      = time.time()
    show_graphs = False   # toggle with G

    cv2.namedWindow('DECXIN Dual IMU', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('DECXIN Dual IMU', W, H)
    print(f"\n{len(caps)} cameras, ICM-42688 @ ~600 Hz.  Q=quit  R=reset  G=graphs")

    while True:
        now = time.time()
        fps_hist.append(1.0/max(now-t_last,1e-6))
        t_last = now

        frames = []
        for c in caps:
            ret, f = c.read()
            if not ret: break
            frames.append(f)
        if len(frames) < len(caps): break

        canvas = np.zeros((H, W, 3), np.uint8)
        eye_w  = W // 2
        cam_data = []

        for ci, frame in enumerate(frames):
            # ── Decode IMU ─────────────────────────────────────────────────
            pkts, exp_start, exp_end = decode_imu_frame(frame)
            roll, pitch, yaw = filters[ci].update(pkts)

            vals6 = [0.0]*6
            if pkts:
                p = pkts[-1]
                vals6 = [p['ax'], p['ay'], p['az'], p['gx'], p['gy'], p['gz']]
            for ch, v in enumerate(vals6):
                histories[ci][ch].append(v)

            cam_data.append((pkts, roll, pitch, yaw, vals6, exp_start, exp_end))

            # ── Video display ──────────────────────────────────────────────
            fh, fw = frame.shape[:2]
            half   = fw // 2
            # Left eye: cols STRIP_CROP..half (crop the encoded strip rows at top)
            # Right eye: cols half..fw
            STRIP_CROP = 0   # let user see the strip as-is; it's only top ~32 rows
            l_raw = frame[:, STRIP_CROP:half, :]
            r_raw = frame[:, half:, :]
            row_y = ci * EYE_H
            canvas[row_y:row_y+EYE_H,    :eye_w] = cv2.resize(l_raw, (eye_w, EYE_H))
            canvas[row_y:row_y+EYE_H, eye_w:W  ] = cv2.resize(r_raw, (eye_w, EYE_H))

            # Overlay: strip data visible at top
            n_pkts = len(pkts)
            t_span = (pkts[-1]['t_us']-pkts[0]['t_us']) if n_pkts>1 else 0
            imu_hz = (n_pkts-1)/max(t_span*1e-6,1e-6) if n_pkts>1 else 0
            col = CAM_COLORS[ci]
            cv2.putText(canvas, f'CAM {ci}  L', (8, row_y+18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
            cv2.putText(canvas, f'CAM {ci}  R', (eye_w+8, row_y+18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
            cv2.putText(canvas, f'IMU {n_pkts}pkt {imu_hz:.0f}Hz',
                        (eye_w-140, row_y+18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)
            cv2.line(canvas,(eye_w,row_y),(eye_w,row_y+EYE_H),(80,80,80),1)

        cv2.line(canvas,(0,VIDEO_H),(W,VIDEO_H),(70,70,70),1)

        # ── Bottom panel ──────────────────────────────────────────────────
        graph_w = W - len(caps)*BOX_W if show_graphs else 0

        if show_graphs:
            all_h = [(histories[ci], CAM_COLORS[ci]) for ci in range(len(caps))]
            draw_graphs(canvas, all_h, 0, VIDEO_H+2, graph_w, PANEL_H-30)
            for i,(nm,col) in enumerate(zip(AXIS_NAMES, AXIS_COLS)):
                lx = 4 + i*120
                cv2.rectangle(canvas,(lx,H-24),(lx+12,H-12),col,-1)
                cv2.putText(canvas, nm,(lx+15,H-13),
                            cv2.FONT_HERSHEY_SIMPLEX,0.38,col,1,cv2.LINE_AA)
            cv2.putText(canvas,'AX AY AZ = g    GX GY GZ = °/s   range ±2',
                        (4,H-2), cv2.FONT_HERSHEY_SIMPLEX,0.35,(100,100,100),1,cv2.LINE_AA)
        else:
            cv2.putText(canvas, 'G = show IMU graphs',
                        (8, VIDEO_H + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (80,80,80), 1, cv2.LINE_AA)

        # ── 3D boxes ──────────────────────────────────────────────────────
        for ci, (pkts, roll, pitch, yaw, vals6, exp_start, exp_end) in enumerate(cam_data):
            bx0 = graph_w + ci*BOX_W
            cv2.rectangle(canvas,(bx0,VIDEO_H),(bx0+BOX_W,H),(18,18,30),-1)
            cv2.line(canvas,(bx0,VIDEO_H),(bx0,H),(60,60,70),1)

            cv2.putText(canvas, f'CAM {ci}  ICM-42688',
                        (bx0+8, VIDEO_H+20),
                        cv2.FONT_HERSHEY_SIMPLEX,0.48,CAM_COLORS[ci],1,cv2.LINE_AA)

            # Exposure timestamps
            if exp_start or exp_end:
                exp_dur_us = exp_end - exp_start if exp_end > exp_start else 0
                cv2.putText(canvas, f'ES {exp_start/1e6:.6f}s',
                            (bx0+6, VIDEO_H+36),
                            cv2.FONT_HERSHEY_SIMPLEX,0.32,(160,160,220),1,cv2.LINE_AA)
                cv2.putText(canvas, f'EE {exp_end/1e6:.6f}s  dur={exp_dur_us}us',
                            (bx0+6, VIDEO_H+50),
                            cv2.FONT_HERSHEY_SIMPLEX,0.32,(160,160,220),1,cv2.LINE_AA)

            R  = euler_to_R(roll, pitch, yaw)
            cx = bx0 + BOX_W//2
            cy = VIDEO_H + int(PANEL_H*0.52)
            draw_box(canvas, R, cx, cy, size=80, tint=CAM_COLORS[ci])

            # Live values
            ax,ay,az,gx,gy,gz = vals6
            lines = [
                f'Ax={ax:+.3f}g  Ay={ay:+.3f}g  Az={az:+.3f}g',
                f'Gx={gx:+6.1f}  Gy={gy:+6.1f}  Gz={gz:+6.1f} °/s',
                f'Roll={np.degrees(roll):+6.1f}° Pitch={np.degrees(pitch):+6.1f}°',
                f'Yaw ={np.degrees(yaw):+6.1f}°',
            ]
            for i,ln in enumerate(lines):
                cv2.putText(canvas, ln, (bx0+6, H-60+i*16),
                            cv2.FONT_HERSHEY_SIMPLEX,0.36,(180,210,180),1,cv2.LINE_AA)

        # ── HUD ───────────────────────────────────────────────────────────
        fps = np.mean(fps_hist)
        cv2.putText(canvas,f'{fps:.1f} fps',(W-80,16),
                    cv2.FONT_HERSHEY_SIMPLEX,0.45,(150,255,150),1,cv2.LINE_AA)
        cv2.putText(canvas,'Q=quit  R=reset',(W-130,H-2),
                    cv2.FONT_HERSHEY_SIMPLEX,0.38,(80,80,80),1,cv2.LINE_AA)

        cv2.imshow('DECXIN Dual IMU', canvas)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'),27): break
        elif key == ord('r'):
            for f in filters: f.reset()
            print("Orientations reset")
        elif key == ord('g'):
            show_graphs = not show_graphs

    for c in caps: c.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
