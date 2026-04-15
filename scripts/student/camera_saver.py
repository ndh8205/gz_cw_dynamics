#!/usr/bin/env python3
"""Camera saver — starter.

Subscribes to a deputy's camera (gz-transport topic, bridged or direct via
cv_bridge + ros_gz_image) and saves each frame to disk as PNG.

By default uses the gz-transport topic `/<deputy>/camera` published by the
satellite mesh model's built-in sensor.

Requires:
    apt install ros-jazzy-ros-gz-image   # bridges gz Image to ROS 2 sensor_msgs/Image
or use the gz.transport Python bindings directly.

Example:
    ros2 run gz_cw_dynamics camera_saver.py \
        --deputy deputy_formation --out /tmp/frames
"""

import argparse
import os
import sys
from threading import Event, Lock

try:
    from gz.msgs10.image_pb2 import Image as GzImage
    from gz.transport13 import Node as GzNode
    HAVE_GZ = True
except ImportError:
    HAVE_GZ = False


def save_pgm_or_png(path, width, height, step, data_bytes, pixel_format):
    # Minimal PNG write via PIL if available; else raw PGM dump.
    try:
        from PIL import Image
        # Gazebo pixel format 3 = RGB_INT8. Step = width*3 usually.
        mode = 'RGB' if pixel_format == 3 else 'L'
        if mode == 'RGB':
            img = Image.frombytes('RGB', (width, height), data_bytes)
        else:
            img = Image.frombytes('L', (width, height), data_bytes)
        img.save(path)
    except ImportError:
        # Fallback: PPM (P6) for RGB, PGM (P5) for mono.
        with open(path.replace('.png', '.ppm'), 'wb') as f:
            f.write(f'P6\n{width} {height}\n255\n'.encode())
            f.write(data_bytes)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--deputy', default='deputy_formation')
    ap.add_argument('--out',    default='/tmp/frames')
    ap.add_argument('--topic',  default=None,
        help='override topic; default is /<deputy>/camera')
    ap.add_argument('--max',    type=int, default=0,
        help='stop after N frames (0 = unlimited)')
    args = ap.parse_args()

    if not HAVE_GZ:
        print('ERROR: python3-gz-transport13/python3-gz-msgs10 missing',
              file=sys.stderr)
        return 2

    os.makedirs(args.out, exist_ok=True)
    # Deputies use included model cameras; topic name depends on model.
    # For nasa_satellite: /nasa_satellite/camera. For nasa_satellite2:
    # /nasa_satellite2/camera. User can override via --topic.
    default_topic = {
        'deputy_formation': '/nasa_satellite/camera',
        'deputy_docking':   '/nasa_satellite2/camera',
    }.get(args.deputy, f'/{args.deputy}/camera')
    topic = args.topic or default_topic

    gz = GzNode()
    counter = {'n': 0}
    lock = Lock()
    done = Event()

    def on_image(msg: GzImage):
        with lock:
            counter['n'] += 1
            idx = counter['n']
        filename = os.path.join(args.out, f'frame_{idx:06d}.png')
        save_pgm_or_png(filename, msg.width, msg.height,
                        msg.step, msg.data, msg.pixel_format_type)
        if idx % 10 == 0:
            print(f'[camera] saved {filename}')
        if args.max and idx >= args.max:
            done.set()

    if not gz.subscribe(GzImage, topic, on_image):
        print(f'ERROR: failed to subscribe {topic}', file=sys.stderr)
        return 3

    print(f'[camera] subscribing {topic} -> {args.out}')
    try:
        done.wait()
    except KeyboardInterrupt:
        pass
    print(f'[camera] total frames saved: {counter["n"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
