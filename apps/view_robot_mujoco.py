"""Subscribe to the driver's dedicated visual PUB endpoint. Never sends commands."""
import argparse
import math
from pathlib import Path
import time

import zmq
from continuum_sdk.control.mujoco_visual import KinematicMirror


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--endpoint', default='tcp://127.0.0.1:5557')
    parser.add_argument('--xml', type=Path, default=Path(__file__).resolve().parents[2]/
                        'surgical_continuum_robot/model/continuum_robot/mjcf/scene_main.xml')
    parser.add_argument('--source', choices=['feedback','target'], default='target')
    parser.add_argument('--view', choices=['wide','tip','overview'], default='wide',
                        help='Initial camera framing; mouse controls remain available.')
    parser.add_argument('--fps', type=float, default=60,
                        help='Requested viewer refresh rate (1..60); does not change driver or feedback rates.')
    parser.add_argument('--stale-s', type=float, default=.5)
    args = parser.parse_args()
    if not math.isfinite(args.fps) or not 1 <= args.fps <= 60 or not math.isfinite(args.stale_s) or args.stale_s <= 0:
        parser.error('fps must be 1..60; stale-s must be finite and positive.')
    import mujoco.viewer
    mirror = KinematicMirror(args.xml)
    print(f'Visual only | {args.source} | {args.endpoint} | requested refresh {args.fps:g} FPS | no physics or hardware access')
    print('Model reconstruction from the selected source, not measured robot shape. Stale input freezes the last pose.')
    with zmq.Context() as context:
        with context.socket(zmq.SUB) as socket:
            socket.setsockopt(zmq.LINGER,0)
            socket.setsockopt(zmq.CONFLATE,1)
            socket.setsockopt(zmq.SUBSCRIBE,b'')
            socket.connect(args.endpoint)
            with mujoco.viewer.launch_passive(mirror.model, mirror.data, show_left_ui=False, show_right_ui=False) as viewer:
                with viewer.lock():
                    viewer.cam.type = mirror.mj.mjtCamera.mjCAMERA_FREE
                    if args.view == 'overview':
                        viewer.cam.lookat[:] = [0, .25, .08]
                        viewer.cam.distance = .65
                        viewer.cam.azimuth = 180
                        viewer.cam.elevation = -30
                    elif args.view == 'wide':
                        viewer.cam.lookat[:] = [0, .32, .13]
                        viewer.cam.distance = .70
                        viewer.cam.azimuth = 135
                        viewer.cam.elevation = -25
                    else:
                        # Fixed close-up of the stock model's bending sections.
                        # Do not follow the tip: a fixed view helps compare directions.
                        viewer.cam.lookat[:] = [0, .49, .207]
                        viewer.cam.distance = .24
                        viewer.cam.azimuth = 135
                        viewer.cam.elevation = -25
                last_received = None; report = ''; last_message = None
                while viewer.is_running():
                    start = time.monotonic()
                    try:
                        message = socket.recv_json(flags=zmq.NOBLOCK)
                        if message.get('kind') != 'continuum_visual' or message.get('version') != 1:
                            raise ValueError('Unsupported visual message; use the dedicated PUB port.')
                        age = float(message['feedback_age_s'])
                        if not math.isfinite(age) or age < 0:
                            raise ValueError('Invalid feedback age.')
                        if args.source != 'feedback' or age <= args.stale_s:
                            with viewer.lock():
                                mirror.update(message[args.source+'_joint'])
                        last_received = start; last_message = message
                    except zmq.Again:
                        pass
                    stale = (last_received is None or start-last_received > args.stale_s or
                             (args.source=='feedback' and float(last_message['feedback_age_s'])+start-last_received > args.stale_s))
                    text = ('WAITING / STALE: last pose frozen' if stale else
                            f'LIVE {args.source} ({last_message["source"]})')
                    if text != report:
                        print(text,flush=True);report=text
                    viewer.sync()
                    time.sleep(max(0,1/args.fps-(time.monotonic()-start)))


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
