"""Subscribe to a simulation or driver visual PUB endpoint. Never sends commands."""
import argparse
from contextlib import ExitStack
import math
from pathlib import Path
import sys
import time

import zmq
from continuum_sdk.control.mujoco_visual import KinematicMirror


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--endpoint', default='tcp://127.0.0.1:5557')
    parser.add_argument('--xml', type=Path, default=Path(__file__).resolve().parents[2]/
                        'surgical_continuum_robot/model/continuum_robot/mjcf/scene_main.xml')
    parser.add_argument('--source', choices=['ideal','feedback','target'], default='ideal')
    parser.add_argument('--initial-pose', choices=['zero','source'], default='source',
                        help='Zero starts straight at model zero; source preserves the first driver pose.')
    parser.add_argument('--display-gain', type=float, default=1.0,
                        help='Visual-only multiplier for motion from the first received pose; 1 restores unscaled geometry.')
    parser.add_argument('--view', choices=['wide','tip','overview'], default='wide',
                        help='Initial camera framing; mouse controls remain available.')
    parser.add_argument('--fps', type=float, default=60,
                        help='Requested viewer refresh rate (1..60); does not change driver or feedback rates.')
    parser.add_argument('--stale-s', type=float, default=.5)
    parser.add_argument('--render-quality', choices=['light','full'], default='light',
                        help='Light disables shadows and reflections for smoother viewing.')
    parser.add_argument('--wait-mode', choices=['auto','system','windows-highres'], default='auto',
                        help='Auto uses a high-resolution wait on Windows, system sleep elsewhere.')
    args = parser.parse_args()
    if args.source == 'ideal' and (args.initial_pose != 'source' or args.display_gain != 1.):
        parser.error('Ideal simulation display requires initial-pose source and display-gain 1.')
    if not math.isfinite(args.display_gain) or args.display_gain <= 0:
        parser.error('display-gain must be finite and positive.')
    if not math.isfinite(args.fps) or not 1 <= args.fps <= 60 or not math.isfinite(args.stale_s) or args.stale_s <= 0:
        parser.error('fps must be 1..60; stale-s must be finite and positive.')
    import mujoco.viewer
    mirror = KinematicMirror(args.xml, initial_pose=args.initial_pose)
    print(f'Visual only | {args.source} | {args.endpoint} | requested refresh {args.fps:g} FPS | no physics or hardware access')
    print('Model reconstruction from the selected source, not measured robot shape. Stale input freezes the last pose.')
    print(f'Display gain: {args.display_gain:g}x (motion relative to first received pose; hardware commands unchanged)')
    print(f'Initial pose: {args.initial_pose} | first valid source pose establishes the motion reference')
    with ExitStack() as cleanup, zmq.Context() as context:
        sleep = time.sleep
        wait_mode = ('windows-highres' if sys.platform == 'win32' else 'system') if args.wait_mode == 'auto' else args.wait_mode
        if wait_mode == 'windows-highres':
            from pmac_sdk.core.waiting import WindowsHighResolutionWait
            timer = WindowsHighResolutionWait()
            cleanup.callback(timer.close)
            sleep = timer.sleep
        print(f'Render quality: {args.render_quality} | wait: {wait_mode}')
        with context.socket(zmq.SUB) as socket:
            socket.setsockopt(zmq.LINGER,0)
            socket.setsockopt(zmq.CONFLATE,1)
            socket.setsockopt(zmq.SUBSCRIBE,b'')
            socket.connect(args.endpoint)
            with mujoco.viewer.launch_passive(mirror.model, mirror.data, show_left_ui=False, show_right_ui=False) as viewer:
                with viewer.lock():
                    if args.render_quality == 'light':
                        viewer.user_scn.flags[mirror.mj.mjtRndFlag.mjRND_SHADOW] = 0
                        viewer.user_scn.flags[mirror.mj.mjtRndFlag.mjRND_REFLECTION] = 0
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
                        if args.source == 'ideal' and 'ideal_joint' not in message:
                            raise ValueError('Ideal display requires omega_reference_sim.py; '
                                             'use --source target for the legacy driver.')
                        age = float(message['feedback_age_s'])
                        if not math.isfinite(age) or age < 0:
                            raise ValueError('Invalid feedback age.')
                        if args.source != 'feedback' or age <= args.stale_s:
                            with viewer.lock():
                                mirror.update(message[args.source+'_joint'], display_gain=args.display_gain)
                        last_received = start; last_message = message
                    except zmq.Again:
                        pass
                    stale = (last_received is None or start-last_received > args.stale_s or
                             (args.source=='feedback' and float(last_message['feedback_age_s'])+start-last_received > args.stale_s))
                    text = ('WAITING / STALE: last pose frozen' if stale else
                            f'LIVE {args.source} ({last_message.get("ideal_source", "independent_reference_ik") if args.source == "ideal" else last_message["source"]}) | display {args.display_gain:g}x')
                    if text != report:
                        print(text,flush=True);report=text
                    viewer.sync()
                    remaining = 1/args.fps-(time.monotonic()-start)
                    if remaining > 0:
                        sleep(remaining)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
