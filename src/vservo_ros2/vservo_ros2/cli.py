import argparse
from pathlib import Path
import sys
import cv2
import numpy as np
from .geometry import read_json, write_json
from .board import Board
from .calibration import intrinsic, handeye, reprojection, stats, evaluate_handeye, validate_intrinsic
from .accuracy import report


def main(args=None):
    p = argparse.ArgumentParser(description='Offline calibration and accuracy evaluation (never moves robot)')
    sub = p.add_subparsers(dest='cmd', required=True)
    b = sub.add_parser('board')
    b.add_argument('--out', required=True)
    b.add_argument('--square-m', type=float, default=0.015)
    b.add_argument('--marker-m', type=float, default=0.011)
    for name in ('solve', 'check-intrinsics', 'check-handeye'):
        s = sub.add_parser(name)
        s.add_argument('--session', required=True, help='samples.json from independent capture session')
        s.add_argument('--out', required=True)
        if name != 'solve':
            s.add_argument('--calibration', required=True)
        if name == 'check-intrinsics':
            s.add_argument('--max-px', type=float, default=1.0)
        if name == 'check-handeye':
            s.add_argument('--max-mm', type=float, default=10.0)
            s.add_argument('--max-deg', type=float, default=2.0)
    s = sub.add_parser('accuracy')
    s.add_argument('--input', required=True)
    s.add_argument('--out', required=True)
    s.add_argument('--max-mm', type=float, default=5.0)
    s.add_argument('--max-deg', type=float, default=2.0)
    a = p.parse_args(args)
    try:
        if a.cmd == 'board':
            board = Board(square_m=a.square_m, marker_m=a.marker_m)
            image = board.board.generateImage((1400, 1000), marginSize=0, borderBits=1)
            Path(a.out).expanduser().parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(Path(a.out).expanduser()), image):
                raise IOError('Cannot write board')
            print(f'Board saved. Print active chessboard area at {7*a.square_m*1000:.1f} x {5*a.square_m*1000:.1f} mm; verify with ruler.')
            return
        if a.cmd == 'accuracy':
            data = read_json(a.input)
            result = report(data['poses'], data.get('references'))
            result['passed'] = result['translation_mm']['p95'] <= a.max_mm and result['rotation_deg']['p95'] <= a.max_deg
        else:
            data = read_json(a.session)
            samples = data['samples']
            if a.cmd == 'solve':
                if data['mode'] == 'intrinsic':
                    result = intrinsic([(s['object_points'], s['image_points']) for s in samples], data['image_size'])
                else:
                    result = handeye([s['base_T_flange'] for s in samples],
                                     [s['camera_T_board'] for s in samples], data['role'])
                result.update(camera=data['camera'], board=data['board'], schema_version=1,
                              sample_count=len(samples), session_dir=str(Path(a.session).parent.resolve()))
            else:
                calibration = read_json(a.calibration)
                if (calibration['camera']['serial'] != data['camera']['serial']
                        or calibration['camera']['role'] != data['camera']['role']
                        or calibration['board'] != data['board']):
                    raise ValueError('Calibration/session serial, role or board mismatch')
                if len(samples) < 5:
                    raise ValueError('Need >=5 independent validation samples')
                if a.cmd == 'check-intrinsics':
                    K, D = validate_intrinsic(calibration, data['role'], data['camera']['serial'], data['image_size'])
                    values = [reprojection(np.array(s['object_points'], np.float32),
                                           np.array(s['image_points'], np.float32), K, D) for s in samples]
                    result = dict(reprojection_px=stats(values), passed=max(values) <= a.max_px)
                else:
                    if data['mode'] != 'handeye' or calibration['mode'] != data['role']:
                        raise ValueError('Need matching hand-eye validation session')
                    result = evaluate_handeye([np.array(s['base_T_flange']) for s in samples],
                                              [np.array(s['camera_T_board']) for s in samples],
                                              np.array(calibration['matrix']), data['role'],
                                              np.array(calibration['reference_matrix']))
                    result['passed'] = (result['translation_m']['maximum']*1000 <= a.max_mm
                                        and result['rotation_deg']['maximum'] <= a.max_deg)
                result['independent_session'] = str(Path(a.session).resolve()) != str(Path(calibration['session_dir']).resolve() / 'samples.json')
                if not result['independent_session']:
                    raise ValueError('Use a NEW capture session for independent evaluation')
        write_json(a.out, result)
        print(a.out)
        if result.get('passed') is False:
            raise SystemExit(2)
    except (ValueError, KeyError, OSError, cv2.error) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        raise SystemExit(1)
