#!/usr/bin/env python3
"""Render the shipped Unity scene and encode its 15-second preview (Unity + ffmpeg)."""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--unity', required=True, type=Path, help='Unity 6000.6.5f1 editor executable')
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
output = root / 'Documentation/HydroValidation/demo.mp4'
font = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
font_option = f'fontfile={font}' if Path(font).exists() else 'font=Sans'
base = f'drawtext={font_option}:fontsize=26:fontcolor=white:box=1:boxcolor=black@0.4:boxborderw=12:x=24:y=28:'
filters = [
    base + "text='IRYS | PVC rigide 350 x 70 x 1 mm'",
    base + "text='Repos':y=76:enable='lt(t,2)'",
    base + "text='Propulsion u = 0.8':y=76:enable='between(t,2,8)'",
    base + "text='Virage v = +0.7':y=76:enable='between(t,8,11)'",
    base + "text='Arret progressif':y=76:enable='gte(t,11)'",
    f"drawtext={font_option}:fontsize=18:fontcolor=white:x=24:y=h-40:text='Simulation visuelle - coefficients et masses non calibres'",
]
with tempfile.TemporaryDirectory(prefix='irys-hydro-') as directory:
    env = os.environ.copy()
    env['IRYS_HYDRO_FRAMES'] = directory
    subprocess.run([str(args.unity.resolve()), '-batchmode', '-force-glcore', '-projectPath', str(root / 'Unity_robot'),
                    '-executeMethod', 'HydroPreview.Render', '-quit', '-logFile', '/tmp/irys-hydro-preview.log'], env=env, check=True)
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', '25', '-i', str(Path(directory) / 'frame-%04d.png'),
                    '-vf', ','.join(filters), '-c:v', 'libx264', '-crf', '21', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output)], check=True)
print(output)
