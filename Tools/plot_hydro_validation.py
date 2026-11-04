#!/usr/bin/env python3
"""Render the command arc and the measured Unity acceptance traces (matplotlib required)."""
from pathlib import Path
import csv
import json
import re
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parents[1]
out = root / 'Documentation/HydroValidation'
report = json.loads((out / 'report.json').read_text())
if report['status'] != 'PASS':
    raise SystemExit('A passing Unity report is required.')
source = (root / 'Unity_robot/Assets/HydroSimulation/RobotHydroDefinition.cs').read_text()
def constant(name):
    return float(re.search(r'\b' + name + r'\s*=\s*([.\d]+)f', source)[1])
def data(name):
    with (out / (name + '.csv')).open() as stream:
        rows = list(csv.DictReader(stream))
    return {key: np.array([float(row[key]) for row in rows]) for key in rows[0]}

plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
ax = axes[0, 0]
u = np.linspace(0, 1, 201)
A = constant('MaxAmplitudeDeg') * np.sin(np.pi * u / 2)
f = constant('MaxFrequencyHz') * (1 - np.cos(np.pi * u / 2))
ax.plot(f, A, color='#126a79', linewidth=2)
for q in [0, .25, .5, .75, 1]:
    fq = constant('MaxFrequencyHz') * (1 - np.cos(np.pi * q / 2))
    aq = constant('MaxAmplitudeDeg') * np.sin(np.pi * q / 2)
    ax.scatter(fq, aq, color='#c48316', zorder=3)
    ax.annotate(f'u={q:g}', (fq, aq), xytext=(5, -14 if q == 1 else 5), textcoords='offset points')
ax.set(xlabel='Fréquence f (Hz)', ylabel='Demi-amplitude A (°)', title='Arc de commande à régime établi', xlim=(-.04, 1.37), ylim=(-1, 32))

straight = data('straight_stop')
ax = axes[0, 1]
ax.plot(straight['time'], straight['target_deg'], label='Cible limitée', color='#126a79')
ax.plot(straight['time'], straight['servo_deg'], label='Angle réel du servo', color='#c48316', alpha=.85)
ax.set(xlabel='Temps (s)', ylabel='Angle (°)', title='Commande, suivi et arrêt progressif')
ax.axvline(4, color='.6', linestyle=':', label='Mise en propulsion / arrêt')
ax.axvline(14, color='.6', linestyle=':')
ax.legend(fontsize=8)

ax = axes[1, 0]
for name, label, color in [('turn_positive', 'v = +0,7 (gauche)', '#126a79'), ('turn_negative', 'v = −0,7 (droite)', '#c48316')]:
    d = data(name)
    ax.plot(d['x'], d['z'], label=label, color=color)
    ax.scatter(d['x'][-1], d['z'][-1], color=color, s=20)
ax.set(xlabel='Déplacement latéral x (m)', ylabel='Avancée z (m)', title='Trajectoires obtenues, u = 0,8')
ax.set_aspect('equal', adjustable='datalim')
ax.legend(fontsize=8)

ax = axes[1, 1]
refined = data('straight_stop_half_step')
ax.plot(straight['time'], straight['speed'], label='Pas 2,5 ms', color='#126a79')
ax.plot(refined['time'], refined['speed'], label='Pas 1,25 ms', color='#c48316', alpha=.8)
ax.set(xlabel='Temps (s)', ylabel='Vitesse (m/s)', title='Sensibilité numérique du modèle')
ax.legend(fontsize=8)
for ax in axes.flat:
    ax.grid(alpha=.18)
fig.suptitle('IRYS — commande et simulation hydro visuelle\nRésultats numériques ; masses et coefficients non calibrés', fontsize=14)
fig.savefig(out / 'command-and-validation.png', dpi=160)
print(out / 'command-and-validation.png')
