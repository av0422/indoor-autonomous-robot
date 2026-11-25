"""Summarise the OFF / ON trial CSVs and plot collisions per route."""

import os

import matplotlib
import pandas as pd

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RUNS = [
    ('OFF', 'results/baseline_off.csv'),
    ('ON run 1', 'results/perception_on_run1.csv'),
    ('ON rerun', 'results/perception_on_v2.csv'),
]
PAIRS = range(5)
CHART = 'docs/images/collisions_by_pair.png'
PAIR_LABELS = ['backpack W-E', 'suitcase W-E', 'ball SW-NE', 'backpack NW-SE', 'control']


def load(path):
    """Read one trial CSV relative to the repository root."""
    return pd.read_csv(os.path.join(ROOT, path))


def summarise(label, frame):
    """Return one summary row (a dict) for a run."""
    won = frame[frame['success'] == 1]
    return {
        'run': label,
        'trials': len(frame),
        'successes': int(frame['success'].sum()),
        'collisions': int(frame['collision_count'].sum()),
        'trials_w_collision': int((frame['collision_count'] > 0).sum()),
        'time_mean': won['time_to_goal'].mean(),
        'time_std': won['time_to_goal'].std(),
        'path_mean': won['path_length_m'].mean(),
        'path_std': won['path_length_m'].std(),
        'recoveries': int(frame['recoveries'].sum()),
    }


def plot(table):
    """Save a grouped bar chart of collisions per pair, one bar per run."""
    ax = table.plot.bar(figsize=(8, 5), width=0.8)
    ax.set_xlabel('Route')
    ax.set_xticklabels(PAIR_LABELS)
    for container in ax.containers:
        ax.bar_label(container, fmt='%d', padding=2)
    ax.set_ylabel('Collisions (total over trials)')
    ax.set_title('Collisions per route: perception OFF vs ON')
    ax.legend(title='Run')
    ax.tick_params(axis='x', rotation=0)
    ax.figure.tight_layout()
    ax.figure.savefig(os.path.join(ROOT, CHART), dpi=150)


def main():
    """Print the summary and per-pair tables and write the chart."""
    frames = {label: load(path) for label, path in RUNS}
    summary = pd.DataFrame([summarise(label, frame) for label, frame in frames.items()])
    per_pair = pd.DataFrame({
        label: frame.groupby('pair')['collision_count'].sum().reindex(PAIRS, fill_value=0)
        for label, frame in frames.items()
    })
    per_pair.index.name = 'pair'
    with pd.option_context('display.width', 200, 'display.max_columns', None):
        print(summary.round(3).to_string(index=False))
        print('\nCollisions per route:')
        print(per_pair.to_string())
    plot(per_pair)
    print(f'\nSaved {CHART}')


if __name__ == '__main__':
    main()
