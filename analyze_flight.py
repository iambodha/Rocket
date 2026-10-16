"""Validate a GY-86 CSV export and summarize measured acceleration and pressure.

Standard library only. No attitude estimate, gravity removal, or velocity integration.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import sys

COLUMNS = ('Time(ms)', 'AccelX(g)', 'AccelY(g)', 'AccelZ(g)', 'Pressure(Pa)')


def read_samples(path):
    with Path(path).open(newline='', encoding='utf-8-sig') as stream:
        reader = csv.DictReader(stream)
        missing = set(COLUMNS) - set(reader.fieldnames or [])
        if missing:
            raise ValueError('Missing CSV columns: ' + ', '.join(sorted(missing)))
        samples = []
        for line, row in enumerate(reader, start=2):
            try:
                sample = {key: float(row[key]) for key in COLUMNS}
            except (ValueError, TypeError) as exc:
                raise ValueError(f'Line {line}: expected numeric sensor values') from exc
            if not all(math.isfinite(value) for value in sample.values()):
                raise ValueError(f'Line {line}: values must be finite')
            if sample['Time(ms)'] < 0 or sample['Pressure(Pa)'] <= 0:
                raise ValueError(f'Line {line}: time must be nonnegative and pressure positive')
            if samples and sample['Time(ms)'] <= samples[-1]['Time(ms)']:
                raise ValueError(f'Line {line}: timestamps must strictly increase')
            samples.append(sample)
    if len(samples) < 2:
        raise ValueError('At least two samples are required')
    return samples


def summarize(samples):
    intervals = [(b['Time(ms)'] - a['Time(ms)']) / 1000 for a, b in zip(samples, samples[1:])]
    magnitudes = [math.hypot(*(sample[key] for key in COLUMNS[1:4])) for sample in samples]
    pressures = [sample['Pressure(Pa)'] for sample in samples]
    median_interval = statistics.median(intervals)
    return {
        'samples': len(samples),
        'duration_s': (samples[-1]['Time(ms)'] - samples[0]['Time(ms)']) / 1000,
        'median_sample_rate_hz': 1 / median_interval,
        'largest_interval_s': max(intervals),
        'peak_measured_acceleration_g': max(magnitudes),
        'minimum_pressure_pa': min(pressures),
        'maximum_pressure_pa': max(pressures),
        'notes': [
            'Acceleration magnitude includes gravity; it is not net vertical acceleration.',
            'Pressure is a sensor reading, not a calibrated altitude or launch pressure.',
            'Sampling rate is derived from timestamps, not assumed from firmware settings.',
        ],
    }


def write_plot(samples, destination):
    """Write a portable SVG of measured acceleration, without plotting dependencies."""
    times = [s['Time(ms)'] / 1000 for s in samples]
    values = [math.hypot(*(s[key] for key in COLUMNS[1:4])) for s in samples]
    top = max(max(values) * 1.1, 1)
    points = ' '.join(f'{60 + (t-times[0])/(times[-1]-times[0])*790:.2f},{260-v/top*200:.2f}' for t, v in zip(times, values))
    Path(destination).write_text(f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 900 320" role="img" aria-labelledby="title desc">
<title id="title">Measured acceleration magnitude</title><desc id="desc">Acceleration includes gravity. Time in seconds, acceleration in g.</desc>
<rect width="900" height="320" fill="#faf9f6"/><g fill="#242424" font-family="sans-serif">
<text x="60" y="30" font-size="20">Measured acceleration magnitude · includes gravity</text>
<text x="60" y="55">{top:.2f} g</text><text x="60" y="282">{times[0]:.2f} s</text><text x="780" y="282">{times[-1]:.2f} s</text>
<text x="60" y="306">Input CSV · {len(samples)} samples · no gravity or orientation correction</text></g>
<path d="M60 60V260H850" fill="none" stroke="#888"/><polyline points="{points}" fill="none" stroke="#216b68" stroke-width="2"/>
</svg>''', encoding='utf-8')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--json', type=Path, help='Write summary JSON')
    parser.add_argument('--svg', type=Path, help='Write acceleration plot')
    args = parser.parse_args(argv)
    try:
        samples = read_samples(args.csv)
        summary = json.dumps(summarize(samples), indent=2, allow_nan=False)
        if args.json:
            args.json.write_text(summary + '\n', encoding='utf-8')
        if args.svg:
            write_plot(samples, args.svg)
        print(summary)
    except (ValueError, OSError) as exc:
        print(f'Error: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
