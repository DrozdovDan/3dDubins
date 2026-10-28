"""Plot saved optimization measurements; never execute the planner.

python -m examples.plot_native_speed outputs/optimization_20261027
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from statistics import median

VERSIONS = ('baseline', 'optimized')
INK, MUTED = '#142637', '#516576'


def mission_pairs(report, field):
    """One paired observation per mission; retain both altitude directions."""
    if field not in ('wall_ms', 'mean_length_m'):
        raise ValueError('expected wall_ms or mean_length_m')
    return [tuple((run[version][field]/1000/(run['agents'] if field == 'wall_ms' else 1)
                   if run[version].get(field) is not None and
                   (field == 'wall_ms' or (run[version]['complete'] and run[version]['safety_valid']))
                   else None) for version in VERSIONS) for run in report['runs']]


def panel(axis, pairs, *, labels, color, failures=None):
    from examples.plot_benchmark_fleet import draw_boxes

    values = [[pair[i] for pair in pairs if pair[i] is not None] for i in range(2)]
    draw_boxes(axis, values, labels, color, show_fliers=False)
    # Old/new use consistent visual identities; all observations are drawn once.
    if axis.patches:
        axis.patches[0].set_facecolor('#c8d2dc')
    for i, pair in enumerate(pairs):
        jitter = -.10+.20*i/max(1, len(pairs)-1)
        failed = failures is not None and failures[i]
        dot = '#be3c43' if failed else '#31596f'
        if all(value is not None for value in pair):
            axis.plot([1+jitter, 2+jitter], pair, color=dot, alpha=.16, linewidth=.9, zorder=1)
        for position, value in enumerate(pair, 1):
            if value is not None:
                axis.scatter(position+jitter, value, color=dot, marker='x' if failed else 'o',
                             s=35, alpha=.85, zorder=3)
    axis.tick_params(labelsize=14)
    return values


def slide(title, subtitle):
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(16, 9), facecolor='white')
    fig.text(.055, .935, title, fontsize=27, weight='bold', color=INK)
    fig.text(.055, .882, subtitle, fontsize=14, color=MUTED)
    axes = [fig.add_axes((left, .285, .385, .485)) for left in (.075, .565)]
    return fig, axes


def number(value):
    return f'{value:.2f}'.replace('.', ',')


def finish_panel(fig, axis, left, title, ylabel, values, unit):
    from matplotlib.ticker import FuncFormatter

    axis.set_title(title, loc='left', fontsize=21, weight='bold', color=INK, pad=22)
    axis.set_ylabel(ylabel, fontsize=15, color=MUTED, labelpad=12)
    axis.yaxis.set_major_formatter(FuncFormatter(lambda value, _: f'{value:g}'.replace('.', ',')))
    medians = [number(median(v)) if v else '—' for v in values]
    fig.text(left, .17, f'Медианы: {medians[0]} → {medians[1]} {unit}', fontsize=15, color=INK)


def save(fig, destination, name):
    import matplotlib.pyplot as plt

    for suffix in ('png', 'svg'):
        fig.savefig(destination/f'{name}.{suffix}', dpi=240)
    plt.close(fig)


def plot_missions(report, destination):
    runs = report['runs']
    if not runs:
        raise ValueError('no measured missions')
    counts = sorted({r['agents'] for r in runs})
    agents = '/'.join(map(str, counts))
    maps = len({r['topology'] for r in runs})
    fig, axes = slide('Ускорение роевой маршрутизации',
                      f'{len(runs)} парные миссии · {agents} агентов · {maps} карты · '
                      'куб 1×1×1 км · подъём и спуск вместе')
    labels = ['До\nстарая DLL', 'После\nновая DLL']
    for axis, left, field, title, ylabel, color, unit in (
            (axes[0], .075, 'wall_ms', 'Время решения', 'T/N, с на агента', '#80b5db', 'с/агент'),
            (axes[1], .565, 'mean_length_m', 'Средняя длина пути', 'ΣLᵢ/N, км на агента', '#8bbfa6', 'км/агент')):
        values = panel(axis, mission_pairs(report, field), labels=labels, color=color)
        finish_panel(fig, axis, left, title, ylabel, values, unit)
        if field == 'wall_ms':
            axis.set_ylim(bottom=0)
    successes = [sum(r[v]['complete'] and r[v]['safety_valid'] for r in runs) for v in VERSIONS]
    fig.text(.055, .105, f'Полные безопасные миссии: {successes[0]}/{len(runs)} до и '
             f'{successes[1]}/{len(runs)} после. Линии соединяют одну и ту же задачу.', fontsize=12, color=MUTED)
    fig.text(.055, .069, 'Ящик — средние 50%; линия в ящике — медиана; усы — 1,5 IQR. '
             'Все измерения показаны точками.', fontsize=12, color=MUTED)
    fig.text(.055, .033, f'T включает весь цикл с перепланированием и проверкой. '
             f'Показаны {len(runs)} контрольные миссии, не полный набор из 144.', fontsize=12, color=MUTED)
    save(fig, destination, 'metrics')


def plot_local(ordinary, heavy, destination):
    reports = (ordinary, heavy)
    fig, axes = slide('Время отдельного поиска RRT*',
                      'Одинаковые входные данные для двух DLL · статический и time-aware поиск · '
                      'подъём и спуск вместе')
    for report, axis, left in zip(reports, axes, (.075, .565)):
        runs = report['runs']
        budgets = {r['budget'] for r in runs}
        if len(budgets) != 1 or not runs:
            raise ValueError('each local panel needs one explicit nonempty iteration budget')
        budget, = budgets
        divisor = 1 if budget == 600 else 1000
        unit = 'мс' if divisor == 1 else 'с'
        pairs = [(r['baseline_ms']/divisor, r['optimized_ms']/divisor) for r in runs]
        values = panel(axis, pairs, labels=['До\nстарая DLL', 'После\nновая DLL'], color='#80b5db',
                       failures=[not r['succeeded'] for r in runs])
        finish_panel(fig, axis, left, f'{budget} итераций · {len(runs)} вызова',
                     f'Время одного поиска, {unit}', values, unit)
        axis.set_ylim(bottom=0)
        if divisor == 1:
            axis.axhline(100, color='#be3c43', linestyle='--', alpha=.7, linewidth=1)
            axis.text(.98, 100, ' 100 мс', transform=axis.get_yaxis_transform(),
                      ha='right', va='bottom', fontsize=11, color='#be3c43')
    count = sum(len(r['runs']) for r in reports)
    matches = sum(r['exact_match'] for report in reports for r in report['runs'])
    fig.text(.055, .105, f'{matches}/{count} результатов полностью совпали. Красный × — одинаковый NoPath '
             'в обеих версиях; его время сохранено.', fontsize=12, color=MUTED)
    fig.text(.055, .069, '600 итераций: шесть топологий, оба направления. '
             '3000: агент B, узкий проход, спуск; два режима поиска.', fontsize=12, color=MUTED)
    fig.text(.055, .033, 'Линии соединяют одинаковые входы. Ящик — средние 50%; усы — 1,5 IQR. '
             'Это локальное время, не T/N миссии.', fontsize=12, color=MUTED)
    save(fig, destination, 'local_time')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    import matplotlib
    matplotlib.use('Agg')

    data = [json.loads((args.source/name).read_text(encoding='utf-8'))
            for name in ('missions.json', 'local_searches.json', 'local_heavy.json')]
    for version in VERSIONS:
        if len({r['libraries'][version]['sha256'] for r in data}) != 1:
            raise ValueError('cannot pool measurements of different library versions')
    destination = args.output or args.source/'figures'
    destination.mkdir(parents=True, exist_ok=True)
    plot_missions(data[0], destination)
    plot_local(data[1], data[2], destination)
    print(f'Saved: {destination}/metrics.png and local_time.png (also SVG)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
