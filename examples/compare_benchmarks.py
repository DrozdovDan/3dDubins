"""Compare saved bidirectional/fast mission measurements without planning.

python -m examples.compare_benchmarks
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from statistics import median


def compare_reports(before, after):
    """Align workloads, allow declared solver/network differences, retain failures."""
    for key in ('topologies', 'scenarios', 'directions', 'agents', 'iterations',
                'retry_iterations', 'max_route_attempts', 'planning_window_s',
                'planner_seed', 'flight_bounds_m', 'spatial_replanning',
                'shared_goal_regions_m', 'formation_required'):
        if before['configuration'].get(key) != after['configuration'].get(key):
            raise ValueError(f'different workloads/settings: {key}')
    indexed = []
    for report in (before, after):
        rows = {r['id']: r for r in report['runs']}
        if not rows or len(rows) != len(report['runs']):
            raise ValueError('expected nonempty reports with unique mission IDs')
        indexed.append(rows)
    if indexed[0].keys() != indexed[1].keys():
        raise ValueError('different mission IDs; cannot compare selected subsets')
    pairs = []
    for key in sorted(indexed[0]):
        a, b = (rows[key] for rows in indexed)
        if any(a.get(k) != b.get(k) for k in ('agents', 'topology', 'scenario', 'direction', 'requests')):
            raise ValueError(f'different mission endpoints/roster: {key}')
        if a['agents'] <= 0 or a['wall_ms'] <= 0 or b['wall_ms'] <= 0:
            raise ValueError('positive fleet sizes and measured wall times required')
        pairs.append((a, b))
    summary = []
    for n in sorted({a['agents'] for a, _ in pairs}):
        selected = [(a, b) for a, b in pairs if a['agents'] == n]
        timing = [[pair[i]['wall_ms']/n for pair in selected] for i in (0, 1)]
        lengths = [[pair[i]['total_length_m']/n/1000 for pair in selected
                    if pair[i]['complete'] and pair[i]['safety_valid']
                    and pair[i]['total_length_m'] is not None] for i in (0, 1)]
        medians = [median(v) for v in timing]
        summary.append(dict(agents=n, missions=len(selected),
                            time_median_ms=medians, median_time_ratio=medians[0]/medians[1],
                            length_median_km=[median(v) if v else None for v in lengths],
                            safe_complete=[len(v) for v in lengths]))
    return pairs, summary


def settings_label(report):
    c = report['configuration']; network = c['network']; gap = c.get('relative_gap', 0)
    solver = f'ранняя остановка {100*gap:g}%' if gap else 'полный бюджет'
    return (f"{c['iterations']}/{c['retry_iterations']} итераций, {solver}; "
            f"доставка {1000*network['minimum_delay']:g}–{1000*network['maximum_delay']:g} мс, "
            f"потери {100*network['loss']:g}%, дубликаты {100*network['duplicates']:g}%")


def render(before, after, destination):
    pairs, summary = compare_reports(before, after)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch
    from matplotlib.ticker import FixedLocator, FuncFormatter, NullFormatter
    from examples.plot_benchmark_fleet import box_statistics

    destination.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(16, 9), facecolor='white')
    ink, muted = '#142637', '#516576'
    colors = ('#a9b5c2', '#3b94ad')
    fig.text(.055, .935, 'Архивный и текущий прогоны', fontsize=27, weight='bold', color=ink)
    fig.text(.055, .885, f'{len(pairs)} одинаковых заданий на версию · 1×1×1 км · подъём и спуск объединены',
             fontsize=14, color=muted)
    fig.legend(handles=[Patch(facecolor=colors[0], label='benchmark_swarm_1km_bidirectional'),
                        Patch(facecolor=colors[1], label='benchmark_swarm_1km_fast')],
               loc='upper left', bbox_to_anchor=(.055, .849), ncol=2, frameon=False, fontsize=12.5)
    axes = [fig.add_axes((.075, .30, .435, .41)), fig.add_axes((.62, .30, .33, .41))]
    counts = [row['agents'] for row in summary]
    for index, axis in enumerate(axes):
        for position, n in enumerate(counts, 1):
            selected = [pair for pair in pairs if pair[0]['agents'] == n]
            for version in (0, 1):
                group = [pair[version] for pair in selected]
                if index:
                    group = [r for r in group if r['complete'] and r['safety_valid']
                             and r['total_length_m'] is not None]
                values = [r['total_length_m']/n/1000 if index else r['wall_ms']/n for r in group]
                x = position + (-.18 if version == 0 else .18)
                if values:
                    axis.bxp([box_statistics(values)], positions=[x], widths=.29, manage_ticks=False,
                             showfliers=False, patch_artist=True,
                             boxprops=dict(facecolor=colors[version], edgecolor='#30475a', alpha=.8),
                             medianprops=dict(color=ink, linewidth=2),
                             whiskerprops=dict(color='#53687a'), capprops=dict(color='#53687a'))
                for i, (run, value) in enumerate(zip(group, values)):
                    good = run['complete'] and run['safety_valid']
                    axis.scatter(x-.085+.17*i/max(1, len(group)-1), value,
                                 color=colors[version] if good else '#b84642',
                                 marker='o' if good else 'x', s=13, alpha=.65, zorder=3)
        axis.set_xticks(range(1, len(counts)+1), [str(n) for n in counts])
        axis.set_xlim(.5, len(counts)+.5)
        axis.set_xlabel('Число агентов', fontsize=13, color=muted, labelpad=10)
        axis.tick_params(labelsize=12)
        axis.spines[['top', 'right']].set_visible(False)
        axis.grid(axis='y', color='#e5ebf0', linewidth=.8)
        axis.set_axisbelow(True)
    axes[0].set_title('Полное время на агента', loc='left', fontsize=20, weight='bold', color=ink, pad=19)
    axes[0].set_ylabel('T/N, мс/агент · логарифмическая шкала', fontsize=12, color=muted, labelpad=9)
    axes[0].set_yscale('log')
    axes[0].yaxis.set_major_locator(FixedLocator([20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000]))
    axes[0].yaxis.set_major_formatter(FuncFormatter(lambda value, _: f'{value:g}'))
    axes[0].yaxis.set_minor_formatter(NullFormatter())
    axes[1].set_title('Длина пути на агента', loc='left', fontsize=18, weight='bold', color=ink, pad=19)
    axes[1].set_ylabel('ΣLᵢ/N, км/агент', fontsize=12, color=muted, labelpad=8)
    axes[1].yaxis.set_major_formatter(FuncFormatter(lambda value, _: f'{value:g}'.replace('.', ',')))
    fig.text(.075, .207, 'Медианы: архив → текущий, мс/агент; отношение медиан в скобках', fontsize=10.5, color=muted)
    fig.text(.62, .207, 'Медианы: архив → текущий, км/агент', fontsize=10.5, color=muted)
    for line, rows in enumerate((summary[:2], summary[2:])):
        times = '  ·  '.join(f"{r['agents']}: {r['time_median_ms'][0]:.0f} → {r['time_median_ms'][1]:.0f} "
                             f"(×{r['median_time_ratio']:.1f})" for r in rows).replace('.', ',')
        lengths = '  ·  '.join(f"{r['agents']}: " + ' → '.join('—' if v is None else f'{v:.2f}'
                                                           for v in r['length_median_km']) for r in rows).replace('.', ',')
        fig.text(.075, .170-line*.034, times, fontsize=11.5, color=ink)
        fig.text(.62, .170-line*.034, lengths, fontsize=11.5, color=ink)
    successes = [sum(r['complete'] and r['safety_valid'] for pair in pairs for r in (pair[i],)) for i in (0, 1)]
    fig.text(.055, .096, f'Полные безопасные миссии: {successes[0]}/{len(pairs)} и {successes[1]}/{len(pairs)}. '
             'Ящики — средние 50%; линия — медиана; усы — 1,5 IQR; показаны все точки.', fontsize=10.5, color=muted)
    fig.text(.055, .066, 'Архив: ' + settings_label(before), fontsize=10.2, color=muted)
    fig.text(.055, .042, 'Текущий: ' + settings_label(after), fontsize=10.2, color=muted)
    fig.text(.055, .017, 'Разные даты и фоновая нагрузка: отношение медиан описывает прогоны целиком, '
             'а не чистый эффект отдельной C++-оптимизации.', fontsize=10, color=muted)
    for suffix in ('png', 'svg'):
        fig.savefig(destination/f'comparison_bidirectional.{suffix}', dpi=240)
    plt.close(fig)
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before', type=Path, default=Path('outputs/benchmark_swarm_1km_bidirectional/results.json'))
    parser.add_argument('--after', type=Path, default=Path('outputs/benchmark_swarm_1km_fast/results.json'))
    parser.add_argument('--output', type=Path, default=Path('outputs/benchmark_swarm_1km_fast/report'))
    args = parser.parse_args(argv)
    sources = [p.read_bytes() for p in (args.before, args.after)]
    before, after = [json.loads(data.decode('utf-8')) for data in sources]
    summary = render(before, after, args.output)
    metadata = dict(sources=[dict(path=str(p), sha256=hashlib.sha256(data).hexdigest())
                             for p, data in zip((args.before, args.after), sources)],
                    configurations=[r['configuration'] for r in (before, after)], summary=summary)
    (args.output/'comparison_bidirectional.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
    lines = ['# Сравнение архивного и текущего прогонов', '',
             '![Сравнительный график](comparison_bidirectional.png)', '',
             '[PNG 16:9](comparison_bidirectional.png) · [SVG](comparison_bidirectional.svg)', '',
             'Совпадение заданий проверено по ID, числу агентов и всем стартовым/конечным состояниям.', '',
             'Архив: '+settings_label(before)+'.', 'Текущий: '+settings_label(after)+'.', '',
             'Измерения сделаны в разное время. Отношение медиан характеризует различие сохранённых '
             'прогонов целиком; оно не является парным замером ускорения только C++.', '',
             '| Агентов | Миссий на версию | T/N архив, мс | T/N текущий, мс | Отношение медиан | Длина архив, км | Длина текущая, км |',
             '|---:|---:|---:|---:|---:|---:|---:|']
    for r in summary:
        lengths = ['—' if v is None else f'{v:.3f}' for v in r['length_median_km']]
        lines.append(f"| {r['agents']} | {r['missions']} | {r['time_median_ms'][0]:.2f} | {r['time_median_ms'][1]:.2f} "
                     f"| {r['median_time_ratio']:.2f}× | {lengths[0]} | {lengths[1]} |")
    lines.extend(['', '[Исходные файлы, конфигурации и проверяемые хеши](comparison_bidirectional.json)', ''])
    (args.output/'comparison_bidirectional.md').write_text('\n'.join(lines), encoding='utf-8')
    print(args.output/'comparison_bidirectional.png')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
