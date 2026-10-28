"""Plot saved benchmark JSONs without running the planner again.

python -m examples.plot_benchmark_fleet outputs/benchmark_fleet/normal.json --output outputs/benchmark_fleet/normal_figures
Matplotlib is optional; maps additionally require the built C++ backend.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
from math import ceil
import os
from pathlib import Path
from statistics import median

FAMILY_NAMES = {"reference": "Эталон", "slope": "Склон", "terrain": "Горный рельеф",
                "gates": "Ворота", "combined": "Рельеф + ворота"}
PROFILE_NAMES = {"ideal": "Идеальная сеть", "normal": "Обычная сеть", "slow": "Медленная сеть"}
PROFILE_COLORS = {"ideal": "#3382c4", "normal": "#42a578", "slow": "#df9a35"}
AGENT_COLORS = {"A": "#318ce7", "B": "#e5a02b", "C": "#a756c7", "D": "#258c62", "E": "#d93b42"}
SCENARIO_NAMES = {**FAMILY_NAMES, "reference": "Исходный горный\nсценарий",
                  "slope": "Склон\nбез препятствий", "gates": "Ворота\nна склоне",
                  "terrain": "Горный\nрельеф", "combined": "Рельеф\nи ворота"}
BOX_GUIDE = ("Линия в ящике — медиана; ящик — средние 50% значений (от 25-го до 75-го процентиля).\n"
             "Усы — крайние наблюдения в пределах 1,5 высоты ящика от его границ; кружки за усами — остальные наблюдения.")


def call_kind(call):
    """An experimental run is NOT a replan; static retries are NOT traffic replans."""
    if call.get("attempt", 1) == 1:
        return "initial"
    return "traffic" if call.get("mode") == "traffic" else "static_retry"


def box_statistics(values):
    """Tukey boxes with interpolated quartiles; whiskers are not min/max or p95."""
    from examples.benchmark_fleet import quantile

    if not values:
        raise ValueError("a box needs at least one observation")
    ordered = sorted(values)
    q1, q3 = quantile(ordered, .25), quantile(ordered, .75)
    span = q3-q1
    inside = [x for x in ordered if q1-1.5*span <= x <= q3+1.5*span]
    return dict(med=median(ordered), q1=q1, q3=q3,
                whislo=min(q1, min(inside, default=q1)), whishi=max(q3, max(inside, default=q3)),
                fliers=[x for x in ordered if x < q1-1.5*span or x > q3+1.5*span])


def grouped_runs(report):
    """Keep exact cases and networks separate; never merge input report files."""
    groups, cases = defaultdict(list), {}
    for run in report["runs"]:
        groups[run["case_id"], run["network"]].append(run)
        cases.setdefault(run["case_id"], run)
    return groups, cases


def case_label(run):
    return f"{FAMILY_NAMES.get(run['family'], run['family'])}\nкарта {run['map_seed']}, агентов {run['agents']}"


def sample_note(report):
    runs = report["runs"]
    seed_values = sorted({r["seed"] for r in runs})
    seeds = len(seed_values)
    seed_text = ", ".join(map(str, seed_values)) if seeds <= 5 else f"{seed_values[0]}…{seed_values[-1]} ({seeds} значений)"
    maps = len({(r["family"], r["map_seed"]) for r in runs})
    return (f"Миссий: {len(runs)} · геометрий: {maps} · RRT-seed: {seed_text}. "
            + ("ПРОБНЫЕ ЗАПУСКИ: недостаточно для статистических выводов." if seeds < 30 else
               "Описательные результаты выбранной серии, не гарантия времени/успеха."))


def performance_rows(report):
    """Ordinary operation: one fixed communication profile, algorithm metrics."""
    if {r["network"] for r in report["runs"]} != {"normal"}:
        raise ValueError("use a normal-only report for ordinary operation; --compare-links is only for optional stress tests")
    groups, cases = grouped_runs(report)
    rows = []
    for (case, _), runs in groups.items():
        good = [r for r in runs if r["agreement_complete"] and r["safety_valid"]]
        calls = [c for r in runs for c in r["calls"]]
        call_groups = {kind: [c for c in calls if call_kind(c) == kind]
                       for kind in ("initial", "traffic", "static_retry")}
        rows.append({"label": case_label(cases[case]), "case": cases[case],
                     "call_groups": call_groups, "runs": runs, "good": good,
                     "initial": [c["elapsed_ms"] for c in calls if c.get("attempt", 1) == 1],
                     "repeat": [c["elapsed_ms"] for c in calls if c.get("attempt", 1) > 1],
                     "wall": [r["wall_ms"]/1000 for r in runs],
                     "lengths": [r["total_length_m"]/1000 for r in good if r.get("total_length_m") is not None],
                     "arrival": [r["makespan_s"] for r in good if r.get("makespan_s") is not None],
                     "repeats": [sum(c.get("attempt", 1) > 1 for c in r["calls"]) for r in runs]})
    return rows


def draw_boxes(axis, datasets, labels, color, *, show_fliers=True):
    """One empirical distribution per scenario, with no fabricated empty boxes."""
    for position, values in enumerate(datasets, 1):
        if values:
            axis.bxp([box_statistics(values)], positions=[position], widths=.46,
                     patch_artist=True, manage_ticks=False, showfliers=show_fliers,
                     boxprops=dict(facecolor=color, edgecolor="#30475a", alpha=.85),
                     medianprops=dict(color="#142637", linewidth=2.2),
                     whiskerprops=dict(color="#30475a"), capprops=dict(color="#30475a"),
                     flierprops=dict(marker="o", markersize=4, markerfacecolor="white", markeredgecolor="#607889"))
        else:
            axis.text(position, .5, "Нет наблюдений", transform=axis.get_xaxis_transform(),
                      ha="center", fontsize=10, color="#617180")
    axis.set_xticks(range(1, len(labels)+1), labels, fontsize=11)
    axis.set_xlim(.4, len(labels)+.6)
    axis.grid(axis="y", color="#e3e8ed", linewidth=.8)
    axis.set_axisbelow(True)
    axis.spines[["top", "right"]].set_visible(False)
    axis.tick_params(axis="x", length=0, pad=12)
    axis.margins(y=.18)


def plot_performance(report, output):
    """Three readable box-plot figures; raw measurements are never rerun here."""
    import matplotlib.pyplot as plt

    rows = performance_rows(report)
    paths = []
    agent_counts = sorted({r["agents"] for r in report["runs"]})
    context = "Самолётов в эксперименте: " + ", ".join(map(str, agent_counts))
    for page, offset in enumerate(range(0, len(rows), 5), 1):
        selected = rows[offset:offset+5]
        suffix = f"_{page}" if page > 1 else ""
        names = [SCENARIO_NAMES.get(r["case"]["family"], r["case"]["family"]) for r in selected]
        # Include map/count only if the panel would otherwise have duplicate labels.
        if len(set(names)) < len(names) or len(agent_counts) > 1:
            names = [f"{n}\nкарта {r['case']['map_seed']}, агентов {r['case']['agents']}"
                     for n, r in zip(names, selected)]

        def save(figure, kind):
            path = Path(output).with_name(Path(output).stem+"_"+kind+suffix+".png")
            figure.savefig(path, dpi=180)
            plt.close(figure)
            paths.append(path)

        figure, axis = plt.subplots(figsize=(13, 7.5))
        labels = [f"{n}\nполных: {len(r['good'])}/{len(r['runs'])}"
                  for n, r in zip(names, selected)]
        draw_boxes(axis, [r["wall"] for r in selected], labels, "#80b5db")
        for position, row in enumerate(selected, 1):
            failed = [r["wall_ms"]/1000 for r in row["runs"]
                      if not (r["agreement_complete"] and r["safety_valid"])]
            axis.scatter([position+.1]*len(failed), failed, marker="x", s=48,
                         color="#bf3d42", linewidths=1.6, zorder=4)
        axis.set_ylim(bottom=0)
        axis.set_ylabel("Время работы всей системы, секунды", fontsize=12)
        figure.suptitle("Сколько времени нужно, чтобы спланировать всю группу?", x=.08, y=.96,
                        ha="left", fontsize=18, weight="bold")
        figure.text(.08, .885, context+". Одно наблюдение — один эксперимент с нуля.\n"
                    "Поиск путей + согласование + перестроения + итоговая проверка безопасности.", fontsize=11)
        figure.text(.08, .105, BOX_GUIDE, fontsize=10, linespacing=1.5)
        figure.text(.08, .035, "Красный × — эксперимент завершился без решения для всей группы; его время также входит в ящик.\n"
                    "Подготовка карты и визуализация не входят в замер. Это реальное время вычислений, не полёта.", fontsize=10)
        figure.tight_layout(rect=(.015, .20, .99, .87))
        save(figure, "timing")

        figure, axis = plt.subplots(figsize=(13, 7.5))
        labels = [f"{n}\nучтено: {len(r['good'])}/{len(r['runs'])}"
                  for n, r in zip(names, selected)]
        draw_boxes(axis, [r["lengths"] for r in selected], labels, "#8bbfa6")
        axis.set_ylabel("Сумма длин всех маршрутов, километры", fontsize=12)
        figure.suptitle("Какой длины маршруты получает группа?", x=.08, y=.96,
                        ha="left", fontsize=18, weight="bold")
        figure.text(.08, .885, context+". Одно наблюдение — сумма 3D-длин путей в одной полной миссии.\n"
                    "Меньше — короче общий путь; известного оптимума для сравнения здесь нет.", fontsize=11)
        figure.text(.08, .105, BOX_GUIDE, fontsize=10, linespacing=1.5)
        figure.text(.08, .035, "Только эксперименты с безопасным согласованным решением для всех самолётов. Неудачи учтены в подписях.\n"
                    "Длина считается от выхода со стартового круга до входа в конечный; полёт по самим кругам не учитывается.", fontsize=10)
        figure.tight_layout(rect=(.015, .20, .99, .87))
        save(figure, "quality")

        figure, axes = plt.subplots(2, 1, figsize=(13, 11.5), sharey=True)
        for axis, kind, title, color in (
                (axes[0], "initial", "Первое построение пути: ещё без маршрутов соседей", "#80b5db"),
                (axes[1], "traffic", "Перестроение пути: с учётом полученных маршрутов соседей", "#e6b574")):
            groups = [r["call_groups"][kind] for r in selected]
            labels = [f"{n}\nвызовов: {len(g)}\nпуть найден: {sum(c.get('succeeded', False) for c in g)}"
                      for n, g in zip(names, groups)]
            draw_boxes(axis, [[c["elapsed_ms"] for c in g] for g in groups], labels, color)
            for position, calls in enumerate(groups, 1):
                failed = [c["elapsed_ms"] for c in calls if not c.get("succeeded", False)]
                axis.scatter([position+.1]*len(failed), failed, marker="x", s=35,
                             color="#bf3d42", linewidths=1.3, zorder=4)
            axis.set_ylim(bottom=0)
            axis.axhline(100, color="#bf3d42", linestyle="--", linewidth=1)
            axis.set_title(title, loc="left", fontsize=13, pad=15)
            axis.set_ylabel("Время одного вызова, мс", fontsize=11)
        # set_ylim(bottom=0) freezes autoscaling on shared axes: use BOTH datasets
        # explicitly so a slow replan cannot be clipped by the initial-search range.
        largest = max((c["elapsed_ms"] for r in selected for kind in ("initial", "traffic")
                       for c in r["call_groups"][kind]), default=100)
        axes[0].set_ylim(0, max(100, largest)*1.15)
        figure.suptitle("Сколько длится поиск одного пути?", x=.08, y=.975,
                        ha="left", fontsize=18, weight="bold")
        figure.text(.08, .93, "Одно наблюдение — один вызов планировщика одного самолёта, а не эксперимент всей группы.\n"
                    "Верхние поиски выполнялись параллельно; нижние — последовательно. Это разные задачи.", fontsize=11)
        figure.text(.08, .095, BOX_GUIDE, fontsize=10, linespacing=1.5)
        retries = sum(len(r["call_groups"]["static_retry"]) for r in selected)
        figure.text(.08, .025, "Красный × — путь не найден за бюджет поиска; такие вызовы включены. Пунктир — цель 100 мс на вызов.\n"
                    f"Новых попыток после неудачного статического поиска: {retries}; они разобраны отдельно в отчёте.\n"
                    "Число вызовов не равно числу экспериментов: в каждом эксперименте несколько самолётов и возможны перестроения.", fontsize=10)
        figure.tight_layout(rect=(.015, .17, .99, .90), h_pad=3.0)
        save(figure, "search")

    text_path = write_performance_summary(report, rows, output, paths)
    return [*paths, text_path]


def write_performance_summary(report, rows, output, paths):
    """Explain the experiment before showing numbers; distinguish three searches."""
    def number(values):
        return f"{median(values):.2f}" if values else "—"

    def label(row):
        return row["label"].replace("Эталон", "Исходный горный сценарий").replace("\n", " · ")

    configuration = report.get("configuration", {})
    budgets = sorted({r["iterations"] for r in report["runs"] if "iterations" in r})
    n_complete = sum(len(r["good"]) for r in rows)
    summary = ["# Что именно мы измерили", "",
               "## Сначала — что такое один эксперимент", "",
               "1. Берём готовую карту и задания самолётам: у каждого свой стартовый и конечный круг.",
               "2. С нуля строим первоначальные пути; самолёты делают это параллельно.",
               "3. Агенты обмениваются путями. При конфликте агент строит другой путь с учётом полученных путей соседей.",
               "4. Проверяем безопасность результата. Успех — все самолёты получили согласованные безопасные маршруты.", "",
               "**30 экспериментов — не 30 перепланирований одного полёта.** На каждом сценарии всё планирование группы запускается "
               "с нуля 30 раз (если выбрано `--runs 30`). Карта, задания и ограничения остаются теми же; меняются случайные выборки RRT*. "
               "Это нужно, чтобы видеть разброс случайного алгоритма, а не одно удачное число. Деревья между экспериментами не сохраняются.", "",
               sample_note(report), "",
               f"Полное безопасное решение: **{n_complete}/{len(report['runs'])} экспериментов**. "
               f"Нарушений итоговой проверки безопасности: **{sum(not r['safety_valid'] for r in report['runs'])}**.", "",
               "## Три разных величины — их нельзя смешивать", "",
               "| Величина | Что означает одно наблюдение | Что входит |",
               "|---|---|---|",
               "| Время всей системы, с | Один эксперимент для всей группы | Все поиски, обмен сообщениями в симуляторе, проверки конфликтов, перестроения и финальная проверка |",
               "| Время поиска одного пути, мс | Один вызов планировщика одного самолёта | C++-поиск и Python-обвязка вызова, включая проверки внутри поиска; без протокола согласования |",
               "| Суммарная длина, км | Одно полное безопасное решение для всей группы | Сумма пространственных, а не горизонтальных, длин всех маршрутов |", "",
               "Время — прошедшее время по часам на ноутбуке, не сумма процессорного времени потоков. "
               "Пять параллельных поисков нельзя складывать и объявлять полученную сумму временем всей группы. "
               "Карта подготовлена до замера; рисование графиков, запись JSON и дополнительные диагностические проверки после симуляции не входят. "
               "Внутренняя финальная проверка симулятора входит.", "",
               "В длину входят подъём/спуск и повороты от выхода со стартового круга до входа в конечный; "
               "круги до вылета и после прибытия не входят. Время полёта не является временем вычислений.", "",
               "## Главный результат", "",
               "В таблице — медианы (половина наблюдений не больше этого числа, половина не меньше). "
               "Время учитывает все эксперименты, в том числе завершившиеся без полного решения; "
               "длины — только полные безопасные решения. Поэтому успешность обязательно стоит рядом.", "",
               "| Сценарий | Полные решения | Время всей системы, с (медиана) | Сумма длин, км (медиана) |",
               "|---|---:|---:|---:|"]
    for row in rows:
        summary.append(f"| {label(row)} | {len(row['good'])}/{len(row['runs'])} | {number(row['wall'])} | {number(row['lengths'])} |")
    summary.extend(["", "## Как читать ящики с усами", "",
                    "* Линия внутри — медиана.",
                    "* Нижняя и верхняя границы ящика — 25-й и 75-й процентили: внутри средние 50% значений.",
                    "* Разность этих границ — межквартильный размах (IQR). Усы доходят до крайних наблюдений в пределах 1,5 IQR от границ ящика.",
                    "* Кружки за усами — наблюдения вне этих границ. Они не удалены и не обязательно ошибочны.",
                    "* Красный крест — неполная/небезопасная миссия на общем времени или ненайденный путь на времени отдельного вызова.",
                    "* Усы **не** означают минимум–максимум, p95 или доверительный интервал. Узкий ящик означает небольшой разброс, не гарантию времени.", ""])
    for path in paths:
        summary.extend([f"![{path.stem}]({path.name})", ""])
    summary.extend(["## Какие именно поиски бывают внутри эксперимента", "",
                    "1. **Первое построение** — первая попытка одного самолёта, без маршрутов соседей. На 30 экспериментов с пятью самолётами получается 150 вызовов.",
                    "2. **Перестроение с соседями** — поиск другого пространственного пути с временными ограничениями от полученных маршрутов. "
                    "Возможны несколько попыток; старое дерево не используется.",
                    "3. **Новая попытка после неудачи** — первый поиск не нашёл путь за выделенный бюджет, поэтому выполняется новый статический поиск с другим seed. "
                    "Это не исправление столкновения с соседом.", "",
                    "Эти три типа раньше были неудачно смешаны под словом «повторные». Теперь они раздельны. "
                    "Первые поиски выполняются параллельно, последующие — последовательно в обработчике событий. "
                    "Начальный/статический поиск проверяет восемь корней на стартовом круге, перестроение — один фиксированный корень. "
                    "Поэтому различие времени между ними не является ускорением на одинаковой задаче.", "",
                    "| Сценарий | Тип поиска | Всего вызовов | Путь найден | Не найден | Время вызова, мс (медиана всех вызовов) |",
                    "|---|---|---:|---:|---:|---:|"])
    for row in rows:
        for kind, name in (("initial", "Первое построение"), ("traffic", "Перестроение с соседями"),
                           ("static_retry", "Новая попытка после неудачи")):
            calls = row["call_groups"][kind]
            found = sum(c.get("succeeded", False) for c in calls)
            summary.append(f"| {label(row)} | {name} | {len(calls)} | {found} | {len(calls)-found} "
                           f"| {number([c['elapsed_ms'] for c in calls])} |")
    summary.extend(["", "Время неудачных вызовов не скрыто. «Путь найден» относится к одному локальному поиску, "
                    "а не к успешному завершению всей миссии. 150 вызовов — не 150 независимых экспериментов: "
                    "это вызовы нескольких агентов внутри 30 экспериментов. Ящики показывают распределение наблюдений, не статистическую достоверность.", "",
                    "## Длины по самолётам и время полёта", "",
                    "Только полные безопасные миссии; все значения — медианы. Последнее прибытие — момент, "
                    "когда последний самолёт достигает конечного круга, от начала моделируемой миссии (t=0). Это не ожидание программы.", "",
                    "| Сценарий | A, км | B, км | C, км | D, км | E, км | Последнее прибытие, с |",
                    "|---|---:|---:|---:|---:|---:|---:|"])
    for row in rows:
        lengths = [number([a["length_m"]/1000 for r in row["good"] for a in r.get("per_agent", [])
                           if a["name"] == name and a.get("length_m") is not None]) for name in AGENT_COLORS]
        summary.append(f"| {label(row)} | " + " | ".join(lengths) + f" | {number(row['arrival'])} |")
    summary.extend(["", "## Что означает неудача", "",
                    "Неполная миссия — хотя бы один самолёт не получил согласованный маршрут. Уже подтверждённые пути сохраняются, "
                    "остальные агенты продолжают исходные круги. Отсутствие нарушения безопасности не означает, что все выполнили задания. "
                    "Ненайденный путь означает неудачу в выделенном бюджете, а не доказательство невозможности маршрута.", ""])
    failed = [r for r in report["runs"] if not (r["agreement_complete"] and r["safety_valid"])]
    if failed:
        summary.extend(["| Сценарий | Seed эксперимента | Получили подтверждённый путь | Причина |", "|---|---:|---|---|"])
        for run in failed:
            failure = run.get("failure") or "Итоговая проверка/согласование не завершены успешно"
            if "local attempt budget" in failure:
                failure = "Исчерпаны попытки найти безопасный путь при фиксированном времени вылета"
            summary.append(f"| {FAMILY_NAMES.get(run['family'], run['family'])} | {run['seed']} "
                           f"| {', '.join(run.get('committed_agents', [])) or 'никто'} | {failure.replace('|', '/')} |")
    else:
        summary.append("В этой серии неполных или небезопасных миссий нет.")
    summary.extend(["", "## Условия и границы вывода", "",
                    f"Бэкенд: {report.get('environment', {}).get('backend', configuration.get('backend', 'не указан'))}. "
                    f"Итераций на одну попытку поиска: {', '.join(map(str, budgets)) or 'не указано'}. "
                    "Для сохранённой серии normal.json: пять карт по 6,4 × 2,4 км, клетки 10 × 10 м, перепад высот 1 км. "
                    "Это пять конкретных карт, не проверка всех возможных сред.", "",
                    "Все эксперименты используют один фиксированный профиль связи: симулируемая задержка 0–150 мс, "
                    "10% потерь и 10% дубликатов, network seed 17. Здесь не сравниваются разные сети и нет настоящего сетевого транспорта. "
                    "Симулируемая задержка пакета не равна реальной паузе ожидания на ноутбуке.", "",
                    "Не измерен внешний алгоритм для сравнения и не известен оптимум длины. Времена зависят от ноутбука и его нагрузки. "
                    "Замер одного вызова в параллельной системе — не изолированный тест чистого C++-ядра. "
                    "Точка готовности пути зависит от wall time, поэтому одинаковые seed не гарантируют побитовую идентичность событий на другой машине.", "",
                    "Источник: сохранённый JSON (ссылка в index.md), сырые времена вызовов, параметры карт и хэши кода внутри него. "
                    "При создании этого отчёта планировщик заново не запускался.", ""])
    text_path = Path(output).with_name(Path(output).stem+"_summary.md")
    text_path.write_text("\n".join(summary), encoding="utf-8")
    return text_path


def plot_results(report, output):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch
    import numpy as np

    groups, cases = grouped_runs(report)
    keys = list(cases)
    profiles = [p for p in PROFILE_NAMES if any(network == p for _, network in groups)]
    saved = []
    for page, offset in enumerate(range(0, len(keys), 8), 1):
        selected = keys[offset:offset+8]
        figure, (time_axis, outcome_axis) = plt.subplots(1, 2, figsize=(16, 7),
                                                        gridspec_kw={"width_ratios": [1.5, 1]})
        xs, width = np.arange(len(selected)), .8/max(1, len(profiles))
        values = np.full((len(selected), len(profiles)), np.nan)
        for j, profile in enumerate(profiles):
            for i, case in enumerate(selected):
                rows = groups.get((case, profile), [])
                if not rows:
                    continue
                seconds = [r["wall_ms"]/1000 for r in rows]
                partial = any(not r["agreement_complete"] for r in rows)
                x = xs[i]+(j-(len(profiles)-1)/2)*width
                time_axis.bar(x, median(seconds), width*.92, color=PROFILE_COLORS[profile],
                              edgecolor="#666666" if partial else "white", hatch="///" if partial else None)
                time_axis.text(x, median(seconds), f"{median(seconds):.2f}", ha="center", va="bottom", fontsize=8)
                if len(rows) > 1:
                    time_axis.plot([x, x], [min(seconds), max(seconds)], color="#303030", linewidth=1)
                committed = sum(len(r["committed_agents"]) for r in rows)
                total = sum(r["agents"] for r in rows)
                complete = sum(r["agreement_complete"] and r["safety_valid"] for r in rows)
                values[i, j] = committed/total
                unsafe = any(not r["safety_valid"] for r in rows)
                outcome_axis.text(j, i, ("НЕБЕЗОПАСНО\n" if unsafe else "")
                                  + f"COMMIT {committed}/{total}\nвсе: {complete}/{len(rows)}",
                                  ha="center", va="center", fontsize=10,
                                  color="#a00000" if unsafe else "white" if values[i, j] > .6 else "#163823", zorder=3)
        time_axis.set_xticks(xs, [case_label(cases[c]) for c in selected], fontsize=9)
        time_axis.set_ylabel("Реальное время симулятора, с")
        time_axis.set_title("Время полного прогона — не только поиск пути")
        time_axis.set_ylim(bottom=0)
        time_axis.margins(y=.17)
        time_axis.grid(axis="y", alpha=.2)
        time_axis.set_axisbelow(True)
        time_axis.legend(handles=[Patch(facecolor=PROFILE_COLORS[p], label=PROFILE_NAMES[p]) for p in profiles]
                         + [Patch(facecolor="white", edgecolor="#666666", hatch="///", label="Есть неполные миссии")],
                         fontsize=9, loc="upper left")
        outcome_axis.imshow(np.ma.masked_invalid(values), cmap="YlGn", vmin=0, vmax=1, aspect="auto")
        outcome_axis.set_xticks(range(len(profiles)), [PROFILE_NAMES[p].replace(" ", "\n", 1) for p in profiles])
        outcome_axis.set_yticks(range(len(selected)), [case_label(cases[c]) for c in selected], fontsize=9)
        outcome_axis.set_title("Разрешённые полёты и полные безопасные миссии")
        figure.suptitle(f"AirLogistics-3D · результаты {Path(output).stem}", fontsize=17)
        figure.text(.02, .04, sample_note(report), fontsize=10)
        figure.text(.02, .012, "Числа над столбцами — медианы; отрезки — min–max при повторах. "
                    "Штриховка не означает нарушение безопасности: миссия могла согласоваться частично.", fontsize=9)
        figure.tight_layout(rect=(0, .09, 1, .93), w_pad=2)
        path = Path(output).with_name(Path(output).stem+(f"_{page}" if page > 1 else "")+".png")
        figure.savefig(path, dpi=150)
        plt.close(figure)
        saved.append(path)
    return saved


def plot_latency(report, output):
    import matplotlib.pyplot as plt
    import numpy as np

    _, cases = grouped_runs(report)
    saved = []
    modes = (("calls", "static", "Первичный\nв системе\n(параллельно)"),
             ("calls", "traffic", "Повторный\nв системе\n(последовательно)"),
             ("isolated_calls", "static", "Первичный\nотдельно"),
             ("isolated_calls", "traffic", "Повторный\nотдельно"))
    # A separate panel for each case AND network; do not mix network profiles.
    panels = list(grouped_runs(report)[0].items())
    for page, offset in enumerate(range(0, len(panels), 4), 1):
        selected = panels[offset:offset+4]
        columns = min(2, len(selected))
        figure, axes = plt.subplots(ceil(len(selected)/columns), columns, squeeze=False,
                                    figsize=(16, 5*ceil(len(selected)/columns)))
        for axis, ((case, profile), runs) in zip(axes.flat, selected):
            largest = 100.0
            for x, (field, mode, _) in enumerate(modes):
                calls = [c for r in runs for c in r.get(field, []) if c["mode"] == mode]
                if not calls:
                    axis.text(x, 15, "нет вызовов", ha="center", fontsize=9, color="#777777")
                    continue
                times = [c["elapsed_ms"] for c in calls]
                largest = max(largest, max(times))
                positions = x+np.linspace(-.13, .13, len(calls))
                axis.plot([x, x], [min(times), max(times)], color="#333333", linewidth=1)
                axis.plot([x-.16, x+.16], [median(times)]*2, color="#333333", linewidth=2)
                for at, call in zip(positions, calls):
                    axis.scatter(at, call["elapsed_ms"], s=35,
                                 color=AGENT_COLORS.get(call["agent"], "#333333") if call["succeeded"] else "#bd2525",
                                 marker="o" if call["succeeded"] else "x", zorder=3)
                axis.annotate(f"n={len(calls)}", (x, max(times)), xytext=(0, 8), textcoords="offset points",
                              ha="center", fontsize=9)
            axis.axhline(100, color="#c43d3d", linestyle="--", linewidth=1.2)
            axis.set_xticks(range(4), [m[2] for m in modes], fontsize=10)
            axis.set_xlim(-.5, 3.5)
            axis.set_ylim(0, largest*1.22)
            axis.set_ylabel("Реальное время одного вызова, мс")
            axis.set_title(case_label(cases[case]).replace("\n", " · ")+" · "+PROFILE_NAMES[profile])
            axis.grid(axis="y", alpha=.2)
            axis.set_axisbelow(True)
        for axis in list(axes.flat)[len(selected):]:
            axis.set_visible(False)
        figure.suptitle("Локальное планирование: реальные вызовы и изолированный повтор", fontsize=17)
        figure.text(.02, .048, sample_note(report), fontsize=10)
        figure.text(.02, .012, "Точки — отдельные вызовы (× — неудача); черта — медиана, отрезок — min–max. "
                    "Красная линия — 100 мс. Первичный поиск: 8 корней, повторный: 1; это разные задачи.", fontsize=9)
        figure.tight_layout(rect=(0, .105, 1, .92))
        path = Path(output).with_name(Path(output).stem+(f"_{page}" if page > 1 else "")+".png")
        figure.savefig(path, dpi=150)
        plt.close(figure)
        saved.append(path)
    return saved


def plot_maps(manifest, output):
    """Rebuild representative terrain/boxes, not trajectories or planner runs."""
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap
    from matplotlib.patches import Rectangle
    import numpy as np
    from examples.benchmark_fleet import FAMILIES, make_case

    chosen = {}
    for instance in manifest["instances"]:
        family = instance["parameters"]["family"]
        if family not in chosen or (len(instance["agents"]) > len(chosen[family]["agents"])):
            chosen[family] = instance
    terrain_colors = LinearSegmentedColormap.from_list("height", ["#326649", "#8ba65c", "#c4b577", "#af805e", "#f4f1df"])
    figure, axes = plt.subplots(3, 2, figsize=(16, 10))
    image = None
    for axis, family in zip(axes.flat, [f for f in FAMILIES if f in chosen]):
        instance = chosen[family]
        world, requests, descriptor = make_case(family, instance["parameters"]["map_seed"], len(instance["agents"]))
        if descriptor["id"] != instance["id"]:
            raise ValueError("manifest parameters differ from the current generator; cannot silently redraw")
        grid = world.terrain
        image = axis.imshow(np.asarray(grid.heights), origin="lower", interpolation="nearest",
                            extent=[grid.x_min/1000, grid.x_max/1000, grid.y_min/1000, grid.y_max/1000],
                            cmap=terrain_colors, vmin=0, vmax=1000, aspect="equal")
        for box in world.obstacles:
            axis.add_patch(Rectangle((box.x_min/1000, box.y_min/1000),
                                     (box.x_max-box.x_min)/1000, (box.y_max-box.y_min)/1000,
                                     facecolor="#743a35", edgecolor="#632822", alpha=.85))
        for r in requests:
            color = AGENT_COLORS[r.name]
            a, b = (r.start.x/1000, r.start.y/1000), (r.goal.x/1000, r.goal.y/1000)
            axis.annotate("", xy=b, xytext=a, arrowprops=dict(arrowstyle="->", color=color, lw=1, linestyle="--", alpha=.75))
            axis.scatter(*a, s=20, c=color, edgecolors="white", linewidths=.5, zorder=4)
            axis.scatter(*b, s=30, facecolors="none", edgecolors=color, linewidths=1.5, zorder=4)
            axis.text(a[0]+.07, a[1]+.07, r.name, color=color, weight="bold", fontsize=10)
        axis.set_title(f"{FAMILY_NAMES[family]} · карта {instance['parameters']['map_seed']} · {len(requests)} агентов")
        axis.set_xlabel("Восток, км")
        axis.set_ylabel("Север, км")
    for axis in list(axes.flat)[len(chosen):]:
        axis.set_axis_off()
    legend_axis = axes.flat[-1]
    legend_axis.text(.03, .9, "ОДИНАКОВЫЙ МАСШТАБ", transform=legend_axis.transAxes, fontsize=15, weight="bold")
    legend_axis.text(.03, .73, "6,4 × 2,4 км · клетка 10 × 10 м\nРельеф 0–1000 м · каждый рейс меняет высоту на 1 км\n"
                     "Коричневые объёмы — запрещённые препятствия\nТочка — старт, кольцо — цель\n"
                     "Пунктирные стрелки — ЗАДАНИЯ, НЕ рассчитанные маршруты", transform=legend_axis.transAxes,
                     fontsize=10.5, va="top", linespacing=1.45)
    if image is not None:
        color_axis = legend_axis.inset_axes([.04, .015, .82, .07])
        figure.colorbar(image, cax=color_axis, orientation="horizontal", label="Высота рельефа, м")
    figure.suptitle("AirLogistics-3D · представители семейств карт", fontsize=18)
    figure.tight_layout(rect=(0, 0, 1, .95))
    figure.savefig(output, dpi=150)
    plt.close(figure)
    return Path(output)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reports", type=Path, nargs="+")
    parser.add_argument("--manifest", type=Path, help="also draw representative maps from this manifest")
    parser.add_argument("--compare-links", action="store_true", help="optional legacy communication stress-test charts")
    parser.add_argument("--output", type=Path, default=Path("outputs/benchmark_fleet/figures"))
    args = parser.parse_args(argv)
    try:
        import matplotlib
    except ImportError:
        parser.error("Matplotlib is required: python -m pip install -r requirements.txt")
    matplotlib.use("Agg")
    matplotlib.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "axes.titlesize": 12})
    args.output.mkdir(parents=True, exist_ok=True)
    catalog = ["# Визуализации AirLogistics-3D", "",
               "Графики построены из сохранённых JSON: планирование повторно не запускалось.", "",
               "Реальное время вычислений отделено от модельного времени полёта.",
               "Частично согласованная миссия не считается полной успешной миссией.", ""]
    stems = set()
    for source in args.reports:
        if source.stem in stems:
            parser.error("input reports must have different filename stems")
        stems.add(source.stem)
        report = json.loads(source.read_text(encoding="utf-8"))
        if not report.get("runs"):
            parser.error(f"{source}: no measured missions; a manifest is not a results report")
        relative = Path(os.path.relpath(source.resolve(), args.output.resolve())).as_posix()
        catalog.extend([f"## {source.stem}", "", f"[Исходный JSON]({relative})", "", sample_note(report), ""])
        if args.compare_links:
            plots = [*plot_results(report, args.output/(source.stem+"_results.png")),
                     *plot_latency(report, args.output/(source.stem+"_latency.png"))]
        else:
            try:
                plots = plot_performance(report, args.output/source.stem)
            except ValueError as error:
                parser.error(str(error))
            catalog.extend([f"**[Начать здесь: что измеряли, как читать графики и результаты]({source.stem}_summary.md)**", "",
                            "Один эксперимент — планирование всей группы с нуля. Несколько экспериментов нужны для оценки разброса RRT*. "
                            "Перестроения путей происходят внутри эксперимента и считаются отдельно.", ""])
        for path in plots:
            print(path)
            if not args.compare_links and path.suffix == ".md":
                continue
            title = path.name
            if not args.compare_links:
                title = ("Время планирования всей группы — ящики с усами" if "_timing" in path.stem else
                         "Суммарная длина маршрутов — ящики с усами" if "_quality" in path.stem else
                         "Время поиска одного пути: первое построение и перестроение — ящики с усами")
            catalog.append(f"- [{title}]({path.name})")
        catalog.append("")
    if args.manifest:
        os.environ["DUBINS_BACKEND"] = "cpp"
        print(plot_maps(json.loads(args.manifest.read_text(encoding="utf-8")), args.output/"maps.png"))
        catalog.extend(["## Карты", "", "[Представители семейств](maps.png)", "",
                        "Стрелки на картах показывают задания старт → цель, НЕ рассчитанные пути.", ""])
    index = args.output/"index.md"
    index.write_text("\n".join(catalog), encoding="utf-8")
    print(index)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
