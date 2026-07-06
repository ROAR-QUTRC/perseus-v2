# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Curses front-end: tabs, key handling, and the render loop."""

# This is the only module that imports curses. All display logic funnels
# through `_addstr`, which clips to the window and swallows curses errors so a
# tiny or resizing terminal can never crash the loop.

import curses
from pathlib import Path

from perseus_lite_tui import config, jobs, layout, persistence, pixi_env
from perseus_lite_tui.registry import (
    BOOL,
    build_launch_command,
    build_task_command,
    CHOICE,
    normalize_bool,
)

TAB_LAUNCH = 0
TAB_TASKS = 1
TAB_LOGS = 2

_TAB_TITLES = ['Launch', 'Tasks', 'Logs']

_HINTS = {
    TAB_LAUNCH: '↑↓ pick  ←→ option  l/enter launch  r reset  1-3 tabs  q quit',
    TAB_TASKS: '↑↓ pick  l/enter run  1-3 tabs  q quit',
    TAB_LOGS: '[ ] job  ↑↓ scroll  k stop  x clear finished  1-3 tabs  q quit',
}


class App:
    """Owns TUI state and drives the curses render/input loop."""

    def __init__(self, repo_root: Path):
        self.repo_root = Path(repo_root)
        self.ros_ws = str(pixi_env.ros_ws_dir(self.repo_root))
        self.profiles = pixi_env.visible_profiles(config.PROFILES)
        self.tasks = list(config.TASKS)
        self.jobs = jobs.JobManager(noise_filter=None)
        self.state_file = persistence.state_path()
        self.selections = persistence.load_selections(self.state_file)

        self.active = TAB_LAUNCH
        self.launch_sel = 0
        self.opt_focus = 0
        self.task_sel = 0
        self.log_sel = 0
        self.log_scroll = 0
        self.status = ''
        self.colors = {}
        self.stdscr = None

    # -- lifecycle ------------------------------------------------------------

    def run(self) -> int:
        """Run the curses loop, stopping all jobs on exit."""
        try:
            curses.wrapper(self._loop)
        finally:
            self.jobs.stop_all()
            persistence.save_selections(self.state_file, self.selections)
        return 0

    def _loop(self, stdscr) -> None:
        self.stdscr = stdscr
        curses.curs_set(0)
        stdscr.timeout(100)
        self._init_colors()
        while True:
            self._render()
            key = stdscr.getch()
            if key == -1 or key == curses.KEY_RESIZE:
                continue
            if self._handle_key(key):
                return

    def _init_colors(self) -> None:
        try:
            curses.start_color()
            curses.use_default_colors()
        except curses.error:
            return
        pairs = {
            'header': (curses.COLOR_WHITE, curses.COLOR_BLUE),
            'group': (curses.COLOR_CYAN, -1),
            'running': (curses.COLOR_GREEN, -1),
            'bad': (curses.COLOR_RED, -1),
            'dim': (curses.COLOR_WHITE, -1),
        }
        for index, (name, (fg, bg)) in enumerate(pairs.items(), start=1):
            try:
                curses.init_pair(index, fg, bg)
                self.colors[name] = curses.color_pair(index)
            except curses.error:
                self.colors[name] = 0
        self.colors['dim'] = self.colors.get('dim', 0) | curses.A_DIM

    def _color(self, name: str) -> int:
        return self.colors.get(name, 0)

    # -- drawing --------------------------------------------------------------

    def _addstr(self, y: int, x: int, text: str, attr: int = 0) -> None:
        maxy, maxx = self.stdscr.getmaxyx()
        if y < 0 or y >= maxy or x >= maxx or x < 0:
            return
        clipped = text[: max(0, maxx - x)]
        try:
            self.stdscr.addstr(y, x, clipped, attr)
        except curses.error:
            pass

    def _render(self) -> None:
        self.stdscr.erase()
        maxy, maxx = self.stdscr.getmaxyx()
        self._addstr(
            0,
            0,
            ' perseus-lite mission control '.ljust(maxx),
            self._color('header') | curses.A_BOLD,
        )
        self._draw_tabs(1)
        body_top, body_bottom = 3, maxy - 2
        if self.active == TAB_LAUNCH:
            self._draw_launch(body_top, body_bottom)
        elif self.active == TAB_TASKS:
            self._draw_tasks(body_top, body_bottom)
        else:
            self._draw_logs(body_top, body_bottom)
        self._addstr(maxy - 1, 0, self.status[:maxx], curses.A_BOLD)
        hint = _HINTS.get(self.active, '')
        self._addstr(maxy - 1, max(0, maxx - len(hint) - 1), hint, curses.A_DIM)
        self.stdscr.refresh()

    def _draw_tabs(self, y: int) -> None:
        x = 0
        for text, is_active in layout.tab_bar(_TAB_TITLES, self.active):
            attr = curses.A_REVERSE if is_active else curses.A_DIM
            self._addstr(y, x, text, attr)
            x += len(text) + 1

    def _draw_launch(self, top: int, bottom: int) -> None:
        rows = self._launch_rows()
        detail_height = 4
        list_bottom = bottom - detail_height
        height = max(1, list_bottom - top)
        target = self._selected_row_index(rows)
        offset = target - height + 1 if target >= height else 0
        offset = layout.clamp_scroll(offset, len(rows), height)
        end = offset + height
        window = rows[offset:end]
        y = top
        for kind, text, ref in window:
            if kind == 'group':
                self._addstr(y, 0, text, self._color('group') | curses.A_BOLD)
            else:
                selected = ref is self._selected_profile()
                attr = curses.A_REVERSE if selected else 0
                if ref.disabled:
                    attr = curses.A_DIM
                self._addstr(y, 0, '  ' + text, attr)
                self._addstr(y, 2 + len(text) + 2, self._job_badge(ref.slug))
            y += 1
        self._draw_launch_detail(list_bottom + 1, bottom)

    def _draw_launch_detail(self, top: int, bottom: int) -> None:
        prof = self._selected_profile()
        if prof is None:
            return
        self._addstr(top, 0, '─' * 40, curses.A_DIM)
        if prof.disabled:
            self._addstr(top + 1, 0, prof.note or 'Disabled.', self._color('dim'))
            return
        if not prof.options:
            self._addstr(top + 1, 0, '(no options)', curses.A_DIM)
        x = 0
        for index, option in enumerate(prof.options):
            value = self._opt_value(prof, option)
            label = f'{option.label}: {value}'
            focused = index == self.opt_focus
            attr = curses.A_REVERSE if focused else 0
            self._addstr(top + 1, x, label, attr)
            x += len(label) + 3
        if prof.note:
            self._addstr(top + 2, 0, prof.note, curses.A_DIM)

    def _draw_tasks(self, top: int, bottom: int) -> None:
        y = top
        for index, task in enumerate(self.tasks):
            if y > bottom:
                break
            selected = index == self.task_sel
            attr = curses.A_REVERSE if selected else 0
            self._addstr(y, 0, '  ' + task.label, attr)
            self._addstr(y, 40, self._job_badge('task:' + task.slug))
            if task.note:
                self._addstr(y, 56, task.note, curses.A_DIM)
            y += 1

    def _draw_logs(self, top: int, bottom: int) -> None:
        all_jobs = self.jobs.jobs()
        if not all_jobs:
            self._addstr(
                top,
                0,
                'No jobs yet. Launch something from the Launch tab.',
                curses.A_DIM,
            )
            return
        self.log_sel = max(0, min(self.log_sel, len(all_jobs) - 1))
        x = 0
        for index, job in enumerate(all_jobs):
            marker = '▸' if index == self.log_sel else ' '
            label = f'{marker}{job.label}'
            attr = curses.A_REVERSE if index == self.log_sel else self._job_attr(job)
            self._addstr(top, x, label, attr)
            x += len(label) + 2
        job = all_jobs[self.log_sel]
        self._addstr(
            top + 1,
            0,
            f'{job.label} — {job.status_line()}',
            self._job_attr(job) | curses.A_BOLD,
        )
        self._draw_log_body(job, top + 2, bottom)

    def _draw_log_body(self, job, top: int, bottom: int) -> None:
        height = max(1, bottom - top + 1)
        lines = job.snapshot_log()
        if self.log_scroll <= 0:
            view = layout.tail(lines, height)
        else:
            end = max(0, len(lines) - self.log_scroll)
            start = max(0, end - height)
            view = lines[start:end]
        for offset, line in enumerate(view):
            self._addstr(top + offset, 0, line)

    # -- row/selection helpers ------------------------------------------------

    def _launch_rows(self):
        rows = []
        last_group = None
        for prof in self.profiles:
            if prof.group != last_group:
                rows.append(('group', prof.group, None))
                last_group = prof.group
            rows.append(('profile', prof.label, prof))
        return rows

    def _profile_rows(self):
        return [r for r in self._launch_rows() if r[0] == 'profile']

    def _selected_profile(self):
        profiles = [r[2] for r in self._profile_rows()]
        if not profiles:
            return None
        self.launch_sel = max(0, min(self.launch_sel, len(profiles) - 1))
        return profiles[self.launch_sel]

    def _selected_row_index(self, rows) -> int:
        prof = self._selected_profile()
        for index, (kind, _text, ref) in enumerate(rows):
            if kind == 'profile' and ref is prof:
                return index
        return 0

    def _job_badge(self, key: str) -> str:
        job = self.jobs.get(key)
        return '' if job is None else f'[{job.status_line()}]'

    def _job_attr(self, job) -> int:
        if job.running:
            return self._color('running')
        return self._color('bad') if job.returncode else self._color('dim')

    def _opt_value(self, prof, option) -> str:
        return self.selections.get(prof.slug, {}).get(option.key, option.default)

    def _set_opt(self, prof, option, value: str) -> None:
        self.selections.setdefault(prof.slug, {})[option.key] = value
        persistence.save_selections(self.state_file, self.selections)

    # -- actions --------------------------------------------------------------

    def _cycle_option(self, direction: int) -> None:
        prof = self._selected_profile()
        if prof is None or not prof.options:
            return
        self.opt_focus = max(0, min(self.opt_focus, len(prof.options) - 1))
        option = prof.options[self.opt_focus]
        value = self._opt_value(prof, option)
        if option.kind == BOOL:
            self._set_opt(
                prof, option, 'False' if normalize_bool(value) == 'True' else 'True'
            )
        elif option.kind == CHOICE and option.choices:
            current = option.choices.index(value) if value in option.choices else 0
            nxt = (current + direction) % len(option.choices)
            self._set_opt(prof, option, option.choices[nxt])

    def _reset_options(self) -> None:
        prof = self._selected_profile()
        if prof is None:
            return
        self.selections.pop(prof.slug, None)
        persistence.save_selections(self.state_file, self.selections)
        self.status = f'Reset options for {prof.label}'

    def _launch_selected(self) -> None:
        prof = self._selected_profile()
        if prof is None:
            return
        if prof.disabled:
            self.status = prof.note or 'This entry cannot be launched here.'
            return
        if not pixi_env.install_setup_exists(self.repo_root):
            self.status = 'Workspace not built — run Tasks → Build first.'
            self.active = TAB_TASKS
            return
        selections = self.selections.get(prof.slug, {})
        argv = build_launch_command(prof, selections, self.ros_ws)
        _, started = self.jobs.start(prof.slug, prof.label, argv, str(self.repo_root))
        self.status = (
            f'Launched {prof.label}' if started else f'{prof.label} already running'
        )
        self._focus_logs(prof.slug)

    def _run_task(self) -> None:
        if not self.tasks:
            return
        task = self.tasks[self.task_sel]
        argv = build_task_command(task)
        key = 'task:' + task.slug
        _, started = self.jobs.start(key, task.label, argv, str(self.repo_root))
        self.status = (
            f'Running {task.label}' if started else f'{task.label} already running'
        )
        self._focus_logs(key)

    def _focus_logs(self, key: str) -> None:
        self.active = TAB_LOGS
        self.log_scroll = 0
        all_jobs = self.jobs.jobs()
        for index, job in enumerate(all_jobs):
            if job.key == key:
                self.log_sel = index

    # -- input ----------------------------------------------------------------

    def _handle_key(self, key: int) -> bool:
        if key in (ord('q'), 27):
            return True
        if key in (ord('1'), ord('2'), ord('3')):
            self.active = key - ord('1')
            return False
        if key == ord('\t'):
            self.active = (self.active + 1) % len(_TAB_TITLES)
            return False
        if key == curses.KEY_BTAB:
            self.active = (self.active - 1) % len(_TAB_TITLES)
            return False
        if self.active == TAB_LAUNCH:
            self._handle_launch_key(key)
        elif self.active == TAB_TASKS:
            self._handle_tasks_key(key)
        else:
            self._handle_logs_key(key)
        return False

    def _handle_launch_key(self, key: int) -> None:
        profiles = self._profile_rows()
        if key == curses.KEY_UP:
            self.launch_sel = max(0, self.launch_sel - 1)
            self.opt_focus = 0
        elif key == curses.KEY_DOWN:
            self.launch_sel = min(len(profiles) - 1, self.launch_sel + 1)
            self.opt_focus = 0
        elif key == curses.KEY_LEFT:
            self._cycle_option(-1)
        elif key in (curses.KEY_RIGHT, ord(' ')):
            self._cycle_option(1)
        elif key in (ord('l'), curses.KEY_ENTER, 10, 13):
            self._launch_selected()
        elif key == ord('r'):
            self._reset_options()

    def _handle_tasks_key(self, key: int) -> None:
        if key == curses.KEY_UP:
            self.task_sel = max(0, self.task_sel - 1)
        elif key == curses.KEY_DOWN:
            self.task_sel = min(len(self.tasks) - 1, self.task_sel + 1)
        elif key in (ord('l'), curses.KEY_ENTER, 10, 13):
            self._run_task()

    def _handle_logs_key(self, key: int) -> None:
        all_jobs = self.jobs.jobs()
        if key == ord('[') and all_jobs:
            self.log_sel = max(0, self.log_sel - 1)
            self.log_scroll = 0
        elif key == ord(']') and all_jobs:
            self.log_sel = min(len(all_jobs) - 1, self.log_sel + 1)
            self.log_scroll = 0
        elif key == curses.KEY_UP:
            self.log_scroll += 1
        elif key == curses.KEY_DOWN:
            self.log_scroll = max(0, self.log_scroll - 1)
        elif key == ord('k') and all_jobs:
            all_jobs[self.log_sel].stop()
            self.status = 'Stopping job…'
        elif key == ord('x'):
            removed = self.jobs.clear_finished()
            self.status = f'Cleared {removed} finished job(s)'


def _resolve_repo_root() -> Path:
    for start in (Path.cwd(), Path(__file__).resolve()):
        root = pixi_env.find_repo_root(start)
        if root is not None:
            return root
    return None


def main() -> int:
    """Console-script entry point for the perseus_tui command."""
    root = _resolve_repo_root()
    if root is None:
        print('perseus_tui: could not find pixi.toml (run from the repo).')
        return 1
    return App(root).run()
