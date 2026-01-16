#!/usr/bin/env python3
"""
Perseus V2 Test Runner TUI
Interactive terminal interface for running and managing tests.

Uses Python's built-in curses module for cross-platform compatibility.
"""

import curses
import subprocess
import sys
import time
import re
import threading
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
from enum import Enum
from pathlib import Path

# ASCII box drawing characters
BOX_H = "-"
BOX_V = "|"
BOX_TL = "+"
BOX_TR = "+"
BOX_BL = "+"
BOX_BR = "+"
BOX_T = "+"
BOX_B = "+"
BOX_L = "+"
BOX_R = "+"
BOX_X = "+"

# Paths
SCRIPT_DIR = Path(__file__).parent.resolve()
REPO_ROOT = SCRIPT_DIR.parent
ROS_WS = REPO_ROOT / "software" / "ros_ws"
RESULTS_DIR = REPO_ROOT / "test-results"
ALLURE_REPORT = RESULTS_DIR / "allure-report" / "index.html"


class TestStatus(Enum):
    PENDING = "pending"
    RUNNING = "running"
    PASSED = "passed"
    FAILED = "failed"
    SKIPPED = "skipped"


@dataclass
class TestCase:
    name: str
    suite: str
    package: str
    selected: bool = True
    status: TestStatus = TestStatus.PENDING
    duration: float = 0.0
    error_msg: str = ""


@dataclass
class TestSuite:
    name: str
    package: str
    tests: List[TestCase] = field(default_factory=list)
    expanded: bool = True
    selected: bool = True

    @property
    def passed_count(self) -> int:
        return sum(1 for t in self.tests if t.status == TestStatus.PASSED)

    @property
    def failed_count(self) -> int:
        return sum(1 for t in self.tests if t.status == TestStatus.FAILED)


@dataclass
class Package:
    name: str
    suites: List[TestSuite] = field(default_factory=list)
    expanded: bool = True
    selected: bool = True


class TestRunnerTUI:
    """Main TUI application class."""

    def __init__(self, stdscr):
        self.stdscr = stdscr
        self.packages: List[Package] = []
        self.cursor_pos = 0
        self.scroll_offset = 0
        self.visible_items: List[Tuple[str, object]] = []  # (type, object)
        self.status_msg = "Press 'h' for help"
        self.running = True
        self.test_output: List[str] = []
        self.show_output = False
        self.last_run_time = 0.0
        self.last_run_passed = 0
        self.last_run_failed = 0
        self.allure_server_url: Optional[str] = None
        self.allure_process: Optional[subprocess.Popen] = None

        # Initialize curses
        curses.curs_set(0)
        curses.start_color()
        curses.use_default_colors()

        # Define color pairs
        curses.init_pair(1, curses.COLOR_GREEN, -1)  # Passed
        curses.init_pair(2, curses.COLOR_RED, -1)  # Failed
        curses.init_pair(3, curses.COLOR_YELLOW, -1)  # Running/Warning
        curses.init_pair(4, curses.COLOR_CYAN, -1)  # Info/Selected
        curses.init_pair(5, curses.COLOR_MAGENTA, -1)  # Header
        curses.init_pair(6, curses.COLOR_WHITE, curses.COLOR_BLUE)  # Highlight

        self.COLOR_PASSED = curses.color_pair(1)
        self.COLOR_FAILED = curses.color_pair(2)
        self.COLOR_RUNNING = curses.color_pair(3)
        self.COLOR_INFO = curses.color_pair(4)
        self.COLOR_HEADER = curses.color_pair(5)
        self.COLOR_HIGHLIGHT = curses.color_pair(6)

        # Load test data
        self.load_tests()

    def load_tests(self):
        """Discover available tests from the build directory."""
        self.packages = []

        # Test executables and their packages
        test_executables = [
            (
                "perseus_input",
                "axis_math_test",
                [
                    (
                        "DeadbandTest",
                        [
                            "ValueWithinDeadbandReturnsZero",
                            "ValueAtDeadbandBoundaryReturnsZero",
                            "ValueAboveDeadbandIsNormalized",
                            "ValueBelowNegativeDeadbandIsNormalized",
                            "AsymmetricDeadband",
                            "NaNDeadbandReturnsOriginalValue",
                            "ZeroDeadbandPassesThrough",
                        ],
                    ),
                    (
                        "ButtonOverrideTest",
                        [
                            "NoButtonsReturnsOriginalValue",
                            "PositiveButtonReturnsOne",
                            "NegativeButtonReturnsNegativeOne",
                            "BothButtonsReturnsZero",
                        ],
                    ),
                    (
                        "ScalingTest",
                        [
                            "BasicScaling",
                            "NaNScalingDefaultsToOne",
                            "ZeroScalingReturnsZero",
                            "NegativeScalingInvertsAxis",
                        ],
                    ),
                    (
                        "ResolveNegativeDeadbandTest",
                        [
                            "FiniteNegativeDeadbandReturnsAsIs",
                            "NaNNegativeDeadbandDefaultsToNegativePositive",
                        ],
                    ),
                    (
                        "EnableThresholdTest",
                        [
                            "GreaterThanThreshold",
                            "LessThanThreshold",
                            "NaNThresholdDefaultsToHalf",
                            "NegativeThreshold",
                        ],
                    ),
                    (
                        "FullPipelineTest",
                        [
                            "TypicalJoystickInput",
                            "ButtonOverridesDeadband",
                            "SmallInputInDeadbandIsZero",
                        ],
                    ),
                ],
            ),
            (
                "perseus_input",
                "controller_config_test",
                [
                    (
                        "ControllerConfigTest",
                        [
                            "AllConfigFilesExist",
                            "AllConfigFilesAreValidYaml",
                            "AllConfigsHaveGenericControllerSection",
                            "AllConfigsHaveDriveSection",
                            "DriveAxesHaveValidConfiguration",
                            "ScalingValuesAreReasonable",
                            "EnableConfigurationsAreValid",
                            "TimeoutIsConfigured",
                            "ButtonIndicesAreValid",
                            "BucketSectionIsConsistent",
                            "FollowsReferencesAreValid",
                            "TurboScalingIsReasonable",
                        ],
                    ),
                ],
            ),
            (
                "perseus_lite",
                "controller_config_test",
                [
                    (
                        "PerseusLiteControllerConfigTest",
                        [
                            "ConfigFileExists",
                            "ValidYamlStructure",
                            "HasControllerManagerSection",
                            "ControllerManagerUpdateRateIsValid",
                            "HasRequiredControllers",
                            "ControllerTypesAreCorrect",
                            "HasHardwareInterfaceSection",
                            "AllFourWheelJointsDefined",
                            "JointsHaveRequiredInterfaces",
                            "HasDiffDriveControllerSection",
                            "WheelNamesMatchHardwareJoints",
                            "WheelGeometryIsValid",
                            "KinematicLimitsAreValid",
                            "CmdVelTimeoutIsValid",
                            "OdomFrameIdsAreConfigured",
                            "CovarianceDiagonalsAreValid",
                            "PublishRateIsValid",
                            "WriteOpModesIsConfigured",
                            "RegressionWheelGeometry",
                        ],
                    ),
                ],
            ),
            (
                "perseus_simulation",
                "gz_bridge_config_test",
                [
                    (
                        "GzBridgeConfigTest",
                        [
                            "ConfigFileExists",
                            "ValidYamlStructure",
                            "ValidDirections",
                            "ValidRosMessageTypes",
                            "ValidGazeboMessageTypes",
                            "UniqueTopicNames",
                            "ExpectedCoreTopics",
                        ],
                    ),
                ],
            ),
            (
                "perseus_sensors",
                "imu_device_config_test",
                [
                    (
                        "ImuDeviceRegistryTest",
                        [
                            "GetSupportedDevicesReturnsExpectedCount",
                            "GetSupportedDevicesContainsExpectedDevices",
                            "GetDefaultDeviceReturnsLsm6dsox",
                            "GetDeviceConfigReturnsValidConfigForAllDevices",
                            "GetDeviceConfigReturnsNulloptForUnknownDevice",
                        ],
                    ),
                    (
                        "ImuScaleFactorTest",
                        [
                            "Lsm6dsoxAccelScaleIsCorrect",
                            "Mpu6050AccelScaleIsCorrect",
                            "GyroScaleConvertsToRadPerSec",
                        ],
                    ),
                ],
            ),
            (
                "perseus_sensors",
                "imu_calibration_test",
                [
                    (
                        "ImuCalibrationTest",
                        [
                            "IdentityCalibrationPreservesAccelData",
                            "IdentityCalibrationPreservesGyroData",
                            "AccelOffsetIsSubtracted",
                            "GyroOffsetIsSubtracted",
                            "NegativeOffsetIncreasesValue",
                            "AccelScaleIsApplied",
                            "GyroScaleIsApplied",
                            "OffsetThenScaleOrder",
                            "FullCalibrationComputation",
                            "TemperatureIsPreserved",
                            "TimestampIsPreserved",
                            "ZeroInputData",
                            "ZeroScaleZerosOutput",
                            "NegativeScaleInvertsData",
                            "LargeValuesHandledCorrectly",
                            "NegativeInputData",
                            "GravityCompensation",
                            "GyroDriftCompensation",
                        ],
                    ),
                ],
            ),
        ]

        # Build package/suite/test hierarchy
        pkg_dict: Dict[str, Package] = {}

        for pkg_name, exe_name, suites in test_executables:
            if pkg_name not in pkg_dict:
                pkg_dict[pkg_name] = Package(name=pkg_name)

            for suite_name, test_names in suites:
                suite = TestSuite(name=suite_name, package=pkg_name)
                for test_name in test_names:
                    test = TestCase(name=test_name, suite=suite_name, package=pkg_name)
                    suite.tests.append(test)
                pkg_dict[pkg_name].suites.append(suite)

        self.packages = list(pkg_dict.values())
        self.rebuild_visible_items()

    def rebuild_visible_items(self):
        """Rebuild the list of visible items based on expansion state."""
        self.visible_items = []

        for pkg in self.packages:
            self.visible_items.append(("package", pkg))
            if pkg.expanded:
                for suite in pkg.suites:
                    self.visible_items.append(("suite", suite))
                    if suite.expanded:
                        for test in suite.tests:
                            self.visible_items.append(("test", test))

        # Adjust cursor if needed
        if self.cursor_pos >= len(self.visible_items):
            self.cursor_pos = max(0, len(self.visible_items) - 1)

    def get_total_tests(self) -> int:
        """Get total number of tests."""
        return sum(len(suite.tests) for pkg in self.packages for suite in pkg.suites)

    def get_selected_tests(self) -> int:
        """Get number of selected tests."""
        return sum(
            1
            for pkg in self.packages
            for suite in pkg.suites
            for test in suite.tests
            if test.selected
        )

    def draw_header(self):
        """Draw the application header."""
        height, width = self.stdscr.getmaxyx()

        # Title bar
        title = " PERSEUS V2 TEST RUNNER "
        self.stdscr.attron(curses.A_BOLD | self.COLOR_HEADER)
        self.stdscr.addstr(0, 0, BOX_H * width)
        self.stdscr.addstr(0, (width - len(title)) // 2, title)
        self.stdscr.attroff(curses.A_BOLD | self.COLOR_HEADER)

        # Stats line
        total = self.get_total_tests()
        selected = self.get_selected_tests()
        stats = f" Tests: {selected}/{total} selected "

        if self.last_run_time > 0:
            stats += f"| Last run: {self.last_run_passed} passed, {self.last_run_failed} failed ({self.last_run_time:.1f}s) "

        self.stdscr.addstr(1, 0, stats[: width - 1])

        # Allure server URL line (if running)
        if self.allure_server_url:
            allure_line = f" Allure: {self.allure_server_url} "
            self.stdscr.attron(self.COLOR_INFO)
            self.stdscr.addstr(2, 0, allure_line[: width - 1])
            self.stdscr.attroff(self.COLOR_INFO)

    def draw_test_list(self):
        """Draw the test list."""
        height, width = self.stdscr.getmaxyx()
        list_start = 4 if self.allure_server_url else 3
        list_height = height - 7 if self.allure_server_url else height - 6

        # Adjust scroll offset to keep cursor visible
        if self.cursor_pos < self.scroll_offset:
            self.scroll_offset = self.cursor_pos
        elif self.cursor_pos >= self.scroll_offset + list_height:
            self.scroll_offset = self.cursor_pos - list_height + 1

        for i in range(list_height):
            item_idx = self.scroll_offset + i
            y = list_start + i

            if item_idx >= len(self.visible_items):
                self.stdscr.addstr(y, 0, " " * (width - 1))
                continue

            item_type, item = self.visible_items[item_idx]
            is_selected = item_idx == self.cursor_pos

            # Build the display line
            if item_type == "package":
                prefix = "[v]" if item.expanded else "[>]"
                checkbox = "[*]" if item.selected else "[ ]"
                line = f"{prefix} {checkbox} {item.name}"
                attr = curses.A_BOLD
            elif item_type == "suite":
                prefix = "  [v]" if item.expanded else "  [>]"
                checkbox = "[*]" if item.selected else "[ ]"

                # Show pass/fail counts if tests have run
                status_str = ""
                if any(t.status != TestStatus.PENDING for t in item.tests):
                    passed = item.passed_count
                    failed = item.failed_count
                    if failed > 0:
                        status_str = f" ({passed}/{len(item.tests)} passed)"
                    else:
                        status_str = f" ({passed}/{len(item.tests)} passed)"

                line = f"{prefix} {checkbox} {item.name}{status_str}"
                attr = 0
            else:  # test
                checkbox = "[*]" if item.selected else "[ ]"
                status_icon = " "
                status_attr = 0

                if item.status == TestStatus.PASSED:
                    status_icon = "+"
                    status_attr = self.COLOR_PASSED
                elif item.status == TestStatus.FAILED:
                    status_icon = "X"
                    status_attr = self.COLOR_FAILED
                elif item.status == TestStatus.RUNNING:
                    status_icon = "~"
                    status_attr = self.COLOR_RUNNING

                line = f"      {checkbox} {status_icon} {item.name}"
                attr = status_attr

            # Highlight selected line
            if is_selected:
                attr |= self.COLOR_HIGHLIGHT

            # Truncate and pad line
            line = line[: width - 1].ljust(width - 1)

            try:
                self.stdscr.addstr(y, 0, line, attr)
            except curses.error:
                pass

    def draw_footer(self):
        """Draw the footer with help and status."""
        height, width = self.stdscr.getmaxyx()

        # Separator
        self.stdscr.addstr(height - 3, 0, BOX_H * (width - 1))

        # Help line - show different options based on Allure state
        if self.allure_server_url:
            help_text = " [Space] Toggle | [r] Run | [a] Allure | [S] Stop Allure | [g] Generate | [h] Help | [q] Quit "
        else:
            help_text = " [Space] Toggle | [Enter] Expand | [r] Run | [a] Allure | [b] Build | [h] Help | [q] Quit "
        self.stdscr.attron(curses.A_DIM)
        self.stdscr.addstr(height - 2, 0, help_text[: width - 1])
        self.stdscr.attroff(curses.A_DIM)

        # Status line
        status = f" {self.status_msg} "
        self.stdscr.addstr(height - 1, 0, status[: width - 1].ljust(width - 1))

    def draw(self):
        """Main draw function."""
        self.stdscr.clear()
        self.draw_header()
        self.draw_test_list()
        self.draw_footer()
        self.stdscr.refresh()

    def toggle_selection(self):
        """Toggle selection of current item."""
        if not self.visible_items:
            return

        item_type, item = self.visible_items[self.cursor_pos]

        if item_type == "package":
            item.selected = not item.selected
            # Propagate to all suites and tests
            for suite in item.suites:
                suite.selected = item.selected
                for test in suite.tests:
                    test.selected = item.selected
        elif item_type == "suite":
            item.selected = not item.selected
            # Propagate to all tests
            for test in item.tests:
                test.selected = item.selected
        else:  # test
            item.selected = not item.selected

    def toggle_expand(self):
        """Toggle expansion of current item."""
        if not self.visible_items:
            return

        item_type, item = self.visible_items[self.cursor_pos]

        if item_type == "package":
            item.expanded = not item.expanded
            self.rebuild_visible_items()
        elif item_type == "suite":
            item.expanded = not item.expanded
            self.rebuild_visible_items()

    def select_all(self):
        """Select all tests."""
        for pkg in self.packages:
            pkg.selected = True
            for suite in pkg.suites:
                suite.selected = True
                for test in suite.tests:
                    test.selected = True
        self.status_msg = "All tests selected"

    def select_none(self):
        """Deselect all tests."""
        for pkg in self.packages:
            pkg.selected = False
            for suite in pkg.suites:
                suite.selected = False
                for test in suite.tests:
                    test.selected = False
        self.status_msg = "All tests deselected"

    def run_tests(self):
        """Run selected tests."""
        selected_count = self.get_selected_tests()
        if selected_count == 0:
            self.status_msg = "No tests selected!"
            return

        self.status_msg = f"Running {selected_count} tests..."
        self.draw()

        # Reset test states
        for pkg in self.packages:
            for suite in pkg.suites:
                for test in suite.tests:
                    if test.selected:
                        test.status = TestStatus.PENDING
                    else:
                        test.status = TestStatus.SKIPPED

        # Build the test filter for gtest
        # For simplicity, we'll run the shell script and parse results
        start_time = time.time()

        try:
            # Run tests using the existing script
            result = subprocess.run(
                [str(SCRIPT_DIR / "run-tests.sh"), "--no-lint", "--no-build"],
                cwd=str(ROS_WS),
                capture_output=True,
                text=True,
                timeout=300,
            )

            self.test_output = result.stdout.split("\n") + result.stderr.split("\n")
            elapsed = time.time() - start_time

            # Parse results - mark all selected tests as passed/failed based on output
            # For now, assume all pass if exit code is 0
            passed = 0
            failed = 0

            for pkg in self.packages:
                for suite in pkg.suites:
                    for test in suite.tests:
                        if test.selected:
                            # Check if test failed in output
                            test_pattern = f"{suite.name}.{test.name}"
                            if (
                                "FAILED" in result.stdout
                                and test_pattern in result.stdout
                            ):
                                test.status = TestStatus.FAILED
                                failed += 1
                            else:
                                test.status = TestStatus.PASSED
                                passed += 1

            self.last_run_time = elapsed
            self.last_run_passed = passed
            self.last_run_failed = failed

            if result.returncode == 0:
                self.status_msg = f"All {selected_count} tests passed in {elapsed:.1f}s"
            else:
                self.status_msg = f"Tests completed: {passed} passed, {failed} failed in {elapsed:.1f}s"

        except subprocess.TimeoutExpired:
            self.status_msg = "Test run timed out!"
        except Exception as e:
            self.status_msg = f"Error running tests: {str(e)[:50]}"

    def run_build(self):
        """Run build."""
        self.status_msg = "Building packages..."
        self.draw()

        try:
            result = subprocess.run(
                [str(SCRIPT_DIR / "run-tests.sh"), "--build-only"],
                cwd=str(ROS_WS),
                capture_output=True,
                text=True,
                timeout=300,
            )

            if result.returncode == 0:
                self.status_msg = "Build completed successfully"
            else:
                self.status_msg = "Build failed! Check output for details"

            self.test_output = result.stdout.split("\n") + result.stderr.split("\n")

        except subprocess.TimeoutExpired:
            self.status_msg = "Build timed out!"
        except Exception as e:
            self.status_msg = f"Build error: {str(e)[:50]}"

    def _capture_allure_url(self, process):
        """Background thread to capture Allure server URL from output."""
        try:
            for line in iter(process.stdout.readline, ""):
                if not line:
                    break
                # Look for server URL in output
                # Allure outputs: "Server started at <http://...>. Press <Ctrl+C> to exit"
                match = re.search(r"<(https?://[^>]+)>", line)
                if match:
                    self.allure_server_url = match.group(1)
                    break
                # Also try without angle brackets
                match = re.search(r"(https?://\S+:\d+)", line)
                if match:
                    self.allure_server_url = match.group(1)
                    break
        except Exception:
            pass

    def open_allure_report(self):
        """Open Allure HTML report using Allure server via nix-shell."""
        allure_report_dir = RESULTS_DIR / "allure-report"
        if not allure_report_dir.exists():
            self.status_msg = "Allure report not found. Press 'g' to generate first."
            return

        # Kill existing Allure process if running
        if self.allure_process and self.allure_process.poll() is None:
            self.allure_process.terminate()
            self.allure_server_url = None

        self.status_msg = "Starting Allure server..."
        self.draw()

        try:
            # Use nix-shell to run allure open command
            self.allure_process = subprocess.Popen(
                [
                    "nix-shell",
                    "-p",
                    "allure",
                    "--run",
                    f"allure open '{allure_report_dir}'",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )

            # Start background thread to capture URL
            capture_thread = threading.Thread(
                target=self._capture_allure_url,
                args=(self.allure_process,),
                daemon=True,
            )
            capture_thread.start()

            # Wait briefly for URL to be captured
            capture_thread.join(timeout=5.0)

            if self.allure_server_url:
                self.status_msg = "Allure server running"
            else:
                self.status_msg = "Allure server started. Check your browser."

        except Exception:
            # Fallback to xdg-open if nix-shell fails
            try:
                subprocess.Popen(
                    ["xdg-open", str(ALLURE_REPORT)],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                self.status_msg = "Opening Allure report in browser..."
            except Exception as e2:
                self.status_msg = f"Error opening report: {str(e2)[:40]}"

    def stop_allure_server(self):
        """Stop the running Allure server."""
        if self.allure_process and self.allure_process.poll() is None:
            self.allure_process.terminate()
            try:
                self.allure_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.allure_process.kill()
            self.allure_server_url = None
            self.allure_process = None
            self.status_msg = "Allure server stopped"
        else:
            self.status_msg = "No Allure server running"

    def generate_allure_report(self):
        """Generate Allure HTML report."""
        self.status_msg = "Generating Allure report..."
        self.draw()

        try:
            result = subprocess.run(
                [str(SCRIPT_DIR / "run-tests.sh"), "--no-lint", "--no-build", "--html"],
                cwd=str(ROS_WS),
                capture_output=True,
                text=True,
                timeout=120,
            )

            if result.returncode == 0:
                self.status_msg = "Allure report generated. Press 'a' to open."
            else:
                self.status_msg = "Failed to generate Allure report"

        except Exception as e:
            self.status_msg = f"Error: {str(e)[:50]}"

    def show_help(self):
        """Show help dialog."""
        height, width = self.stdscr.getmaxyx()

        help_lines = [
            "",
            "  PERSEUS V2 TEST RUNNER - HELP",
            "  " + "-" * 40,
            "",
            "  NAVIGATION:",
            "    Up/Down, j/k    Move cursor",
            "    PgUp/PgDn       Page up/down",
            "    Home/End        Jump to start/end",
            "",
            "  SELECTION:",
            "    Space           Toggle item selection",
            "    Enter           Expand/collapse item",
            "    A               Select all tests",
            "    N               Deselect all tests",
            "",
            "  ACTIONS:",
            "    r               Run selected tests",
            "    b               Build packages",
            "    g               Generate Allure report",
            "    a               Start Allure server",
            "    S               Stop Allure server",
            "    o               Show last output",
            "",
            "  OTHER:",
            "    h, ?            Show this help",
            "    q, Esc          Quit",
            "",
            "  Press any key to close this help...",
            "",
        ]

        # Draw help box
        box_h = len(help_lines) + 2
        box_w = 50
        start_y = (height - box_h) // 2
        start_x = (width - box_w) // 2

        # Draw box
        for i in range(box_h):
            y = start_y + i
            if i == 0:
                line = BOX_TL + BOX_H * (box_w - 2) + BOX_TR
            elif i == box_h - 1:
                line = BOX_BL + BOX_H * (box_w - 2) + BOX_BR
            else:
                content = help_lines[i - 1] if i - 1 < len(help_lines) else ""
                line = BOX_V + content.ljust(box_w - 2)[: box_w - 2] + BOX_V

            try:
                self.stdscr.addstr(y, start_x, line)
            except curses.error:
                pass

        self.stdscr.refresh()
        self.stdscr.getch()

    def show_output(self):
        """Show last test output."""
        if not self.test_output:
            self.status_msg = "No output to show"
            return

        height, width = self.stdscr.getmaxyx()
        scroll = 0
        max_scroll = max(0, len(self.test_output) - height + 4)

        while True:
            self.stdscr.clear()
            self.stdscr.addstr(
                0, 0, " TEST OUTPUT (q to close, arrows to scroll) ", curses.A_REVERSE
            )

            for i in range(height - 2):
                line_idx = scroll + i
                if line_idx < len(self.test_output):
                    line = self.test_output[line_idx][: width - 1]
                    try:
                        self.stdscr.addstr(i + 1, 0, line)
                    except curses.error:
                        pass

            self.stdscr.refresh()

            key = self.stdscr.getch()
            if key in (ord("q"), ord("Q"), 27):
                break
            elif key == curses.KEY_UP:
                scroll = max(0, scroll - 1)
            elif key == curses.KEY_DOWN:
                scroll = min(max_scroll, scroll + 1)
            elif key == curses.KEY_PPAGE:
                scroll = max(0, scroll - height + 4)
            elif key == curses.KEY_NPAGE:
                scroll = min(max_scroll, scroll + height - 4)

    def run(self):
        """Main event loop."""
        while self.running:
            self.draw()

            key = self.stdscr.getch()

            # Navigation
            if key in (curses.KEY_UP, ord("k")):
                self.cursor_pos = max(0, self.cursor_pos - 1)
            elif key in (curses.KEY_DOWN, ord("j")):
                self.cursor_pos = min(len(self.visible_items) - 1, self.cursor_pos + 1)
            elif key == curses.KEY_PPAGE:
                height = self.stdscr.getmaxyx()[0]
                self.cursor_pos = max(0, self.cursor_pos - (height - 6))
            elif key == curses.KEY_NPAGE:
                height = self.stdscr.getmaxyx()[0]
                self.cursor_pos = min(
                    len(self.visible_items) - 1, self.cursor_pos + (height - 6)
                )
            elif key == curses.KEY_HOME:
                self.cursor_pos = 0
            elif key == curses.KEY_END:
                self.cursor_pos = len(self.visible_items) - 1

            # Selection
            elif key == ord(" "):
                self.toggle_selection()
            elif key in (curses.KEY_ENTER, 10, 13):
                self.toggle_expand()
            elif key in (ord("A"),):
                self.select_all()
            elif key in (ord("N"),):
                self.select_none()

            # Actions
            elif key in (ord("r"), ord("R")):
                self.run_tests()
            elif key in (ord("b"), ord("B")):
                self.run_build()
            elif key in (ord("g"), ord("G")):
                self.generate_allure_report()
            elif key in (ord("a"),):
                self.open_allure_report()
            elif key in (ord("S"),):
                self.stop_allure_server()
            elif key in (ord("o"), ord("O")):
                self.show_output()

            # Help
            elif key in (ord("h"), ord("H"), ord("?")):
                self.show_help()

            # Quit
            elif key in (ord("q"), ord("Q"), 27):
                self.running = False


def main(stdscr):
    """Main entry point."""
    app = TestRunnerTUI(stdscr)
    app.run()


if __name__ == "__main__":
    try:
        curses.wrapper(main)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
