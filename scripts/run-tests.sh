#!/usr/bin/env bash
# Test runner script for Perseus V2
# Run this from the repository root or software/ros_ws directory

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_WS="$REPO_ROOT/software/ros_ws"
RESULTS_DIR="$REPO_ROOT/test-results"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
RESULTS_FILE="$RESULTS_DIR/test_results_${TIMESTAMP}.txt"
ALLURE_RESULTS_DIR="$RESULTS_DIR/allure-results"
ALLURE_REPORT_DIR="$RESULTS_DIR/allure-report"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
BOLD='\033[1m'
DIM='\033[2m'
NC='\033[0m' # No Color

# For file output (no colors)
print_to_file() {
    echo "$1" >> "$RESULTS_FILE"
}

print_header() {
    echo -e "\n${BLUE}${BOLD}═══════════════════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}${BOLD}  $1${NC}"
    echo -e "${BLUE}${BOLD}═══════════════════════════════════════════════════════════════${NC}\n"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

usage() {
    cat << EOF
Usage: $(basename "$0") [OPTIONS]

Run tests for the Perseus V2 project.

OPTIONS:
    -h, --help          Show this help message
    -b, --build         Build before running tests (default: skip if already built)
    -v, --verbose       Show verbose test output
    -p, --package PKG   Run tests only for a specific package
    -o, --output FILE   Write results to specific file (default: test-results/test_results_TIMESTAMP.txt)
    --no-lint           Skip linting tests (copyright, formatting, etc.)
    --clean             Clean build directory before building
    --no-build          Skip build even if packages are missing
    --html              Generate Allure HTML report (requires allure in PATH or nix-shell)

EXAMPLES:
    $(basename "$0")                        # Run all tests
    $(basename "$0") -b                     # Build first, then run tests
    $(basename "$0") -v                     # Run tests with verbose output
    $(basename "$0") -p perseus_simulation  # Test only perseus_simulation package
    $(basename "$0") --no-lint              # Skip lint tests, run only unit tests
    $(basename "$0") --html                 # Generate HTML report with Allure
    $(basename "$0") --clean -b             # Clean build, then test

RESULTS:
    Text results:  test-results/test_results_TIMESTAMP.txt
    Latest:        test-results/latest.txt
    HTML report:   test-results/allure-report/index.html (with --html)

EOF
}

# Parse arguments
BUILD=false
BUILD_ONLY=false
VERBOSE=false
CLEAN=false
NO_BUILD=false
NO_LINT=false
GENERATE_HTML=false
PACKAGE=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            usage
            exit 0
            ;;
        -b|--build)
            BUILD=true
            shift
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -p|--package)
            PACKAGE="$2"
            shift 2
            ;;
        -o|--output)
            RESULTS_FILE="$2"
            shift 2
            ;;
        --no-lint)
            NO_LINT=true
            shift
            ;;
        --html)
            GENERATE_HTML=true
            shift
            ;;
        --clean)
            CLEAN=true
            BUILD=true
            shift
            ;;
        --no-build)
            NO_BUILD=true
            shift
            ;;
        --build-only)
            BUILD=true
            BUILD_ONLY=true
            shift
            ;;
        *)
            print_error "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

# Create results directory
mkdir -p "$RESULTS_DIR"

# Initialize results file
cat > "$RESULTS_FILE" << EOF
================================================================================
PERSEUS V2 TEST RESULTS
================================================================================
Date: $(date)
Host: $(hostname)
User: $(whoami)
Branch: $(cd "$REPO_ROOT" && git branch --show-current 2>/dev/null || echo "unknown")
Commit: $(cd "$REPO_ROOT" && git rev-parse --short HEAD 2>/dev/null || echo "unknown")
================================================================================

EOF

# Change to ROS workspace
cd "$ROS_WS"

# Check if we're in a nix shell or have ROS2 available
if ! command -v colcon &> /dev/null; then
    print_error "colcon not found. Please enter the nix development shell first:"
    echo "    cd $ROS_WS && nix develop ./nix-packages"
    echo "  or from repo root:"
    echo "    nix develop ./software/ros_ws/nix-packages"
    exit 1
fi

# Clean if requested
if $CLEAN; then
    print_header "Cleaning build directories"
    rm -rf build install log
    print_success "Clean complete"
fi

# Check if build is needed
if [[ ! -d "build" ]] && ! $NO_BUILD; then
    print_warning "Build directory not found, building first..."
    BUILD=true
fi

# Build if requested or needed
if $BUILD; then
    print_header "Building packages"

    BUILD_ARGS=("--symlink-install" "--cmake-args" "-DBUILD_TESTING=ON")

    if [[ -n "$PACKAGE" ]]; then
        BUILD_ARGS+=("--packages-select" "$PACKAGE")
    fi

    colcon build "${BUILD_ARGS[@]}"
    print_success "Build complete"

    # Exit if build-only mode
    if [[ "$BUILD_ONLY" == "true" ]]; then
        echo ""
        echo -e "${GREEN}Build completed successfully.${NC}"
        exit 0
    fi
fi

# Source the workspace
if [[ -f "install/setup.bash" ]]; then
    source install/setup.bash
else
    print_error "install/setup.bash not found. Please build first with: $(basename "$0") -b"
    exit 1
fi

# Run tests
print_header "Running tests"

TEST_ARGS=()

if [[ -n "$PACKAGE" ]]; then
    TEST_ARGS+=("--packages-select" "$PACKAGE")
fi

if $VERBOSE; then
    TEST_ARGS+=("--event-handlers" "console_direct+")
fi

# Run colcon test (don't exit on failure, we want to show results)
set +e
colcon test "${TEST_ARGS[@]}"
TEST_STATUS=$?
set -e

# Parse and display results
print_header "Test Results"

# Arrays to store failure information for later display
declare -a FAILED_TESTS=()
declare -A FAILED_TEST_OUTPUT=()
declare -A FAILED_TEST_COMMAND=()
declare -A FAILED_TEST_PACKAGE=()

# Function to parse XML test results and create table
parse_and_display_results() {
    local total_tests=0
    local total_passed=0
    local total_failed=0
    local total_skipped=0
    local total_errors=0

    # Declare associative arrays for package results
    declare -A pkg_tests_map
    declare -A pkg_passed_map
    declare -A pkg_failed_map
    declare -A pkg_errors_map
    declare -A pkg_skipped_map

    # Parse CTest results (GTest unit tests) from Testing directories
    for pkg_testing_dir in build/*/Testing; do
        if [[ -d "$pkg_testing_dir" ]]; then
            pkg_name=$(basename "$(dirname "$pkg_testing_dir")")

            # Find the most recent Test.xml
            local latest_test_xml=""
            for test_dir in "$pkg_testing_dir"/*/; do
                if [[ -f "${test_dir}Test.xml" ]]; then
                    latest_test_xml="${test_dir}Test.xml"
                fi
            done

            if [[ -n "$latest_test_xml" && -f "$latest_test_xml" ]]; then
                local passed_count=0
                local failed_count=0

                if $NO_LINT; then
                    passed_count=$(grep -A1 'Status="passed"' "$latest_test_xml" 2>/dev/null | grep '<Name>' | grep -v -E -i 'copyright|cpplint|flake8|pep257|xmllint|lint_cmake|uncrustify|cppcheck' | wc -l) || passed_count=0
                    failed_count=$(grep -A1 'Status="failed"' "$latest_test_xml" 2>/dev/null | grep '<Name>' | grep -v -E -i 'copyright|cpplint|flake8|pep257|xmllint|lint_cmake|uncrustify|cppcheck' | wc -l) || failed_count=0
                else
                    passed_count=$(grep -c 'Status="passed"' "$latest_test_xml" 2>/dev/null) || passed_count=0
                    failed_count=$(grep -c 'Status="failed"' "$latest_test_xml" 2>/dev/null) || failed_count=0
                fi

                if [[ $((passed_count + failed_count)) -gt 0 ]]; then
                    pkg_tests_map[$pkg_name]=$((${pkg_tests_map[$pkg_name]:-0} + passed_count + failed_count))
                    pkg_passed_map[$pkg_name]=$((${pkg_passed_map[$pkg_name]:-0} + passed_count))
                    pkg_failed_map[$pkg_name]=$((${pkg_failed_map[$pkg_name]:-0} + failed_count))
                fi
            fi
        fi
    done

    # Parse ament test results (linting, etc.) from test_results directories
    if ! $NO_LINT; then
        for pkg_dir in build/*/test_results; do
            if [[ -d "$pkg_dir" ]]; then
                pkg_name=$(basename "$(dirname "$pkg_dir")")

                for xml_file in "$pkg_dir"/*/*.xml "$pkg_dir"/*.xml; do
                    if [[ -f "$xml_file" ]]; then
                        if grep -q 'testsuite' "$xml_file" 2>/dev/null; then
                            local tests=$(grep -oP 'tests="\K[0-9]+' "$xml_file" 2>/dev/null | head -1 || echo "0")
                            local failures=$(grep -oP 'failures="\K[0-9]+' "$xml_file" 2>/dev/null | head -1 || echo "0")
                            local errors=$(grep -oP 'errors="\K[0-9]+' "$xml_file" 2>/dev/null | head -1 || echo "0")
                            local skipped=$(grep -oP 'skipped="\K[0-9]+' "$xml_file" 2>/dev/null | head -1 || echo "0")

                            tests=${tests:-0}
                            failures=${failures:-0}
                            errors=${errors:-0}
                            skipped=${skipped:-0}

                            pkg_tests_map[$pkg_name]=$((${pkg_tests_map[$pkg_name]:-0} + tests))
                            pkg_failed_map[$pkg_name]=$((${pkg_failed_map[$pkg_name]:-0} + failures))
                            pkg_errors_map[$pkg_name]=$((${pkg_errors_map[$pkg_name]:-0} + errors))
                            pkg_skipped_map[$pkg_name]=$((${pkg_skipped_map[$pkg_name]:-0} + skipped))
                        fi
                    fi
                done
            fi
        done
    fi

    # Table header
    echo ""
    echo -e "${CYAN}${BOLD}┌───────────────────────────────────┬──────────┬──────────┬──────────┬──────────┬────────────┐${NC}"
    echo -e "${CYAN}${BOLD}│ Package                           │  Passed  │  Failed  │  Errors  │ Skipped  │   Status   │${NC}"
    echo -e "${CYAN}${BOLD}├───────────────────────────────────┼──────────┼──────────┼──────────┼──────────┼────────────┤${NC}"

    print_to_file "┌───────────────────────────────────┬──────────┬──────────┬──────────┬──────────┬────────────┐"
    print_to_file "│ Package                           │  Passed  │  Failed  │  Errors  │ Skipped  │   Status   │"
    print_to_file "├───────────────────────────────────┼──────────┼──────────┼──────────┼──────────┼────────────┤"

    for pkg_name in $(echo "${!pkg_tests_map[@]}" | tr ' ' '\n' | sort); do
        local pkg_tests=${pkg_tests_map[$pkg_name]:-0}
        local pkg_failed=${pkg_failed_map[$pkg_name]:-0}
        local pkg_errors=${pkg_errors_map[$pkg_name]:-0}
        local pkg_skipped=${pkg_skipped_map[$pkg_name]:-0}
        local pkg_passed=${pkg_passed_map[$pkg_name]:-0}

        if [[ $pkg_passed -eq 0 && $pkg_tests -gt 0 ]]; then
            pkg_passed=$((pkg_tests - pkg_failed - pkg_errors - pkg_skipped))
            if [[ $pkg_passed -lt 0 ]]; then pkg_passed=0; fi
        fi

        if [[ $pkg_tests -gt 0 ]]; then
            local status_text status_color
            if [[ $pkg_failed -eq 0 && $pkg_errors -eq 0 ]]; then
                status_text="PASS"
                status_color="${GREEN}"
            else
                status_text="FAIL"
                status_color="${RED}"
            fi

            local display_name="$pkg_name"
            if [[ ${#display_name} -gt 33 ]]; then
                display_name="${display_name:0:30}..."
            fi

            printf "${NC}│ %-33s │ %8d │ %8d │ %8d │ %8d │ ${status_color}%10s${NC} │\n" \
                "$display_name" "$pkg_passed" "$pkg_failed" "$pkg_errors" "$pkg_skipped" "$status_text"
            printf "│ %-33s │ %8d │ %8d │ %8d │ %8d │ %10s │\n" \
                "$display_name" "$pkg_passed" "$pkg_failed" "$pkg_errors" "$pkg_skipped" "$status_text" >> "$RESULTS_FILE"

            total_tests=$((total_tests + pkg_tests))
            total_passed=$((total_passed + pkg_passed))
            total_failed=$((total_failed + pkg_failed))
            total_errors=$((total_errors + pkg_errors))
            total_skipped=$((total_skipped + pkg_skipped))
        fi
    done

    echo -e "${CYAN}${BOLD}├───────────────────────────────────┼──────────┼──────────┼──────────┼──────────┼────────────┤${NC}"
    print_to_file "├───────────────────────────────────┼──────────┼──────────┼──────────┼──────────┼────────────┤"

    local total_status total_color
    if [[ $total_failed -eq 0 && $total_errors -eq 0 ]]; then
        total_status="PASS"
        total_color="${GREEN}${BOLD}"
    else
        total_status="FAIL"
        total_color="${RED}${BOLD}"
    fi

    printf "${BOLD}│ %-33s │ %8d │ %8d │ %8d │ %8d │ ${total_color}%10s${NC}${BOLD} │${NC}\n" \
        "TOTAL" "$total_passed" "$total_failed" "$total_errors" "$total_skipped" "$total_status"
    echo -e "${CYAN}${BOLD}└───────────────────────────────────┴──────────┴──────────┴──────────┴──────────┴────────────┘${NC}"

    printf "│ %-33s │ %8d │ %8d │ %8d │ %8d │ %10s │\n" \
        "TOTAL" "$total_passed" "$total_failed" "$total_errors" "$total_skipped" "$total_status" >> "$RESULTS_FILE"
    print_to_file "└───────────────────────────────────┴──────────┴──────────┴──────────┴──────────┴────────────┘"

    # Summary
    echo ""
    echo -e "${BOLD}═══════════════════════════════════════════════════════════════${NC}"
    echo -e "${BOLD}                         SUMMARY${NC}"
    echo -e "${BOLD}═══════════════════════════════════════════════════════════════${NC}"
    echo ""
    echo -e "  Total Tests:    ${BOLD}$total_tests${NC}"
    echo -e "  Passed:         ${GREEN}${BOLD}$total_passed${NC}"
    [[ $total_failed -gt 0 ]] && echo -e "  Failed:         ${RED}${BOLD}$total_failed${NC}" || echo -e "  Failed:         ${BOLD}$total_failed${NC}"
    [[ $total_errors -gt 0 ]] && echo -e "  Errors:         ${RED}${BOLD}$total_errors${NC}" || echo -e "  Errors:         ${BOLD}$total_errors${NC}"
    echo -e "  Skipped:        ${YELLOW}${BOLD}$total_skipped${NC}"
    echo ""
    [[ $total_failed -eq 0 && $total_errors -eq 0 ]] && echo -e "  ${GREEN}${BOLD}✓ ALL TESTS PASSED${NC}" || echo -e "  ${RED}${BOLD}✗ SOME TESTS FAILED${NC}"
    echo ""
    echo -e "${BOLD}═══════════════════════════════════════════════════════════════${NC}"

    print_to_file ""
    print_to_file "═══════════════════════════════════════════════════════════════"
    print_to_file "                         SUMMARY"
    print_to_file "═══════════════════════════════════════════════════════════════"
    print_to_file ""
    print_to_file "  Total Tests:    $total_tests"
    print_to_file "  Passed:         $total_passed"
    print_to_file "  Failed:         $total_failed"
    print_to_file "  Errors:         $total_errors"
    print_to_file "  Skipped:        $total_skipped"
    print_to_file ""
    [[ $total_failed -eq 0 && $total_errors -eq 0 ]] && print_to_file "  ✓ ALL TESTS PASSED" || print_to_file "  ✗ SOME TESTS FAILED"
    print_to_file ""
    print_to_file "═══════════════════════════════════════════════════════════════"

    # Detailed test breakdown
    echo ""
    echo -e "${BOLD}═══════════════════════════════════════════════════════════════${NC}"
    echo -e "${BOLD}                      TEST DETAILS${NC}"
    echo -e "${BOLD}═══════════════════════════════════════════════════════════════${NC}"
    print_to_file ""
    print_to_file "═══════════════════════════════════════════════════════════════"
    print_to_file "                      TEST DETAILS"
    print_to_file "═══════════════════════════════════════════════════════════════"

    for pkg_testing_dir in build/*/Testing; do
        if [[ -d "$pkg_testing_dir" ]]; then
            local pkg_name=$(basename "$(dirname "$pkg_testing_dir")")
            local latest_test_xml=""
            for test_dir in "$pkg_testing_dir"/*/; do
                [[ -f "${test_dir}Test.xml" ]] && latest_test_xml="${test_dir}Test.xml"
            done

            if [[ -n "$latest_test_xml" && -f "$latest_test_xml" ]]; then
                local has_tests=false
                if $NO_LINT; then
                    grep -A1 'Status=' "$latest_test_xml" 2>/dev/null | grep '<Name>' | grep -v -E -i 'copyright|cpplint|flake8|pep257|xmllint|lint_cmake|uncrustify|cppcheck' | grep -q . && has_tests=true
                else
                    grep -q 'Status=' "$latest_test_xml" 2>/dev/null && has_tests=true
                fi

                if $has_tests; then
                    echo ""
                    echo -e "${CYAN}${BOLD}[$pkg_name]${NC}"
                    print_to_file ""
                    print_to_file "[$pkg_name]"

                    declare -A suite_tests suite_passed suite_failed
                    local current_status="" current_test_name="" current_command="" current_output=""
                    local in_measurement=false

                    while IFS= read -r line; do
                        if [[ "$line" =~ Status=\"([^\"]+)\" ]]; then
                            current_status="${BASH_REMATCH[1]}"
                        elif [[ "$line" =~ \<Name\>([^<]+)\</Name\> ]]; then
                            current_test_name="${BASH_REMATCH[1]}"
                        elif [[ "$line" =~ \<FullCommandLine\>([^<]+)\</FullCommandLine\> ]]; then
                            current_command="${BASH_REMATCH[1]}"
                        elif [[ "$line" =~ \<Measurement\> ]]; then
                            in_measurement=true
                            current_output=""
                        elif [[ "$line" =~ \</Measurement\> ]]; then
                            in_measurement=false
                        elif $in_measurement && [[ "$line" =~ \<Value\>(.*)\</Value\> ]]; then
                            current_output="${BASH_REMATCH[1]}"
                        elif [[ "$line" =~ \</Test\> ]]; then
                            if [[ -n "$current_test_name" ]]; then
                                if $NO_LINT && [[ "$current_test_name" =~ (copyright|cpplint|flake8|pep257|xmllint|lint_cmake|uncrustify|cppcheck) ]]; then
                                    current_test_name=""
                                    continue
                                fi

                                local suite_name="${current_test_name%%.*}"
                                local method_name="${current_test_name#*.}"
                                suite_tests[$suite_name]+="${method_name}|${current_status};"

                                [[ "$current_status" == "passed" ]] && suite_passed[$suite_name]=$((${suite_passed[$suite_name]:-0} + 1)) || suite_failed[$suite_name]=$((${suite_failed[$suite_name]:-0} + 1))

                                # Store failure info
                                if [[ "$current_status" == "failed" ]]; then
                                    FAILED_TESTS+=("$current_test_name")
                                    FAILED_TEST_OUTPUT["$current_test_name"]="$current_output"
                                    FAILED_TEST_COMMAND["$current_test_name"]="$current_command"
                                    FAILED_TEST_PACKAGE["$current_test_name"]="$pkg_name"
                                fi
                            fi
                            current_test_name=""
                            current_command=""
                            current_output=""
                        fi
                    done < "$latest_test_xml"

                    for suite in $(echo "${!suite_tests[@]}" | tr ' ' '\n' | sort); do
                        local passed=${suite_passed[$suite]:-0}
                        local failed=${suite_failed[$suite]:-0}
                        local total=$((passed + failed))

                        if [[ $failed -eq 0 ]]; then
                            echo -e "  ${GREEN}✓${NC} ${BOLD}$suite${NC} ($passed/$total passed)"
                            print_to_file "  ✓ $suite ($passed/$total passed)"
                        else
                            echo -e "  ${RED}✗${NC} ${BOLD}$suite${NC} ($passed/$total passed, $failed failed)"
                            print_to_file "  ✗ $suite ($passed/$total passed, $failed failed)"
                        fi

                        IFS=';' read -ra tests <<< "${suite_tests[$suite]}"
                        for test_entry in "${tests[@]}"; do
                            if [[ -n "$test_entry" ]]; then
                                local method="${test_entry%|*}"
                                local status="${test_entry#*|}"
                                if [[ "$status" == "passed" ]]; then
                                    echo -e "      ${GREEN}✓${NC} $method"
                                    print_to_file "      ✓ $method"
                                else
                                    echo -e "      ${RED}✗${NC} $method"
                                    print_to_file "      ✗ $method"
                                fi
                            fi
                        done
                    done
                    unset suite_tests suite_passed suite_failed
                fi
            fi
        fi
    done

    echo ""
    print_to_file ""

    # Return appropriate exit code
    [[ $total_failed -gt 0 || $total_errors -gt 0 ]] && return 1 || return 0
}

# Function to display failure details
display_failure_details() {
    if [[ ${#FAILED_TESTS[@]} -eq 0 ]]; then
        return
    fi

    echo ""
    echo -e "${RED}${BOLD}═══════════════════════════════════════════════════════════════${NC}"
    echo -e "${RED}${BOLD}                    FAILURE DETAILS${NC}"
    echo -e "${RED}${BOLD}═══════════════════════════════════════════════════════════════${NC}"
    print_to_file ""
    print_to_file "═══════════════════════════════════════════════════════════════"
    print_to_file "                    FAILURE DETAILS"
    print_to_file "═══════════════════════════════════════════════════════════════"

    for test_name in "${FAILED_TESTS[@]}"; do
        local pkg="${FAILED_TEST_PACKAGE[$test_name]}"
        local output="${FAILED_TEST_OUTPUT[$test_name]}"
        local command="${FAILED_TEST_COMMAND[$test_name]}"

        echo ""
        echo -e "${RED}${BOLD}✗ $test_name${NC}"
        echo -e "  ${DIM}Package: $pkg${NC}"
        print_to_file ""
        print_to_file "✗ $test_name"
        print_to_file "  Package: $pkg"

        # Extract and display failure message from output
        if [[ -n "$output" ]]; then
            # Decode HTML entities
            output=$(echo "$output" | sed 's/&lt;/</g; s/&gt;/>/g; s/&amp;/\&/g; s/&quot;/"/g')

            # Extract failure lines (GTest format)
            local failure_info=$(echo "$output" | grep -A5 -E '(FAILED|Expected|Actual|Value of|Which is)' | head -10)
            if [[ -n "$failure_info" ]]; then
                echo -e "  ${YELLOW}Failure:${NC}"
                print_to_file "  Failure:"
                while IFS= read -r line; do
                    echo -e "    ${DIM}$line${NC}"
                    print_to_file "    $line"
                done <<< "$failure_info"
            fi
        fi

        # Show re-run command
        if [[ -n "$command" ]]; then
            echo ""
            echo -e "  ${CYAN}To re-run this test:${NC}"
            echo -e "    ${BOLD}$command${NC}"
            print_to_file ""
            print_to_file "  To re-run this test:"
            print_to_file "    $command"
        fi

        echo ""
        print_to_file ""
    done
}

# Function to display next steps guidance
display_next_steps() {
    if [[ ${#FAILED_TESTS[@]} -eq 0 ]]; then
        return
    fi

    echo ""
    echo -e "${YELLOW}${BOLD}═══════════════════════════════════════════════════════════════${NC}"
    echo -e "${YELLOW}${BOLD}                      NEXT STEPS${NC}"
    echo -e "${YELLOW}${BOLD}═══════════════════════════════════════════════════════════════${NC}"
    echo ""
    echo -e "  ${BOLD}1.${NC} Review the failure details above to understand what went wrong"
    echo ""
    echo -e "  ${BOLD}2.${NC} Re-run specific failing tests using the commands shown above"
    echo ""
    echo -e "  ${BOLD}3.${NC} For more verbose output, run:"
    echo -e "     ${CYAN}./scripts/run-tests.sh -v --no-lint${NC}"
    echo ""
    echo -e "  ${BOLD}4.${NC} To run tests for a specific package:"
    echo -e "     ${CYAN}./scripts/run-tests.sh -p <package_name> --no-lint${NC}"
    echo ""
    echo -e "  ${BOLD}5.${NC} Test source files are located in:"
    echo -e "     ${CYAN}software/ros_ws/src/<package>/tests/${NC}"
    echo ""
    echo -e "  ${BOLD}6.${NC} For debugging, run the test executable directly with:"
    echo -e "     ${CYAN}./build/<package>/<test_executable> --gtest_filter=\"TestSuite.TestName\"${NC}"
    echo ""
    echo -e "${YELLOW}${BOLD}═══════════════════════════════════════════════════════════════${NC}"

    print_to_file ""
    print_to_file "═══════════════════════════════════════════════════════════════"
    print_to_file "                      NEXT STEPS"
    print_to_file "═══════════════════════════════════════════════════════════════"
    print_to_file ""
    print_to_file "  1. Review the failure details above to understand what went wrong"
    print_to_file ""
    print_to_file "  2. Re-run specific failing tests using the commands shown above"
    print_to_file ""
    print_to_file "  3. For more verbose output, run:"
    print_to_file "     ./scripts/run-tests.sh -v --no-lint"
    print_to_file ""
    print_to_file "  4. To run tests for a specific package:"
    print_to_file "     ./scripts/run-tests.sh -p <package_name> --no-lint"
    print_to_file ""
    print_to_file "  5. Test source files are located in:"
    print_to_file "     software/ros_ws/src/<package>/tests/"
    print_to_file ""
    print_to_file "  6. For debugging, run the test executable directly with:"
    print_to_file "     ./build/<package>/<test_executable> --gtest_filter=\"TestSuite.TestName\""
    print_to_file ""
    print_to_file "═══════════════════════════════════════════════════════════════"
}

# Function to generate Allure report
generate_allure_report() {
    if ! $GENERATE_HTML; then
        return
    fi

    print_header "Generating Allure HTML Report"

    # Check if allure is available
    local allure_cmd=""
    if command -v allure &> /dev/null; then
        allure_cmd="allure"
    else
        echo -e "${YELLOW}Allure not found in PATH. Attempting to use nix-shell...${NC}"
        if command -v nix-shell &> /dev/null; then
            allure_cmd="nix-shell -p allure --run allure"
        else
            print_error "Allure not available. Install it or use nix-shell."
            return 1
        fi
    fi

    # Prepare Allure results directory
    rm -rf "$ALLURE_RESULTS_DIR"
    mkdir -p "$ALLURE_RESULTS_DIR"

    echo -e "Running GTest executables to generate JUnit XML..."

    # Find and run GTest executables directly to get proper JUnit XML output
    # This ensures we get complete test results, not individual CTest entries
    local gtest_executables=(
        "perseus_input:axis_math_test"
        "perseus_input:controller_config_test"
        "perseus_lite:controller_config_test"
        "perseus_simulation:gz_bridge_config_test"
        "perseus_sensors:imu_device_config_test"
        "perseus_sensors:imu_calibration_test"
    )

    for entry in "${gtest_executables[@]}"; do
        local pkg_name="${entry%%:*}"
        local test_name="${entry##*:}"
        local test_exe="build/${pkg_name}/${test_name}"
        local build_dir="build/${pkg_name}"

        if [[ -x "$test_exe" ]]; then
            local output_file="$ALLURE_RESULTS_DIR/${pkg_name}_${test_name}.xml"
            echo -e "  Running ${CYAN}${pkg_name}/${test_name}${NC}..."
            # Run from the package build directory so relative paths work
            (cd "$build_dir" && "./${test_name}" --gtest_output=xml:"$output_file" > /dev/null 2>&1) || true
        fi
    done

    # Only copy lint test results if --no-lint was NOT specified
    if [[ "$NO_LINT" != "true" ]]; then
        for pkg_dir in build/*/test_results; do
            if [[ -d "$pkg_dir" ]]; then
                local pkg_name=$(basename "$(dirname "$pkg_dir")")
                for xml_file in "$pkg_dir"/*/*.xunit.xml "$pkg_dir"/*.xunit.xml; do
                    if [[ -f "$xml_file" ]]; then
                        local base_name=$(basename "$xml_file")
                        cp "$xml_file" "$ALLURE_RESULTS_DIR/${pkg_name}_${base_name}" 2>/dev/null || true
                    fi
                done
            fi
        done
    fi

    # Generate Allure report
    echo -e "Generating report from: ${CYAN}$ALLURE_RESULTS_DIR${NC}"

    if [[ "$allure_cmd" == "allure" ]]; then
        allure generate "$ALLURE_RESULTS_DIR" -o "$ALLURE_REPORT_DIR" --clean 2>/dev/null
    else
        nix-shell -p allure --run "allure generate '$ALLURE_RESULTS_DIR' -o '$ALLURE_REPORT_DIR' --clean" 2>/dev/null
    fi

    if [[ -f "$ALLURE_REPORT_DIR/index.html" ]]; then
        print_success "Allure report generated successfully"
        echo ""
        echo -e "  ${BOLD}HTML Report:${NC} ${CYAN}$ALLURE_REPORT_DIR/index.html${NC}"
        echo ""
        echo -e "  To view the report, run:"
        echo -e "    ${CYAN}xdg-open $ALLURE_REPORT_DIR/index.html${NC}"
        echo -e "  or:"
        if [[ "$allure_cmd" == "allure" ]]; then
            echo -e "    ${CYAN}allure open $ALLURE_REPORT_DIR${NC}"
        else
            echo -e "    ${CYAN}nix-shell -p allure --run \"allure open '$ALLURE_REPORT_DIR'\"${NC}"
        fi

        print_to_file ""
        print_to_file "HTML Report: $ALLURE_REPORT_DIR/index.html"
    else
        print_warning "Allure report generation may have encountered issues"
    fi
}

# Run the parser
parse_and_display_results
FINAL_STATUS=$?

# Display failure details if any
display_failure_details

# Display next steps if there were failures
display_next_steps

# Generate Allure report if requested
generate_allure_report

# Print results file location
echo ""
echo -e "Results saved to: ${CYAN}$RESULTS_FILE${NC}"
print_to_file ""
print_to_file "Results file: $RESULTS_FILE"

# Create symlink to latest results
ln -sf "$(basename "$RESULTS_FILE")" "$RESULTS_DIR/latest.txt"
echo -e "Latest results: ${CYAN}$RESULTS_DIR/latest.txt${NC}"

exit $FINAL_STATUS
