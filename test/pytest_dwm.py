import re

_ANSI = re.compile(rb"\x1b\[[0-9;]*m")
_CASE = re.compile(r"^[^:\r\n]+:\d+:(?P<name>.+):(?P<result>PASS|FAIL|IGNORE)\b", re.M)


def _report(subtests, cases):
    for name, result in cases:
        with subtests.test(msg=name):
            assert result == "PASS", f"{name} -> {result}"


def test_dwm(dut, subtests, request):
    # device: run_all_single_board_cases drives the menu per-case (records
    # durations and attributes crashes) and fills dut.testsuite; replay those
    # into subtests so each case still prints individually. It hangs on the
    # linux target, so host runs the whole suite at once and parses it itself.
    if request.config.getoption("target") != "linux":
        try:
            dut.run_all_single_board_cases()
        finally:
            _report(subtests, [(c.name, c.result) for c in dut.testsuite.testcases])
        return

    dut.expect_exact("Press ENTER to see the list of tests")
    dut.write("*")
    dut.expect(rb"\d+ Tests \d+ Failures \d+ Ignored")

    output = _ANSI.sub(b"", dut.pexpect_proc.before).decode(errors="replace")
    cases = _CASE.findall(output)
    assert cases, "no unity test cases parsed from output"
    _report(subtests, cases)
