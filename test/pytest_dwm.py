import re

_ANSI = re.compile(rb"\x1b\[[0-9;]*m")
_CASE = re.compile(r"^[^:\r\n]+:\d+:(?P<name>.+):(?P<result>PASS|FAIL|IGNORE)\b", re.M)


def test_dwm(dut, subtests):
    # the app is unity_run_menu(); "*" runs the whole suite, printing one
    # "<file>:<line>:<name>:PASS|FAIL" line per case. Report each as a subtest so
    # every case shows up individually (host + device, no per-case menu reset).
    dut.expect_exact("Press ENTER to see the list of tests")
    dut.write("*")
    dut.expect(rb"\d+ Tests \d+ Failures \d+ Ignored")

    output = _ANSI.sub(b"", dut.pexpect_proc.before).decode(errors="replace")
    cases = _CASE.findall(output)
    assert cases, "no unity test cases parsed from output"

    for name, result in cases:
        with subtests.test(msg=name):
            assert result == "PASS", f"{name} -> {result}"
