from __future__ import annotations

from pathlib import Path
from datetime import datetime
import re

ROOT = Path(__file__).resolve().parent
MAIN = ROOT / "main" / "main.c"
TMP117 = ROOT / "components" / "i2c_with_tmp117" / "i2c_with_tmp117.c"
INA226 = ROOT / "components" / "INA226" / "INA226.c"
MODE = ROOT / "components" / "error_handler" / "mode_manager.c"
MONITOR = ROOT / "components" / "system_monitor" / "system_monitor.c"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="ignore")


def assert_contains(text: str, patterns: list[str], label: str) -> tuple[bool, list[str]]:
    missing: list[str] = []
    for p in patterns:
        if re.search(p, text, flags=re.MULTILINE) is None:
            missing.append(p)
    return (len(missing) == 0, missing)


def run_tests() -> list[tuple[str, bool, str]]:
    results: list[tuple[str, bool, str]] = []

    main = read(MAIN)
    tmp117 = read(TMP117)
    ina226 = read(INA226)
    mode = read(MODE)
    monitor = read(MONITOR)

    # Test 1: System bootstrap + module initialization chain
    ok, missing = assert_contains(
        main,
        [
            r"mode_manager_init\s*\(\s*\)",
            r"system_monitor_init\s*\(\s*\)",
            r"tmp117_init\s*\(\s*\)",
            r"ina226_init\s*\(\s*\)",
            r"pump_Operation_init\s*\(\s*\)",
        ],
        "bootstrap",
    )
    results.append((
        "T1 Bootstrap Integration",
        ok,
        "All core subsystems are initialized from initialize_system()." if ok else f"Missing init calls: {missing}",
    ))

    # Test 2: Callback registration and safety handlers
    ok_reg, missing_reg = assert_contains(
        main,
        [
            r"tmp117_register_error_callback\s*\(\s*on_sensor_error\s*\)",
            r"tmp117_register_overheat_callback\s*\(\s*on_sensor_overheat\s*\)",
            r"ina226_register_high_current_callback\s*\(\s*on_high_current_alert\s*\)",
        ],
        "callbacks",
    )
    ok_handlers, missing_handlers = assert_contains(
        main,
        [
            r"void\s+on_sensor_error\s*\([^\)]*\)\s*\{[\s\S]*mode_manager_set_mode\s*\(\s*MODE_ERROR\s*\)",
            r"void\s+on_sensor_overheat\s*\([^\)]*\)\s*\{[\s\S]*mode_manager_set_mode\s*\(\s*MODE_COOLDOWN\s*\)",
            r"void\s+on_high_current_alert\s*\([^\)]*\)\s*\{[\s\S]*mode_manager_set_mode\s*\(\s*MODE_ERROR\s*\)",
        ],
        "handlers",
    )
    ok = ok_reg and ok_handlers
    missing = missing_reg + missing_handlers
    results.append((
        "T2 Safety Callback Wiring",
        ok,
        "Callback registration and mode-transition handlers are wired." if ok else f"Missing callback logic: {missing}",
    ))

    # Test 3: Main FSM mode coverage
    ok, missing = assert_contains(
        main,
        [
            r"case\s+MODE_MANUAL_LOW",
            r"case\s+MODE_MANUAL_MEDIUM",
            r"case\s+MODE_MANUAL_HIGH",
            r"case\s+MODE_AUTOMATIC",
            r"case\s+MODE_COOLDOWN",
            r"case\s+MODE_ERROR",
            r"mode_manager_get_mode\s*\(\s*\)",
        ],
        "fsm",
    )
    results.append((
        "T3 State-Machine Coverage",
        ok,
        "All expected operating modes are present in app_main() switch-case." if ok else f"Missing mode branches: {missing}",
    ))

    # Test 4: Automatic task cadence and sensor pipeline
    ok, missing = assert_contains(
        main,
        [
            r"void\s+automatic_mode_task\s*\([^\)]*\)",
            r"tmp117_read_temperature\s*\(\s*\)",
            r"tmp117_read_temperature_device2\s*\(\s*\)",
            r"ina226_read_current\s*\(\s*\)",
            r"ina226_read_power\s*\(\s*\)",
            r"system_monitor_update\s*\(",
            r"vTaskDelay\s*\(\s*pdMS_TO_TICKS\s*\(\s*500\s*\)\s*\)",
        ],
        "auto",
    )
    results.append((
        "T4 Automatic Control Loop",
        ok,
        "Automatic task reads sensors and updates monitor every 500 ms." if ok else f"Missing automatic loop elements: {missing}",
    ))

    # Test 5: Remote sensor (I2C) access implementation details
    ok_tmp, miss_tmp = assert_contains(
        tmp117,
        [
            r"device_address\s*=\s*0x48",
            r"device_address\s*=\s*0x49",
            r"i2c_master_transmit_receive\s*\(",
        ],
        "tmp117_i2c",
    )
    ok_ina, miss_ina = assert_contains(
        ina226,
        [
            r"#define\s+INA226_I2C_ADDR\s+0x40",
            r"i2c_master_transmit_receive\s*\(",
            r"ina226_read_current\s*\(",
            r"ina226_read_power\s*\(",
        ],
        "ina226_i2c",
    )
    ok = ok_tmp and ok_ina
    missing = miss_tmp + miss_ina
    results.append((
        "T5 Remote Sensor Access Path",
        ok,
        "Remote I2C sensor paths for TMP117 (x2) and INA226 are implemented." if ok else f"Missing sensor access elements: {missing}",
    ))

    return results


def main_run() -> int:
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print("Software-Only System Test Log")
    print(f"Timestamp: {ts}")
    print(f"Project: {ROOT}")
    print("Hardware dependency: NONE (static software integration validation)")
    print("=" * 72)

    results = run_tests()
    passed = 0
    for name, ok, details in results:
        status = "PASS" if ok else "FAIL"
        if ok:
            passed += 1
        print(f"[{status}] {name}")
        print(f"  {details}")

    print("-" * 72)
    print(f"Summary: {passed}/{len(results)} tests passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main_run())
