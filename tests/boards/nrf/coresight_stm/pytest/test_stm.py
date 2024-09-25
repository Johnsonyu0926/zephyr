#
# Copyright (c) 2024 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: Apache-2.0
#

import logging
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path
from time import sleep
from typing import Optional

import psutil
from twister_harness import DeviceAdapter

logger = logging.getLogger(__name__)


@dataclass
class STMLimits:
    log_0_arg: Optional[float]
    log_1_arg: Optional[float]
    log_2_arg: Optional[float]
    log_3_arg: Optional[float]
    log_str: Optional[float]
    tracepoint: Optional[float]
    tracepoint_d32: Optional[float]
    tolerance: Optional[float]


# https://github.com/zephyrproject-rtos/zephyr/blob/main/drivers/misc/coresight/nrf_etr.c#L102
STM_M_ID = {
    "sec": 33,
    "app": 34,
    "rad": 35,
    "mod": 36,
    "sys": 44,
    "flpr": 45,
    "ppr": 46,
    "hw": 128,
}


def _check_benchmark_results(output: str, core: str, constraints: STMLimits) -> None:
    """
    Use regular expressions to parse 'output' string.
    Search for benchmark results related to 'core' coprocessor.
    Check that benchamrk results are lower than limits provided in 'constraints'.
    """

    latency_msg_0_str = re.search(
        rf"{core}: Timing for log message with 0 arguments: (.+)us", output
    ).group(1)
    assert latency_msg_0_str is not None, "Timing for log message with 0 arguments NOT found"
    # check value
    latency_msg_0 = float(latency_msg_0_str)
    threshold = (constraints.log_0_arg) * (1 + constraints.tolerance)
    assert (
        latency_msg_0 < threshold
    ), f"Timing for log message with 0 arguments {latency_msg_0} us exceeds threshold {threshold} us"


    latency_msg_1_str = re.search(
        rf"{core}: Timing for log message with 1 argument: (.+)us", output
    ).group(1)
    assert latency_msg_1_str is not None, "Timing for log message with 1 argument NOT found"
    # check value
    latency_msg_1 = float(latency_msg_1_str)
    threshold = (constraints.log_1_arg) * (1 + constraints.tolerance)
    assert (
        latency_msg_1 < threshold
    ), f"Timing for log message with 1 argument {latency_msg_1} us exceeds threshold {threshold} us"


    latency_msg_2_str = re.search(
        rf"{core}: Timing for log message with 2 arguments: (.+)us", output
    ).group(1)
    assert latency_msg_2_str is not None, "Timing for log message with 2 arguments NOT found"
    # check value
    latency_msg_2 = float(latency_msg_2_str)
    threshold = (constraints.log_2_arg) * (1 + constraints.tolerance)
    assert (
        latency_msg_2 < threshold
    ), f"Timing for log message with 2 arguments {latency_msg_2} us exceeds threshold {threshold} us"


    latency_msg_3_str = re.search(
        rf"{core}: Timing for log message with 3 arguments: (.+)us", output
    ).group(1)
    assert latency_msg_3_str is not None, "Timing for log message with 3 arguments NOT found"
    # check value
    latency_msg_3 = float(latency_msg_3_str)
    threshold = (constraints.log_3_arg) * (1 + constraints.tolerance)
    assert (
        latency_msg_3 < threshold
    ), f"Timing for log message with 3 arguments {latency_msg_3} us exceeds threshold {threshold} us"


    latency_msg_string_str = re.search(
        rf"{core}: Timing for log_message with string: (.+)us", output
    ).group(1)
    assert latency_msg_string_str is not None, "Timing for log_message with string NOT found"
    # check value
    latency_msg_string = float(latency_msg_string_str)
    threshold = (constraints.log_str) * (1 + constraints.tolerance)
    assert (
        latency_msg_string < threshold
    ), f"Timing for log message with string {latency_msg_string} us exceeds threshold {threshold} us"


    latency_tracepoint_str = re.search(
        rf"{core}: Timing for tracepoint: (.+)us", output
    ).group(1)
    assert latency_tracepoint_str is not None, "Timing for tracepoint NOT found"
    # check value
    latency_tracepoint = float(latency_tracepoint_str)
    threshold = (constraints.tracepoint) * (1 + constraints.tolerance)
    assert (
        latency_tracepoint < threshold
    ), f"Timing for tracepoint {latency_tracepoint} us exceeds threshold {threshold} us"


    latency_tracepoint_d32_str = re.search(
        rf"{core}: Timing for tracepoint_d32: (.+)us", output
    ).group(1)
    assert latency_tracepoint_d32_str is not None, "Timing for tracepoint_d32 NOT found"
    # check value
    latency_tracepoint_d32 = float(latency_tracepoint_d32_str)
    threshold = (constraints.tracepoint_d32) * (1 + constraints.tolerance)
    assert (
        latency_tracepoint_d32 < threshold
    ), f"Timing for tracepoint_d32 {latency_tracepoint_d32} us exceeds threshold {threshold} us"


# nrfutil starts children processes
# when subprocess.terminate(nrfutil_process) is executed, only the parent terminates
# this blocks serial port for other uses
def _kill(proc):
    try:
        for child in psutil.Process(proc.pid).children(recursive=True):
            child.kill()
        proc.kill()
    except Exception as e:
        logger.exception(f'Could not kill nrfutil - {e}')


def _nrfutil_dictionary_from_serial(
    dut: DeviceAdapter,
    decoded_file_name: str = "output.txt",
    collect_time: float = 60.0,
) -> None:
    UART_PATH = dut.device_config.serial
    UART_BAUDRATE = dut.device_config.baud
    dut.close()

    logger.debug(f"Using serial: {UART_PATH}")

    if Path(f"{decoded_file_name}").exists():
        logger.warning("Output file already exists!")

    # prepare database config string
    BUILD_DIR = str(dut.device_config.build_dir)
    logger.debug(f"{BUILD_DIR=}")
    config_str = f"{STM_M_ID['app']}:{BUILD_DIR}/coresight_stm/zephyr/log_dictionary.json"
    config_str = config_str + f",{STM_M_ID['rad']}:{BUILD_DIR}/remote_1/zephyr/log_dictionary.json"
    config_str = config_str + f",{STM_M_ID['ppr']}:{BUILD_DIR}/remote_2/zephyr/log_dictionary.json"
    logger.debug(f"{config_str=}")

    cmd = (
        "nrfutil trace stm --database-config "
        f"{config_str} "
        f"--input-serialport {UART_PATH} --baudrate {UART_BAUDRATE} "
        f"--output-ascii {decoded_file_name}"
    )
    try:
        # run nrfutil trace in background non-blocking
        logger.info(f"Executing:\n{cmd}")
        proc = subprocess.Popen(cmd.split(), stdout=subprocess.DEVNULL)
    except OSError as exc:
        logger.error(f"Unable to start nrfutil trace:\n{cmd}\n{exc}")
    try:
        proc.wait(collect_time)
    except subprocess.TimeoutExpired:
        pass
    finally:
        _kill(proc)


def test_STM_decoded(dut: DeviceAdapter):
    """
    Run sample.boards.nrf.coresight_stm from samples/boards/nrf/coresight_stm sample.
    Both Application and Radio cores use STM for logging.
    STM proxy (Application core) decodes logs from all domains.
    After reset, coprocessors execute code in expected way and Application core
    outputs STM traces on UART port.
    """
    app_constraints = STMLimits(
        # all values in us
        log_0_arg=1.7,
        log_1_arg=1.9,
        log_2_arg=2.0,
        log_3_arg=2.1,
        log_str=4.5,
        tracepoint=0.5,
        tracepoint_d32=0.5,
        tolerance=0.5,  # 50 %
    )
    rad_constraints = STMLimits(
        # all values in us
        log_0_arg=5.1,
        log_1_arg=5.8,
        log_2_arg=6.0,
        log_3_arg=6.4,
        log_str=7.1,
        tracepoint=0.5,
        tracepoint_d32=0.5,
        tolerance=0.5,
    )
    ppr_constraints = STMLimits(
        # all values in us
        log_0_arg=25.4,
        log_1_arg=26.3,
        log_2_arg=27.0,
        log_3_arg=59.4,
        log_str=111.8,
        tracepoint=1.7,
        tracepoint_d32=1.65,
        tolerance=0.5,
    )
    # nrf54h20 prints immediately after it is flashed.
    # Wait a bit to skipp logs from previous test.
    sleep(4)

    # Get output from serial port
    output = "\n".join(dut.readlines())

    # check results on Application core
    _check_benchmark_results(
        output=output,
        core='app',
        constraints=app_constraints
    )

    # check results on Radio core
    _check_benchmark_results(
        output=output,
        core='rad',
        constraints=rad_constraints
    )

    # check results on PPR core
    _check_benchmark_results(
        output=output,
        core='ppr',
        constraints=ppr_constraints
    )


def test_STM_dictionary_mode(dut: DeviceAdapter):
    """
    Run sample.boards.nrf.coresight_stm.dict from samples/boards/nrf/coresight_stm sample.
    Both Application and Radio cores use STM for logging.
    STM proxy (Application core) prints on serial port raw logs from all domains.
    Nrfutil trace is used to decode STM logs.
    After reset, coprocessors execute code in expected way and Application core
    outputs STM traces on UART port.
    """
    BUILD_DIR = str(dut.device_config.build_dir)
    test_filename = f"{BUILD_DIR}/coresight_stm_dictionary.txt"
    COLLECT_TIMEOUT = 10.0
    app_constraints = STMLimits(
        # all values in us
        log_0_arg=0.7,
        log_1_arg=0.8,
        log_2_arg=0.9,
        log_3_arg=1.5,
        log_str=3.4,
        tracepoint=0.5,
        tracepoint_d32=0.5,
        tolerance=0.5,  # 50 %
    )
    rad_constraints = STMLimits(
        # all values in us
        log_0_arg=0.8,
        log_1_arg=0.9,
        log_2_arg=1.0,
        log_3_arg=1.9,
        log_str=4.2,
        tracepoint=0.5,
        tracepoint_d32=0.5,
        tolerance=0.5,
    )

    ppr_constraints = STMLimits(
        # all values in us
        log_0_arg=7.8,
        log_1_arg=8.2,
        log_2_arg=8.4,
        log_3_arg=56.8,
        log_str=82.9,
        tracepoint=1.6,
        tracepoint_d32=1.6,
        tolerance=0.5,
    )

    # use nrfutil trace to decode logs
    _nrfutil_dictionary_from_serial(
        dut=dut,
        decoded_file_name=f"{test_filename}",
        collect_time=COLLECT_TIMEOUT,
    )

    # read decoded logs
    with open(f"{test_filename}", errors="ignore") as decoded_file:
        decoded_file_content = decoded_file.read()

    # if nothing in decoded_file, stop test
    assert(
        len(decoded_file_content) > 0
    ), f"File {test_filename} is empty"

    # check results on Application core
    _check_benchmark_results(
        output=decoded_file_content,
        core='app',
        constraints=app_constraints
    )

    # check results on Radio core
    _check_benchmark_results(
        output=decoded_file_content,
        core='rad',
        constraints=rad_constraints
    )

    # check results on PPR core
    _check_benchmark_results(
        output=decoded_file_content,
        core='ppr',
        constraints=ppr_constraints
    )
