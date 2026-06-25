# Windows Setup and RobStride Bench Test Guide

This guide is for testing RobStride03, RobStride04, and RobStride06 from a
Windows PC using this fork of `actuator`.

The first test must be communication/feedback only. Do not enable torque, set
zero, write parameters, or command motion until the CAN adapter and motor
feedback are confirmed.

## 1. Install Required Software

Install these on Windows:

1. **Git for Windows**
   - https://git-scm.com/download/win

2. **Python 3.12 64-bit**
   - https://www.python.org/downloads/windows/
   - Check "Add python.exe to PATH" during install.
   - Python 3.11 is also OK. Avoid Python 3.14 for now.

3. **Rust**
   - https://rustup.rs/
   - Install the default stable MSVC toolchain.

4. **Visual Studio Build Tools 2022**
   - https://visualstudio.microsoft.com/visual-cpp-build-tools/
   - Select: `Desktop development with C++`
   - This is needed for Rust/PyO3 Python extension builds on Windows.

5. **USB-CAN adapter driver**
   - If your adapter appears as a COM port, Windows has loaded a serial driver.
   - For CH340/CH341 adapters, install the WCH driver if Windows does not show a COM port.
   - If your USB-CAN adapter has vendor software, install it too. It can help identify bitrate/protocol.

## 2. Clone This Fork and Branch

Open PowerShell:

```powershell
cd $HOME\Documents
git clone https://github.com/AlexanderEkeberg/actuator.git
cd actuator
git checkout robstride06-bench
```

Verify that the RS06 commit is present:

```powershell
git log --oneline -5
```

You should see:

```text
Add RobStride06 support and safe radian-native bindings
```

## 3. Create Python Virtual Environment

From the repo root:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\activate
```

If Python 3.12 is not installed, use Python 3.11:

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\activate
```

Upgrade build tools:

```powershell
python -m pip install -U pip setuptools wheel
python -m pip install setuptools-rust pytest pyserial
```

Install this package in editable mode:

```powershell
python -m pip install -e ".[dev]"
```

Check import:

```powershell
python -c "import actuator; print(actuator.__version__)"
```

## 4. Rust Build and Tests

Run:

```powershell
cargo build
cargo test
cargo build -p bindings --features extension-module
```

Run Python tests:

```powershell
pytest tests/test_robstride.py
```

These tests do not move motors and do not require hardware.

## 5. Find the USB-CAN COM Port

Plug in the USB-CAN adapter.

In PowerShell:

```powershell
python -m serial.tools.list_ports -v
```

Look for a port like:

```text
COM3
COM4
COM5
```

You can also check Device Manager:

```text
Device Manager -> Ports (COM & LPT)
```

Use the COM name in test commands, for example `COM3`.

## 6. Safe Wiring Checklist

Keep battery disconnected while wiring.

Power:

```text
Battery/XT60 red   -> motor thick red   = power +
Battery/XT60 black -> motor thick black = power - / GND
```

CAN:

```text
USB-CAN CANH -> motor CANH
USB-CAN CANL -> motor CANL
USB-CAN GND  -> battery/motor minus, same point as motor thick black
```

If using a 3-port Wago on minus/GND:

```text
Black Wago:
- XT60/battery black
- motor thick black
- USB-CAN GND
```

Do not connect USB-CAN GND to CANH, CANL, or battery plus.

Before connecting the battery:

```text
- No loose copper strands
- No short between power red and black
- CANH/CANL not connected to battery plus or minus
- CANH-to-CANL resistance is roughly 60-120 ohm
```

Your measured `119 ohm` is acceptable for a short bench test.

## 7. First Feedback Test

Do not enable torque. Do not command motion.

For RobStride06, using motor ID 1 and port COM3:

```powershell
python -m examples.supervisor --port-name COM3 --motor-id 1 --motor-type 6
```

For RobStride04:

```powershell
python -m examples.supervisor --port-name COM3 --motor-id 1 --motor-type 4
```

For RobStride03:

```powershell
python -m examples.supervisor --port-name COM3 --motor-id 1 --motor-type 3
```

Stop with:

```text
Ctrl+C
```

Let the first test run only 5-10 seconds.

Expected successful output should contain a non-empty state list. Empty output:

```text
State (rad, rad/s, Nm): []
```

means the script is running but has not received cached feedback for that motor ID.

## 8. If You Still Get `[]`

Check in this order:

1. Correct COM port.
2. Motor has power.
3. USB-CAN GND is connected to battery/motor minus.
4. CANH/CANL are not swapped.
5. CANH-to-CANL resistance is around 60-120 ohm with battery disconnected.
6. Try a few motor IDs.
7. Confirm the USB-CAN adapter protocol is supported.

Try swapped CANH/CANL:

```text
USB-CAN CANH -> motor other CAN wire
USB-CAN CANL -> motor first CAN wire
```

Try IDs one by one:

```powershell
python -m examples.supervisor --port-name COM3 --motor-id 0 --motor-type 6
python -m examples.supervisor --port-name COM3 --motor-id 1 --motor-type 6
python -m examples.supervisor --port-name COM3 --motor-id 2 --motor-type 6
python -m examples.supervisor --port-name COM3 --motor-id 127 --motor-type 6
```

## 9. Important USB-CAN Adapter Note

This repo does not support every USB-CAN adapter automatically.

On Windows/macOS, the current path uses `CH341Transport`, which expects a
specific serial protocol:

```text
921600 baud
"AT" + extended CAN id + length + data + "\r\n"
```

Many USB-CAN adapters show up as a COM port but use a different protocol, such as
SLCAN, candleLight, Waveshare/vendor protocol, or a proprietary binary protocol.
If the adapter protocol does not match `CH341Transport`, the COM port opens but
the motor never receives valid CAN frames.

If RS03, RS04, and RS06 all return `[]`, and wiring/termination/GND are correct,
the most likely issue is the USB-CAN adapter protocol.

## 10. Things Not To Do Yet

Do not run any code that:

```text
- enables torque
- calls command_actuators
- sets zero position
- writes parameters
- commands position/velocity/torque motion
```

RS06 constants are provisional and must be hardware-validated before dynamic
motion.

## 11. Units

The Python API in this branch is radian-native:

```text
position: rad
velocity: rad/s
torque: Nm
current: A
voltage: V
time: seconds
```

Degrees should only be used for display/debug helpers.

