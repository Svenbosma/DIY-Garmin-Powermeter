from __future__ import annotations

import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


SOURCE_FILE = Path(
    r"C:\Users\Sven Bosma\Documents\DIY-Garmin-Powermeter\nRF52840_PowerMeter3\build\Seeeduino.nrf52.xiaonRF52840SensePlus\nRF52840_PowerMeter3.ino.zip"
)
CHECK_INTERVAL_SECONDS = 5
SEND_DELAY_SECONDS = 5


@dataclass(frozen=True)
class FileSignature:
    modified_ns: int
    size: int


def get_signature(path: Path) -> FileSignature | None:
    if not path.exists():
        return None

    stat = path.stat()
    return FileSignature(modified_ns=stat.st_mtime_ns, size=stat.st_size)


def run_powershell(script: str) -> None:
    subprocess.run(
        [
            "powershell",
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-Command",
            script,
        ],
        check=True,
    )


def send_file_via_phone_link(path: Path) -> None:
    if not path.exists():
        raise FileNotFoundError(f"File does not exist: {path}")

    escaped_path = str(path).replace("'", "''")
    powershell_script = f"""
Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName Microsoft.VisualBasic

function Get-PhoneLinkSendWindow {{
    $root = [Windows.Automation.AutomationElement]::RootElement
    $phoneCond = New-Object Windows.Automation.PropertyCondition(
        [Windows.Automation.AutomationElement]::NameProperty,
        'Phone Link'
    )
    $windows = $root.FindAll([Windows.Automation.TreeScope]::Children, $phoneCond)

    for ($i = 0; $i -lt $windows.Count; $i++) {{
        $window = $windows[$i]

        $selectFilesButton = $window.FindFirst(
            [Windows.Automation.TreeScope]::Descendants,
            (New-Object Windows.Automation.AndCondition(
                (New-Object Windows.Automation.PropertyCondition(
                    [Windows.Automation.AutomationElement]::ControlTypeProperty,
                    [Windows.Automation.ControlType]::Button
                )),
                (New-Object Windows.Automation.PropertyCondition(
                    [Windows.Automation.AutomationElement]::NameProperty,
                    'Select files'
                ))
            ))
        )

        $openDialog = $window.FindFirst(
            [Windows.Automation.TreeScope]::Descendants,
            (New-Object Windows.Automation.PropertyCondition(
                [Windows.Automation.AutomationElement]::NameProperty,
                'Open'
            ))
        )

        if ($selectFilesButton -or $openDialog) {{
            return $window
        }}
    }}

    throw 'Could not find the Phone Link "Send to phone" window.'
}}

function Get-OpenDialog($window) {{
    return $window.FindFirst(
        [Windows.Automation.TreeScope]::Descendants,
        (New-Object Windows.Automation.PropertyCondition(
            [Windows.Automation.AutomationElement]::NameProperty,
            'Open'
        ))
    )
}}

$path = '{escaped_path}'
$window = Get-PhoneLinkSendWindow
$openDialog = Get-OpenDialog $window

if (-not $openDialog) {{
    $selectFilesButton = $window.FindFirst(
        [Windows.Automation.TreeScope]::Descendants,
        (New-Object Windows.Automation.AndCondition(
            (New-Object Windows.Automation.PropertyCondition(
                [Windows.Automation.AutomationElement]::ControlTypeProperty,
                [Windows.Automation.ControlType]::Button
            )),
            (New-Object Windows.Automation.PropertyCondition(
                [Windows.Automation.AutomationElement]::NameProperty,
                'Select files'
            ))
        ))
    )

    if (-not $selectFilesButton) {{
        throw 'The "Select files" button was not found in Phone Link.'
    }}

    $invoke = $selectFilesButton.GetCurrentPattern([Windows.Automation.InvokePattern]::Pattern)
    $invoke.Invoke()

    $deadline = (Get-Date).AddSeconds(10)
    do {{
        Start-Sleep -Milliseconds 250
        $openDialog = Get-OpenDialog $window
    }} while (-not $openDialog -and (Get-Date) -lt $deadline)
}}

if (-not $openDialog) {{
    throw 'The file picker did not appear after clicking "Select files".'
}}

$wshell = New-Object -ComObject WScript.Shell
[Microsoft.VisualBasic.Interaction]::AppActivate('Phone Link') | Out-Null
Start-Sleep -Milliseconds 300
Set-Clipboard -Value $path
$wshell.SendKeys('%n')
Start-Sleep -Milliseconds 200
$wshell.SendKeys('^a')
Start-Sleep -Milliseconds 100
$wshell.SendKeys('^v')
Start-Sleep -Milliseconds 200
$wshell.SendKeys('~')
"""

    run_powershell(powershell_script)


def send_file_to_windows_share(path: Path) -> None:
    if not path.exists():
        raise FileNotFoundError(f"File does not exist: {path}")

    escaped_path = str(path).replace("'", "''")
    powershell_script = f"""
$path = '{escaped_path}'
$shell = New-Object -ComObject Shell.Application
$folder = $shell.Namespace((Split-Path $path))
$item = $folder.ParseName((Split-Path $path -Leaf))
if (-not $item) {{
    throw "Could not find the file in Shell.Application: $path"
}}
$shareVerb = $item.Verbs() | Where-Object {{ $_.Name.Replace('&', '') -eq 'Share' }} | Select-Object -First 1
if (-not $shareVerb) {{
    throw "The Windows Share action is not available for: $path"
}}
$shareVerb.DoIt()
"""

    run_powershell(powershell_script)


def monitor_file(path: Path) -> None:
    last_seen_signature = get_signature(path)

    print(f"Monitoring: {path}")
    if last_seen_signature is None:
        print("File is not there yet. Waiting for it to be created...")
    else:
        print(
            "Initial file state:"
            f" modified_ns={last_seen_signature.modified_ns},"
            f" size={last_seen_signature.size}"
        )

    while True:
        current_signature = get_signature(path)

        if current_signature != last_seen_signature:
            if current_signature is None:
                print("File was removed. Waiting for it to appear again...")
                last_seen_signature = None
            else:
                print(
                    "Change detected. Waiting"
                    f" {SEND_DELAY_SECONDS} seconds before sharing..."
                )
                time.sleep(SEND_DELAY_SECONDS)

                stable_signature = get_signature(path)
                if stable_signature is None:
                    print("File disappeared before it could be shared.")
                    last_seen_signature = None
                else:
                    print(
                        "Updated file detected:"
                        f" {path.name} (size={stable_signature.size})"
                    )
                    try:
                        print("Trying Phone Link automation...")
                        send_file_via_phone_link(path)
                        print("Phone Link automation triggered.")
                    except Exception as exc:
                        print(
                            "Phone Link automation failed, falling back to"
                            f" Windows Share. Reason: {exc}"
                        )
                        send_file_to_windows_share(path)
                        print(
                            "Share window opened. If Phone Link is available there,"
                            " choose your phone to finish sending."
                        )
                    last_seen_signature = stable_signature

        time.sleep(CHECK_INTERVAL_SECONDS)


def main() -> int:
    try:
        monitor_file(SOURCE_FILE)
    except KeyboardInterrupt:
        print("\nStopped by user.")
        return 0
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
