# Serial monitor for the STM32F769I-Discovery ST-LINK virtual COM port.
# Defaults match the firmware (main.cpp: USART1, 9600 baud, UART_WORDLENGTH_8B incl. ODD parity
# => 7 data bits + odd parity + 1 stop bit on the host side).
# Ctrl+C to quit.
param(
  [string]$Port = "",           # e.g. COM8; empty = auto-detect the ST-LINK VCP
  [int]$Baud = 9600,
  [int]$DataBits = 7,
  [ValidateSet("None", "Odd", "Even", "Mark", "Space")]
  [string]$Parity = "Odd",
  [ValidateSet("One", "Two", "OnePointFive")]
  [string]$StopBits = "One"
)

# Built into Windows PowerShell 5.1; a separate assembly in PowerShell 7
try { Add-Type -AssemblyName System.IO.Ports -ErrorAction Stop } catch {}

if (-not $Port) {
  $dev = Get-CimInstance Win32_PnPEntity -Filter "Name LIKE '%(COM%'" |
    Where-Object { $_.Name -match 'STLink|ST-Link|STMicroelectronics' } |
    Select-Object -First 1
  if (-not $dev) {
    Write-Host "ST-LINK virtual COM port not found. Is the board plugged in? Use -Port COMx to override." -ForegroundColor Red
    exit 1
  }
  $Port = [regex]::Match($dev.Name, 'COM\d+').Value
}

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, ([System.IO.Ports.Parity]::$Parity), $DataBits, ([System.IO.Ports.StopBits]::$StopBits)
$sp.ReadTimeout = 200
$sp.Encoding = [System.Text.Encoding]::UTF8

# The port may be briefly unavailable right after flashing/reset: retry for a few seconds
$deadline = (Get-Date).AddSeconds(10)
while ($true) {
  try { $sp.Open(); break }
  catch {
    if ((Get-Date) -gt $deadline) {
      Write-Host "Cannot open ${Port}: $($_.Exception.Message)" -ForegroundColor Red
      exit 1
    }
    Start-Sleep -Milliseconds 500
  }
}

Write-Host "--- Monitoring $Port ($Baud $DataBits$($Parity.Substring(0,1))$(@{One='1';Two='2';OnePointFive='1.5'}[$StopBits])) - Ctrl+C to quit ---" -ForegroundColor Cyan
try {
  while ($true) {
    $data = $sp.ReadExisting()
    if ($data) { [Console]::Write($data) }
    else { Start-Sleep -Milliseconds 20 }
  }
}
finally {
  $sp.Close()
  Write-Host "`n--- $Port closed ---" -ForegroundColor Cyan
}
