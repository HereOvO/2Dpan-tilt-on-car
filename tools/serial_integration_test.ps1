param(
    [string]$DebugPort = "COM19",
    [string]$CdcPort = "COM18",
    [string]$ProgrammerCli = "D:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
)

$ErrorActionPreference = "Stop"
$script:CdcRxBuffer = New-Object System.Collections.Generic.List[byte]
$script:AckParamValues = @{}

function Write-Log {
    param([string]$Message)
    $ts = Get-Date -Format "HH:mm:ss.fff"
    Write-Host "[$ts] $Message"
}

function Open-SerialPort {
    param([string]$PortName)
    $port = New-Object System.IO.Ports.SerialPort
    $port.PortName = $PortName
    $port.BaudRate = 115200
    $port.Parity = [System.IO.Ports.Parity]::None
    $port.DataBits = 8
    $port.StopBits = [System.IO.Ports.StopBits]::One
    $port.ReadTimeout = 50
    $port.WriteTimeout = 500
    $port.DtrEnable = $false
    $port.RtsEnable = $false
    $port.Open()
    return $port
}

function Open-SerialPortWithRetry {
    param(
        [string]$PortName,
        [int]$TimeoutMs = 5000,
        [int]$RetryIntervalMs = 200
    )

    $deadline = (Get-Date).AddMilliseconds($TimeoutMs)
    $lastError = $null

    while ((Get-Date) -lt $deadline) {
        try {
            return Open-SerialPort -PortName $PortName
        }
        catch {
            $lastError = $_
            Start-Sleep -Milliseconds $RetryIntervalMs
        }
    }

    if ($null -ne $lastError) {
        throw $lastError
    }

    throw "Failed to open serial port $PortName within ${TimeoutMs}ms"
}

function New-Frame {
    param(
        [byte]$MsgType,
        [byte]$Flags,
        [UInt16]$FrameId,
        [UInt32]$TimestampMs,
        [byte[]]$Payload
    )

    if ($null -eq $Payload) {
        $Payload = [byte[]]@()
    }

    $bytes = New-Object System.Collections.Generic.List[byte]
    $bytes.Add(0xAA)
    $bytes.Add(0x55)
    $bytes.Add($MsgType)
    $bytes.Add($Flags)
    $bytes.Add([byte]$Payload.Length)
    $bytes.AddRange([BitConverter]::GetBytes($FrameId))
    $bytes.AddRange([BitConverter]::GetBytes($TimestampMs))
    if ($Payload.Length -gt 0) {
        $bytes.AddRange($Payload)
    }

    [byte]$checksum = 0
    for ($i = 2; $i -lt $bytes.Count; $i++) {
        $checksum = $checksum -bxor $bytes[$i]
    }

    $bytes.Add($checksum)
    $bytes.Add(0x0D)
    return $bytes.ToArray()
}

function Send-Frame {
    param(
        [System.IO.Ports.SerialPort]$Port,
        [byte]$MsgType,
        [byte]$Flags,
        [UInt16]$FrameId,
        [UInt32]$TimestampMs,
        [byte[]]$Payload,
        [string]$Tag
    )

    $frame = New-Frame -MsgType $MsgType -Flags $Flags -FrameId $FrameId -TimestampMs $TimestampMs -Payload $Payload
    $Port.Write($frame, 0, $frame.Length)
    $hex = ($frame | ForEach-Object { $_.ToString("X2") }) -join " "
    Write-Log "TX $Tag => $hex"
}

function Decode-State {
    param([byte]$State)
    switch ($State) {
        0 { return "BOOT_CENTERING" }
        1 { return "STANDBY" }
        2 { return "TRACKING" }
        3 { return "HOLD_LAST" }
        default { return "UNKNOWN($State)" }
    }
}

function Decode-AckCode {
    param([byte]$Code)
    switch ($Code) {
        0 { return "OK" }
        1 { return "BAD_LENGTH" }
        2 { return "BAD_PARAM" }
        3 { return "BAD_CMD" }
        4 { return "BAD_STATE" }
        5 { return "BAD_MSG" }
        6 { return "BUFFER_TOO_SMALL" }
        default { return "UNKNOWN($Code)" }
    }
}

function Decode-ParamName {
    param([byte]$ParamId)
    switch ($ParamId) {
        0x01 { return "DEADBAND" }
        0x02 { return "MAX_STEP_US" }
        0x03 { return "KP_NUM" }
        0x04 { return "KP_DEN" }
        0x05 { return "CENTER_US" }
        0x06 { return "HOME_US" }
        0x07 { return "MIN_US" }
        0x08 { return "MAX_US" }
        0x09 { return "INVERT" }
        0x0A { return "KALMAN_ENABLE" }
        0x20 { return "STATUS_PERIOD_MS" }
        0x21 { return "TARGET_TIMEOUT_MS" }
        0x22 { return "BOOT_CENTER_MS" }
        0x23 { return "KALMAN_Q_MILLI" }
        0x24 { return "KALMAN_R_MILLI" }
        default { return ("UNKNOWN_PARAM_0x{0:X2}" -f $ParamId) }
    }
}

function Decode-AxisName {
    param([byte]$AxisId)
    switch ($AxisId) {
        0x00 { return "PAN" }
        0x01 { return "TILT" }
        0xFF { return "ALL" }
        default { return ("UNKNOWN_AXIS_0x{0:X2}" -f $AxisId) }
    }
}

function Parse-ParamRecords {
    param(
        [byte[]]$Detail,
        [string]$ContextTag = "ACK_DETAIL"
    )

    if ($null -eq $Detail -or $Detail.Length -eq 0) {
        Write-Log "$ContextTag no param records"
        return
    }

    if (($Detail.Length % 6) -ne 0) {
        $hex = ($Detail | ForEach-Object { $_.ToString("X2") }) -join " "
        Write-Log "$ContextTag malformed param detail len=$($Detail.Length) raw=$hex"
        return
    }

    for ($i = 0; $i -lt $Detail.Length; $i += 6) {
        $paramId = [byte]$Detail[$i]
        $axisId = [byte]$Detail[$i + 1]
        $value = [BitConverter]::ToInt32($Detail, $i + 2)
        $paramName = Decode-ParamName $paramId
        $axisName = Decode-AxisName $axisId
        $key = "{0:X2}:{1:X2}" -f $paramId, $axisId
        $script:AckParamValues[$key] = $value
        Write-Log "$ContextTag param=0x$($paramId.ToString('X2'))/$paramName axis=0x$($axisId.ToString('X2'))/$axisName value=$value"
    }
}

function Assert-ParamValue {
    param(
        [byte]$ParamId,
        [byte]$AxisId,
        [int]$ExpectedValue,
        [string]$SourceTag
    )

    $key = "{0:X2}:{1:X2}" -f $ParamId, $AxisId
    if (-not $script:AckParamValues.ContainsKey($key)) {
        throw "$SourceTag missing param record for key=$key"
    }

    $actual = [int]$script:AckParamValues[$key]
    if ($actual -ne $ExpectedValue) {
        throw "$SourceTag expected $ExpectedValue but got $actual for key=$key"
    }

    Write-Log "$SourceTag verified key=$key value=$actual"
}

function Clear-ParamValue {
    param(
        [byte]$ParamId,
        [byte]$AxisId
    )

    $key = "{0:X2}:{1:X2}" -f $ParamId, $AxisId
    if ($script:AckParamValues.ContainsKey($key)) {
        $script:AckParamValues.Remove($key) | Out-Null
    }
}

function Parse-CdcBuffer {
    param([System.Collections.Generic.List[byte]]$Buffer)

    while ($Buffer.Count -ge 13) {
        if (($Buffer[0] -ne 0xAA) -or ($Buffer[1] -ne 0x55)) {
            $Buffer.RemoveAt(0)
            continue
        }

        $payloadLen = [int]$Buffer[4]
        $totalLen = 13 + $payloadLen
        if ($Buffer.Count -lt $totalLen) {
            break
        }

        $frameBytes = $Buffer.GetRange(0, $totalLen).ToArray()
        $tail = $frameBytes[$totalLen - 1]
        if ($tail -ne 0x0D) {
            Write-Log ("RX CDC malformed tail => " + (($frameBytes | ForEach-Object { $_.ToString("X2") }) -join " "))
            $Buffer.RemoveAt(0)
            continue
        }

        [byte]$checksum = 0
        for ($i = 2; $i -lt ($totalLen - 2); $i++) {
            $checksum = $checksum -bxor $frameBytes[$i]
        }
        if ($checksum -ne $frameBytes[$totalLen - 2]) {
            Write-Log ("RX CDC checksum mismatch => " + (($frameBytes | ForEach-Object { $_.ToString("X2") }) -join " "))
            $Buffer.RemoveRange(0, $totalLen)
            continue
        }

        $msgType = $frameBytes[2]
        $flags = $frameBytes[3]
        $frameId = [BitConverter]::ToUInt16($frameBytes, 5)
        $ts = [BitConverter]::ToUInt32($frameBytes, 7)
        $payload = if ($payloadLen -gt 0) { $frameBytes[11..(10 + $payloadLen)] } else { [byte[]]@() }

        $hex = ($frameBytes | ForEach-Object { $_.ToString("X2") }) -join " "
        Write-Log "RX CDC raw <= $hex"

        switch ($msgType) {
            0x81 {
                $state = Decode-State $payload[0]
                $statusFlags = $payload[1]
                $lastFrameId = [BitConverter]::ToUInt16($payload, 2)
                $errX = [BitConverter]::ToInt16($payload, 4)
                $errY = [BitConverter]::ToInt16($payload, 6)
                $panUs = [BitConverter]::ToUInt16($payload, 8)
                $tiltUs = [BitConverter]::ToUInt16($payload, 10)
                Write-Log "RX STATUS frame_id=$frameId state=$state flags=0x$($statusFlags.ToString('X2')) last_track=$lastFrameId err=($errX,$errY) pan_us=$panUs tilt_us=$tiltUs"
            }
            0x82 {
                $ackedType = $payload[0]
                $ackCodeByte = $payload[1]
                $ackCode = Decode-AckCode $ackCodeByte
                $detail = if ($payloadLen -gt 2) { [byte[]]$payload[2..($payloadLen - 1)] } else { [byte[]]@() }
                Write-Log "RX ACK frame_id=$frameId acked_type=0x$($ackedType.ToString('X2')) code=$ackCode detail_len=$($payloadLen - 2)"
                if (($ackCodeByte -eq 0) -and (($ackedType -eq 0x02) -or ($ackedType -eq 0x03))) {
                    Parse-ParamRecords -Detail $detail -ContextTag "RX ACK PARAM"
                }
            }
            default {
                Write-Log "RX frame type=0x$($msgType.ToString('X2')) flags=0x$($flags.ToString('X2')) frame_id=$frameId payload_len=$payloadLen"
            }
        }

        $Buffer.RemoveRange(0, $totalLen)
    }
}

function Drain-Ports {
    param(
        [System.IO.Ports.SerialPort]$DebugPortObj,
        [System.IO.Ports.SerialPort]$CdcPortObj,
        [int]$DurationMs
    )

    $deadline = (Get-Date).AddMilliseconds($DurationMs)

    while ((Get-Date) -lt $deadline) {
        if ($null -ne $DebugPortObj -and $DebugPortObj.IsOpen -and $DebugPortObj.BytesToRead -gt 0) {
            $text = $DebugPortObj.ReadExisting()
            if (-not [string]::IsNullOrEmpty($text)) {
                $text = $text.Replace("`r", "\r").Replace("`n", "\n")
                Write-Log "RX DBG <= $text"
            }
        }

        if ($null -ne $CdcPortObj -and $CdcPortObj.IsOpen -and $CdcPortObj.BytesToRead -gt 0) {
            $count = $CdcPortObj.BytesToRead
            $buf = New-Object byte[] $count
            $read = $CdcPortObj.Read($buf, 0, $count)
            if ($read -gt 0) {
                if ($read -lt $buf.Length) {
                    $buf = $buf[0..($read - 1)]
                }
                $script:CdcRxBuffer.AddRange($buf)
                Parse-CdcBuffer -Buffer $script:CdcRxBuffer
            }
        }

        Start-Sleep -Milliseconds 20
    }
}

function Reset-Board {
    param([string]$CliPath)
    Write-Log "Issuing STM32 software reset"
    & $CliPath -c port=SWD -rst | Out-Null
}

$debugSerial = $null
$cdcSerial = $null

try {
    $script:CdcRxBuffer.Clear()
    $script:AckParamValues.Clear()
    $debugSerial = Open-SerialPort -PortName $DebugPort

    $debugSerial.DiscardInBuffer()

    Reset-Board -CliPath $ProgrammerCli
    Drain-Ports -DebugPortObj $debugSerial -CdcPortObj $null -DurationMs 1500

    Write-Log "Opening CDC port after reset"
    $cdcSerial = Open-SerialPortWithRetry -PortName $CdcPort -TimeoutMs 8000 -RetryIntervalMs 250
    $cdcSerial.DiscardInBuffer()
    $cdcSerial.DiscardOutBuffer()
    Drain-Ports -DebugPortObj $debugSerial -CdcPortObj $cdcSerial -DurationMs 300

    $tick = [UInt32][Environment]::TickCount

    Send-Frame -Port $cdcSerial -MsgType 0x04 -Flags 0x00 -FrameId 1 -TimestampMs $tick -Payload ([byte[]]@(0x05)) -Tag "FORCE_STATUS"
    Drain-Ports -DebugPortObj $debugSerial -CdcPortObj $cdcSerial -DurationMs 700

    Send-Frame -Port $cdcSerial -MsgType 0x03 -Flags 0x00 -FrameId 2 -TimestampMs ($tick + 1) -Payload ([byte[]]@()) -Tag "PARAM_GET_ALL"
    Drain-Ports -DebugPortObj $debugSerial -CdcPortObj $cdcSerial -DurationMs 1000

    $trackValidPayload = New-Object System.Collections.Generic.List[byte]
    $trackValidPayload.AddRange([BitConverter]::GetBytes([Int16]200))
    $trackValidPayload.AddRange([BitConverter]::GetBytes([Int16](-160)))
    Send-Frame -Port $cdcSerial -MsgType 0x01 -Flags 0x07 -FrameId 3 -TimestampMs ($tick + 2) -Payload ($trackValidPayload.ToArray()) -Tag "TRACK_VALID"
    Drain-Ports -DebugPortObj $debugSerial -CdcPortObj $cdcSerial -DurationMs 1200

    $trackInvalidPayload = New-Object System.Collections.Generic.List[byte]
    $trackInvalidPayload.AddRange([BitConverter]::GetBytes([Int16]0))
    $trackInvalidPayload.AddRange([BitConverter]::GetBytes([Int16]0))
    Send-Frame -Port $cdcSerial -MsgType 0x01 -Flags 0x06 -FrameId 4 -TimestampMs ($tick + 3) -Payload ($trackInvalidPayload.ToArray()) -Tag "TRACK_INVALID"
    Drain-Ports -DebugPortObj $debugSerial -CdcPortObj $cdcSerial -DurationMs 800

    $paramSetPayload = New-Object System.Collections.Generic.List[byte]
    $paramSetPayload.Add(0x01)
    $paramSetPayload.Add(0x00)
    $paramSetPayload.AddRange([BitConverter]::GetBytes([Int32]5))
    Clear-ParamValue -ParamId 0x01 -AxisId 0x00
    Send-Frame -Port $cdcSerial -MsgType 0x02 -Flags 0x00 -FrameId 5 -TimestampMs ($tick + 4) -Payload ($paramSetPayload.ToArray()) -Tag "PARAM_SET_PAN_DEADBAND_5"
    Drain-Ports -DebugPortObj $debugSerial -CdcPortObj $cdcSerial -DurationMs 800
    Assert-ParamValue -ParamId 0x01 -AxisId 0x00 -ExpectedValue 5 -SourceTag "PARAM_SET_PAN_DEADBAND_5"

    $paramGetPayload = [byte[]]@(0x01, 0x00)
    Clear-ParamValue -ParamId 0x01 -AxisId 0x00
    Send-Frame -Port $cdcSerial -MsgType 0x03 -Flags 0x00 -FrameId 6 -TimestampMs ($tick + 5) -Payload $paramGetPayload -Tag "PARAM_GET_PAN_DEADBAND"
    Drain-Ports -DebugPortObj $debugSerial -CdcPortObj $cdcSerial -DurationMs 800
    Assert-ParamValue -ParamId 0x01 -AxisId 0x00 -ExpectedValue 5 -SourceTag "PARAM_GET_PAN_DEADBAND"

    $kalmanSetPayload = New-Object System.Collections.Generic.List[byte]
    $kalmanSetPayload.Add(0x23)
    $kalmanSetPayload.Add(0xFF)
    $kalmanSetPayload.AddRange([BitConverter]::GetBytes([Int32]16000))
    Clear-ParamValue -ParamId 0x23 -AxisId 0xFF
    Send-Frame -Port $cdcSerial -MsgType 0x02 -Flags 0x00 -FrameId 7 -TimestampMs ($tick + 6) -Payload ($kalmanSetPayload.ToArray()) -Tag "PARAM_SET_KALMAN_Q_16000"
    Drain-Ports -DebugPortObj $debugSerial -CdcPortObj $cdcSerial -DurationMs 800
    Assert-ParamValue -ParamId 0x23 -AxisId 0xFF -ExpectedValue 16000 -SourceTag "PARAM_SET_KALMAN_Q_16000"

    $kalmanGetPayload = [byte[]]@(0x23, 0xFF)
    Clear-ParamValue -ParamId 0x23 -AxisId 0xFF
    Send-Frame -Port $cdcSerial -MsgType 0x03 -Flags 0x00 -FrameId 8 -TimestampMs ($tick + 7) -Payload $kalmanGetPayload -Tag "PARAM_GET_KALMAN_Q"
    Drain-Ports -DebugPortObj $debugSerial -CdcPortObj $cdcSerial -DurationMs 800
    Assert-ParamValue -ParamId 0x23 -AxisId 0xFF -ExpectedValue 16000 -SourceTag "PARAM_GET_KALMAN_Q"

    $goHomePayload = [byte[]]@(0x01)
    Send-Frame -Port $cdcSerial -MsgType 0x04 -Flags 0x00 -FrameId 9 -TimestampMs ($tick + 8) -Payload $goHomePayload -Tag "GO_HOME"
    Drain-Ports -DebugPortObj $debugSerial -CdcPortObj $cdcSerial -DurationMs 1000

    Write-Log "Integration test sequence finished"
}
finally {
    if ($debugSerial -ne $null -and $debugSerial.IsOpen) { $debugSerial.Close() }
    if ($cdcSerial -ne $null -and $cdcSerial.IsOpen) { $cdcSerial.Close() }
}
