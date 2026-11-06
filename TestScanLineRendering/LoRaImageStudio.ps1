Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing

[System.Windows.Forms.Application]::EnableVisualStyles()

$W = 120
$H = 90
$SCALE = 3
$b64Chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"

# Global state
$script:serialPort = $null
$script:isStreaming = $false
$script:currentPass = 1
$script:queueIndex = 0
$script:scanOrder = @()
$script:scanlines = @{}
$script:missedRows = New-Object System.Collections.Generic.List[int]
$script:sentCount = 0
$script:dropCount = 0
$script:noiseCount = 0
$script:previewBmp = New-Object System.Drawing.Bitmap ($W * $SCALE), ($H * $SCALE)
$script:sourceBmp = New-Object System.Drawing.Bitmap $W, $H
$script:rng = New-Object System.Random

# Precompute 64-color RGB222 palette
$script:palette = @{}
for ($idx = 0; $idx -lt 64; $idx++) {
    $r = (($idx -shr 4) -band 3) * 85
    $g = (($idx -shr 2) -band 3) * 85
    $b = ($idx -band 3) * 85
    $script:palette[$b64Chars[$idx]] = [System.Drawing.Color]::FromArgb($r, $g, $b)
}

function Convert-BitmapToScanlines([System.Drawing.Bitmap]$bmp) {
    $resized = New-Object System.Drawing.Bitmap $bmp, $W, $H
    $gPrev = [System.Drawing.Graphics]::FromImage($script:previewBmp)
    $gPrev.InterpolationMode = [System.Drawing.Drawing2D.InterpolationMode]::NearestNeighbor

    for ($y = 0; $y -lt $H; $y++) {
        $sb = New-Object System.Text.StringBuilder
        for ($x = 0; $x -lt $W; $x++) {
            $c = $resized.GetPixel($x, $y)
            $r2 = [Math]::Min(3, [Math]::Round($c.R / 85.0))
            $g2 = [Math]::Min(3, [Math]::Round($c.G / 85.0))
            $b2 = [Math]::Min(3, [Math]::Round($c.B / 85.0))
            $idx = ([int]$r2 -shl 4) -bor ([int]$g2 -shl 2) -bor [int]$b2
            $ch = $b64Chars[$idx]
            [void]$sb.Append($ch)

            $quantColor = $script:palette[$ch]
            $script:sourceBmp.SetPixel($x, $y, $quantColor)
            $brush = New-Object System.Drawing.SolidBrush $quantColor
            $gPrev.FillRectangle($brush, $x * $SCALE, $y * $SCALE, $SCALE, $SCALE)
            $brush.Dispose()
        }
        $script:scanlines[$y] = $sb.ToString()
    }
    $gPrev.Dispose()
    $resized.Dispose()
    $picPreview.Image = $script:previewBmp
    $picPreview.Invalidate()
}

function New-PresetImage([string]$presetName) {
    $bmp = New-Object System.Drawing.Bitmap $W, $H
    $g = [System.Drawing.Graphics]::FromImage($bmp)
    $font = New-Object System.Drawing.Font("Consolas", 7, [System.Drawing.FontStyle]::Bold)

    if ($presetName -eq "Thermal IR Target") {
        $g.Clear([System.Drawing.Color]::FromArgb(0, 0, 85))
        $g.FillRectangle((New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(0, 85, 0))), 0, 58, 120, 32)
        $g.FillEllipse((New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(0, 170, 85))), -10, 48, 70, 30)
        $g.FillEllipse((New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(0, 170, 85))), 50, 45, 80, 35)
        $penCyan = New-Object System.Drawing.Pen ([System.Drawing.Color]::FromArgb(0, 255, 255)), 1
        $g.DrawEllipse($penCyan, 42, 22, 36, 36)
        $g.DrawLine($penCyan, 60, 14, 60, 66)
        $g.DrawLine($penCyan, 34, 40, 86, 40)
        $g.FillEllipse([System.Drawing.Brushes]::Yellow, 52, 32, 16, 16)
        $g.FillEllipse([System.Drawing.Brushes]::Red, 56, 36, 8, 8)
        $g.DrawString("SAR-CAM IR", $font, [System.Drawing.Brushes]::Yellow, 4, 2)
        $g.DrawString("51.5N 0.1W", $font, [System.Drawing.Brushes]::White, 4, 76)
    }
    elseif ($presetName -eq "Drone Recon Grid") {
        $g.Clear([System.Drawing.Color]::FromArgb(0, 85, 85))
        $penGrid = New-Object System.Drawing.Pen ([System.Drawing.Color]::FromArgb(0, 170, 170)), 1
        for ($x = 0; $x -lt $W; $x += 15) { $g.DrawLine($penGrid, $x, 0, $x, $H) }
        for ($y = 0; $y -lt $H; $y += 15) { $g.DrawLine($penGrid, 0, $y, $W, $y) }
        # River & landing zone
        $penRiver = New-Object System.Drawing.Pen ([System.Drawing.Color]::FromArgb(0, 85, 255)), 6
        $g.DrawLine($penRiver, 0, 20, 120, 75)
        $g.FillRectangle([System.Drawing.Brushes]::Yellow, 75, 20, 26, 26)
        $g.DrawString("H", (New-Object System.Drawing.Font("Consolas", 14, [System.Drawing.FontStyle]::Bold)), [System.Drawing.Brushes]::Red, 80, 22)
        $g.DrawString("LZ ALPHA", $font, [System.Drawing.Brushes]::White, 4, 76)
    }
    elseif ($presetName -eq "Color Bars Test") {
        $colors = @(
            [System.Drawing.Color]::White,
            [System.Drawing.Color]::Yellow,
            [System.Drawing.Color]::Cyan,
            [System.Drawing.Color]::Lime,
            [System.Drawing.Color]::Magenta,
            [System.Drawing.Color]::Red,
            [System.Drawing.Color]::Blue,
            [System.Drawing.Color]::Black
        )
        $barW = [int]($W / $colors.Count)
        for ($i = 0; $i -lt $colors.Count; $i++) {
            $b = New-Object System.Drawing.SolidBrush $colors[$i]
            $g.FillRectangle($b, $i * $barW, 0, $barW, 68)
        }
        $g.FillRectangle([System.Drawing.Brushes]::Black, 0, 68, $W, 22)
        $g.DrawString("LORA SSTV TEST", $font, [System.Drawing.Brushes]::Lime, 18, 73)
    }
    elseif ($presetName -eq "SOS Emergency Alert") {
        $g.Clear([System.Drawing.Color]::FromArgb(170, 0, 0))
        $g.FillRectangle([System.Drawing.Brushes]::Yellow, 8, 8, 104, 74)
        $g.FillRectangle([System.Drawing.Brushes]::Black, 12, 12, 96, 66)
        $fontBig = New-Object System.Drawing.Font("Consolas", 22, [System.Drawing.FontStyle]::Bold)
        $g.DrawString("S O S", $fontBig, [System.Drawing.Brushes]::Red, 16, 18)
        $g.DrawString("SURVIVOR LOCATED", $font, [System.Drawing.Brushes]::Yellow, 14, 56)
    }

    $g.Dispose()
    Convert-BitmapToScanlines $bmp
    $bmp.Dispose()
}

function Build-ScanOrder([string]$mode) {
    $order = New-Object System.Collections.Generic.List[int]
    if ($mode -like "Progressive*") {
        for ($y = 0; $y -lt $H; $y++) { $order.Add($y) }
    }
    elseif ($mode -like "Interlaced*") {
        for ($y = 0; $y -lt $H; $y += 4) { $order.Add($y) }
        for ($y = 2; $y -lt $H; $y += 4) { $order.Add($y) }
        for ($y = 1; $y -lt $H; $y += 2) { $order.Add($y) }
    }
    elseif ($mode -like "Random*") {
        $temp = @(0..($H - 1))
        $shuffled = $temp | Sort-Object { $script:rng.Next() }
        foreach ($y in $shuffled) { $order.Add($y) }
    }
    $script:scanOrder = $order.ToArray()
}

# Build Main Window
$form = New-Object System.Windows.Forms.Form
$form.Text = "SAR Drone - LoRa Scanline & Link Simulator Studio"
$form.Size = New-Object System.Drawing.Size(780, 560)
$form.BackColor = [System.Drawing.Color]::FromArgb(17, 24, 39)
$form.ForeColor = [System.Drawing.Color]::White
$form.StartPosition = "CenterScreen"
$form.FormBorderStyle = "FixedSingle"
$form.MaximizeBox = $false

$fontTitle = New-Object System.Drawing.Font("Segoe UI", 11, [System.Drawing.FontStyle]::Bold)
$fontNormal = New-Object System.Drawing.Font("Segoe UI", 9, [System.Drawing.FontStyle]::Regular)
$fontBold = New-Object System.Drawing.Font("Segoe UI", 9, [System.Drawing.FontStyle]::Bold)

# --- LEFT COLUMN: Image Preview & Selection ---
$grpImage = New-Object System.Windows.Forms.GroupBox
$grpImage.Text = " 1. Image Source & Live Transmit Preview "
$grpImage.ForeColor = [System.Drawing.Color]::FromArgb(16, 185, 129)
$grpImage.Font = $fontBold
$grpImage.Location = New-Object System.Drawing.Point(15, 12)
$grpImage.Size = New-Object System.Drawing.Size(390, 405)
$form.Controls.Add($grpImage)

$picPreview = New-Object System.Windows.Forms.PictureBox
$picPreview.Location = New-Object System.Drawing.Point(15, 25)
$picPreview.Size = New-Object System.Drawing.Size(($W * $SCALE), ($H * $SCALE))
$picPreview.BorderStyle = "FixedSingle"
$grpImage.Controls.Add($picPreview)

$lblPreset = New-Object System.Windows.Forms.Label
$lblPreset.Text = "Preset Scene:"
$lblPreset.ForeColor = [System.Drawing.Color]::LightGray
$lblPreset.Font = $fontNormal
$lblPreset.Location = New-Object System.Drawing.Point(15, 310)
$lblPreset.Size = New-Object System.Drawing.Size(85, 22)
$grpImage.Controls.Add($lblPreset)

$cmbPreset = New-Object System.Windows.Forms.ComboBox
$cmbPreset.Location = New-Object System.Drawing.Point(100, 307)
$cmbPreset.Size = New-Object System.Drawing.Size(165, 24)
$cmbPreset.DropDownStyle = "DropDownList"
$cmbPreset.Font = $fontNormal
[void]$cmbPreset.Items.AddRange(@("Thermal IR Target", "Drone Recon Grid", "Color Bars Test", "SOS Emergency Alert"))
$cmbPreset.SelectedIndex = 0
$grpImage.Controls.Add($cmbPreset)

$btnBrowse = New-Object System.Windows.Forms.Button
$btnBrowse.Text = "Browse Photo..."
$btnBrowse.Location = New-Object System.Drawing.Point(272, 305)
$btnBrowse.Size = New-Object System.Drawing.Size(103, 27)
$btnBrowse.BackColor = [System.Drawing.Color]::FromArgb(55, 65, 81)
$btnBrowse.ForeColor = [System.Drawing.Color]::White
$btnBrowse.FlatStyle = "Flat"
$btnBrowse.Font = $fontNormal
$grpImage.Controls.Add($btnBrowse)

$btnClearPi = New-Object System.Windows.Forms.Button
$btnClearPi.Text = "Clear Pi Screen Over-The-Air (CMD:CLEAR)"
$btnClearPi.Location = New-Object System.Drawing.Point(15, 345)
$btnClearPi.Size = New-Object System.Drawing.Size(360, 32)
$btnClearPi.BackColor = [System.Drawing.Color]::FromArgb(75, 85, 99)
$btnClearPi.ForeColor = [System.Drawing.Color]::White
$btnClearPi.FlatStyle = "Flat"
$btnClearPi.Font = $fontNormal
$grpImage.Controls.Add($btnClearPi)

# --- RIGHT COLUMN: Radio & Error Simulation Controls ---
$grpConn = New-Object System.Windows.Forms.GroupBox
$grpConn.Text = " 2. ESP32 Connection & LoRa Air Data Rate "
$grpConn.ForeColor = [System.Drawing.Color]::FromArgb(59, 130, 246)
$grpConn.Font = $fontBold
$grpConn.Location = New-Object System.Drawing.Point(418, 12)
$grpConn.Size = New-Object System.Drawing.Size(335, 145)
$form.Controls.Add($grpConn)

$lblPort = New-Object System.Windows.Forms.Label
$lblPort.Text = "ESP32 Port:"
$lblPort.ForeColor = [System.Drawing.Color]::LightGray
$lblPort.Font = $fontNormal
$lblPort.Location = New-Object System.Drawing.Point(15, 28)
$lblPort.Size = New-Object System.Drawing.Size(75, 22)
$grpConn.Controls.Add($lblPort)

$cmbPort = New-Object System.Windows.Forms.ComboBox
$cmbPort.Location = New-Object System.Drawing.Point(92, 25)
$cmbPort.Size = New-Object System.Drawing.Size(95, 24)
$cmbPort.Font = $fontNormal
$ports = [System.IO.Ports.SerialPort]::GetPortNames()
if ($ports.Count -gt 0) {
    [void]$cmbPort.Items.AddRange($ports)
    if ($ports -contains "COM5") { $cmbPort.SelectedItem = "COM5" } else { $cmbPort.SelectedIndex = 0 }
} else {
    [void]$cmbPort.Items.Add("COM5")
    $cmbPort.SelectedIndex = 0
}
$grpConn.Controls.Add($cmbPort)

$btnConnect = New-Object System.Windows.Forms.Button
$btnConnect.Text = "Connect ESP32"
$btnConnect.Location = New-Object System.Drawing.Point(195, 23)
$btnConnect.Size = New-Object System.Drawing.Size(125, 27)
$btnConnect.BackColor = [System.Drawing.Color]::FromArgb(16, 185, 129)
$btnConnect.ForeColor = [System.Drawing.Color]::Black
$btnConnect.FlatStyle = "Flat"
$btnConnect.Font = $fontBold
$grpConn.Controls.Add($btnConnect)

$lblLevel = New-Object System.Windows.Forms.Label
$lblLevel.Text = "LoRa Speed:"
$lblLevel.ForeColor = [System.Drawing.Color]::LightGray
$lblLevel.Font = $fontNormal
$lblLevel.Location = New-Object System.Drawing.Point(15, 65)
$lblLevel.Size = New-Object System.Drawing.Size(75, 22)
$grpConn.Controls.Add($lblLevel)

$cmbLevel = New-Object System.Windows.Forms.ComboBox
$cmbLevel.Location = New-Object System.Drawing.Point(92, 62)
$cmbLevel.Size = New-Object System.Drawing.Size(228, 24)
$cmbLevel.DropDownStyle = "DropDownList"
$cmbLevel.Font = $fontNormal
[void]$cmbLevel.Items.AddRange(@(
    "LEVEL 0 - 336 bps (Extreme Range ~8km)",
    "LEVEL 1 - 1,075 bps (Very Long Range)",
    "LEVEL 2 - 2,149 bps (Default Long Range)",
    "LEVEL 3 - 6,250 bps (Medium Range)",
    "LEVEL 4 - 10,417 bps (Medium-Fast)",
    "LEVEL 5 - 18,230 bps (Fast)",
    "LEVEL 6 - 37,500 bps (Very Fast)",
    "LEVEL 7 - 62,500 bps (Maximum Bandwidth)"
))
$cmbLevel.SelectedIndex = 2
$grpConn.Controls.Add($cmbLevel)

$btnApplyLevel = New-Object System.Windows.Forms.Button
$btnApplyLevel.Text = "Sync Both Radios to Selected Speed"
$btnApplyLevel.Location = New-Object System.Drawing.Point(15, 98)
$btnApplyLevel.Size = New-Object System.Drawing.Size(305, 30)
$btnApplyLevel.BackColor = [System.Drawing.Color]::FromArgb(37, 99, 235)
$btnApplyLevel.ForeColor = [System.Drawing.Color]::White
$btnApplyLevel.FlatStyle = "Flat"
$btnApplyLevel.Font = $fontBold
$grpConn.Controls.Add($btnApplyLevel)

# --- Group 3: Scan Rate & Channel Error Simulation ---
$grpSim = New-Object System.Windows.Forms.GroupBox
$grpSim.Text = " 3. Scan Rate & Weak-Signal Simulator (Live) "
$grpSim.ForeColor = [System.Drawing.Color]::FromArgb(245, 158, 11)
$grpSim.Font = $fontBold
$grpSim.Location = New-Object System.Drawing.Point(418, 165)
$grpSim.Size = New-Object System.Drawing.Size(335, 252)
$form.Controls.Add($grpSim)

$lblOrder = New-Object System.Windows.Forms.Label
$lblOrder.Text = "Scan Mode:"
$lblOrder.ForeColor = [System.Drawing.Color]::LightGray
$lblOrder.Font = $fontNormal
$lblOrder.Location = New-Object System.Drawing.Point(15, 28)
$lblOrder.Size = New-Object System.Drawing.Size(75, 22)
$grpSim.Controls.Add($lblOrder)

$cmbOrder = New-Object System.Windows.Forms.ComboBox
$cmbOrder.Location = New-Object System.Drawing.Point(95, 25)
$cmbOrder.Size = New-Object System.Drawing.Size(225, 24)
$cmbOrder.DropDownStyle = "DropDownList"
$cmbOrder.Font = $fontNormal
[void]$cmbOrder.Items.AddRange(@(
    "Interlaced (4-Pass Progressive Build)",
    "Progressive (Top-to-Bottom Scan)",
    "Random Scatter (Random Scanlines)"
))
$cmbOrder.SelectedIndex = 0
$grpSim.Controls.Add($cmbOrder)

# Scan Delay Slider
$lblDelay = New-Object System.Windows.Forms.Label
$lblDelay.Text = "Scanline Interval (Delay): 650 ms / line"
$lblDelay.ForeColor = [System.Drawing.Color]::White
$lblDelay.Font = $fontNormal
$lblDelay.Location = New-Object System.Drawing.Point(15, 58)
$lblDelay.Size = New-Object System.Drawing.Size(300, 20)
$grpSim.Controls.Add($lblDelay)

$trkDelay = New-Object System.Windows.Forms.TrackBar
$trkDelay.Location = New-Object System.Drawing.Point(15, 76)
$trkDelay.Size = New-Object System.Drawing.Size(305, 35)
$trkDelay.Minimum = 150
$trkDelay.Maximum = 1500
$trkDelay.TickFrequency = 150
$trkDelay.Value = 650
$trkDelay.AutoSize = $false
$grpSim.Controls.Add($trkDelay)

# Simulated Packet Loss Slider
$lblDrop = New-Object System.Windows.Forms.Label
$lblDrop.Text = "Simulated Missed Scanlines (Drop %): 15%"
$lblDrop.ForeColor = [System.Drawing.Color]::FromArgb(252, 165, 165)
$lblDrop.Font = $fontNormal
$lblDrop.Location = New-Object System.Drawing.Point(15, 114)
$lblDrop.Size = New-Object System.Drawing.Size(300, 20)
$grpSim.Controls.Add($lblDrop)

$trkDrop = New-Object System.Windows.Forms.TrackBar
$trkDrop.Location = New-Object System.Drawing.Point(15, 132)
$trkDrop.Size = New-Object System.Drawing.Size(305, 35)
$trkDrop.Minimum = 0
$trkDrop.Maximum = 80
$trkDrop.TickFrequency = 10
$trkDrop.Value = 15
$trkDrop.AutoSize = $false
$grpSim.Controls.Add($trkDrop)

# Simulated Pixel Static / Bit Corruption Slider
$lblNoise = New-Object System.Windows.Forms.Label
$lblNoise.Text = "Simulated Radio Static (Pixel Noise %): 10%"
$lblNoise.ForeColor = [System.Drawing.Color]::FromArgb(253, 224, 71)
$lblNoise.Font = $fontNormal
$lblNoise.Location = New-Object System.Drawing.Point(15, 170)
$lblNoise.Size = New-Object System.Drawing.Size(300, 20)
$grpSim.Controls.Add($lblNoise)

$trkNoise = New-Object System.Windows.Forms.TrackBar
$trkNoise.Location = New-Object System.Drawing.Point(15, 188)
$trkNoise.Size = New-Object System.Drawing.Size(305, 35)
$trkNoise.Minimum = 0
$trkNoise.Maximum = 60
$trkNoise.TickFrequency = 10
$trkNoise.Value = 10
$trkNoise.AutoSize = $false
$grpSim.Controls.Add($trkNoise)

$chkSelfHeal = New-Object System.Windows.Forms.CheckBox
$chkSelfHeal.Text = "Auto-Heal Clean Pass on Pass 2+ (Fix Static & Drops)"
$chkSelfHeal.ForeColor = [System.Drawing.Color]::LightGreen
$chkSelfHeal.Font = $fontNormal
$chkSelfHeal.Location = New-Object System.Drawing.Point(18, 222)
$chkSelfHeal.Size = New-Object System.Drawing.Size(310, 24)
$chkSelfHeal.Checked = $true
$grpSim.Controls.Add($chkSelfHeal)

# --- BOTTOM BAR: Start / Stop & Telemetry Status ---
$btnStream = New-Object System.Windows.Forms.Button
$btnStream.Text = "START SCANLINE STREAM"
$btnStream.Location = New-Object System.Drawing.Point(15, 428)
$btnStream.Size = New-Object System.Drawing.Size(390, 45)
$btnStream.BackColor = [System.Drawing.Color]::FromArgb(16, 185, 129)
$btnStream.ForeColor = [System.Drawing.Color]::Black
$btnStream.FlatStyle = "Flat"
$btnStream.Font = $fontTitle
$form.Controls.Add($btnStream)

$lblStatus = New-Object System.Windows.Forms.Label
$lblStatus.Text = "Status: Disconnected. Click 'Connect ESP32' to open COM5."
$lblStatus.ForeColor = [System.Drawing.Color]::Cyan
$lblStatus.Font = $fontBold
$lblStatus.Location = New-Object System.Drawing.Point(420, 428)
$lblStatus.Size = New-Object System.Drawing.Size(335, 45)
$form.Controls.Add($lblStatus)

$lblStats = New-Object System.Windows.Forms.Label
$lblStats.Text = "Pass: - | Sent: 0 | Dropped: 0 | Static Lines: 0"
$lblStats.ForeColor = [System.Drawing.Color]::LightGray
$lblStats.Font = $fontNormal
$lblStats.Location = New-Object System.Drawing.Point(15, 482)
$lblStats.Size = New-Object System.Drawing.Size(740, 25)
$form.Controls.Add($lblStats)

# Timer for streaming scanlines without freezing the UI
$timer = New-Object System.Windows.Forms.Timer
$timer.Interval = 650

# Event Handlers
$trkDelay.Add_Scroll({
    $timer.Interval = $trkDelay.Value
    $lblDelay.Text = "Scanline Interval (Delay): $($trkDelay.Value) ms / line"
})

$trkDrop.Add_Scroll({
    $lblDrop.Text = "Simulated Missed Scanlines (Drop %): $($trkDrop.Value)%"
})

$trkNoise.Add_Scroll({
    $lblNoise.Text = "Simulated Radio Static (Pixel Noise %): $($trkNoise.Value)%"
})

$cmbPreset.Add_SelectedIndexChanged({
    New-PresetImage $cmbPreset.SelectedItem.ToString()
})

$btnBrowse.Add_Click({
    $ofd = New-Object System.Windows.Forms.OpenFileDialog
    $ofd.Filter = "Images (*.png;*.jpg;*.jpeg;*.bmp)|*.png;*.jpg;*.jpeg;*.bmp"
    if ($ofd.ShowDialog() -eq "OK") {
        $loaded = New-Object System.Drawing.Bitmap $ofd.FileName
        Convert-BitmapToScanlines $loaded
        $loaded.Dispose()
    }
})

$btnConnect.Add_Click({
    if ($script:serialPort -and $script:serialPort.IsOpen) {
        $timer.Stop()
        $script:isStreaming = $false
        $btnStream.Text = "START SCANLINE STREAM"
        $btnStream.BackColor = [System.Drawing.Color]::FromArgb(16, 185, 129)
        $script:serialPort.Close()
        $btnConnect.Text = "Connect ESP32"
        $btnConnect.BackColor = [System.Drawing.Color]::FromArgb(16, 185, 129)
        $lblStatus.Text = "Status: Disconnected."
        return
    }
    try {
        $portName = $cmbPort.Text
        $script:serialPort = New-Object System.IO.Ports.SerialPort $portName, 115200, 'None', 8, 'One'
        $script:serialPort.Open()
        $lblStatus.Text = "Opening $portName... waiting 3.5s for ESP32 boot..."
        $form.Refresh()
        Start-Sleep -Milliseconds 3500
        $null = $script:serialPort.ReadExisting()

        # Default to selected level on connect
        $lvl = $cmbLevel.SelectedIndex
        $script:serialPort.Write("+++`r`n")
        Start-Sleep -Milliseconds 500
        $null = $script:serialPort.ReadExisting()
        $script:serialPort.Write("AT+LEVEL$lvl`r`n")
        Start-Sleep -Milliseconds 400
        $null = $script:serialPort.ReadExisting()
        $script:serialPort.Write("AT+RESET`r`n")
        Start-Sleep -Milliseconds 700
        $null = $script:serialPort.ReadExisting()

        $btnConnect.Text = "Disconnect"
        $btnConnect.BackColor = [System.Drawing.Color]::FromArgb(239, 68, 68)
        $lblStatus.Text = "Connected on $portName! Ready to stream."
    }
    catch {
        [System.Windows.Forms.MessageBox]::Show("Could not open $($cmbPort.Text): $($_.Exception.Message)", "Serial Error")
    }
})

$btnApplyLevel.Add_Click({
    if (-not ($script:serialPort -and $script:serialPort.IsOpen)) {
        [System.Windows.Forms.MessageBox]::Show("Click 'Connect ESP32' first!", "Not Connected")
        return
    }
    $lvl = $cmbLevel.SelectedIndex
    $wasStreaming = $script:isStreaming
    if ($wasStreaming) { $timer.Stop() }

    $lblStatus.Text = "Syncing Pi & ESP32 to LEVEL $lvl..."
    $form.Refresh()

    # 1. Tell Raspberry Pi over the air to switch to LEVEL $lvl
    $script:serialPort.Write("CMD:LEVEL:$lvl`r`n")
    Start-Sleep -Milliseconds 700
    $script:serialPort.Write("CMD:LEVEL:$lvl`r`n")
    Start-Sleep -Milliseconds 800

    # 2. Switch local ESP32 DX-LR22 to LEVEL $lvl
    $script:serialPort.Write("+++`r`n")
    Start-Sleep -Milliseconds 600
    $null = $script:serialPort.ReadExisting()
    $script:serialPort.Write("AT+LEVEL$lvl`r`n")
    Start-Sleep -Milliseconds 500
    $null = $script:serialPort.ReadExisting()
    $script:serialPort.Write("AT+RESET`r`n")
    Start-Sleep -Milliseconds 800
    $null = $script:serialPort.ReadExisting()

    # Adjust recommended scanline delay automatically for chosen level
    $recDelays = @(2200, 950, 650, 350, 280, 220, 180, 160)
    $trkDelay.Value = $recDelays[$lvl]
    $timer.Interval = $trkDelay.Value
    $lblDelay.Text = "Scanline Interval (Delay): $($trkDelay.Value) ms / line"

    $lblStatus.Text = "Synced both radios to LEVEL $lvl!"
    if ($wasStreaming) { $timer.Start() }
})

$btnClearPi.Add_Click({
    if ($script:serialPort -and $script:serialPort.IsOpen) {
        $script:serialPort.Write("CMD:CLEAR`r`n")
        $lblStatus.Text = "Sent CMD:CLEAR to Raspberry Pi!"
    }
})

$btnStream.Add_Click({
    if (-not ($script:serialPort -and $script:serialPort.IsOpen)) {
        [System.Windows.Forms.MessageBox]::Show("Click 'Connect ESP32' first!", "Not Connected")
        return
    }
    if ($script:isStreaming) {
        $timer.Stop()
        $script:isStreaming = $false
        $btnStream.Text = "START SCANLINE STREAM"
        $btnStream.BackColor = [System.Drawing.Color]::FromArgb(16, 185, 129)
        $lblStatus.Text = "Stream paused."
    }
    else {
        Build-ScanOrder $cmbOrder.SelectedItem.ToString()
        $script:queueIndex = 0
        $script:currentPass = 1
        $script:sentCount = 0
        $script:dropCount = 0
        $script:noiseCount = 0
        $script:isStreaming = $true
        $timer.Interval = $trkDelay.Value
        $btnStream.Text = "STOP STREAM"
        $btnStream.BackColor = [System.Drawing.Color]::FromArgb(239, 68, 68)
        $lblStatus.Text = "Streaming Pass 1..."
        $timer.Start()
    }
})

$timer.Add_Tick({
    if (-not ($script:serialPort -and $script:serialPort.IsOpen)) {
        $timer.Stop()
        return
    }

    if ($script:queueIndex -ge $script:scanOrder.Length) {
        $script:currentPass++
        $script:queueIndex = 0
        Build-ScanOrder $cmbOrder.SelectedItem.ToString()
    }

    $y = $script:scanOrder[$script:queueIndex]
    $script:queueIndex++

    # Determine if we apply simulated drops/noise (Pass 1 always uses sliders; Pass 2+ heals if checked)
    $useErrors = ($script:currentPass -eq 1) -or (-not $chkSelfHeal.Checked)

    if ($useErrors -and ($script:rng.Next(0, 100) -lt $trkDrop.Value)) {
        $script:dropCount++
        $lblStatus.Text = "Pass $($script:currentPass): [DROPPED] Row $y"
        $lblStats.Text = "Pass: $($script:currentPass) | Sent: $($script:sentCount) | Dropped: $($script:dropCount) | Static Lines: $($script:noiseCount)"
        return
    }

    $lineStr = $script:scanlines[$y]

    # Inject simulated pixel noise if active
    if ($useErrors -and ($trkNoise.Value -gt 0)) {
        $chars = $lineStr.ToCharArray()
        $corrupted = $false
        for ($i = 0; $i -lt $chars.Length; $i++) {
            if ($script:rng.Next(0, 100) -lt $trkNoise.Value) {
                $chars[$i] = $b64Chars[$script:rng.Next(0, 64)]
                $corrupted = $true
            }
        }
        if ($corrupted) {
            $script:noiseCount++
            $lineStr = -join $chars
        }
    }

    $pkt = "L:$y`:$lineStr`r`n"
    try {
        $script:serialPort.Write($pkt)
        $script:sentCount++
        $lblStatus.Text = "Pass $($script:currentPass): Sent Row $y ($($script:queueIndex)/$H)"
        $lblStats.Text = "Pass: $($script:currentPass) | Sent: $($script:sentCount) | Dropped: $($script:dropCount) | Static Lines: $($script:noiseCount)"
    }
    catch {
        $timer.Stop()
        $lblStatus.Text = "Serial write error!"
    }
})

$form.Add_FormClosing({
    $timer.Stop()
    if ($script:serialPort -and $script:serialPort.IsOpen) {
        $script:serialPort.Close()
    }
})

# Initialize default preset image
New-PresetImage "Thermal IR Target"

[void]$form.ShowDialog()
