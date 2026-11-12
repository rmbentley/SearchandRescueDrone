Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing

# Fast C# True 24-Bit JPEG (16.7 Million Colors) + Optimal Scanline Encoder
$csharpCode = @"
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using System.IO;
using System.Text;

public class JpegTileResult {
    public string[] Bodies;
    public int[] QualityIndices;
}

public class SpanNode {
    public int XStart;
    public int Run;
    public int SumR, SumG, SumB;
    public int StartR, StartG, StartB;
    public int EndR, EndG, EndB;

    public int AvgR { get { return SumR / Run; } }
    public int AvgG { get { return SumG / Run; } }
    public int AvgB { get { return SumB / Run; } }
}

public class SmoothScanlineEncoder {
    private static readonly string B64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    private static readonly int[] QLevels = new int[] { 85, 75, 65, 55, 45, 35, 25, 15 };
    private static ImageCodecInfo jpegCodec = null;

    private static ImageCodecInfo GetJpegCodec() {
        if (jpegCodec != null) return jpegCodec;
        foreach (ImageCodecInfo ici in ImageCodecInfo.GetImageEncoders()) {
            if (ici.MimeType == "image/jpeg") {
                jpegCodec = ici;
                break;
            }
        }
        return jpegCodec;
    }

    public static string ComputeChecksum2(string data) {
        int crc = 0xACE1;
        for (int i = 0; i < data.Length; i++) {
            crc ^= ((int)data[i] << 4);
            for (int b = 0; b < 8; b++) {
                if ((crc & 0x8000) != 0)
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF;
                else
                    crc = ((crc << 1) & 0xFFFF);
            }
        }
        int v1 = (crc >> 6) & 0x3F;
        int v2 = crc & 0x3F;
        return "" + B64[v1] + B64[v2];
    }

    // Encodes 300 64x16 True 24-Bit JPEG Tiles (stripping the 623-byte static header + 2-byte EOI so each tile fits in 1 single LoRa packet!)
    public static JpegTileResult EncodeJpegTiles64x16(Bitmap src, Bitmap previewOut, int width, int height) {
        int tilesX = width / 64;
        int tilesY = height / 16;
        int total = tilesX * tilesY;
        string[] bodies = new string[total];
        int[] qIndices = new int[total];
        ImageCodecInfo codec = GetJpegCodec();

        using (Bitmap resized = new Bitmap(width, height)) {
            using (Graphics g = Graphics.FromImage(resized)) {
                g.InterpolationMode = InterpolationMode.HighQualityBicubic;
                g.PixelOffsetMode = PixelOffsetMode.HighQuality;
                g.DrawImage(src, 0, 0, width, height);
            }

            using (Graphics gPrev = (previewOut != null) ? Graphics.FromImage(previewOut) : null) {
                for (int ty = 0; ty < tilesY; ty++) {
                    for (int tx = 0; tx < tilesX; tx++) {
                        int idx = ty * tilesX + tx;
                        Rectangle rect = new Rectangle(tx * 64, ty * 16, 64, 16);
                        using (Bitmap tileBmp = resized.Clone(rect, PixelFormat.Format24bppRgb)) {
                            byte[] bestFullBytes = null;
                            string bestB64 = "";
                            int bestQIdx = 0;

                            for (int qi = 0; qi < QLevels.Length; qi++) {
                                using (EncoderParameters ep = new EncoderParameters(1)) {
                                    ep.Param[0] = new EncoderParameter(System.Drawing.Imaging.Encoder.Quality, (long)QLevels[qi]);
                                    using (MemoryStream ms = new MemoryStream()) {
                                        tileBmp.Save(ms, codec, ep);
                                        byte[] fullBytes = ms.ToArray();
                                        int bodyLen = fullBytes.Length - 623 - 2;
                                        if (bodyLen < 1) bodyLen = 1;
                                        string b64 = Convert.ToBase64String(fullBytes, 623, bodyLen).TrimEnd('=');

                                        bestFullBytes = fullBytes;
                                        bestB64 = b64;
                                        bestQIdx = qi;

                                        // Fits cleanly inside a single 255-byte SX1262 LoRa packet!
                                        if (b64.Length <= 220) {
                                            break;
                                        }
                                    }
                                }
                            }

                            bodies[idx] = bestB64;
                            qIndices[idx] = bestQIdx;

                            if (gPrev != null && bestFullBytes != null) {
                                using (MemoryStream msDec = new MemoryStream(bestFullBytes)) {
                                    using (Image decImg = Image.FromStream(msDec)) {
                                        gPrev.DrawImage(decImg, tx * 64, ty * 16, 64, 16);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        return new JpegTileResult { Bodies = bodies, QualityIndices = qIndices };
    }

    // Encodes the entire 640x480 frame as a single True 24-Bit JPEG and splits into 215-char Base64 chunks
    public static string[] EncodeFullFrameJpeg(Bitmap src, Bitmap previewOut, int width, int height, int quality) {
        ImageCodecInfo codec = GetJpegCodec();
        byte[] jpgBytes = null;

        using (Bitmap resized = new Bitmap(width, height)) {
            using (Graphics g = Graphics.FromImage(resized)) {
                g.InterpolationMode = InterpolationMode.HighQualityBicubic;
                g.PixelOffsetMode = PixelOffsetMode.HighQuality;
                g.DrawImage(src, 0, 0, width, height);
            }
            using (EncoderParameters ep = new EncoderParameters(1)) {
                ep.Param[0] = new EncoderParameter(System.Drawing.Imaging.Encoder.Quality, (long)quality);
                using (MemoryStream ms = new MemoryStream()) {
                    resized.Save(ms, codec, ep);
                    jpgBytes = ms.ToArray();
                }
            }
        }

        if (previewOut != null && jpgBytes != null) {
            using (MemoryStream msDec = new MemoryStream(jpgBytes)) {
                using (Image decImg = Image.FromStream(msDec)) {
                    using (Graphics gPrev = Graphics.FromImage(previewOut)) {
                        gPrev.DrawImage(decImg, 0, 0, width, height);
                    }
                }
            }
        }

        string fullB64 = Convert.ToBase64String(jpgBytes).TrimEnd('=');
        int chunkSize = 216; // Multiple of 4 for clean Base64 slicing!
        int numChunks = (fullB64.Length + chunkSize - 1) / chunkSize;
        string[] chunks = new string[numChunks];
        for (int i = 0; i < numChunks; i++) {
            int start = i * chunkSize;
            int len = Math.Min(chunkSize, fullB64.Length - start);
            chunks[i] = fullB64.Substring(start, len);
        }
        return chunks;
    }

    private static void EmitCode(StringBuilder sb, int run, int r4, int g4, int b4, ref int prevR4, ref int prevG4, ref int prevB4) {
        r4 = Math.Max(0, Math.Min(15, r4));
        g4 = Math.Max(0, Math.Min(15, g4));
        b4 = Math.Max(0, Math.Min(15, b4));
        int c1 = (r4 << 2) | (g4 >> 2);
        int c2 = ((g4 & 3) << 4) | b4;
        sb.Append(B64[run - 1]);
        sb.Append(B64[c1]);
        sb.Append(B64[c2]);
        prevR4 = r4;
        prevG4 = g4;
        prevB4 = b4;
    }

    private static string EncodeRowOptimal(Color[] rowPixels, int width, int maxSpans) {
        List<SpanNode> nodes = new List<SpanNode>(width);
        int x = 0;
        while (x < width) {
            Color c0 = rowPixels[x];
            int r4_0 = (int)Math.Round(c0.R / 17.0);
            int g4_0 = (int)Math.Round(c0.G / 17.0);
            int b4_0 = (int)Math.Round(c0.B / 17.0);

            SpanNode n = new SpanNode {
                XStart = x, Run = 1,
                SumR = c0.R, SumG = c0.G, SumB = c0.B,
                StartR = c0.R, StartG = c0.G, StartB = c0.B,
                EndR = c0.R, EndG = c0.G, EndB = c0.B
            };
            x++;
            while (x < width && n.Run < 64) {
                Color cx = rowPixels[x];
                if ((int)Math.Round(cx.R / 17.0) == r4_0 &&
                    (int)Math.Round(cx.G / 17.0) == g4_0 &&
                    (int)Math.Round(cx.B / 17.0) == b4_0) {
                    n.Run++;
                    n.SumR += cx.R; n.SumG += cx.G; n.SumB += cx.B;
                    n.EndR = cx.R; n.EndG = cx.G; n.EndB = cx.B;
                    x++;
                } else {
                    break;
                }
            }
            nodes.Add(n);
        }

        int targetBaseSpans = Math.Max(2, (int)(maxSpans * 0.85));
        while (nodes.Count > targetBaseSpans) {
            int bestIdx = -1;
            double bestCost = double.MaxValue;
            for (int i = 0; i < nodes.Count - 1; i++) {
                SpanNode a = nodes[i];
                SpanNode b = nodes[i + 1];
                if (a.Run + b.Run > 64) continue;
                double dr = Math.Abs(a.AvgR - b.AvgR);
                double dg = Math.Abs(a.AvgG - b.AvgG);
                double db = Math.Abs(a.AvgB - b.AvgB);
                double colorDiff = dr + dg + db;
                double lenWeight = Math.Min(a.Run, b.Run) + 0.25 * (a.Run + b.Run);
                double cost = colorDiff * colorDiff * lenWeight;
                if (cost < bestCost) {
                    bestCost = cost;
                    bestIdx = i;
                }
            }
            if (bestIdx < 0) break;
            SpanNode left = nodes[bestIdx];
            SpanNode right = nodes[bestIdx + 1];
            left.Run += right.Run;
            left.SumR += right.SumR;
            left.SumG += right.SumG;
            left.SumB += right.SumB;
            left.EndR = right.EndR;
            left.EndG = right.EndG;
            left.EndB = right.EndB;
            nodes.RemoveAt(bestIdx + 1);
        }

        StringBuilder sb = new StringBuilder(maxSpans * 3);
        int prevR4 = -1, prevG4 = -1, prevB4 = -1;
        for (int i = 0; i < nodes.Count; i++) {
            SpanNode s = nodes[i];
            int avgR4 = (int)Math.Round(s.AvgR / 17.0);
            int avgG4 = (int)Math.Round(s.AvgG / 17.0);
            int avgB4 = (int)Math.Round(s.AvgB / 17.0);
            EmitCode(sb, s.Run, avgR4, avgG4, avgB4, ref prevR4, ref prevG4, ref prevB4);
        }
        return sb.ToString();
    }

    public static string[] EncodeSmooth480p(Bitmap src, Bitmap previewOut, int width, int height) {
        string[] lines = new string[height];
        using (Bitmap resized = new Bitmap(width, height)) {
            using (Graphics g = Graphics.FromImage(resized)) {
                g.InterpolationMode = InterpolationMode.HighQualityBicubic;
                g.PixelOffsetMode = PixelOffsetMode.HighQuality;
                g.DrawImage(src, 0, 0, width, height);
            }
            for (int y = 0; y < height; y++) {
                Color[] rowPixels = new Color[width];
                for (int x = 0; x < width; x++) {
                    rowPixels[x] = resized.GetPixel(x, y);
                }
                string encoded = EncodeRowOptimal(rowPixels, width, 76);
                lines[y] = encoded;
                if (previewOut != null) {
                    RenderPreviewLine(previewOut, y, encoded, width);
                }
            }
        }
        return lines;
    }

    private static void RenderPreviewLine(Bitmap bmp, int y, string encoded, int width) {
        int x = 0;
        int prevR = -1, prevG = -1, prevB = -1;
        for (int i = 0; i + 2 < encoded.Length; i += 3) {
            int run = B64.IndexOf(encoded[i]) + 1;
            int c1 = B64.IndexOf(encoded[i + 1]);
            int c2 = B64.IndexOf(encoded[i + 2]);
            int r = ((c1 >> 2) & 0x0F) * 17;
            int g = (((c1 & 0x03) << 2) | ((c2 >> 4) & 0x03)) * 17;
            int b = (c2 & 0x0F) * 17;

            if (prevR < 0 || Math.Abs(r - prevR) + Math.Abs(g - prevG) + Math.Abs(b - prevB) > 34) {
                for (int k = 0; k < run && x < width; k++) {
                    bmp.SetPixel(x++, y, Color.FromArgb(r, g, b));
                }
            } else {
                for (int k = 1; k <= run && x < width; k++) {
                    double t = (double)k / run;
                    int ir = (int)(prevR + (r - prevR) * t);
                    int ig = (int)(prevG + (g - prevG) * t);
                    int ib = (int)(prevB + (b - prevB) * t);
                    bmp.SetPixel(x++, y, Color.FromArgb(ir, ig, ib));
                }
            }
            prevR = r; prevG = g; prevB = b;
        }
    }
}
"@

Add-Type -TypeDefinition $csharpCode -ReferencedAssemblies System.Drawing

[System.Windows.Forms.Application]::EnableVisualStyles()

$W = 640
$H = 480
$b64Chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"

$script:serialPort = $null
$script:isStreaming = $false
$script:currentPass = 1
$script:queueIndex = 0
$script:scanPlan = @()

# 1. True 24-Bit 64x16 JPEG Tiles (300 tiles, 1 packet per tile)
$script:jpegTileBodies = New-Object string[] 300
$script:jpegTileQIdx = New-Object int[] 300
$script:jpegTileChecksums = New-Object string[] 300

# 2. Full-Frame 640x480 24-Bit JPEG Chunks
$script:fullJpegChunks = @()
$script:fullJpegChecksums = @()

# 3. Legacy Scanlines (S: packets)
$script:scanlines = New-Object string[] $H
$script:checksums = New-Object string[] $H

$script:sentCount = 0
$script:dropCount = 0
$script:noiseCount = 0
$script:dedupSaved = 0
$script:previewJpegTileBmp = New-Object System.Drawing.Bitmap $W, $H
$script:previewFullJpegBmp = New-Object System.Drawing.Bitmap $W, $H
$script:previewScanBmp = New-Object System.Drawing.Bitmap $W, $H
$script:rng = New-Object System.Random

function Update-PreviewAndInfo() {
    $mode = if ($cmbOrder -and $cmbOrder.SelectedItem) { $cmbOrder.SelectedItem.ToString() } else { "True 24-Bit JPEG Tiles" }
    Build-ScanOrder $mode
    if ($mode -like "True 24-Bit JPEG Tiles*") {
        $picPreview.Image = $script:previewJpegTileBmp
        $lblCompInfo.Text = "True 24-Bit JPEG Tiles (16.7M Colors): $($script:scanPlan.Length) packets ($($script:dedupSaved) tiles merged)!"
    }
    elseif ($mode -like "Full-Frame 24-Bit JPEG*") {
        $picPreview.Image = $script:previewFullJpegBmp
        $lblCompInfo.Text = "Full-Frame 640x480 24-Bit JPEG: $($script:scanPlan.Length) packets (100% Photographic Quality)!"
    }
    else {
        $picPreview.Image = $script:previewScanBmp
        $lblCompInfo.Text = "Legacy 480p Scanlines: $($script:scanPlan.Length) packets ($($script:dedupSaved) rows deduped)!"
    }
    $picPreview.Invalidate()
}

function Convert-BitmapToScanlines([System.Drawing.Bitmap]$bmp) {
    # 1. Encode 300 True 24-Bit JPEG 64x16 Tiles
    $res = [SmoothScanlineEncoder]::EncodeJpegTiles64x16($bmp, $script:previewJpegTileBmp, $W, $H)
    $script:jpegTileBodies = $res.Bodies
    $script:jpegTileQIdx = $res.QualityIndices
    for ($i = 0; $i -lt 300; $i++) {
        $script:jpegTileChecksums[$i] = [SmoothScanlineEncoder]::ComputeChecksum2($script:jpegTileBodies[$i])
    }

    # 2. Encode Full-Frame 640x480 24-Bit JPEG (Q=68)
    $script:fullJpegChunks = [SmoothScanlineEncoder]::EncodeFullFrameJpeg($bmp, $script:previewFullJpegBmp, $W, $H, 68)
    $script:fullJpegChecksums = New-Object string[] $script:fullJpegChunks.Length
    for ($i = 0; $i -lt $script:fullJpegChunks.Length; $i++) {
        $script:fullJpegChecksums[$i] = [SmoothScanlineEncoder]::ComputeChecksum2($script:fullJpegChunks[$i])
    }

    # 3. Encode Legacy Scanlines
    $script:scanlines = [SmoothScanlineEncoder]::EncodeSmooth480p($bmp, $script:previewScanBmp, $W, $H)
    for ($y = 0; $y -lt $H; $y++) {
        $script:checksums[$y] = [SmoothScanlineEncoder]::ComputeChecksum2($script:scanlines[$y])
    }

    Update-PreviewAndInfo
}

function New-PresetImage([string]$presetName) {
    $bmp = New-Object System.Drawing.Bitmap $W, $H
    $g = [System.Drawing.Graphics]::FromImage($bmp)
    $g.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
    $fontTitle = New-Object System.Drawing.Font("Segoe UI", 22, [System.Drawing.FontStyle]::Bold)
    $fontHud = New-Object System.Drawing.Font("Consolas", 15, [System.Drawing.FontStyle]::Bold)

    if ($presetName -like "Thermal IR*") {
        $skyBrush = New-Object System.Drawing.Drawing2D.LinearGradientBrush(
            (New-Object System.Drawing.Point 0, 0),
            (New-Object System.Drawing.Point 0, 320),
            [System.Drawing.Color]::FromArgb(8, 12, 55),
            [System.Drawing.Color]::FromArgb(55, 20, 120)
        )
        $g.FillRectangle($skyBrush, 0, 0, $W, 320)

        $groundBrush = New-Object System.Drawing.Drawing2D.LinearGradientBrush(
            (New-Object System.Drawing.Point 0, 260),
            (New-Object System.Drawing.Point 0, 480),
            [System.Drawing.Color]::FromArgb(15, 130, 75),
            [System.Drawing.Color]::FromArgb(5, 40, 25)
        )
        $g.FillEllipse($groundBrush, -60, 260, 450, 220)
        $g.FillEllipse($groundBrush, 240, 250, 460, 240)
        $g.FillRectangle($groundBrush, 0, 340, $W, 140)

        for ($r = 90; $r -gt 10; $r -= 8) {
            $alpha = [int]((100 - $r) * 2.2)
            $col = [System.Drawing.Color]::FromArgb($alpha, 255, [Math]::Min(255, $r * 3), 0)
            $b = New-Object System.Drawing.SolidBrush $col
            $g.FillEllipse($b, 320 - $r, 210 - $r, $r * 2, $r * 2)
            $b.Dispose()
        }
        $g.FillEllipse([System.Drawing.Brushes]::White, 308, 198, 24, 24)

        $penCyan = New-Object System.Drawing.Pen ([System.Drawing.Color]::FromArgb(200, 0, 255, 255)), 2
        $g.DrawEllipse($penCyan, 220, 110, 200, 200)
        $g.DrawLine($penCyan, 320, 80, 320, 340)
        $g.DrawLine($penCyan, 190, 210, 450, 210)

        $g.DrawString("SAR-CAM 24-BIT JPEG 480P", $fontTitle, [System.Drawing.Brushes]::Yellow, 20, 14)
        $g.DrawString("LAT: 51.5074 N  LON: 0.1278 W  ALT: 120m", $fontHud, [System.Drawing.Brushes]::Cyan, 20, 435)
    }
    elseif ($presetName -like "Drone Recon*") {
        $bgBrush = New-Object System.Drawing.Drawing2D.LinearGradientBrush(
            (New-Object System.Drawing.Point 0, 0),
            (New-Object System.Drawing.Point 640, 480),
            [System.Drawing.Color]::FromArgb(20, 75, 50),
            [System.Drawing.Color]::FromArgb(75, 135, 65)
        )
        $g.FillRectangle($bgBrush, 0, 0, $W, $H)
        $penGrid = New-Object System.Drawing.Pen ([System.Drawing.Color]::FromArgb(90, 255, 255, 255)), 2
        for ($x = 0; $x -lt $W; $x += 80) { $g.DrawLine($penGrid, $x, 0, $x, $H) }
        for ($y = 0; $y -lt $H; $y += 80) { $g.DrawLine($penGrid, 0, $y, $W, $y) }

        $penRiver = New-Object System.Drawing.Pen ([System.Drawing.Color]::FromArgb(30, 130, 230)), 34
        $g.DrawLine($penRiver, 0, 100, 640, 400)

        $g.FillEllipse([System.Drawing.Brushes]::Gold, 390, 100, 130, 130)
        $g.DrawString("H", (New-Object System.Drawing.Font("Segoe UI", 60, [System.Drawing.FontStyle]::Bold)), [System.Drawing.Brushes]::Crimson, 415, 108)
        $g.DrawString("RECON SECTOR ALPHA - 24-BIT JPEG", $fontTitle, [System.Drawing.Brushes]::White, 20, 425)
    }
    elseif ($presetName -like "Smooth Rainbow*") {
        for ($x = 0; $x -lt $W; $x++) {
            $r = [int](127 + 127 * [Math]::Sin($x * 0.01))
            $gr = [int](127 + 127 * [Math]::Sin($x * 0.01 + 2.09))
            $b = [int](127 + 127 * [Math]::Sin($x * 0.01 + 4.18))
            $p = New-Object System.Drawing.Pen ([System.Drawing.Color]::FromArgb($r, $gr, $b)), 1
            $g.DrawLine($p, $x, 0, $x, 390)
            $p.Dispose()
        }
        $g.FillRectangle([System.Drawing.Brushes]::Black, 0, 390, $W, 90)
        $g.DrawString("16.7 MILLION COLOR JPEG TEST", $fontHud, [System.Drawing.Brushes]::White, 95, 420)
    }
    elseif ($presetName -like "SOS Emergency*") {
        $g.Clear([System.Drawing.Color]::FromArgb(170, 0, 0))
        $g.FillRectangle([System.Drawing.Brushes]::Yellow, 40, 40, 560, 400)
        $g.FillRectangle([System.Drawing.Brushes]::Black, 60, 60, 520, 360)
        $fontBig = New-Object System.Drawing.Font("Consolas", 96, [System.Drawing.FontStyle]::Bold)
        $g.DrawString("S O S", $fontBig, [System.Drawing.Brushes]::Red, 110, 90)
        $g.DrawString("SURVIVOR LOCATED - 480P", $fontTitle, [System.Drawing.Brushes]::Yellow, 105, 310)
    }

    $g.Dispose()
    Convert-BitmapToScanlines $bmp
    $bmp.Dispose()
}

function Build-ScanOrder([string]$mode) {
    $plan = New-Object System.Collections.Generic.List[object]
    $script:dedupSaved = 0

    if ($mode -like "True 24-Bit JPEG Tiles*") {
        $visited = New-Object bool[] 300
        $tileBlocks = New-Object System.Collections.Generic.List[object]

        for ($ty = 0; $ty -lt 30; $ty++) {
            for ($tx = 0; $tx -lt 10; $tx++) {
                $idx = $ty * 10 + $tx
                if ($visited[$idx]) { continue }
                $body = $script:jpegTileBodies[$idx]
                $qIdx = $script:jpegTileQIdx[$idx]

                $spanW = 1
                while (($tx + $spanW) -lt 10 -and (-not $visited[$ty * 10 + $tx + $spanW]) -and ($script:jpegTileBodies[$ty * 10 + $tx + $spanW] -eq $body) -and ($script:jpegTileQIdx[$ty * 10 + $tx + $spanW] -eq $qIdx)) {
                    $spanW++
                }

                $spanH = 1
                while (($ty + $spanH) -lt 30) {
                    $allMatch = $true
                    for ($dx = 0; $dx -lt $spanW; $dx++) {
                        $checkIdx = ($ty + $spanH) * 10 + ($tx + $dx)
                        if ($visited[$checkIdx] -or ($script:jpegTileBodies[$checkIdx] -ne $body) -or ($script:jpegTileQIdx[$checkIdx] -ne $qIdx)) {
                            $allMatch = $false
                            break
                        }
                    }
                    if ($allMatch) { $spanH++ } else { break }
                }

                for ($dy = 0; $dy -lt $spanH; $dy++) {
                    for ($dx = 0; $dx -lt $spanW; $dx++) {
                        $visited[($ty + $dy) * 10 + ($tx + $dx)] = $true
                    }
                }
                $script:dedupSaved += ($spanW * $spanH - 1)

                $distCenter = [Math]::Abs($tx - 4.5) + [Math]::Abs($ty - 14.5) * 0.3
                $priority = $body.Length * 2.0 - $distCenter

                $tileBlocks.Add(@{
                    Type = "JpegTile"
                    TX = $tx
                    TY = $ty
                    SpanW = $spanW
                    SpanH = $spanH
                    Idx = $idx
                    Priority = $priority
                })
            }
        }

        if ($mode -like "*Center/Detail-First*") {
            $sorted = $tileBlocks | Sort-Object { -$_.Priority }
            foreach ($tb in $sorted) { $plan.Add($tb) }
        } else {
            foreach ($tb in $tileBlocks) { $plan.Add($tb) }
        }
    }
    elseif ($mode -like "Full-Frame 24-Bit JPEG*") {
        $total = $script:fullJpegChunks.Length
        for ($i = 0; $i -lt $total; $i++) {
            $plan.Add(@{
                Type = "FullJpeg"
                Idx = $i
                Total = $total
            })
        }
    }
    else {
        $blocks = New-Object System.Collections.Generic.List[object]
        $y = 0
        while ($y -lt $H) {
            $span = 1
            while (($y + $span) -lt $H -and ($span -lt 16) -and ($script:scanlines[$y + $span] -eq $script:scanlines[$y])) {
                $span++
                $script:dedupSaved++
            }
            $blocks.Add(@{ Type = "Scanline"; Y = $y; SpanH = $span })
            $y += $span
        }
        foreach ($b in $blocks) { $plan.Add($b) }
    }

    $script:scanPlan = $plan.ToArray()
}

$form = New-Object System.Windows.Forms.Form
$form.Text = "SAR Drone - True 24-Bit JPEG 480p LoRa Studio (16.7M Colors)"
$form.Size = New-Object System.Drawing.Size(860, 580)
$form.BackColor = [System.Drawing.Color]::FromArgb(17, 24, 39)
$form.ForeColor = [System.Drawing.Color]::White
$form.StartPosition = "CenterScreen"
$form.FormBorderStyle = "FixedSingle"
$form.MaximizeBox = $false

$fontTitle = New-Object System.Drawing.Font("Segoe UI", 11, [System.Drawing.FontStyle]::Bold)
$fontNormal = New-Object System.Drawing.Font("Segoe UI", 9, [System.Drawing.FontStyle]::Regular)
$fontBold = New-Object System.Drawing.Font("Segoe UI", 9, [System.Drawing.FontStyle]::Bold)

# --- LEFT COLUMN ---
$grpImage = New-Object System.Windows.Forms.GroupBox
$grpImage.Text = " 1. True 24-Bit JPEG 480p Source & Exact Receiver Preview "
$grpImage.ForeColor = [System.Drawing.Color]::FromArgb(16, 185, 129)
$grpImage.Font = $fontBold
$grpImage.Location = New-Object System.Drawing.Point(15, 12)
$grpImage.Size = New-Object System.Drawing.Size(460, 425)
$form.Controls.Add($grpImage)

$picPreview = New-Object System.Windows.Forms.PictureBox
$picPreview.Location = New-Object System.Drawing.Point(15, 25)
$picPreview.Size = New-Object System.Drawing.Size(430, 322)
$picPreview.SizeMode = "StretchImage"
$picPreview.BorderStyle = "FixedSingle"
$grpImage.Controls.Add($picPreview)

$cmbPreset = New-Object System.Windows.Forms.ComboBox
$cmbPreset.Location = New-Object System.Drawing.Point(15, 355)
$cmbPreset.Size = New-Object System.Drawing.Size(210, 24)
$cmbPreset.DropDownStyle = "DropDownList"
$cmbPreset.Font = $fontNormal
[void]$cmbPreset.Items.AddRange(@("Thermal IR Target (Smooth)", "Drone Recon Map (Smooth)", "Smooth Rainbow Gradient", "SOS Emergency Alert"))
$cmbPreset.SelectedIndex = 0
$grpImage.Controls.Add($cmbPreset)

$btnBrowse = New-Object System.Windows.Forms.Button
$btnBrowse.Text = "Browse Photo..."
$btnBrowse.Location = New-Object System.Drawing.Point(232, 353)
$btnBrowse.Size = New-Object System.Drawing.Size(105, 27)
$btnBrowse.BackColor = [System.Drawing.Color]::FromArgb(55, 65, 81)
$btnBrowse.ForeColor = [System.Drawing.Color]::White
$btnBrowse.FlatStyle = "Flat"
$btnBrowse.Font = $fontNormal
$grpImage.Controls.Add($btnBrowse)

$btnClearPi = New-Object System.Windows.Forms.Button
$btnClearPi.Text = "Clear Receivers"
$btnClearPi.Location = New-Object System.Drawing.Point(345, 353)
$btnClearPi.Size = New-Object System.Drawing.Size(100, 27)
$btnClearPi.BackColor = [System.Drawing.Color]::FromArgb(75, 85, 99)
$btnClearPi.ForeColor = [System.Drawing.Color]::White
$btnClearPi.FlatStyle = "Flat"
$btnClearPi.Font = $fontNormal
$grpImage.Controls.Add($btnClearPi)

$lblCompInfo = New-Object System.Windows.Forms.Label
$lblCompInfo.Text = "True 24-Bit JPEG (16.7M Colors): Photographic 480p Quality!"
$lblCompInfo.ForeColor = [System.Drawing.Color]::LightGreen
$lblCompInfo.Font = $fontNormal
$lblCompInfo.Location = New-Object System.Drawing.Point(15, 390)
$lblCompInfo.Size = New-Object System.Drawing.Size(435, 22)
$grpImage.Controls.Add($lblCompInfo)

# --- RIGHT COLUMN ---
$grpConn = New-Object System.Windows.Forms.GroupBox
$grpConn.Text = " 2. ESP32-S3 SX1262 SPI LoRa Link (868.0 MHz) "
$grpConn.ForeColor = [System.Drawing.Color]::FromArgb(59, 130, 246)
$grpConn.Font = $fontBold
$grpConn.Location = New-Object System.Drawing.Point(488, 12)
$grpConn.Size = New-Object System.Drawing.Size(345, 145)
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
    if ($ports -contains "COM6") { $cmbPort.SelectedItem = "COM6" }
    elseif ($ports -contains "COM5") { $cmbPort.SelectedItem = "COM5" }
    else { $cmbPort.SelectedIndex = ($ports.Count - 1) }
} else {
    [void]$cmbPort.Items.Add("COM6")
    $cmbPort.SelectedIndex = 0
}
$cmbPort.Add_DropDown({
    $cur = $cmbPort.Text
    $cmbPort.Items.Clear()
    $p = [System.IO.Ports.SerialPort]::GetPortNames()
    if ($p.Count -gt 0) { [void]$cmbPort.Items.AddRange($p) }
    if ($cmbPort.Items -contains $cur) { $cmbPort.SelectedItem = $cur }
})
$grpConn.Controls.Add($cmbPort)

$btnConnect = New-Object System.Windows.Forms.Button
$btnConnect.Text = "Connect ESP32"
$btnConnect.Location = New-Object System.Drawing.Point(195, 23)
$btnConnect.Size = New-Object System.Drawing.Size(135, 27)
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
$cmbLevel.Size = New-Object System.Drawing.Size(238, 24)
$cmbLevel.DropDownStyle = "DropDownList"
$cmbLevel.Font = $fontNormal
[void]$cmbLevel.Items.AddRange(@(
    "LEVEL 0 - SF12 / 125kHz (Extreme Range)",
    "LEVEL 1 - SF11 / 125kHz (Very Long Range)",
    "LEVEL 2 - SF10 / 125kHz (Long Range)",
    "LEVEL 3 - SF9  / 250kHz (Medium Range)",
    "LEVEL 4 - SF8  / 250kHz (Medium-Fast)",
    "LEVEL 5 - SF7  / 250kHz (Fast SPI LoRa)",
    "LEVEL 6 - SF7  / 500kHz (Very Fast SPI LoRa)",
    "LEVEL 7 - SF6  / 500kHz (Max Turbo SPI LoRa)"
))
$cmbLevel.SelectedIndex = 5
$grpConn.Controls.Add($cmbLevel)

$btnApplyLevel = New-Object System.Windows.Forms.Button
$btnApplyLevel.Text = "Set ESP32 Speed (+ Broadcast to Receivers)"
$btnApplyLevel.Location = New-Object System.Drawing.Point(15, 98)
$btnApplyLevel.Size = New-Object System.Drawing.Size(315, 30)
$btnApplyLevel.BackColor = [System.Drawing.Color]::FromArgb(37, 99, 235)
$btnApplyLevel.ForeColor = [System.Drawing.Color]::White
$btnApplyLevel.FlatStyle = "Flat"
$btnApplyLevel.Font = $fontBold
$grpConn.Controls.Add($btnApplyLevel)

# --- Group 3: Scan Rate & Channel Error Simulation ---
$grpSim = New-Object System.Windows.Forms.GroupBox
$grpSim.Text = " 3. One-Way Broadcast & Error Simulator (Live) "
$grpSim.ForeColor = [System.Drawing.Color]::FromArgb(245, 158, 11)
$grpSim.Font = $fontBold
$grpSim.Location = New-Object System.Drawing.Point(488, 165)
$grpSim.Size = New-Object System.Drawing.Size(345, 272)
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
$cmbOrder.Size = New-Object System.Drawing.Size(235, 24)
$cmbOrder.DropDownStyle = "DropDownList"
$cmbOrder.Font = $fontNormal
[void]$cmbOrder.Items.AddRange(@(
    "True 24-Bit JPEG Tiles: Top-to-Bottom (64x16)",
    "True 24-Bit JPEG Tiles: Center/Detail-First (64x16)",
    "Full-Frame 24-Bit JPEG: Progressive 640x480",
    "Legacy Scanlines: 4,096-Color RLE (No Pi Update)"
))
$cmbOrder.SelectedIndex = 0
$grpSim.Controls.Add($cmbOrder)

$cmbOrder.Add_SelectedIndexChanged({
    Update-PreviewAndInfo
    if ($script:isStreaming) {
        $script:queueIndex = 0
        $script:currentPass = 1
    }
})

$lblDelay = New-Object System.Windows.Forms.Label
$lblDelay.Text = "Packet Interval (Delay): 55 ms / packet (~18.2 pkt/s)"
$lblDelay.ForeColor = [System.Drawing.Color]::White
$lblDelay.Font = $fontNormal
$lblDelay.Location = New-Object System.Drawing.Point(15, 58)
$lblDelay.Size = New-Object System.Drawing.Size(315, 20)
$grpSim.Controls.Add($lblDelay)

$trkDelay = New-Object System.Windows.Forms.TrackBar
$trkDelay.Location = New-Object System.Drawing.Point(15, 76)
$trkDelay.Size = New-Object System.Drawing.Size(315, 35)
$trkDelay.Minimum = 18
$trkDelay.Maximum = 1500
$trkDelay.TickFrequency = 50
$trkDelay.Value = 55
$trkDelay.AutoSize = $false
$grpSim.Controls.Add($trkDelay)

$lblDrop = New-Object System.Windows.Forms.Label
$lblDrop.Text = "Simulated Missed Packets (Drop %): 0%"
$lblDrop.ForeColor = [System.Drawing.Color]::FromArgb(252, 165, 165)
$lblDrop.Font = $fontNormal
$lblDrop.Location = New-Object System.Drawing.Point(15, 114)
$lblDrop.Size = New-Object System.Drawing.Size(315, 20)
$grpSim.Controls.Add($lblDrop)

$trkDrop = New-Object System.Windows.Forms.TrackBar
$trkDrop.Location = New-Object System.Drawing.Point(15, 132)
$trkDrop.Size = New-Object System.Drawing.Size(315, 35)
$trkDrop.Minimum = 0
$trkDrop.Maximum = 80
$trkDrop.TickFrequency = 10
$trkDrop.Value = 0
$trkDrop.AutoSize = $false
$grpSim.Controls.Add($trkDrop)

$lblNoise = New-Object System.Windows.Forms.Label
$lblNoise.Text = "Simulated Radio Static (Noise %): 0%"
$lblNoise.ForeColor = [System.Drawing.Color]::FromArgb(253, 224, 71)
$lblNoise.Font = $fontNormal
$lblNoise.Location = New-Object System.Drawing.Point(15, 170)
$lblNoise.Size = New-Object System.Drawing.Size(315, 20)
$grpSim.Controls.Add($lblNoise)

$trkNoise = New-Object System.Windows.Forms.TrackBar
$trkNoise.Location = New-Object System.Drawing.Point(15, 188)
$trkNoise.Size = New-Object System.Drawing.Size(315, 35)
$trkNoise.Minimum = 0
$trkNoise.Maximum = 60
$trkNoise.TickFrequency = 10
$trkNoise.Value = 0
$trkNoise.AutoSize = $false
$grpSim.Controls.Add($trkNoise)

$chkSelfHeal = New-Object System.Windows.Forms.CheckBox
$chkSelfHeal.Text = "Keep Errors Active on Pass 2+ (Receivers Self-Lock Clean Tiles!)"
$chkSelfHeal.ForeColor = [System.Drawing.Color]::LightGreen
$chkSelfHeal.Font = $fontNormal
$chkSelfHeal.Location = New-Object System.Drawing.Point(18, 230)
$chkSelfHeal.Size = New-Object System.Drawing.Size(320, 24)
$chkSelfHeal.Checked = $false
$grpSim.Controls.Add($chkSelfHeal)

# --- BOTTOM BAR ---
$btnStream = New-Object System.Windows.Forms.Button
$btnStream.Text = "START 24-BIT JPEG 480P STREAM"
$btnStream.Location = New-Object System.Drawing.Point(15, 448)
$btnStream.Size = New-Object System.Drawing.Size(460, 45)
$btnStream.BackColor = [System.Drawing.Color]::FromArgb(16, 185, 129)
$btnStream.ForeColor = [System.Drawing.Color]::Black
$btnStream.FlatStyle = "Flat"
$btnStream.Font = $fontTitle
$form.Controls.Add($btnStream)

$lblStatus = New-Object System.Windows.Forms.Label
$lblStatus.Text = "Status: Disconnected. Click 'Connect ESP32' to open COM6."
$lblStatus.ForeColor = [System.Drawing.Color]::Cyan
$lblStatus.Font = $fontBold
$lblStatus.Location = New-Object System.Drawing.Point(490, 448)
$lblStatus.Size = New-Object System.Drawing.Size(345, 45)
$form.Controls.Add($lblStatus)

$lblStats = New-Object System.Windows.Forms.Label
$lblStats.Text = "Pass: - | Sent: 0 | Dropped: 0 | Static: 0 | Dedup Saved: 0"
$lblStats.ForeColor = [System.Drawing.Color]::LightGray
$lblStats.Font = $fontNormal
$lblStats.Location = New-Object System.Drawing.Point(15, 502)
$lblStats.Size = New-Object System.Drawing.Size(810, 25)
$form.Controls.Add($lblStats)

$timer = New-Object System.Windows.Forms.Timer
$timer.Interval = 220

$trkDelay.Add_Scroll({
    $timer.Interval = $trkDelay.Value
    $lps = [Math]::Round(1000.0 / $trkDelay.Value, 1)
    $lblDelay.Text = "Packet Interval (Delay): $($trkDelay.Value) ms / packet (~$lps pkt/s)"
})

$trkDrop.Add_Scroll({
    $lblDrop.Text = "Simulated Missed Packets (Drop %): $($trkDrop.Value)%"
})

$trkNoise.Add_Scroll({
    $lblNoise.Text = "Simulated Radio Static (Noise %): $($trkNoise.Value)%"
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
        $btnStream.Text = "START 24-BIT JPEG 480P STREAM"
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
        $script:serialPort.DtrEnable = $true
        $script:serialPort.RtsEnable = $true
        $script:serialPort.WriteTimeout = 500
        $script:serialPort.ReadTimeout = 500
        $script:serialPort.Open()
        $lblStatus.Text = "Opening $portName... initializing SX1262..."
        $form.Refresh()
        Start-Sleep -Milliseconds 1200
        $null = $script:serialPort.ReadExisting()

        $lvl = $cmbLevel.SelectedIndex
        $script:serialPort.Write("AT+LEVEL$lvl`r`n")
        Start-Sleep -Milliseconds 250
        $resp = $script:serialPort.ReadExisting().Trim()

        $btnConnect.Text = "Disconnect"
        $btnConnect.BackColor = [System.Drawing.Color]::FromArgb(239, 68, 68)
        $lblStatus.Text = "Connected on $portName (SX1262 LEVEL $lvl)! $resp"
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

    $lblStatus.Text = "Setting ESP32 & Receivers to LEVEL $lvl..."
    $form.Refresh()

    $script:serialPort.Write("CMD:LEVEL:$lvl`r`n")
    Start-Sleep -Milliseconds 300
    $script:serialPort.Write("CMD:LEVEL:$lvl`r`n")
    Start-Sleep -Milliseconds 300

    $script:serialPort.Write("AT+LEVEL$lvl`r`n")
    Start-Sleep -Milliseconds 250
    $null = $script:serialPort.ReadExisting()

    $recDelays = @(1100, 600, 320, 140, 85, 55, 35, 25)
    $trkDelay.Value = $recDelays[$lvl]
    $timer.Interval = $trkDelay.Value
    $lps = [Math]::Round(1000.0 / $trkDelay.Value, 1)
    $lblDelay.Text = "Packet Interval (Delay): $($trkDelay.Value) ms / packet (~$lps pkt/s)"

    $lblStatus.Text = "ESP32 & Pi SX1262 now on LEVEL $lvl!"
    if ($wasStreaming) { $timer.Start() }
})

$btnClearPi.Add_Click({
    if ($script:serialPort -and $script:serialPort.IsOpen) {
        $script:serialPort.Write("CMD:CLEAR`r`n")
        $lblStatus.Text = "Broadcasted CMD:CLEAR to all receivers!"
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
        $btnStream.Text = "START 24-BIT JPEG 480P STREAM"
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
        $lblStatus.Text = "Streaming Pass 1 ($($script:scanPlan.Length) packets)..."
        $timer.Start()
    }
})

$timer.Add_Tick({
    if (-not ($script:serialPort -and $script:serialPort.IsOpen)) {
        $timer.Stop()
        return
    }

    if ($script:serialPort.BytesToWrite -gt 0) {
        $timer.Interval = 25
        return
    }
    if ($script:serialPort.BytesToRead -gt 0) {
        $null = $script:serialPort.ReadExisting()
    }

    if ($script:queueIndex -ge $script:scanPlan.Length) {
        $script:currentPass++
        $script:queueIndex = 0
        Build-ScanOrder $cmbOrder.SelectedItem.ToString()
    }

    $item = $script:scanPlan[$script:queueIndex]
    $script:queueIndex++

    $useErrors = ($script:currentPass -eq 1) -or $chkSelfHeal.Checked

    if ($useErrors -and ($script:rng.Next(0, 100) -lt $trkDrop.Value)) {
        $script:dropCount++
        $lblStatus.Text = "Pass $($script:currentPass): [DROPPED] Packet $($script:queueIndex)/$($script:scanPlan.Length)"
        $lblStats.Text = "Pass: $($script:currentPass) | Sent: $($script:sentCount) | Dropped: $($script:dropCount) | Static: $($script:noiseCount) | Dedup Saved: $($script:dedupSaved)"
        return
    }

    if ($item.Type -eq "JpegTile") {
        $idx = $item.Idx
        $bodyStr = $script:jpegTileBodies[$idx]
        $qIdx = $script:jpegTileQIdx[$idx]
        $chkStr = $script:jpegTileChecksums[$idx]

        if ($useErrors -and ($trkNoise.Value -gt 0) -and ($script:rng.Next(0, 100) -lt $trkNoise.Value)) {
            $chars = $bodyStr.ToCharArray()
            if ($chars.Length -gt 8) {
                $pos = $script:rng.Next(4, $chars.Length - 2)
                $chars[$pos] = $b64Chars[$script:rng.Next(0, 64)]
                $bodyStr = -join $chars
                $script:noiseCount++
            }
        }

        # True 24-Bit JPEG 64x16 Tile Packet: J:<tx>:<ty>:<spanW>:<spanH>:<qIdx>:<chk2>:<b64EntropyBody>
        $pkt = "J:$($item.TX):$($item.TY):$($item.SpanW):$($item.SpanH):$qIdx`:$chkStr`:$bodyStr`r`n"
        $desc = "24-Bit JPEG Tile ($($item.TX),$($item.TY))"
    }
    elseif ($item.Type -eq "FullJpeg") {
        $idx = $item.Idx
        $total = $item.Total
        $chunkStr = $script:fullJpegChunks[$idx]
        $chkStr = $script:fullJpegChecksums[$idx]

        if ($useErrors -and ($trkNoise.Value -gt 0) -and ($script:rng.Next(0, 100) -lt $trkNoise.Value)) {
            $chars = $chunkStr.ToCharArray()
            if ($chars.Length -gt 8) {
                $pos = $script:rng.Next(2, $chars.Length - 2)
                $chars[$pos] = $b64Chars[$script:rng.Next(0, 64)]
                $chunkStr = -join $chars
                $script:noiseCount++
            }
        }

        # Full-Frame 24-Bit JPEG Chunk Packet: F:<idx>:<total>:<chk2>:<b64Chunk>
        $pkt = "F:$idx`:$total`:$chkStr`:$chunkStr`r`n"
        $desc = "Full JPEG Chunk $($idx + 1)/$total"
    }
    else {
        $y = $item.Y
        $spanH = $item.SpanH
        $lineStr = $script:scanlines[$y]
        $chkStr = $script:checksums[$y]

        if ($useErrors -and ($trkNoise.Value -gt 0)) {
            $chars = $lineStr.ToCharArray()
            $corrupted = $false
            for ($i = 0; $i + 2 -lt $chars.Length; $i += 3) {
                if ($script:rng.Next(0, 100) -lt $trkNoise.Value) {
                    $chars[$i + 1] = $b64Chars[$script:rng.Next(0, 64)]
                    $chars[$i + 2] = $b64Chars[$script:rng.Next(0, 64)]
                    $corrupted = $true
                }
            }
            if ($corrupted) {
                $script:noiseCount++
                $lineStr = -join $chars
            }
        }

        $pkt = "S:$y`:$spanH`:$chkStr`:$lineStr`r`n"
        $desc = "Row $y (Span $($spanH)px)"
    }

    try {
        $script:serialPort.Write($pkt)
        $script:sentCount++
        $lvl = $cmbLevel.SelectedIndex
        $baseMs = @(1200, 650, 350, 110, 60, 35, 22, 18)[$lvl]
        $perByteMs = @(14.5, 7.5, 4.0, 1.15, 0.65, 0.85, 0.45, 0.28)[$lvl]
        $minAirMs = [int]($baseMs + ($pkt.Length * $perByteMs))
        $userScaledMs = [int]($trkDelay.Value * [Math]::Max(0.45, $pkt.Length / 95.0))
        $timer.Interval = [Math]::Max($minAirMs, $userScaledMs)
        if ($script:sentCount % 2 -eq 0) {
            $lblStatus.Text = "Pass $($script:currentPass): $desc [$($script:queueIndex)/$($script:scanPlan.Length)]"
            $lblStats.Text = "Pass: $($script:currentPass) | Sent: $($script:sentCount) | Dropped: $($script:dropCount) | Static: $($script:noiseCount) | Dedup Saved: $($script:dedupSaved)"
        }
    }
    catch [System.TimeoutException] {
        $timer.Interval = 60
    }
    catch {
        $timer.Stop()
        $lblStatus.Text = "Serial write error: $($_.Exception.Message)"
    }
})

$form.Add_FormClosing({
    $timer.Stop()
    if ($script:serialPort -and $script:serialPort.IsOpen) {
        $script:serialPort.Close()
    }
})

New-PresetImage "Thermal IR Target (Smooth)"

[void]$form.ShowDialog()
