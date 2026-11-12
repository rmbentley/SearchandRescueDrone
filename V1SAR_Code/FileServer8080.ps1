$port = 8080
$root = "C:\Users\eyres\.gemini\antigravity\scratch\lora_image_studio"
$endpoint = New-Object System.Net.IPEndPoint([System.Net.IPAddress]::Any, $port)
$listener = New-Object System.Net.Sockets.TcpListener($endpoint)
$listener.Start()

while ($true) {
    $client = $listener.AcceptTcpClient()
    try {
        $stream = $client.GetStream()
        $reader = New-Object System.IO.StreamReader($stream, [System.Text.Encoding]::ASCII, $false, 1024, $true)
        $requestLine = $reader.ReadLine()
        while ($true) {
            $hdr = $reader.ReadLine()
            if ([string]::IsNullOrEmpty($hdr)) { break }
        }
        if ($requestLine -match "^GET\s+/([^\s\?]*)\s+HTTP") {
            $rel = [System.Uri]::UnescapeDataString($matches[1])
            if ([string]::IsNullOrWhiteSpace($rel)) {
                $files = Get-ChildItem -Path $root -File | Where-Object { $_.Name -ne "FileServer8080.ps1" }
                $html = "<html><head><title>SAR Drone LoRa Files</title><style>body{font-family:sans-serif;background:#0f172a;color:#fff;padding:24px} a{display:inline-block;margin:8px 0;padding:12px 18px;background:#10b981;color:#000;font-weight:bold;text-decoration:none;border-radius:8px}</style></head><body><h2>Download SAR Drone Files to Raspberry Pi</h2>"
                foreach ($f in $files) {
                    $html += "<div><a href='/$($f.Name)' download='$($f.Name)'>Download $($f.Name) ($([Math]::Round($f.Length/1024.0, 1)) KB)</a></div>"
                }
                $html += "</body></html>"
                $body = [System.Text.Encoding]::UTF8.GetBytes($html)
                $header = "HTTP/1.1 200 OK`r`nContent-Type: text/html; charset=utf-8`r`nContent-Length: $($body.Length)`r`nConnection: close`r`n`r`n"
                $hdrBytes = [System.Text.Encoding]::ASCII.GetBytes($header)
                $stream.Write($hdrBytes, 0, $hdrBytes.Length)
                $stream.Write($body, 0, $body.Length)
            } else {
                $filePath = Join-Path $root $rel
                if (Test-Path $filePath -PathType Leaf) {
                    $body = [System.IO.File]::ReadAllBytes($filePath)
                    $fname = [System.IO.Path]::GetFileName($filePath)
                    $header = "HTTP/1.1 200 OK`r`nContent-Type: application/octet-stream`r`nContent-Disposition: attachment; filename=`"$fname`"`r`nContent-Length: $($body.Length)`r`nConnection: close`r`n`r`n"
                    $hdrBytes = [System.Text.Encoding]::ASCII.GetBytes($header)
                    $stream.Write($hdrBytes, 0, $hdrBytes.Length)
                    $stream.Write($body, 0, $body.Length)
                } else {
                    $msg = [System.Text.Encoding]::ASCII.GetBytes("HTTP/1.1 404 Not Found`r`nContent-Length: 9`r`nConnection: close`r`n`r`nNot Found")
                    $stream.Write($msg, 0, $msg.Length)
                }
            }
        }
        $stream.Flush()
    } catch {}
    finally {
        $client.Close()
    }
}
