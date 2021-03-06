$url = "https://github.com/andrey-val-rodin/RotatingTable.Arduino/archive/refs/heads/main.zip"
$archive = "./main.zip"
$name = "./RotatingTable.Arduino-main"

# Download files
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
try
{
    Invoke-WebRequest -Uri $url -OutFile $archive
}
catch
{
    Write-Host $PSItem.Exception -ForegroundColor RED
    return
}

# Delete old files
Remove-Item -Path $name -Force -Recurse -ErrorAction Ignore

# Expand zip
Expand-Archive $archive "./"

# Delete zip
Remove-Item -Path $archive -Force -Recurse