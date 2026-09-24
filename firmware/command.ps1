$file = ".\BlueBot_v1.0.11.py"
$size = (Get-Item $file).Length
$hash = (Get-FileHash $file -Algorithm SHA256).Hash.ToLower()

Write-Host "fileSize:" $size
Write-Host "sha256:" $hash