cd c:\
md repos
cd c:\repos
py -3 -m pip install robotpy
py -3 -m pip install --upgrade robotpy
 
git clone https://github.com/Team-4536/FRC-2024.git

cd c:\repos\FRC-2024\src

py -3 -m robotpy sync 


start https://www.ni.com/en/support/downloads/drivers/download/packaged.frc-game-tools.500107.html

cd C:\Users\minut\Downloads

start ni-frc-2024-game-tools_24.0_online.exe