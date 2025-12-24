#!/usr/bin/bash
cd
wt.exe -p wsl bash -c "source ~/.venv/bin/activate && cd && cd airsim-unreal5.5.4 && python3 connector.py" `
wt.exe -p wsl bash -c "source ~/.venv/bin/activate && cd && cd airsim-unreal5.5.4 && python3 collision.py" `
wt.exe -p wsl bash -c "source ~/.venv/bin/activate && cd && cd airsim-unreal5.5.4 && python3 gimbal.py" `
#wt.exe -p wsl bash -c "source ~/.venv/bin/activate && cd && cd airsim-unreal5.5.4 && python3 create_logs.py" `
wt.exe -p wsl bash -c "source  ~/.venv/bin/activate && cd && cd cosys-airsim && python3 dvd.py"

