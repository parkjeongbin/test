#!/usr/bin/env python3
import tkinter as tk
import subprocess

# 이동할 YAML 파일 목록
yaml_files = ["move1.yaml", "move2.yaml"]
current_yaml_index = 0  # 현재 선택된 YAML 파일 인덱스

def on_button_click_1():  # 터틀봇 실행 버튼
    global current_yaml_index
    label.config(text="터틀봇이 이동 중입니다.\n 이동이 완료되면 이동 버튼을 다시 눌러주세요. \n 이동중 문제가 생기면 원위치 버튼을 눌러주세요")
    
    if current_yaml_index < len(yaml_files):
        # 실행할 명령어 정의
        command = f"/opt/ros/noetic/bin/rostopic pub /move_base/goal move_base_msgs/MoveBaseActionGoal -f {yaml_files[current_yaml_index]}"

        # 명령어 실행
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", command])
        current_yaml_index += 1
    else:
        label.config(text="모든 이동이 완료되었습니다.")

def on_button_click_2():  # 터틀봇 원상 복구 버튼
    label.config(text="터틀봇에 문제가 생겨 초기 위치로 복귀합니다.")
    global current_yaml_index
    current_yaml_index = 0

# 메인 창 생성
root = tk.Tk()
root.geometry("500x500")
root.title("버튼 페이지")

# 레이블 생성
label = tk.Label(root, text="안녕하세요!\n옆에 작성되어있는 지도를 참조해서 로봇을 움직일 수 있어요\n움직이기 버튼을 통해 특정 목적지로 보낼수 있습니다.\n 원위치 복귀 버튼을 누르면 초기 위치로 돌아옵니다.")
label.pack(pady=10)

# 버튼 생성
button_1 = tk.Button(root, text="터틀봇 움직이기", width=30, height=3, command=on_button_click_1)
button_1.pack(pady=20)
button_2 = tk.Button(root, text="터틀봇 원위치 복귀", width=30, height=3, command=on_button_click_2)
button_2.pack(pady=10)

# 창 실행
root.mainloop()
