# UR3 LLM Control — Bài thực hành 02

Package tạo luồng `natural language → 9Router → JSON plan → validator → robot skills → MoveIt 2`.
LLM chỉ được yêu cầu chọn `pick(object)`, `place(object, zone)`, `home()`; `PlanValidator`
từ chối skill, object, zone hoặc tham số ngoài allow-list trước khi robot chạy.

## Cấu hình và chạy

```bash
export NINEROUTER_API_KEY='your-9router-key'
cd ~/workspaces/ur_gz
source /opt/ros/humble/setup.bash
colcon build --packages-select ur3_llm_control
source install/setup.bash
ros2 launch ur3_llm_control llm_robot.launch.py \
  command:='Put the red cube in zone B.' api_key:=$NINEROUTER_API_KEY
```


Để kiểm tra JSON/validator mà không đưa chuyển động vào thực thi, truyền `execute:=false`.

## Gripper mô phỏng

URDF có gripper hai ngón với hai khớp prismatic được điều khiển bởi `gripper_controller`.
`pick` mở ngàm, tiếp cận khối, đóng ngàm rồi attach collision object vào `tool0`; `place` đưa
khối tới zone, mở ngàm và detach. Cube trong Gazebo là vật thể động để tiếp xúc với ngàm.
Các tọa độ và vật thể được khai báo trong `worlds/task_world.sdf` và `config/scene.yaml`.

Skill gửi goal đến MoveGroup action của MoveIt để lập kế hoạch và thực thi chuyển động tay máy;
ngàm được điều khiển riêng qua FollowJointTrajectory. Mô phỏng gắp vật phụ thuộc tham số tiếp
xúc/friction của Gazebo và có thể cần tinh chỉnh nếu khối trượt khỏi ngàm.
