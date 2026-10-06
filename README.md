# Mô phỏng UR3e: MoveIt 2, Gripper và Lập kế hoạch tác vụ bằng LLM — Bài thực hành 03

Không gian làm việc (workspace) ROS 2 Humble phát triển tiếp từ Bài 02, phục vụ bài toán điều khiển robot UR3e thao tác gắp - đặt tự động trong môi trường mô phỏng Gazebo Sim, tích hợp nhận diện trạng thái môi trường qua Camera Perception, giải quyết xung đột chiếm chỗ (Zone Conflict Resolution), hoạch định quỹ đạo với MoveIt 2 và lập kế hoạch tác vụ bằng LLM Planner.

---

## 1. Kiến trúc hệ thống

Hệ thống tuân thủ luồng kiến trúc phân cấp:

```text
Natural Language Command
        ↓
    LLM Planner  ← (Trạng thái môi trường từ Camera Perception)
        ↓
 Structured Plan (JSON)
        ↓
   Plan Validator (Kiểm tra kế hoạch trước khi chạy)
        ↓
   Robot Skills (check_zone, pick, place, home,...)
        ↓
     MoveIt 2 (Lập kế hoạch quỹ đạo không va chạm)
        ↓
 UR3/UR3e + Gripper (Gazebo Sim)
LLM Planner: Được sử dụng để hiểu yêu cầu của người dùng, phân tích thông tin môi trường động do Camera gửi về, lựa chọn skill, xác định tham số và sắp xếp thứ tự thực hiện hợp lý. Tuyệt đối không sinh trực tiếp joint trajectory hoặc lệnh điều khiển joint.Plan Validator: Kiểm tra kế hoạch sinh ra trước khi thực thi nhằm đảm bảo chỉ có các skill hợp lệ, đối tượng hợp lệ và tham số an toàn được phép chuyển xuống robot.MoveIt 2 & Robot Skills: Đảm nhiệm bài toán động học ngược (IK), tránh va chạm (collision checking) và thực thi quỹ đạo chuyển động.2. Môi trường mô phỏng (Gazebo Task World)Môi trường mô phỏng trong Gazebo bao gồm:01 UR3/UR3e.01 Gripper: Tay kẹp hai ngón thực hiện gắp, giữ, di chuyển và thả vật thể động thật trong Gazebo (điều khiển qua gripper_controller, không dùng kỹ thuật thay đổi trực tiếp pose của object để gian lận thao tác gắp).01 Camera (overhead_camera): Camera quan sát gắn trên cao để theo dõi bàn làm việc.01 Bàn thao tác (Worktable).03 Vùng đặt vật (Zones):zone_a: (0.54, -0.24, 0.526)zone_b: (0.54, 0.00, 0.526)zone_c: (0.54, 0.24, 0.526)05 Khối hộp (Blocks/Cubes):red_cubeyellow_cubeblue_cubegreen_cubepurple_cube3. Camera và Nhận diện trạng thái môi trườngHệ thống bắt buộc sử dụng Camera thông qua node Perception (camera_perception_node) để lấy thông tin thời gian thực về các block và trạng thái của các zone:Nhận diện tọa độ thời gian thực của từng block trên bàn thao tác.Xác định zone nào đang trống và zone nào đang bị chiếm bởi block nào.Hệ thống không khai báo cố định toàn bộ vị trí vật rồi dùng trực tiếp giá trị đó để lập kế hoạch mà cập nhật động qua topic /world_state.4. Danh mục Skill và Logic xử lý xung độtDo chỉ có 03 zone nhưng có tới 05 block, trong quá trình vận hành zone đích có thể đang bị chiếm đóng bởi một vật thể khác. Robot phải kiểm tra trạng thái môi trường và dọn dẹp vật cản trước khi thực hiện hành động chính.Danh mục Robot SkillsSkillTham sốChức năngcheck_zonezoneĐọc dữ liệu từ camera để kiểm tra trạng thái zone (trống hay đang bị chiếm).pickobjectMở ngàm, hạ tiếp cận, kẹp giữ vật thể bằng gripper và nhấc lên.placeobject, zoneDi chuyển đến zone/vị trí chỉ định, hạ xuống, nhả ngàm và rút gripper về vị trí an toàn.homeKhôngĐưa các khớp của cánh tay robot về tư thế nghỉ chuẩn an toàn.Vị trí tạm thời (temporary_position): Khi zone đích bị chiếm, hệ thống xác định một vị trí trống phù hợp trên bàn ((0.40, 0.22, 0.526)) làm nơi trung chuyển để giải phóng zone đích.5. Kịch bản Demo bắt buộcDemo trường hợp robot không thể thực hiện trực tiếp pick → place mà phải xử lý giải phóng vật chiếm chỗ:Tình trạng ban đầu: Khối blue_cube đang nằm chiếm đóng tại zone_b.Yêu cầu người dùng: "Put the red cube in zone B."Quy trình xử lý của hệ thống:Camera phát hiện zone_b đang bị chiếm bởi blue_cube.Xác định vị trí tạm temporary_position phù hợp trên bàn.Di chuyển blue_cube ra vị trí tạm.Gắp red_cube.Đặt red_cube vào zone_b.Robot về vị trí home.Chuỗi kế hoạch (Structured Plan) do LLM sinh ra:JSON[
  {"skill": "check_zone", "zone": "zone_b"},
  {"skill": "pick", "object": "blue_cube"},
  {"skill": "place", "object": "blue_cube", "zone": "temporary_position"},
  {"skill": "pick", "object": "red_cube"},
  {"skill": "place", "object": "red_cube", "zone": "zone_b"},
  {"skill": "home"}
]
6. Cài đặt và Biên dịch WorkspaceBashcd ~/ur3-llm-control-main
source /opt/ros/humble/setup.bash

# Cài đặt phụ thuộc cần thiết
rosdep install --from-paths src --ignore-src -r -y

# Biên dịch package
colcon build --symlink-install --packages-select ur3_llm_control
source install/setup.bash
7. Hướng dẫn khởi chạyTrước khi chạy, dọn sạch tiến trình nền cũ để tránh xung đột cổng truyền thông ROS 2 và controller:Bash# 1. Dọn dẹp tiến trình treo
ros2 daemon stop
killall -9 gz-sim-gui gz-sim-server ruby ign ros2 python3 rviz2 spawner 2>/dev/null
sleep 2
ros2 daemon start

# 2. Khởi chạy hệ thống với câu lệnh demo
source /opt/ros/humble/setup.bash
source ~/ur3-llm-control-main/install/setup.bash

ros2 launch ur3_llm_control llm_robot.launch.py \
  command:="Put the red cube in zone B." \
  endpoint:=http://localhost:20128/v1/chat/completions \
  model:=gemini/gemini-3.6-flash \
  student_id:="23020770" \
  api_key:="sk-fb0d4c642adebee4-rqckie-c213ad64"
Các đối số Launch (Launch Arguments)command: Câu lệnh ngôn ngữ tự nhiên giao việc cho robot.endpoint: Địa chỉ API của LLM Gateway/Router.model: Mô hình LLM xử lý ngôn ngữ tự nhiên.student_id: Mã sinh viên dùng để tính toán hoán vị ánh xạ zone (23020770).execute: Mặc định là true. Đặt false nếu chỉ muốn LLM sinh JSON Plan và kiểm thử qua Validator mà không gửi quỹ đạo đến robot.8. Xử lý sự cố thường gặp (Troubleshooting)Gazebo Physics Clock bị dừng: Đảm bảo nút ở góc dưới bên trái của Gazebo đang ở trạng thái Play (▶) để đồng hồ thời gian /clock hoạt động trơn tru.Lỗi spawner_gripper_controller timed out: Xuất hiện khi có tiến trình nền cũ chưa tắt hết. Chạy khối lệnh dọn dẹp ở Mục 7 rồi khởi chạy lại.Cartesian Planning nới lỏng va chạm khi nhấc: Động tác nhấc thẳng đứng sau khi kẹp vật thể được cấu hình cho phép tiếp xúc cục bộ với phôi (avoid_collisions=False) để tránh MoveIt bị kẹt giải pháp quỹ đạo (0% trajectory).
