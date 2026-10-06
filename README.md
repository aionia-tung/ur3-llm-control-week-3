# Mô phỏng UR3e: MoveIt 2, Tay kẹp (Gripper) và Lập kế hoạch tác vụ bằng LLM

Không gian làm việc (workspace) ROS 2 Humble phục vụ mô phỏng robot UR3e trong Gazebo, lập kế hoạch chuyển động với MoveIt 2 và thực thi các tác vụ gắp - đặt (pick-and-place) được mô tả bằng ngôn ngữ tự nhiên. Kho lưu trữ này cũng bao gồm bản demo hoạch định quỹ đạo Descartes (Cartesian path) vẽ chữ **L**.

---

## 1. Các thành phần bao gồm

- `src/ur3_draw_letter`: Mô tả/cấu hình UR3e, mô phỏng tay kẹp hai ngón (gripper), và demo MoveIt vẽ chữ L bằng quỹ đạo Descartes.
- `src/ur3_llm_control`: Bộ lập kế hoạch LLM (LLM planner), bộ kiểm tra tính hợp lệ của kế hoạch (plan validator), các kỹ năng robot (skills), thế giới tác vụ Gazebo và file launch kết hợp mô phỏng / MoveIt.
- `src/ur_simulation_gz`: Hỗ trợ mô phỏng Gazebo và file launch MoveIt cho dòng Universal Robots, được tích hợp sẵn dưới dạng mã nguồn trong kho lưu trữ này.

Mô hình LLM chỉ có nhiệm vụ sinh ra một chuỗi kế hoạch JSON ngắn gọn sử dụng 3 kỹ năng cơ bản: `pick`, `place`, và `home`. Bộ kiểm tra tính hợp lệ (validator) sẽ từ chối các kỹ năng lạ, vật thể lạ, vùng (zone) lạ và các tham số thừa trước khi tiến hành thực thi. MoveIt đảm nhiệm lập kế hoạch chuyển động của cánh tay; bộ điều khiển tay kẹp điều khiển các ngón kẹp; Gazebo mô phỏng va chạm tiếp xúc và liên kết vật lý của vật thể.

Với mã số sinh viên `23020770`, quy tắc ánh xạ mặc định được cấu hình là:

| Vùng (Zone) | Vật thể (Object) |
| --- | --- |
| A | Khối lập phương đỏ (Red cube) |
| B | Khối lập phương xanh dương (Blue cube) |
| C | Khối lập phương vàng (Yellow cube) |

---

## 2. Yêu cầu hệ thống

- Hệ điều hành: Ubuntu 22.04 LTS với ROS 2 Humble.
- Công cụ: `colcon`, `rosdep`, MoveIt 2, tích hợp Gazebo/Ignition (Ignition Fortress / Gazebo Sim), cùng các gói mô tả và cấu hình MoveIt tiêu chuẩn của Universal Robots.
- Dịch vụ 9Router hoặc một cổng OpenAI-compatible chat-completions endpoint (ví dụ: `http://localhost:20128/v1/chat/completions`) đang hoạt động.

### Cài đặt phụ thuộc và biên dịch workspace

Cài đặt các gói phụ thuộc được khai báo trong workspace bằng `rosdep`:

```bash
cd ~/ur3_buoi2
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y