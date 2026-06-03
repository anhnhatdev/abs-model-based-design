# Hướng Dẫn Vẽ Plant Model trong Simulink — Từng Bước

> Cập nhật: 2026-08-01. Tài liệu song hành với `PROJECT_PROGRESS_PLAN.md` Task 2.5.

> Tài liệu này hướng dẫn thao tác tay trong Simulink GUI.
> Thực hiện tuần tự từ trên xuống. Không bỏ qua bước nào.

---

## Chuẩn bị trước khi mở Simulink

### Bước 0.1 — Load tham số vào workspace

Trong **MATLAB Command Window**, gõ:

```matlab
cd('c:\product\abs-model-based-design')
run('scripts/utils/init_params.m')
```

Kiểm tra kết quả: workspace phải có các biến sau (Home > Workspace hoặc gõ `whos`):

| Biến | Giá trị | Đơn vị |
|---|---|---|
| `m_vehicle` | 612.5 | kg |
| `R_wheel` | 0.385 | m |
| `J_wheel` | 2.80 | kg·m² |
| `K_brake` | 22.0 | N·m/bar |
| `v0` | 27.78 | m/s |
| `Ts` | 0.001 | s |
| `T_sim` | 10.0 | s |

Nếu thiếu bất kỳ biến nào → dừng lại, kiểm tra `init_params.m`.

---

### Bước 0.2 — Tạo model mới và lưu đúng chỗ

1. Trên MATLAB Command Window, gõ: `new_system('plant_model'); open_system('plant_model')`
2. Một cửa sổ Simulink trắng mở ra, tiêu đề là `plant_model`
3. Ngay lập tức lưu vào đúng thư mục:
   - Nhấn **Ctrl + Shift + S** (Save As)
   - Điều hướng đến: `c:\product\abs-model-based-design\models\plant\`
   - Filename: `plant_model`
   - Click **Save**

> Nếu file `plant_model.slx` đã tồn tại trong thư mục đó: xóa nó trước bằng File Explorer, sau đó thực hiện lại bước trên.

---

### Bước 0.3 — Cấu hình Solver

1. Trên thanh menu Simulink: **Modeling > Model Settings** (hoặc nhấn **Ctrl + E**)
2. Panel bên trái: chọn **Solver**
3. Đặt các giá trị:

| Trường | Giá trị cần đặt |
|---|---|
| **Type** | `Fixed-step` |
| **Solver** | `ode4 (Runge-Kutta)` |
| **Fixed-step size** | `0.001` |
| **Stop time** (ở panel Simulation) | `10` |

4. Click **OK**

---

## Phần 1 — Vẽ Subsystem `Vehicle_Dynamics`

### Bước 1.1 — Thêm Subsystem block vào canvas

1. Trong canvas `plant_model`, nhấn đúp chuột vào vùng trống (hoặc nhấn phím **Ctrl + Shift + L** để mở Library Browser)
2. Gõ vào ô tìm kiếm: `Subsystem`
3. Kéo block **Subsystem** (trong Ports & Subsystems) vào canvas, đặt ở vị trí khoảng giữa-trên
4. **Đổi tên block:** Click 1 lần vào chữ "Subsystem" bên dưới block → xóa → gõ `Vehicle_Dynamics` → nhấn Enter

### Bước 1.2 — Mở bên trong Subsystem

Double-click vào block `Vehicle_Dynamics` để vào bên trong.

Simulink sẽ mở một canvas con. Mặc định đã có sẵn 1 Inport và 1 Outport (tên là `In1`, `Out1`). **Xóa cả hai:**
- Click vào `In1` → nhấn **Delete**
- Click vào `Out1` → nhấn **Delete**
- Xóa luôn đường dây nối giữa chúng nếu còn sót

Canvas bên trong phải trống hoàn toàn.

---

### Bước 1.3 — Thêm Inport: `F_traction`

1. Nhấn đúp chuột vào vùng trống trong canvas `Vehicle_Dynamics`
2. Gõ: `In1` → chọn **In1** từ dropdown → block Inport xuất hiện
3. Đổi tên block: click vào chữ `In1` bên dưới block → gõ `F_traction` → Enter
4. Đặt vị trí: kéo block ra góc trái, khoảng Y = 100

---

### Bước 1.4 — Thêm Gain block: `Gain_accel`

**Ý nghĩa vật lý:** `dv/dt = -F_traction / m_vehicle`

1. Đúp chuột vào vùng trống → gõ `Gain` → chọn **Gain** (Math Operations)
2. Đổi tên: `Gain_accel`
3. Double-click vào block `Gain_accel` để mở dialog thông số:
   - **Gain:** gõ `-1/m_vehicle`
   - Click **OK**
4. Đặt block bên phải `F_traction`, cùng hàng ngang

> Gõ `-1/m_vehicle` — Simulink sẽ evaluate biểu thức này từ workspace, ra giá trị `-0.001633`.
> Dấu trừ thể hiện lực ma sát cản lại chiều chuyển động.

---

### Bước 1.5 — Thêm Integrator: `Integrator_velocity`

**Ý nghĩa vật lý:** Tích phân gia tốc → ra vận tốc xe `v_vehicle`

1. Đúp chuột vào vùng trống → gõ `Integrator` → chọn **Integrator** (Continuous)
2. Đổi tên: `Integrator_velocity`
3. Double-click vào block → dialog mở ra:
   - **Initial condition source:** `internal`
   - **Initial condition:** gõ `v0`
   - **Limit output:** tick chọn (bật lên)
   - **Lower saturation limit:** `0`
   - **Upper saturation limit:** `inf`
   - Click **OK**
4. Đặt bên phải `Gain_accel`

> `v0 = 27.78` m/s (100 km/h) — xe bắt đầu phanh từ tốc độ này.
> Lower limit = 0 ngăn vận tốc âm (xe không thể đi lùi trong mô hình này).

---

### Bước 1.6 — Thêm Integrator: `Integrator_position`

**Ý nghĩa vật lý:** Tích phân vận tốc → ra quãng đường `x_distance`

1. Đúp chuột → gõ `Integrator` → chọn **Integrator**
2. Đổi tên: `Integrator_position`
3. Double-click → dialog:
   - **Initial condition:** `0`
   - **Limit output:** không tick
   - Click **OK**
4. Đặt **bên dưới** `Integrator_velocity` (không cùng hàng ngang)

---

### Bước 1.7 — Thêm 2 Outport

**Outport v_vehicle:**
1. Đúp chuột → gõ `Out1` → chọn **Out1**
2. Đổi tên: `v_vehicle`
3. Đặt bên phải `Integrator_velocity`
4. Double-click → **Port number:** `1` → OK

**Outport x_distance:**
1. Đúp chuột → gõ `Out1` → chọn **Out1**
2. Đổi tên: `x_distance`
3. Đặt bên phải `Integrator_position`
4. Double-click → **Port number:** `2` → OK

---

### Bước 1.8 — Nối dây bên trong `Vehicle_Dynamics`

Thao tác nối dây: hover chuột vào **output port** (mũi tên nhỏ bên phải block) cho đến khi con trỏ thành dấu `+` màu xanh → kéo thả vào **input port** của block tiếp theo.

Nối theo thứ tự:

```
F_traction (output) ──────> Gain_accel (input)
Gain_accel (output) ──────> Integrator_velocity (input)
Integrator_velocity (output) ──────> v_vehicle (input)
Integrator_velocity (output) ──────> Integrator_position (input)
Integrator_position (output) ──────> x_distance (input)
```

Để rẽ một dây thành 2 nhánh (Integrator_velocity → 2 nơi):
- Hover vào dây đã nối giữa Integrator_velocity và v_vehicle
- Giữ **Ctrl** + kéo từ điểm giữa dây → kéo vào input của Integrator_position

> Sau khi nối xong, nhấn **Ctrl+D** để update diagram và kiểm tra lỗi.
> Không được có đường dây màu đỏ.

---

### Bước 1.9 — Đặt tên signal

1. Double-click vào **đường dây** ra từ `Integrator_velocity` (đoạn dây trước khi rẽ nhánh)
2. Gõ: `v_vehicle`
3. Nhấn Enter

1. Double-click vào đường dây ra từ `Integrator_position`
2. Gõ: `x_distance`
3. Nhấn Enter

---

### Bước 1.10 — Quay lại model cha

Click vào nút **mũi tên lên** (Navigate Up) trên thanh toolbar, hoặc nhấn **Alt + F4** để đóng cửa sổ con.

**Lưu:** Ctrl+S

---

## Phần 2 — Vẽ Subsystem `Wheel_Dynamics`

### Bước 2.1 — Thêm Subsystem block thứ hai

1. Quay lại canvas `plant_model` (top level)
2. Đúp chuột vào vùng trống → gõ `Subsystem` → kéo vào canvas
3. Đổi tên: `Wheel_Dynamics`
4. Đặt bên dưới `Vehicle_Dynamics` (cùng cột, cách nhau 100 pixel)

### Bước 2.2 — Mở bên trong và xóa nội dung mặc định

Double-click vào `Wheel_Dynamics` → xóa `In1`, `Out1`, và đường dây nối.

---

### Bước 2.3 — Thêm Inport 1: `F_traction`

1. Đúp chuột → `In1` → Inport block
2. Đổi tên: `F_traction`
3. Double-click → Port number: `1` → OK
4. Đặt góc trái, Y = 80

---

### Bước 2.4 — Thêm Inport 2: `P_brake_cmd`

1. Đúp chuột → `In1` → Inport block
2. Đổi tên: `P_brake_cmd`
3. Double-click → Port number: `2` → OK
4. Đặt góc trái, Y = 200 (bên dưới `F_traction`)

---

### Bước 2.5 — Thêm Gain: `Gain_T_traction`

**Ý nghĩa:** `T_traction = F_traction × R_wheel`

1. Đúp chuột → `Gain`
2. Đổi tên: `Gain_T_traction`
3. Double-click → **Gain:** `R_wheel` → OK
4. Đặt bên phải `F_traction` (inport 1), cùng hàng

---

### Bước 2.6 — Thêm Gain: `Gain_T_brake`

**Ý nghĩa:** `T_brake = P_brake_cmd × K_brake`

> **Lưu ý quan trọng:** `K_brake = 22.0 N·m/bar` là hệ số **mô-men trực tiếp** (chuyển áp suất → mô-men), đã bao gồm hực học cụm phanh. Không nhân thêm R_wheel.
> Giải thích: tại P = 160 bar, T_brake = 22 × 160 = **3520 N·m**. Đây lớn hơn mô-men ma sát lốp tại lambda=1 (~1900 N·m), nên bánh bị khóa đúng hướng. (Xác nhận qua thực thi TC-PLANT-02, 2026-08-01)

1. Đúp chuột → `Gain`
2. Đổi tên: `Gain_T_brake`
3. Double-click → **Gain:** `K_brake` → OK
4. Đặt bên phải `P_brake_cmd` (inport 2), cùng hàng

---

### Bước 2.7 — Thêm Sum block: `Sum_torque`

**Ý nghĩa:** `T_net = T_traction - T_brake`

1. Đúp chuột → `Sum`
2. Đổi tên: `Sum_torque`
3. Double-click → **List of signs:** gõ `+-` (dấu cộng rồi dấu trừ) → OK
4. Đặt ở giữa canvas, giữa hai Gain blocks

> Mặc định Sum có dạng tròn. Nếu muốn rectangular: trong dialog, **Icon shape:** `rectangular`.
> Port 1 (trên hoặc trái) = `+` (T_traction vào đây)
> Port 2 (dưới hoặc phải) = `-` (T_brake vào đây)

---

### Bước 2.8 — Thêm Gain: `Gain_alpha`

**Ý nghĩa:** `alpha = T_net / J_wheel` (gia tốc góc)

1. Đúp chuột → `Gain`
2. Đổi tên: `Gain_alpha`
3. Double-click → **Gain:** `1/J_wheel` → OK
4. Đặt bên phải `Sum_torque`

---

### Bước 2.9 — Thêm Integrator: `Integrator_omega`

**Ý nghĩa:** Tích phân gia tốc góc → tốc độ góc `omega`

1. Đúp chuột → `Integrator`
2. Đổi tên: `Integrator_omega`
3. Double-click → dialog:
   - **Initial condition:** `v0 / R_wheel`
   - **Limit output:** tick bật
   - **Lower saturation limit:** `0`
   - **Upper saturation limit:** `inf`
   - Click **OK**
4. Đặt bên phải `Gain_alpha`

> `v0 / R_wheel = 27.78 / 0.385 = 72.16 rad/s`
> Tại t=0, bánh đang lăn tự do với vận tốc tương đương xe.
> Nếu IC sai → lambda ≠ 0 ngay từ t=0 → kết quả không đúng vật lý.

---

### Bước 2.10 — Thêm Gain: `Gain_v_wheel`

**Ý nghĩa:** `v_wheel = omega × R_wheel`

1. Đúp chuột → `Gain`
2. Đổi tên: `Gain_v_wheel`
3. Double-click → **Gain:** `R_wheel` → OK
4. Đặt bên phải `Integrator_omega`, hàng trên

---

### Bước 2.11 — Thêm 2 Outport

**Outport v_wheel:**
1. Đúp chuột → `Out1`
2. Đổi tên: `v_wheel`
3. Double-click → Port number: `1` → OK
4. Đặt cuối hàng trên (sau `Gain_v_wheel`)

**Outport omega:**
1. Đúp chuột → `Out1`
2. Đổi tên: `omega`
3. Double-click → Port number: `2` → OK
4. Đặt cuối hàng giữa (sau `Integrator_omega`, rẽ xuống)

---

### Bước 2.12 — Nối dây bên trong `Wheel_Dynamics`

```
F_traction    (output) ───> Gain_T_traction (input)
P_brake_cmd   (output) ───> Gain_T_brake    (input)

Gain_T_traction (output) ───> Sum_torque (port 1, dấu +)
Gain_T_brake    (output) ───> Sum_torque (port 2, dấu -)

Sum_torque  (output) ───> Gain_alpha        (input)
Gain_alpha  (output) ───> Integrator_omega  (input)

Integrator_omega (output) ───> Gain_v_wheel (input)      [nhánh 1]
Integrator_omega (output) ───> omega (outport 2)         [nhánh 2]

Gain_v_wheel (output) ───> v_wheel (outport 1)
```

Cách rẽ nhánh từ `Integrator_omega`:
- Nối `Integrator_omega` → `Gain_v_wheel` trước
- Giữ **Ctrl** + kéo từ đường dây đó → kéo vào `omega` outport

Nhấn **Ctrl+D** kiểm tra lỗi.

---

### Bước 2.13 — Đặt tên signal

- Dây ra từ `Integrator_omega` → double-click → gõ `omega_rad_s`
- Dây ra từ `Gain_v_wheel` → double-click → gõ `v_wheel`
- Dây ra từ `Gain_T_traction` → double-click → gõ `T_traction`
- Dây ra từ `Gain_T_brake` → double-click → gõ `T_brake`

### Bước 2.14 — Quay lại top-level

Click mũi tên Navigate Up. Lưu: **Ctrl+S**.

---

## Phần 3 — Thêm MATLAB Function Block `Tire_Pacejka_Model`

### Bước 3.1 — Thêm MATLAB Function block

1. Trên canvas top-level, đúp chuột → gõ `MATLAB Function`
2. Chọn **MATLAB Function** (trong User-Defined Functions)
3. Đổi tên block: `Tire_Pacejka_Model`
4. Đặt block ở giữa canvas, giữa `Vehicle_Dynamics` và `Wheel_Dynamics`

---

### Bước 3.2 — Nhập code vào MATLAB Function block

Double-click vào block `Tire_Pacejka_Model` → **MATLAB Editor** mở ra.

Xóa toàn bộ code mặc định. Paste vào đoạn code sau:

```matlab
function [F_traction, lambda_actual] = Tire_Pacejka_Model(v_vehicle, v_wheel, road_type)
%TIRE_PACEJKA_MODEL  Tinh luc keo (F_traction) va slip ratio thuc te (lambda_actual).
%
%  Inputs:
%    v_vehicle   - Van toc xe (than xe) [m/s]
%    v_wheel     - Van toc tuyen tinh banh xe [m/s]
%    road_type   - Loai mat duong: 1=Kho, 2=Uot, 3=Da soi
%
%  Outputs:
%    F_traction  - Luc keo tu lop [N]  (can chuyen dong)
%    lambda_actual - Slip ratio thuc te [-], pham vi [0, 1]
%
%  Thong so vat ly khop voi init_params.m:
%    m_vehicle = 612.5 kg (khoi luong 1/4 xe Ford Everest)
%    g         = 9.81 m/s2

%% Thong so vat ly (hardcode de tuong thich voi code generation)
m_veh = 612.5;   % [kg]
g     = 9.81;    % [m/s2]

%% Tinh slip ratio (co bao ve chia cho 0)
if v_vehicle < 0.1
    % Xe da dung hoac gan dung: khong co slip co nghia
    lambda_actual = 0.0;
else
    lambda_actual = (v_vehicle - v_wheel) / v_vehicle;
end

% Clamp vao pham vi vat ly [0, 1]
lambda_actual = max(0.0, min(1.0, lambda_actual));

%% Chon tham so Pacejka theo loai mat duong
switch road_type
    case 1   % Nhua kho (Dry Asphalt)
        B = 10.0;  C = 1.90;  D = 0.90;  E = 0.97;
    case 2   % Nhua uot (Wet Asphalt)
        B = 8.0;   C = 1.70;  D = 0.50;  E = 0.80;
    case 3   % Da soi / Bang tuyet (Gravel / Ice)
        B = 6.0;   C = 1.50;  D = 0.30;  E = 0.60;
    otherwise
        % Default: nhua kho
        B = 10.0;  C = 1.90;  D = 0.90;  E = 0.97;
end

%% Pacejka Magic Formula
% mu(lambda) = D * sin(C * atan(B*lambda - E*(B*lambda - atan(B*lambda))))
phi = B * lambda_actual;
mu  = D * sin(C * atan(phi - E * (phi - atan(phi))));
mu  = max(0.0, mu);   % He so ma sat khong the am

%% Luc keo tu lop len mat duong
F_traction = mu * m_veh * g;   % [N]

end
```

Sau khi paste xong:
- **Ctrl+S** để lưu code
- Đóng cửa sổ MATLAB Editor

---

### Bước 3.3 — Kiểm tra ports của MATLAB Function block

Sau khi save code, quay lại canvas. Block `Tire_Pacejka_Model` sẽ tự động cập nhật:
- **3 input ports:** `v_vehicle`, `v_wheel`, `road_type` (bên trái block)
- **2 output ports:** `F_traction`, `lambda_actual` (bên phải block)

Nếu ports không xuất hiện đúng: double-click vào block → vào **Ports and Data Manager** (icon bảng trên toolbar editor) → kiểm tra danh sách Input/Output.

---

## Phần 4 — Thêm Constant Block cho `ROAD_TYPE`

### Bước 4.1

1. Canvas top-level → đúp chuột → gõ `Constant`
2. Đổi tên: `Road_Type_Selector`
3. Double-click → **Constant value:** `ROAD_TYPE` → OK
4. Đặt bên trái block `Tire_Pacejka_Model`, hàng giữa

> `ROAD_TYPE = 1` được load từ `init_params.m`. Khi muốn đổi sang mặt đường ướt: thay `ROAD_TYPE = 2` trong workspace rồi chạy lại sim — không cần rebuild model (đáp ứng NFR-MAINT-01).

---

## Phần 5 — Thêm Inport và Outport ở Top Level

### Bước 5.1 — Inport: `P_brake_cmd`

1. Canvas top-level → đúp chuột → `In1`
2. Đổi tên: `P_brake_cmd`
3. Double-click → Port number: `1` → OK
4. Đặt góc trái canvas, hàng giữa

### Bước 5.2 — Các Outport

Thêm 3 Outport, đặt ở góc phải:

| Tên | Port number | Vị trí |
|---|---|---|
| `v_vehicle` | 1 | hàng trên |
| `v_wheel` | 2 | hàng giữa |
| `lambda_actual` | 3 | hàng dưới |

---

## Phần 6 — Nối dây Top-Level (Feedback Loop)

Đây là phần quan trọng nhất. Vẽ theo sơ đồ:

```
P_brake_cmd (Inport 1) ────────────────────────────> Wheel_Dynamics (port 2)

Road_Type_Selector (output) ──────────────────────> Tire_Pacejka_Model (port 3: road_type)

Tire_Pacejka_Model (output 1: F_traction) ──┬──────> Vehicle_Dynamics (port 1: F_traction)
                                             └──────> Wheel_Dynamics   (port 1: F_traction)

Vehicle_Dynamics (output 1: v_vehicle) ─────┬──────> Tire_Pacejka_Model (port 1: v_vehicle)
                                             └──────> Outport v_vehicle

Wheel_Dynamics (output 1: v_wheel) ─────────┬──────> Tire_Pacejka_Model (port 2: v_wheel)
                                             └──────> Outport v_wheel

Tire_Pacejka_Model (output 2: lambda_actual) ──────> Outport lambda_actual
```

### Thứ tự nối dây để tránh nhầm:

**Bước 6.1:** Nối `P_brake_cmd` → `Wheel_Dynamics` (port 2)

**Bước 6.2:** Nối `Road_Type_Selector` → `Tire_Pacejka_Model` (port 3)

**Bước 6.3:** Nối `Tire_Pacejka_Model` (output 1: F_traction) → `Vehicle_Dynamics` (port 1)

**Bước 6.4:** Từ dây F_traction vừa nối: Ctrl + kéo thêm → `Wheel_Dynamics` (port 1)

**Bước 6.5:** Nối `Vehicle_Dynamics` (output 1: v_vehicle) → `Tire_Pacejka_Model` (port 1)

**Bước 6.6:** Từ dây v_vehicle vừa nối: Ctrl + kéo thêm → `Outport v_vehicle`

**Bước 6.7:** Nối `Wheel_Dynamics` (output 1: v_wheel) → `Tire_Pacejka_Model` (port 2)

**Bước 6.8:** Từ dây v_wheel vừa nối: Ctrl + kéo thêm → `Outport v_wheel`

**Bước 6.9:** Nối `Tire_Pacejka_Model` (output 2: lambda_actual) → `Outport lambda_actual`

---

### Bước 6.10 — Đặt tên signal top-level

| Dây | Tên signal |
|---|---|
| Ra từ Vehicle_Dynamics output 1 | `v_vehicle` |
| Ra từ Wheel_Dynamics output 1 | `v_wheel` |
| Ra từ Tire_Pacejka_Model output 1 | `F_traction` |
| Ra từ Tire_Pacejka_Model output 2 | `lambda_actual` |

Double-click vào từng dây → gõ tên → Enter.

---

## Phần 7 — Thêm Scopes và Logging

### Bước 7.1 — Scope cho velocity

1. Đúp chuột → `Scope` → kéo vào canvas
2. Đổi tên: `Scope_velocity`
3. Double-click vào Scope → click icon **Parameters** (bánh răng) → tab **Signals**:
   - Number of input ports: `2`
4. Click OK
5. Nối:
   - `v_vehicle` signal → Scope_velocity port 1
   - `v_wheel` signal → Scope_velocity port 2

### Bước 7.2 — Scope cho slip ratio

1. Thêm **Scope** mới → đổi tên `Scope_lambda`
2. Number of input ports: `1`
3. Nối: `lambda_actual` → `Scope_lambda`

### Bước 7.3 — To Workspace block

1. Đúp chuột → `To Workspace` → kéo vào canvas
2. Đổi tên: `DataLogger`
3. Double-click → dialog:
   - **Variable name:** `sim_out`
   - **Limit data points:** bỏ chọn (để ghi hết)
   - **Save format:** `Structure With Time`
4. Click OK
5. Cần nối nhiều signals vào 1 To Workspace block → dùng **Mux block:**

**Thêm Mux:**
- Đúp chuột → `Mux` → đổi tên `Data_Mux`
- Double-click → **Number of inputs:** `4`
- Nối vào Mux:
  - Port 1: `v_vehicle`
  - Port 2: `v_wheel`
  - Port 3: `lambda_actual`
  - Port 4: `x_distance` (từ Vehicle_Dynamics output 2)
- Nối Mux output → `DataLogger`

---

## Phần 8 — Kiểm tra và Lưu

### Bước 8.1 — Update diagram

Nhấn **Ctrl+D** — Simulink re-parse model, highlight lỗi nếu có.

**Các lỗi thường gặp và cách fix:**

| Lỗi | Nguyên nhân | Fix |
|---|---|---|
| Unconnected port (dấu `?` đỏ trên block) | Còn port chưa nối | Nối dây còn thiếu |
| Algebraic loop warning | Vòng lặp kín | Bình thường với plant model này, Integrator tự break loop |
| Undefined variable 'v0' | init_params.m chưa chạy | Quay lại cmd: `run('scripts/utils/init_params.m')` |

### Bước 8.2 — Lưu model

**Ctrl+S**

---

## Phần 9 — Chạy thử kiểm tra TC-PLANT-01

**Test: Free rolling — không có lực phanh**

Trước khi chạy, cần thêm Constant block tạm thời cho `P_brake_cmd`:

1. Bỏ nối `P_brake_cmd` Inport tạm thời
2. Thêm Constant block: value = `0` → nối vào `Wheel_Dynamics` (port 2)
3. Nhấn **Run** (nút tam giác xanh, hoặc Ctrl+T)
4. Quan sát Scope_velocity:
   - `v_vehicle` phải **không thay đổi** (vì không có lực phanh)
   - `v_wheel` phải bằng `v_vehicle` (lăn tự do, lambda ≈ 0)
5. Quan sát Scope_lambda:
   - `lambda_actual` phải ≈ `0` trong suốt simulation

**Nếu kết quả đúng:** Model Plant chạy được. Chụp màn hình Scope_velocity.

**Nếu v_vehicle giảm dù P_brake_cmd = 0:** Kiểm tra lại `Gain_accel` — giá trị phải là `-1/m_vehicle`, không phải `1/m_vehicle`.

---

## Checklist trước khi commit

Trước khi commit lên git, kiểm tra từng mục:

- [ ] `init_params.m` chạy không lỗi
- [ ] `plant_model.slx` mở không lỗi
- [ ] Ctrl+D không có unconnected ports
- [ ] Simulation với P_brake_cmd = 0: v_vehicle ổn định, lambda ≈ 0
- [ ] Simulation với P_brake_cmd = 160: v_vehicle giảm, v_wheel giảm nhanh hơn, lambda tăng về 1
- [ ] File đã save tại `models/plant/plant_model.slx`

Khi tất cả mục trên đã tick: báo lại để tôi hướng dẫn bước commit và unit test.
