# Đặc Tả Kỹ Thuật Hệ Thống
# Mô hình hoá & Mô phỏng Hệ thống Điều khiển ABS sử dụng Model-Based Design

---

> **Phiên bản tài liệu:** 1.0  
> **Ngày tạo:** 2026-07-31  
> **Trạng thái:** Approved – Technical Specification  
> **Phương pháp luận:** Model-Based Design (MBD) theo ASPICE Level 1–2  
> **Công cụ chính:** MATLAB R2023b / Simulink / Stateflow / Simulink Coder

---

## Mục Lục

1. [Tổng Quan Dự Án](#1-tổng-quan-dự-án)
2. [Phân Tích Yêu Cầu — Software Requirements Specification (SRS)](#2-phân-tích-yêu-cầu)
3. [Kiến Trúc Hệ Thống — System Architecture](#3-kiến-trúc-hệ-thống)
4. [Mô hình hoá Hệ thống — System Modeling](#4-mô-hình-hoá-hệ-thống)
5. [Xử Lý Tín Hiệu — Signal Processing](#5-xử-lý-tín-hiệu)
6. [Thiết Kế Bộ Điều Khiển — Control System Design](#6-thiết-kế-bộ-điều-khiển)
7. [Sinh Mã Tự Động — Code Generation](#7-sinh-mã-tự-động)
8. [Kiểm Thử & Xác Nhận — Verification & Validation](#8-kiểm-thử--xác-nhận)
9. [Requirements Traceability Matrix (RTM)](#9-requirements-traceability-matrix)
10. [Kế Hoạch Thực Hiện — Project Timeline](#10-kế-hoạch-thực-hiện)
11. [Phụ Lục — Appendix](#11-phụ-lục)

---

## 1. Tổng Quan Dự Án

### 1.1 Bối Cảnh & Động Lực

Hệ thống chống bó cứng phanh (Anti-lock Braking System – ABS) là một trong những hệ thống an toàn chủ động quan trọng nhất trong ô tô hiện đại. Khi phanh khẩn cấp, bánh xe có thể bị bó cứng hoàn toàn (wheel lock), gây mất khả năng lái và kéo dài quãng đường phanh. ABS giải quyết vấn đề này bằng cách điều tiết áp suất phanh liên tục để duy trì hệ số trượt (slip ratio) trong vùng tối ưu.

Dự án này áp dụng phương pháp **Model-Based Design (MBD)** — một quy trình phát triển phần mềm được sử dụng rộng rãi trong công nghiệp ô tô (Bosch, Continental, ZF, DENSO...) — để mô hình hoá, mô phỏng, kiểm thử và sinh mã C tự động cho thuật toán điều khiển ABS.

### 1.2 Mục Tiêu Dự Án

| Mục tiêu | Mô tả chi tiết |
|---|---|
| **MT-01** | Xây dựng mô hình vật lý (plant model) của hệ thống xe – bánh xe – mặt đường bằng Simulink |
| **MT-02** | Thiết kế và tích hợp bộ điều khiển ABS (bang-bang + PID) |
| **MT-03** | Mô phỏng xử lý tín hiệu cảm biến tốc độ bánh xe trong điều kiện nhiễu |
| **MT-04** | Thực hiện Verification & Validation theo quy trình MBD (MIL Testing) |
| **MT-05** | Sinh mã C tự động từ model bằng Simulink Coder |
| **MT-06** | So sánh định lượng hiệu quả hệ thống có/không có ABS |

### 1.3 Phạm Vi Dự Án

**Trong phạm vi (In-Scope):**
- Mô hình động lực học xe 1 bánh (quarter-car + single-wheel model)
- Thuật toán điều khiển ABS: bang-bang controller và PID controller
- Signal processing: noise simulation + low-pass filter
- Mô phỏng trên 3 loại mặt đường: nhựa khô, nhựa ướt, đá sỏi
- MIL (Model-in-the-Loop) testing
- Tự động sinh mã C từ Simulink model

**Ngoài phạm vi (Out-of-Scope):**
- Mô hình 4 bánh xe đầy đủ (full vehicle model)
- Mô hình khí động học
- Hardware-in-the-Loop (HIL) và deploy lên vi điều khiển thực
- Thiết kế cơ khí hệ thống phanh

### 1.4 Sản Phẩm Bàn Giao (Deliverables)

| STT | Deliverable | Định dạng | Giai đoạn |
|---|---|---|---|
| D-01 | Software Requirements Specification (SRS) | Tài liệu .docx/.pdf | Phase 1 |
| D-02 | System Architecture Document | Tài liệu + Sơ đồ | Phase 1 |
| D-03 | Simulink Plant Model | File .slx | Phase 2 |
| D-04 | Simulink Controller Model (Bang-Bang + PID) | File .slx | Phase 3 |
| D-05 | Signal Processing Subsystem | File .slx | Phase 3 |
| D-06 | Integrated Full System Model | File .slx | Phase 3 |
| D-07 | Simulation Results & Analysis Report | Báo cáo + Plots | Phase 4 |
| D-08 | Test Cases & MIL Test Results | Bảng test + Scripts | Phase 5 |
| D-09 | Auto-generated C Code | Thư mục code | Phase 6 |
| D-10 | Requirements Traceability Matrix | Bảng RTM | Phase 5 |
| D-11 | Báo cáo kỹ thuật tổng hợp dự án | .docx/.pdf | Cuối |

---

## 2. Phân Tích Yêu Cầu

### 2.1 Tổng Quan Tài Liệu SRS

Tài liệu này được soạn thảo theo cấu trúc **IEEE 830 Software Requirements Specification** và phân loại yêu cầu theo hai cấp: Functional Requirements (FR) và Non-Functional Requirements (NFR), trong đó mỗi yêu cầu có Requirement ID duy nhất, mức ưu tiên (Priority: High / Medium / Low) và tiêu chí chấp nhận (Acceptance Criteria).

### 2.2 Functional Requirements — Yêu Cầu Chức Năng

#### 2.2.1 Nhóm FR-DETECT: Phát hiện trạng thái bánh xe

| Req ID | Description | Priority | Acceptance Criteria |
|---|---|---|---|
| FR-DETECT-01 | Hệ thống phải tính toán slip ratio (λ) từ tốc độ xe và tốc độ bánh xe | High | λ = (v_vehicle - v_wheel) / v_vehicle, trong khoảng [0, 1] |
| FR-DETECT-02 | Hệ thống phải phát hiện trạng thái wheel lock khi λ > 0.35 | High | Output trạng thái "LOCK" khi λ > 0.35 trong ≥ 2 chu kỳ liên tiếp |
| FR-DETECT-03 | Hệ thống phải phân biệt 2 trạng thái: NORMAL và LOCK | High | State machine với 2 state rõ ràng, chuyển trạng thái theo ngưỡng |
| FR-DETECT-04 | Slip ratio tối ưu phải được duy trì trong dải 0.15 ≤ λ ≤ 0.25 | High | 80% thời gian phanh có λ nằm trong dải tối ưu |

#### 2.2.2 Nhóm FR-CONTROL: Điều khiển áp suất phanh

| Req ID | Description | Priority | Acceptance Criteria |
|---|---|---|---|
| FR-CTRL-01 | Bộ điều khiển phải giảm áp suất phanh khi phát hiện wheel lock | High | Áp suất giảm trong vòng 10ms kể từ khi phát hiện lock |
| FR-CTRL-02 | Bộ điều khiển phải tăng áp suất phanh khi bánh xe thoát khỏi lock | High | Áp suất tăng dần (gradient tăng) khi λ < 0.10 |
| FR-CTRL-03 | Bộ điều khiển bang-bang phải hoạt động ổn định (không chattering quá mức) | Medium | Tần số đóng-mở van phanh không vượt quá 15 Hz |
| FR-CTRL-04 | Bộ điều khiển PID phải hội tụ về slip ratio mục tiêu (λ* = 0.20) trong thời gian hợp lý | High | |λ - λ*| ≤ 0.03 sau tối đa 0.5 giây từ khi vào trạng thái lock |
| FR-CTRL-05 | Áp suất phanh phải bị giới hạn trong dải an toàn [P_min, P_max] | High | 0 bar ≤ P_brake ≤ 160 bar tại mọi thời điểm |

#### 2.2.3 Nhóm FR-SIGNAL: Xử lý tín hiệu

| Req ID | Description | Priority | Acceptance Criteria |
|---|---|---|---|
| FR-SIG-01 | Hệ thống phải mô phỏng nhiễu Gaussian trên tín hiệu cảm biến tốc độ bánh xe | Medium | SNR ≥ 20 dB sau khi lọc |
| FR-SIG-02 | Bộ lọc thông thấp (LPF) phải loại bỏ nhiễu tần số cao (> 50 Hz) | High | Attenuation ≥ -40 dB tại tần số > 50 Hz |
| FR-SIG-03 | Tín hiệu tốc độ sau lọc phải có độ trễ (phase lag) không vượt quá 20ms | High | Phase lag ≤ 20ms tại tần số 10 Hz |
| FR-SIG-04 | Hệ thống phải tính toán slip ratio từ tín hiệu đã lọc | High | Slip ratio tính từ tín hiệu lọc và tín hiệu gốc sai lệch ≤ 0.02 |

#### 2.2.4 Nhóm FR-PERF: Hiệu suất hệ thống

| Req ID | Description | Priority | Acceptance Criteria |
|---|---|---|---|
| FR-PERF-01 | Quãng đường phanh với ABS phải ngắn hơn không có ABS ≥ 15% | High | So sánh trên cùng điều kiện: v₀ = 100 km/h, đường khô |
| FR-PERF-02 | Hệ thống ABS phải duy trì khả năng điều khiển hướng lái trong khi phanh | Medium | Lực ngang (lateral force) ≥ 50% giá trị tối đa |
| FR-PERF-03 | Model phải chạy simulation faster-than-realtime (simulation time / real time < 1) | Low | Simulation 5 giây hoàn thành trong < 10 giây wall-clock |

#### 2.2.5 Nhóm FR-SIM: Điều kiện mô phỏng

| Req ID | Description | Priority | Acceptance Criteria |
|---|---|---|---|
| FR-SIM-01 | Hệ thống phải được mô phỏng trên ít nhất 3 loại mặt đường khác nhau | High | Nhựa khô (μ=0.9), nhựa ướt (μ=0.5), đá sỏi (μ=0.3) |
| FR-SIM-02 | Điều kiện phanh khẩn cấp: vận tốc ban đầu v₀ = 100 km/h, toàn bộ lực phanh áp dụng tức thời | High | Mô phỏng từ t=0 đến khi xe dừng hoặc t=10 giây |
| FR-SIM-03 | Hệ thống phải mô phỏng được điều kiện phanh khi đang trên dốc (grade ≤ 10%) | Low | Chênh lệch quãng đường phanh so với mặt phẳng ≤ 5% |

### 2.3 Non-Functional Requirements — Yêu Cầu Phi Chức Năng

| Req ID | Category | Description | Priority | Acceptance Criteria |
|---|---|---|---|---|
| NFR-MOD-01 | Modularity | Model phải được tổ chức thành các subsystem độc lập | High | Mỗi subsystem có interface rõ ràng (input/output ports được đặt tên) |
| NFR-MOD-02 | Modularity | Các tham số vật lý phải được tập trung trong Model Workspace | High | Không hard-code hằng số trong block |
| NFR-DOC-01 | Documentation | Mỗi subsystem phải có DocBlock mô tả chức năng | Medium | 100% subsystem có description |
| NFR-DOC-02 | Documentation | Tên block, signal phải có ý nghĩa rõ ràng | Medium | Không dùng tên mặc định (Gain1, Sum2...) |
| NFR-CODE-01 | Code Quality | Generated C code phải biên dịch không có warning | High | 0 compiler warning với gcc -Wall |
| NFR-CODE-02 | Code Quality | Generated C code phải có MISRA-C comment | Medium | Mô tả MISRA compliance trong báo cáo |
| NFR-MAINT-01 | Maintainability | Thay đổi tham số mặt đường không yêu cầu rebuild toàn bộ model | Medium | Thay đổi μ trong Workspace → chạy lại sim ngay |
| NFR-SAFE-01 | Safety | Áp suất phanh phải được clip trong giới hạn an toàn | High | Saturation block trên mọi đường dẫn áp suất |

---

## 3. Kiến Trúc Hệ Thống

### 3.1 Sơ Đồ Kiến Trúc Tổng Thể

```
┌─────────────────────────────────────────────────────────────────┐
│                    ABS CONTROL SYSTEM                           │
│                                                                 │
│  ┌─────────────┐    ┌──────────────────┐    ┌───────────────┐  │
│  │   DRIVER    │───▶│   PLANT MODEL    │───▶│    OUTPUTS    │  │
│  │  (Inputs)   │    │                  │    │  (Responses)  │  │
│  │             │    │  ┌────────────┐  │    │               │  │
│  │ • Brake Cmd │    │  │  Vehicle   │  │    │ • v_vehicle   │  │
│  │ • v_initial │    │  │  Dynamics  │  │    │ • x_distance  │  │
│  └─────────────┘    │  └──────┬─────┘  │    │ • F_brake     │  │
│                     │         │         │    └───────────────┘  │
│                     │  ┌──────▼─────┐  │                       │
│                     │  │   Wheel    │  │    ┌───────────────┐  │
│                     │  │  Dynamics  │  │───▶│   DISPLAY     │  │
│                     │  └──────┬─────┘  │    │   & LOGGING   │  │
│                     │         │         │    │               │  │
│                     │  ┌──────▼─────┐  │    │ • Scope       │  │
│                     │  │   Tire     │  │    │ • To Workspace│  │
│                     │  │   Model    │  │    └───────────────┘  │
│                     │  └────────────┘  │                       │
│                     └────────┬─────────┘                       │
│                              │ v_wheel (noisy)                  │
│                              ▼                                  │
│                     ┌────────────────┐                          │
│                     │ SIGNAL         │                          │
│                     │ PROCESSING     │                          │
│                     │                │                          │
│                     │ • Noise Add    │                          │
│                     │ • LPF Filter   │                          │
│                     │ • Slip Calc    │                          │
│                     └────────┬───────┘                          │
│                              │ λ (slip ratio, filtered)         │
│                              ▼                                  │
│                     ┌────────────────┐                          │
│                     │  ABS           │                          │
│                     │  CONTROLLER   │                          │
│                     │                │                          │
│                     │ • Stateflow SM │                          │
│                     │ • Bang-Bang    │                          │
│                     │ • PID (alt.)   │                          │
│                     └────────┬───────┘                          │
│                              │ P_brake (pressure command)       │
│                              └──────────────────────────────────┘
│                                       (feedback to Plant)        │
└─────────────────────────────────────────────────────────────────┘
```

### 3.2 Phân Rã Hệ Thống (System Decomposition)

```
ABS_System.slx (Top-Level)
├── Plant_Model/
│   ├── Vehicle_Dynamics/
│   │   ├── Longitudinal_Motion      (Newton's 2nd Law)
│   │   └── Brake_Force_Calculator
│   ├── Wheel_Dynamics/
│   │   ├── Wheel_Rotational_Motion  (Torque balance)
│   │   └── Wheel_Speed_Calc
│   └── Tire_Model/
│       ├── Slip_Ratio_Physical      (từ physics, không nhiễu)
│       └── Friction_Force_Calc      (Simplified Pacejka)
│
├── Signal_Processing/
│   ├── Noise_Generator              (Gaussian noise)
│   ├── LPF_Butterworth              (2nd order, fc = 30Hz)
│   └── Slip_Ratio_Estimator         (từ tín hiệu lọc)
│
├── ABS_Controller/
│   ├── Stateflow_Logic/             (State machine: NORMAL/BUILD/HOLD/DUMP)
│   ├── BangBang_Controller/
│   │   └── Pressure_Logic
│   ├── PID_Controller/              (Alternative, switchable)
│   │   ├── PID_Block
│   │   └── Slip_Error_Calc
│   └── Pressure_Limiter/            (Saturation: 0 – 160 bar)
│
└── Visualization/
    ├── Scope_SlipRatio
    ├── Scope_Velocity
    ├── Scope_Pressure
    └── DataLogger_ToWorkspace
```

### 3.3 Interface Definition

#### Top-Level Signals

| Signal Name | Unit | Range | Description |
|---|---|---|---|
| `v_vehicle` | m/s | [0, 50] | Vận tốc xe (thân xe) |
| `v_wheel` | m/s | [0, 50] | Vận tốc tuyến tính bánh xe (thực tế) |
| `v_wheel_noisy` | m/s | [0, 50] | Tín hiệu tốc độ bánh xe + nhiễu |
| `v_wheel_filtered` | m/s | [0, 50] | Tín hiệu sau lọc LPF |
| `lambda_est` | - | [0, 1] | Slip ratio ước tính (từ tín hiệu lọc) |
| `lambda_actual` | - | [0, 1] | Slip ratio thực tế (từ physics) |
| `P_brake_cmd` | bar | [0, 160] | Lệnh áp suất phanh từ controller |
| `P_brake_actual` | bar | [0, 160] | Áp suất phanh thực tế tại bánh |
| `F_brake` | N | [0, 25000] | Lực phanh tác dụng lên bánh xe |
| `F_traction` | N | [-15000, 15000] | Lực kéo từ lốp (Pacejka) |
| `ABS_state` | enum | {0,1,2,3} | Trạng thái Stateflow (NORMAL/BUILD/HOLD/DUMP) |

---

## 4. Mô Hình Hoá Hệ Thống

### 4.1 Thông Số Vật Lý

| Tham số | Ký hiệu | Giá trị | Đơn vị | Mô tả |
|---|---|---|---|---|
| Khối lượng tổng xe | `m_total_vehicle` | 2450 | kg | Ford Everest (Curb 2300kg + Payload 150kg) |
| Khối lượng 1/4 xe (quarter) | `m_vehicle` | 612.5 | kg | 1/4 khối lượng xe Ford Everest |
| Bán kính bán tải hiệu dụng | `R_wheel` | 0.385 | m | Lốp 255/55R20 |
| Mô-men quán tính vành+lốp | `J_wheel` | 2.80 | kg·m² | Vành hợp kim 20 inch + SUV tire |
| Hệ số mô-men phanh | `K_brake` | 22.0 | N·m/bar | Chuyển đổi áp suất thủy lực -> Mô-men |
| Áp suất phanh tối đa | `P_max` | 160 | bar | Áp suất hệ thống ABS tối đa |
| Gia tốc trọng trường | `g` | 9.81 | m/s² | |
| Vận tốc ban đầu | `v0` | 27.78 | m/s | 100 km/h |

#### Thông Số Mặt Đường (Road Conditions)

| Mặt đường | Ký hiệu | μ_peak | λ_peak | Ghi chú |
|---|---|---|---|---|
| Nhựa khô | `DRY_ASPHALT` | 0.90 | 0.18 | Điều kiện chuẩn |
| Nhựa ướt | `WET_ASPHALT` | 0.50 | 0.15 | Mưa nhẹ |
| Đá sỏi/băng tuyết | `GRAVEL` | 0.30 | 0.12 | Điều kiện trơn |

### 4.2 Mô Hình Động Lực Học Xe (Vehicle Dynamics)

**Phương trình chuyển động tịnh tiến:**

```
m_vehicle * dv/dt = -F_traction - F_aero - F_grade

Trong đó:
  F_traction = μ(λ) * m_vehicle * g   [Lực ma sát lốp - mặt đường]
  F_aero     ≈ 0                       [Bỏ qua lực cản gió]
  F_grade    ≈ 0                       [Bỏ qua độ dốc trong kịch bản chính]
```

**Simulink Implementation:**
```
[v_vehicle] ──▶ [Integrator: 1/s] ──▶ v_vehicle (output)
                     ▲
                     │ dv/dt = -F_traction / m_vehicle
                [Divide by m_vehicle]
                     ▲
                [Sum: -F_traction]
```

### 4.3 Mô Hình Động Lực Học Bánh Xe (Wheel Dynamics)

**Phương trình mô-men bánh xe:**

```
J_wheel * dω/dt = -T_brake + T_traction

Trong đó:
  ω          = tốc độ góc bánh xe [rad/s]
  T_brake    = R_wheel * P_brake * K_brake   [Mô-men phanh]
  T_traction = R_wheel * F_traction           [Mô-men do ma sát lốp]
```

**Quan hệ vận tốc:**
```
v_wheel = ω * R_wheel   [m/s]
```

**Điều kiện biên:**
```
ω ≥ 0   (bánh xe không quay ngược)
v_wheel = 0 khi xe dừng hẳn
```

### 4.4 Mô Hình Lốp Xe (Tire Model — Simplified Pacejka)

Sử dụng **Simplified Pacejka Magic Formula** với dạng rút gọn:

```
μ(λ) = D * sin(C * arctan(B * λ - E * (B * λ - arctan(B * λ))))

Trong đó (cho nhựa khô):
  B = 10.0   (Stiffness Factor)
  C = 1.9    (Shape Factor)
  D = 0.97   (Peak Factor = μ_peak)
  E = 0.97   (Curvature Factor)
```

**Đường cong μ-λ cho từng loại mặt đường:**

```
μ
│
0.9 │   ___
    │  /   \___
0.5 │ /  WET  \_____
    │/
0.3 │ GRAVEL
    └──────────────────── λ
    0   0.15  0.35  1.0
         ↑
      λ_optimal ≈ 0.15-0.20
```

**Hàm rút gọn dùng trong Simulink (MATLAB Function block):**
```matlab
function mu = tire_friction(lambda, road_type)
% Simplified Pacejka model
% Inputs:  lambda    - slip ratio [0, 1]
%          road_type - 1=dry, 2=wet, 3=gravel
% Output:  mu        - friction coefficient

params = struct();
switch road_type
    case 1  % Dry asphalt
        params.B = 10.0; params.C = 1.9; 
        params.D = 0.97; params.E = 0.97;
    case 2  % Wet asphalt
        params.B = 8.0;  params.C = 1.7; 
        params.D = 0.52; params.E = 0.80;
    case 3  % Gravel
        params.B = 6.0;  params.C = 1.5; 
        params.D = 0.32; params.E = 0.60;
end

phi = params.B * lambda;
mu  = params.D * sin(params.C * atan(phi - params.E*(phi - atan(phi))));
end
```

### 4.5 Slip Ratio Definition

```
           v_vehicle - v_wheel
λ = ─────────────────────────────
              v_vehicle

Trường hợp đặc biệt:
  λ = 0   → Lăn tự do (free rolling), không phanh
  λ = 1   → Wheel lock hoàn toàn (v_wheel = 0)
  λ_opt ≈ 0.15-0.20 → Lực phanh + lực ngang tối ưu
```

**Xử lý điều kiện biên (Singularity Prevention):**
```matlab
% Tránh chia cho 0 khi xe đã dừng
if v_vehicle < 0.1  % m/s (ngưỡng dừng)
    lambda = 0;
else
    lambda = (v_vehicle - v_wheel) / v_vehicle;
end
lambda = max(0, min(1, lambda));  % Clamp [0, 1]
```

---

## 5. Xử Lý Tín Hiệu

### 5.1 Mô Hình Nhiễu Cảm Biến

Cảm biến tốc độ bánh xe (Wheel Speed Sensor – WSS) trong thực tế là cảm biến Hall-effect hoặc VRS (Variable Reluctance Sensor), dễ bị nhiễu từ trường và rung động cơ học. Mô phỏng nhiễu bao gồm:

**Nhiễu Gaussian (thermal noise):**
```
v_wheel_noisy(t) = v_wheel(t) + n(t)

Trong đó: n(t) ~ N(0, σ²)
  σ = 0.1 m/s  (noise standard deviation)
  Tương đương SNR ≈ 28 dB tại v_wheel = 27.78 m/s
```

**Cấu hình Simulink Band-Limited White Noise block:**
```
Noise power (σ²):  0.01   [(m/s)²/Hz]
Sample time:       0.001  [s]  (1 kHz)
Seed:              12345  (reproducible)
```

### 5.2 Thiết Kế Bộ Lọc Thông Thấp (Low-Pass Filter)

**Lý do chọn Butterworth 2nd Order:**
- Đáp ứng biên độ phẳng trong dải thông (maximally flat)
- Không có ripple → không gây sai số thêm cho slip ratio
- Triển khai đơn giản bằng Transfer Function block

**Thông số thiết kế:**
```
Loại filter:        Butterworth, bậc 2
Tần số cắt:         fc = 30 Hz
Tần số lấy mẫu:     fs = 1000 Hz (Ts = 1 ms)
Suy hao tại 50 Hz:  ≥ -20 dB
Suy hao tại 100 Hz: ≥ -40 dB
Phase lag tại 10Hz: < 15 ms
```

**Hàm truyền liên tục:**
```
                  ωc²
H(s) = ─────────────────────────
         s² + √2·ωc·s + ωc²

Với ωc = 2π × 30 = 188.5 rad/s

         35529.6
H(s) = ──────────────────────────
        s² + 266.6s + 35529.6
```

**Bilinear Transform (Tustin) sang Z-domain (Ts = 1ms):**

```matlab
% Script thiết kế bộ lọc trong MATLAB
fc = 30;                              % Hz
fs = 1000;                            % Hz
[b, a] = butter(2, fc/(fs/2), 'low');
% Verify: freqz(b, a, [], fs)
```

**Kiểm tra đặc tính bộ lọc:**

| Tần số (Hz) | Gain lý thuyết (dB) | Acceptance Criteria |
|---|---|---|
| 0 – 10 | ≥ -1 dB | Pass band |
| 30 | -3 dB | Cutoff frequency |
| 50 | ≤ -8 dB | Transition band |
| 100 | ≤ -24 dB | Stop band |

### 5.3 Tính Toán Slip Ratio từ Tín Hiệu Lọc

```
                    v_vehicle - v_wheel_filtered
λ_estimated = ─────────────────────────────────────
                          v_vehicle

Sai số ước tính:
  Δλ = |λ_actual - λ_estimated| ≤ 0.02  (yêu cầu FR-SIG-04)
```

**Subsystem Signal Processing — Block Diagram:**

```
v_wheel (từ Plant) ──▶ [+ Sum] ──▶ [LPF: H(s)] ──▶ v_wheel_filtered ──▶ [Slip Calc] ──▶ λ_est
                         ▲
noise: N(0, 0.01) ───────┘

v_vehicle ────────────────────────────────────────────────────────────▶ [Slip Calc]
```

---

## 6. Thiết Kế Bộ Điều Khiển

### 6.1 Kiến Trúc Điều Khiển Tổng Thể

Dự án thiết kế **hai bộ điều khiển song song** có thể chuyển đổi qua tham số `CONTROLLER_TYPE`:

| Controller | Type | Ưu điểm | Nhược điểm |
|---|---|---|---|
| Bang-Bang + Stateflow | On/Off | Đơn giản, robust, đúng thực tế | Chattering, không tối ưu |
| PID | Continuous | Mượt mà, tối ưu hơn | Nhạy cảm với nhiễu, cần tuning |

### 6.2 Stateflow State Machine — Bộ Điều Khiển Chính

#### State Machine Architecture

```
                    [ENTRY: ABS Off, P = P_driver]
                                  │
                           ┌──────▼──────┐
                           │             │
                    ┌──────│   NORMAL    │◀────────────────────────┐
                    │      │  (State 0)  │                         │
                    │      └──────┬──────┘                         │
                    │             │ [λ > 0.35]                     │
                    │             ▼                                 │
                    │      ┌──────────────┐                        │
                    │      │    BUILD     │  [λ > 0.25]            │
                    │      │  (State 1)  │──────────────▶ ─ ─ ─  │
                    │      │ P += ΔP_inc │                        │
                    │      └──────┬──────┘                        │
                    │             │ [λ > 0.35]                    │
                    │             ▼                                │
                    │      ┌──────────────┐                       │
                    │      │     HOLD     │  [0.15 ≤ λ ≤ 0.25]   │
                    │      │  (State 2)  │ (Stay)                │
                    │      │  P = const  │                        │
                    │      └──────┬──────┘                        │
                    │             │ [λ > 0.35]                    │
                    │             ▼                                │
                    │      ┌──────────────┐                        │
                    │      │     DUMP     │──────────────────────▶│
                    │      │  (State 3)  │  [λ < 0.15]           │
                    │      │ P -= ΔP_dec │                        │
                    │      └──────────────┘                        │
                    │                                               │
                    │ [v_vehicle < 0.5 m/s]                        │
                    └──────────────────────────────────────────────┘
                                (xe dừng → thoát ABS)
```

#### State Definitions

| State | ID | Entry Action | During Action | Transition Condition | Exit Action |
|---|---|---|---|---|---|
| NORMAL | 0 | P_cmd = P_driver | — | λ > λ_high (0.35) → DUMP | — |
| BUILD | 1 | — | P_cmd += ΔP_inc * Ts | λ > 0.35 → DUMP; λ < 0.15 → HOLD | — |
| HOLD | 2 | — | P_cmd = P_cmd (hold) | λ > 0.35 → DUMP; λ < 0.15 → BUILD | — |
| DUMP | 3 | — | P_cmd -= ΔP_dec * Ts | λ < 0.15 → BUILD; λ < 0.05 → NORMAL | — |

#### Stateflow Parameters

| Tham số | Ký hiệu | Giá trị | Mô tả |
|---|---|---|---|
| Ngưỡng lock cao | `lambda_high` | 0.35 | Kích hoạt DUMP |
| Ngưỡng slip tối ưu trên | `lambda_opt_high` | 0.25 | Ngưỡng chuyển HOLD→DUMP |
| Ngưỡng slip tối ưu dưới | `lambda_opt_low` | 0.15 | Ngưỡng chuyển DUMP→BUILD |
| Tốc độ tăng áp | `dP_inc` | 800 | bar/s |
| Tốc độ giảm áp | `dP_dec` | 1200 | bar/s |
| Ngưỡng dừng xe | `v_stop` | 0.5 | m/s |

### 6.3 PID Controller — Bộ Điều Khiển Thay Thế

**Mục tiêu:** Điều khiển áp suất phanh P_brake sao cho slip ratio λ tiến về giá trị đặt λ* = 0.20

**Cấu trúc PID:**

```
λ_setpoint (0.20) ─▶ [+ Error] ──▶ [PID Block] ──▶ ΔP_cmd ──▶ [+ Sum] ──▶ P_brake
λ_estimated ────────▶ [- Error]                                     ▲
                                                              P_driver_base
```

**Phương trình PID:**
```
e(t) = λ* - λ(t)

u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt

ΔP_brake = -Kp_pressure · u(t)   [dấu trừ vì tăng P → tăng λ]
```

**Khởi điểm thông số PID (sẽ được tuning qua simulation):**

| Tham số | Giá trị ban đầu | Ghi chú |
|---|---|---|
| Kp | 500 | bar/slip_unit |
| Ki | 50 | bar/(slip_unit·s) |
| Kd | 10 | bar·s/slip_unit |
| Output saturation | [0, 160] bar | Anti-windup |
| Anti-windup method | Clamping | Tránh tích phân bão hoà |

**PID Tuning Strategy:**
1. Bắt đầu với chỉ Kp (P-only), tăng dần đến khi có oscillation
2. Thêm Ki để loại steady-state error
3. Thêm Kd để giảm overshoot
4. Fine-tune trên cả 3 loại mặt đường

### 6.4 Anti-Windup & Pressure Saturation

**Bắt buộc áp dụng cho cả hai bộ điều khiển:**

```simulink
P_brake_raw ──▶ [Saturation: 0 to 160 bar] ──▶ P_brake_cmd
                           │
                    (NFR-SAFE-01)
```

---

## 7. Sinh Mã Tự Động

### 7.1 Mục Tiêu Code Generation

Sử dụng **Simulink Coder** để sinh C code từ `ABS_Controller` subsystem. Đây là điểm cốt lõi của MBD workflow, thể hiện vòng lặp: **Model → Verify → Generate → Deploy**.

### 7.2 Cấu Hình Model cho Code Generation

**Cài đặt Configuration Parameters:**

```
Solver:
  Type:          Fixed-step
  Solver:        ode4 (Runge-Kutta)
  Fixed-step size: 0.001 (1 ms — tương đương Ts thực tế)

Code Generation:
  Target:          ert.tlc (Embedded Real-Time)
  Language:        C
  Interface:       Reusable function

Optimization:
  Inline parameters:    On
  Remove root outports: On
  Signal storage reuse: On

Report:
  Generate code generation report: On
  Launch report automatically:     On
```

### 7.3 Cấu Trúc Code Sinh Ra

Sau khi generate, cấu trúc thư mục dự kiến:

```
ABS_Controller_ert_rtw/
├── ABS_Controller.c          ← Main algorithm (step function)
├── ABS_Controller.h          ← External interface, type definitions
├── ABS_Controller_data.c     ← Parameters, constants
├── ABS_Controller_types.h    ← Custom types, enumerations
├── rtwtypes.h                ← Simulink base types
├── ert_main.c                ← Example main() wrapper
└── codegen_report/
    ├── html/                 ← Code generation report
    └── traceability/         ← Model-to-code traceability
```

### 7.4 Entry Point Functions

```c
/* Initialize: gọi 1 lần khi khởi động */
void ABS_Controller_initialize(void);

/* Step: gọi mỗi chu kỳ 1ms (ISR hoặc scheduler) */
void ABS_Controller_step(void);

/* Terminate: giải phóng tài nguyên */
void ABS_Controller_terminate(void);
```

### 7.5 I/O Interface của Generated Code

```c
/* --- Input Structure --- */
typedef struct {
  real_T lambda_estimated;     /* Slip ratio từ signal processing [0, 1] */
  real_T v_vehicle;            /* Vận tốc xe [m/s] */
  real_T P_driver;             /* Áp suất driver yêu cầu [bar] */
} ExtU_ABS_Controller_T;

/* --- Output Structure --- */
typedef struct {
  real_T P_brake_cmd;          /* Lệnh áp suất phanh [bar] */
  uint8_T ABS_state;           /* Trạng thái Stateflow: 0-3 */
  boolean_T ABS_active;        /* Cờ ABS đang hoạt động */
} ExtY_ABS_Controller_T;

/* --- Global instances --- */
extern ExtU_ABS_Controller_T ABS_Controller_U;
extern ExtY_ABS_Controller_T ABS_Controller_Y;
```

### 7.6 Kiểm Tra Code Thủ Công (Manual Code Review)

Sau khi generate, kiểm tra các điểm sau:

| Checklist Item | Mô tả | Pass Criteria |
|---|---|---|
| CG-CHECK-01 | Compile với gcc -Wall | 0 warnings |
| CG-CHECK-02 | Step function thực thi trong thời gian xác định | Không có vòng lặp vô hạn |
| CG-CHECK-03 | Saturation được bảo toàn trong code | `if (P > 160.0) P = 160.0;` xuất hiện |
| CG-CHECK-04 | Stateflow logic ánh xạ đúng sang switch-case | State transitions trong code khớp model |
| CG-CHECK-05 | Không có dynamic memory allocation | Không có `malloc`, `free` |
| CG-CHECK-06 | Global variable đặt tên rõ ràng | Không có `tmp_1`, `tmp_2` |

### 7.7 Model-to-Code Traceability

Simulink Coder tạo ra **hyperlinks** từ từng dòng C code → block trong model. Trong báo cáo kỹ thuật, đính kèm ít nhất 3 ví dụ traceability:

1. State transition NORMAL→DUMP → dòng C code tương ứng
2. Saturation block → dòng code clamp pressure
3. PID calculation → generated PID update code

---

## 8. Kiểm Thử & Xác Nhận

### 8.1 Chiến Lược V&V Tổng Thể

```
Level 1: Unit Testing (từng Subsystem riêng lẻ)
   ↓
Level 2: Integration Testing (kết nối Plant + Controller)
   ↓
Level 3: MIL (Model-in-the-Loop) Testing (full system vs. requirements)
   ↓
Level 4: Code Review của Generated C Code
```

### 8.2 Test Cases — Unit Testing

#### TC-PLANT: Kiểm thử Plant Model

| TC ID | Test Name | Input | Expected Output | Pass Criteria |
|---|---|---|---|---|
| TC-PLANT-01 | Free rolling (no brake) | P_brake = 0, v0 = 100 km/h | λ ≈ 0, không giảm tốc | |λ| < 0.01 |
| TC-PLANT-02 | Full brake, no ABS | P_brake = 160 bar, v0 = 100 km/h | λ → 1 trong < 0.5s | λ > 0.8 sau 0.5s |
| TC-PLANT-03 | Wheel stop condition | Brake until wheel stops | v_wheel = 0, v_vehicle > 0 | v_wheel = 0, λ = 1 |
| TC-PLANT-04 | Tire friction dry road | λ = 0.20, road = DRY | μ ≈ 0.92 | |μ - 0.92| < 0.05 |
| TC-PLANT-05 | Tire friction wet road | λ = 0.20, road = WET | μ ≈ 0.48 | |μ - 0.48| < 0.05 |
| TC-PLANT-06 | Slip ratio boundary | v_vehicle = 0.05 m/s | λ = 0 (no division by zero) | Không có NaN, Inf |

#### TC-SIG: Kiểm thử Signal Processing

| TC ID | Test Name | Input | Expected Output | Pass Criteria |
|---|---|---|---|---|
| TC-SIG-01 | LPF DC gain | Sine 1 Hz, A = 1 m/s | Output amplitude ≈ 1 m/s | Gain ≥ -0.5 dB |
| TC-SIG-02 | LPF attenuation at 50 Hz | Sine 50 Hz, A = 1 m/s | Output amplitude ≤ 0.5 m/s | Gain ≤ -8 dB |
| TC-SIG-03 | LPF attenuation at 100 Hz | Sine 100 Hz, A = 1 m/s | Output amplitude ≤ 0.06 m/s | Gain ≤ -24 dB |
| TC-SIG-04 | Phase lag at 10 Hz | Sine 10 Hz | Phase lag ≤ 20 ms | Measured lag ≤ 20 ms |
| TC-SIG-05 | Noise rejection | v_wheel + Gaussian noise σ=0.1 | SNR sau lọc ≥ 20 dB | Measured SNR ≥ 20 dB |
| TC-SIG-06 | Slip estimation accuracy | λ_actual = 0.20, with noise | |λ_est - 0.20| ≤ 0.02 | Error ≤ 0.02 |

#### TC-CTRL: Kiểm thử Controller

| TC ID | Test Name | Input | Expected Output | Pass Criteria |
|---|---|---|---|---|
| TC-CTRL-01 | ABS activation | λ increases to 0.40 | ABS state = DUMP, P giảm | State = DUMP trong < 1 step |
| TC-CTRL-02 | ABS deactivation | v_vehicle < 0.5 m/s | State = NORMAL, ABS_active = 0 | Deactivate đúng |
| TC-CTRL-03 | Pressure never exceeds limit | Any input | P_brake ≤ 160 bar | Max(P) ≤ 160 |
| TC-CTRL-04 | Pressure never below 0 | Any input | P_brake ≥ 0 bar | Min(P) ≥ 0 |
| TC-CTRL-05 | State transition DUMP→BUILD | λ drops from 0.40 to 0.10 | State = BUILD | Transition đúng |
| TC-CTRL-06 | PID steady-state error | λ* = 0.20, steady state | |λ_ss - 0.20| ≤ 0.03 | SSE ≤ 0.03 |
| TC-CTRL-07 | PID settling time | Step λ_ref = 0.20 | λ đạt trong ±5% sau ≤ 0.5s | ts ≤ 0.5 s |
| TC-CTRL-08 | Bang-bang frequency | Full ABS engagement | Switching freq ≤ 15 Hz | Measured freq ≤ 15 Hz |

### 8.3 Test Cases — MIL Integration Testing

| TC ID | Test Scenario | Inputs | Required Outcomes | Metrics |
|---|---|---|---|---|
| TC-MIL-01 | Emergency brake, dry road | v0=100 km/h, P=160 bar, dry | ABS duy trì λ ∈ [0.15, 0.25] | % thời gian trong dải |
| TC-MIL-02 | Emergency brake, wet road | v0=100 km/h, P=160 bar, wet | Quãng đường ABS < No-ABS | d_ABS / d_noABS |
| TC-MIL-03 | Emergency brake, gravel | v0=80 km/h, P=160 bar, gravel | Xe dừng an toàn, không spin | λ không ở 1 quá 0.1s |
| TC-MIL-04 | ABS vs No-ABS comparison | v0=100 km/h, dry | d_ABS ≤ 0.85 × d_noABS | Improvement ≥ 15% |
| TC-MIL-05 | Gradual braking (no ABS trigger) | v0=60 km/h, P ramp to 80 bar | ABS không active | ABS_active = 0 toàn bộ |
| TC-MIL-06 | Signal noise robustness | Noise σ = 0.2 m/s (2×) | ABS vẫn hoạt động đúng | Không false activation |

### 8.4 Kịch Bản Mô Phỏng So Sánh

**Scenario A: No ABS (Baseline)**
```
t = 0s:     v0 = 100 km/h, áp full phanh P = 160 bar
t = 0 → stop: bánh xe lock ngay lập tức, xe trượt thẳng
Kết quả ghi lại:
  - Thời gian phanh
  - Quãng đường phanh
  - Diễn biến slip ratio (→ 1)
  - Diễn biến vận tốc xe và bánh
```

**Scenario B: ABS Bang-Bang**
```
Cùng điều kiện, bật ABS
Kết quả ghi lại:
  - Slip ratio oscillate quanh 0.15-0.25
  - Áp suất phanh đóng-mở theo Stateflow
  - Quãng đường phanh ngắn hơn ≥ 15%
```

**Scenario C: ABS PID**
```
Cùng điều kiện, dùng PID controller
Kết quả ghi lại:
  - Slip ratio mượt mà hơn, ít oscillation
  - So sánh với Bang-Bang về settling time, overshoot
```

### 8.5 Định Dạng Test Report

```markdown
## Test Report — TC-MIL-01

**Test ID:**       TC-MIL-01
**Date:**          2026-xx-xx
**Tester:**        [Tên sinh viên]
**Simulink Model:** ABS_System.slx (Version x.x)

### Input Configuration
| Parameter       | Value          |
|----------------|----------------|
| v_initial      | 27.78 m/s (100 km/h) |
| Road condition | Dry asphalt (μ = 0.9) |
| Brake pressure | 160 bar (full brake)  |
| Simulation time| 10 s                  |
| Controller     | Bang-Bang + Stateflow |

### Results
| Metric               | Expected         | Actual          | Status  |
|----------------------|------------------|-----------------|---------|
| Slip ratio min       | ≥ 0.10           | 0.13            | PASS    |
| Slip ratio max (ABS) | ≤ 0.40           | 0.38            | PASS    |
| % time in [0.15,0.25]| ≥ 70%            | 74.2%           | PASS    |
| Stopping distance    | < d_noABS×0.85   | 48.3 m (vs 58.1 m) | PASS |
| ABS activation time  | Within 0.05s     | 0.03s           | PASS    |

### Observations
[Nhận xét về kết quả, bất thường nếu có]

### Conclusion
Test PASSED — All acceptance criteria met.
```

---

## 9. Requirements Traceability Matrix

> RTM là tài liệu quan trọng nhất trong V&V — chứng minh rằng mọi requirement đều được verify bởi ít nhất một test case.

| Req ID | Requirement Description | Test Case(s) | Design Element | Status |
|---|---|---|---|---|
| FR-DETECT-01 | Tính toán slip ratio | TC-PLANT-06, TC-SIG-06 | Slip_Ratio_Estimator | Planned |
| FR-DETECT-02 | Phát hiện wheel lock | TC-CTRL-01 | Stateflow: NORMAL→DUMP | Planned |
| FR-DETECT-03 | Phân biệt NORMAL / LOCK | TC-CTRL-01, TC-CTRL-05 | Stateflow states | Planned |
| FR-DETECT-04 | Duy trì slip 0.15-0.25 | TC-MIL-01 | All states logic | Planned |
| FR-CTRL-01 | Giảm P khi lock | TC-CTRL-01 | DUMP state action | Planned |
| FR-CTRL-02 | Tăng P khi thoát lock | TC-CTRL-05 | BUILD state action | Planned |
| FR-CTRL-03 | Tần số đóng-mở ≤ 15 Hz | TC-CTRL-08 | Stateflow timing | Planned |
| FR-CTRL-04 | PID hội tụ trong 0.5s | TC-CTRL-07 | PID block tuning | Planned |
| FR-CTRL-05 | P ∈ [0, 160] bar | TC-CTRL-03, TC-CTRL-04 | Saturation block | Planned |
| FR-SIG-01 | Mô phỏng nhiễu Gaussian | TC-SIG-05 | Band-Limited WN block | Planned |
| FR-SIG-02 | LPF attenuation > 50 Hz | TC-SIG-02, TC-SIG-03 | LPF H(s) block | Planned |
| FR-SIG-03 | Phase lag ≤ 20 ms | TC-SIG-04 | LPF design | Planned |
| FR-SIG-04 | Slip estimation error ≤ 0.02 | TC-SIG-06 | Slip_Ratio_Estimator | Planned |
| FR-PERF-01 | ABS ngắn hơn ≥ 15% | TC-MIL-04 | Full system | Planned |
| FR-SIM-01 | 3 loại mặt đường | TC-MIL-01,02,03 | Road_Type parameter | Planned |
| FR-SIM-02 | Phanh khẩn cấp v0=100 km/h | TC-MIL-01,04 | Simulation config | Planned |
| NFR-MOD-01 | Subsystem độc lập | Code Review | Model architecture | Planned |
| NFR-SAFE-01 | Saturation bắt buộc | TC-CTRL-03, TC-CTRL-04 | Saturation block | Planned |
| NFR-CODE-01 | Generated code compile | CG-CHECK-01 | Simulink Coder config | Planned |

---

## 10. Kế Hoạch Thực Hiện

### 10.1 Phân Chia Giai Đoạn

| Phase | Tên giai đoạn | Thời gian | Deliverables |
|---|---|---|---|
| Phase 1 | Requirements & Architecture | Tuần 1-2 | D-01, D-02 |
| Phase 2 | Plant Model | Tuần 3-4 | D-03 |
| Phase 3 | Controller + Signal Processing | Tuần 5-7 | D-04, D-05, D-06 |
| Phase 4 | Simulation & Analysis | Tuần 8-9 | D-07 |
| Phase 5 | Verification & Validation | Tuần 10-11 | D-08, D-10 |
| Phase 6 | Code Generation & Review | Tuần 12 | D-09 |
| Phase 7 | Report & Presentation | Tuần 13-14 | D-11 |

### 10.2 Chi Tiết Timeline

```
Tuần 1:  Nghiên cứu tài liệu ABS, MBD, ASPICE
         → Hoàn thành SRS v1.0 (Section 2 của spec này)
         → Setup MATLAB, tạo project template

Tuần 2:  Hoàn thiện kiến trúc hệ thống
         → Architecture Document
         → Xác nhận license Simulink Coder với nhà trường

Tuần 3:  Xây dựng Vehicle Dynamics subsystem
         → Verify Newton's Law model bằng hand calculation
         → Unit test TC-PLANT-01, TC-PLANT-02

Tuần 4:  Xây dựng Wheel Dynamics + Tire Model (Pacejka)
         → Plot đường cong μ-λ cho 3 mặt đường
         → Unit test TC-PLANT-03, TC-PLANT-04, TC-PLANT-05, TC-PLANT-06

Tuần 5:  Xây dựng Signal Processing subsystem
         → Thiết kế LPF, verify bằng bode plot
         → Unit test TC-SIG-01 đến TC-SIG-06

Tuần 6:  Xây dựng Stateflow State Machine + Bang-Bang Controller
         → Unit test TC-CTRL-01 đến TC-CTRL-05, TC-CTRL-08

Tuần 7:  Xây dựng PID Controller, tuning tham số
         → Integrate toàn bộ hệ thống (Plant + Signal + Controller)
         → Unit test TC-CTRL-06, TC-CTRL-07

Tuần 8:  Chạy simulation Scenario A, B, C
         → Thu thập kết quả, vẽ biểu đồ so sánh
         → Phân tích định lượng

Tuần 9:  Viết Simulation Analysis Report (D-07)
         → So sánh bang-bang vs PID
         → Sensitivity analysis (thay đổi tham số)

Tuần 10: Chạy toàn bộ MIL Test Cases (TC-MIL-01 đến TC-MIL-06)
         → Ghi kết quả vào Test Report format

Tuần 11: Hoàn thành RTM (Requirements Traceability Matrix)
         → Xác nhận tất cả requirements được cover

Tuần 12: Code Generation với Simulink Coder
         → Compile test, CG-CHECK-01 đến CG-CHECK-06
         → Viết Code Review documentation

Tuần 13: Viết báo cáo kỹ thuật hoàn chỉnh
         → Chương 1: Giới thiệu & Bối cảnh
         → Chương 2: Cơ sở lý thuyết
         → Chương 3: Thiết kế hệ thống (SRS + Architecture)
         → Chương 4: Implementation (Model + Signal + Control)
         → Chương 5: V&V Results
         → Chương 6: Code Generation
         → Chương 7: Kết luận & Hướng phát triển

Tuần 14: Chuẩn bị slide thuyết trình
         → Demo live simulation trên MATLAB
         → Rehearsal & final check
```

### 10.3 Risk Management

| Risk | Probability | Impact | Mitigation |
|---|---|---|---|
| Không có license Simulink Coder | Medium | High | Liên hệ nhà trường sớm (Tuần 1); Fallback: dùng basic Simulink Coder free trial |
| Mô hình Plant không ổn định số | Medium | High | Dùng solver ode4, fixed-step; kiểm tra stiff system |
| PID không hội tụ | Low | Medium | Có bang-bang controller làm backup; dùng Simulink PID Tuner |
| Tốn nhiều thời gian debug Stateflow | Medium | Medium | Dùng Stateflow Debugger; viết unit test từng state |
| Kết quả simulation không thực tế | Low | Medium | Validate tham số vật lý với tài liệu kỹ thuật ABS thực tế |

---

## 11. Phụ Lục

### 11.1 Danh Sách Tài Liệu Tham Khảo

1. Bosch Automotive Handbook, 10th Edition — Mục "Antilock Braking System (ABS)"
2. Pacejka, H.B. — "Tire and Vehicle Dynamics", 3rd Edition, 2012
3. MathWorks — "Model-Based Design with Simulink" (mathworks.com/solutions/model-based-design)
4. MathWorks — "Simulink Coder User's Guide" (R2023b)
5. MathWorks — "Stateflow User's Guide" (R2023b)
6. Rajamani, R. — "Vehicle Dynamics and Control", 2nd Edition, Springer, 2012
7. AUTOSAR Foundation — "AUTOSAR Software Requirements Specification"
8. Kiencke, U. & Nielsen, L. — "Automotive Control Systems", 2nd Edition, 2005

### 11.2 Danh Sách Ký Hiệu & Viết Tắt

| Ký hiệu / Viết tắt | Định nghĩa |
|---|---|
| ABS | Anti-lock Braking System |
| MBD | Model-Based Design |
| SRS | Software Requirements Specification |
| RTM | Requirements Traceability Matrix |
| MIL | Model-in-the-Loop |
| HIL | Hardware-in-the-Loop |
| LPF | Low-Pass Filter |
| PID | Proportional-Integral-Derivative |
| V&V | Verification & Validation |
| WSS | Wheel Speed Sensor |
| SDLC | Software Development Lifecycle |
| λ | Slip Ratio |
| μ | Friction Coefficient |
| ω | Angular velocity (rad/s) |
| FR | Functional Requirement |
| NFR | Non-Functional Requirement |
| TC | Test Case |
| SNR | Signal-to-Noise Ratio |

### 11.3 Danh Sách Biểu Đồ Cần Có Trong Báo Cáo

| STT | Tên biểu đồ | Nội dung | Section báo cáo |
|---|---|---|---|
| Fig-01 | Đường cong μ-λ (3 mặt đường) | Tire model validation | Ch.3 |
| Fig-02 | Simulink top-level model screenshot | Kiến trúc model | Ch.4 |
| Fig-03 | Stateflow chart | State machine logic | Ch.4 |
| Fig-04 | Bode plot của LPF | Signal processing validation | Ch.4 |
| Fig-05 | Slip ratio vs Time (No ABS) | Baseline scenario | Ch.5 |
| Fig-06 | Slip ratio vs Time (Bang-Bang ABS) | ABS performance | Ch.5 |
| Fig-07 | Slip ratio vs Time (PID ABS) | PID performance | Ch.5 |
| Fig-08 | Velocity comparison (vehicle vs wheel, 3 scenarios) | Key result | Ch.5 |
| Fig-09 | Brake pressure vs Time (Bang-Bang) | Controller output | Ch.5 |
| Fig-10 | Stopping distance comparison bar chart | Main KPI | Ch.5 |
| Fig-11 | Generated C code screenshot với traceability | Code generation | Ch.6 |
| Fig-12 | Model-to-Code traceability example | V&V | Ch.6 |

---

> **Ghi chú cuối tài liệu:**  
> Spec này là tài liệu sống (living document) — cập nhật khi có phát sinh kỹ thuật trong quá trình thực hiện.  
> Mọi thay đổi so với spec gốc phải được ghi lại trong **Change Log** và thông báo cho Lead Architect.

---
*Tài liệu được soạn thảo theo chuẩn IEEE 830 SRS & ASPICE Level 1-2 Process Framework*  
*Model-Based Design workflow reference: MathWorks MBD V-Model*
