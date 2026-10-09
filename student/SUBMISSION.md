# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Tran Cao Quoc Dinh
- MSSV: 2A202602939
- Email: kentranpr4@gmail.com
- Link repo (fork): https://github.com/kentranpr4-crypto/K4-L2L3-DAY23-TranCaoQuocDinh-2A202602939-SensorFusion
- Commit hash nộp (`git rev-parse HEAD`): lấy hash của commit CP6 cuối cùng và ghi trên LMS (commit không thể tự chứa hash của chính nó).

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: `compare`, `[0, 198]`, `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`, `0`.
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: `0.9700934579439252`, `0.7004048582995951`, `519/16/222`.
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: `0.1502780791727643` m, `502`, `11.336917542087521` m², `0`, `239`, `2.522613065326633`.
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: `0.13584776691057274` m, `502`, `9.26421711884383` m², `0`, `239`, `2.522613065326633`.
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss: Fused giảm RMSE `0.014430312262191575` m so với LiDAR-only. Hai mode đều ghép `502` cặp, có `0` ghost và `239` missed GT frame, nên lần chạy này camera cải thiện độ chính xác vị trí của các track được ghép, không thay đổi số track được ghép. `precision_track = 502/(502+0) = 1.0`; `coverage = 502/519 = 0.9672447013487476`. Detection bỏ sót `222` nhãn xe, vì vậy vẫn cần đọc tracking cùng detection recall, không chỉ RMSE.

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. LiDAR đo tâm hộp 3D trong hệ cảm biến: `z` có 3 phần tử (m), `R` là ma trận 3x3 với phương sai `sigma_lidar_x/y/z^2`. Camera đo tâm hộp ảnh: `z` có 2 phần tử `(u, v)` (pixel), `R` là 2x2 với phương sai `sigma_cam_i/j^2`. Cả hai cùng cập nhật trạng thái 6D, nhưng hàm đo camera là phép chiếu phi tuyến và cần Jacobian.
2. Khoảng cách Mahalanobis bình phương `gamma.T @ S^{-1} @ gamma` xét cả sai lệch đo lẫn độ bất định của track và sensor. Cổng chi-bình-phương loại cặp quá xa trước khi ghép greedy, tránh một đo sai kéo trạng thái hoặc ID sang xe khác. Trong `association_cost_matrix`, cặp ngoài FOV bị loại trước khi tính khoảng cách.
3. Đây là track-then-fuse: một danh sách track được predict một lần mỗi frame; LiDAR ghép và update trước, camera ghép và update trên cùng track sau. `fusion_lab/scripts/run_lab.py` gọi `KF.predict`, `associate_and_update(..., lidar_sensor)`, rồi `associate_and_update(..., camera_sensor)`. `grade_run.log` có 398 record: đúng một record `lidar` và một record `fused` cho mỗi frame 0–198. Tổng hai mode đều có 502 matches; sự khác biệt nằm ở `sum_sq_err` (11.3369 so với 9.2642 m²).
4. Extrinsic camera lệch làm tọa độ dự báo `h(x)` sai, nên innovation `z - h(x)` bị lệch có hệ thống. Khoảng cách Mahalanobis có thể tăng, khiến nhiều cặp camera bị gating loại; nếu vẫn lọt cổng, update camera có thể kéo track sai và tăng RMSE fused.
5. Khi `meas_list` rỗng, vẫn phải gọi `manage_tracks` với `sensor` tường minh để phân biệt lượt LiDAR với lượt camera. LiDAR miss trong FOV làm giảm score và có thể xóa track; đo LiDAR chưa ghép có thể tạo track mới. Camera chỉ sửa `x, P` của track đã ghép, không tăng/giảm score và không tạo/xóa track vì nhãn camera trong lab là đo mô phỏng.
6. Track mới có state `initialized`, score `1/window`. LiDAR hit tăng `1/window` (tối đa 1); score lớn hơn `confirmed_threshold` thì thành `confirmed`. LiDAR miss trong FOV giảm `1/window`, nhưng state confirmed được giữ. Xóa nếu `P[0,0]` hoặc `P[1,1]` lớn hơn `max_P`; hoặc confirmed có score nhỏ hơn `delete_threshold`; hoặc track chưa confirmed có score nhỏ hơn hay bằng 0.

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- Không.

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): OpenAI Codex.
- Dùng cho phần nào (hàm, câu hỏi, debug): Cài Part E-H, tạo notebook Colab, giải thích thuật toán và chuẩn bị báo cáo.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): Đối chiếu công thức với tài liệu trong repo; `pytest student/tests -q` đạt 128 passed trên Python 3.12. Chạy Waymo đủ frame 0–198 với `--fusion compare --seed 0`; đối chiếu 398 record trong `grade_run.log` với `metrics.json` bằng `validate_metrics_records`.

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không sửa
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền số liệu, giải thích E–H và khai báo AI; commit hash cuối nộp trên LMS
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [x] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP` sau commit CP6
- [ ] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
