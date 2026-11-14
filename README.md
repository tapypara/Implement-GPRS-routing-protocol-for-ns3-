# Mô phỏng lại GPSR trên ns-3

Bài tập lớn học phần Đánh giá hiệu năng mạng (2627I_INT3308_1).
Sinh viên: Trần Anh Khoa – 24022806.

Cài đặt giao thức GPSR (B. Karp, H. T. Kung, *GPSR: Greedy Perimeter Stateless Routing for Wireless Networks*, ACM MobiCom 2000) trên ns-3.43 và chạy lại các thí nghiệm ở mục 3 của bài báo.

## Cấu trúc

| Đường dẫn | Nội dung |
| --- | --- |
| `src/gpsr/` | Module GPSR: beacon, bảng nút lân cận, greedy, perimeter (RNG/GG), phản hồi lỗi MAC |
| `scratch/gpsr-paper-scenario.cc` | Kịch bản mô phỏng theo mục 3 của bài báo và các phép đo |
| `scratch/gpsr-void-test.cc` | Kiểm thử chế độ perimeter trên topo có vùng trống |
| `run_sim_and_plot.py` | Chạy song song các lần mô phỏng, tính trung bình và vẽ đồ thị |
| `results/` | Số liệu tổng hợp (`simulation_summary.csv`) và đồ thị của 102 lần chạy |
| `report/` | Báo cáo PDF và mã LaTeX (biên dịch bằng XeLaTeX) |

## Cài đặt

Chép `src/gpsr` và hai tệp trong `scratch/` vào thư mục ns-3.43, rồi build:

```bash
cp -r src/gpsr        ~/ns-allinone-3.43/ns-3.43/src/
cp scratch/*.cc       ~/ns-allinone-3.43/ns-3.43/scratch/
cp run_sim_and_plot.py ~/ns-allinone-3.43/ns-3.43/
cd ~/ns-allinone-3.43/ns-3.43
./ns3 configure -d release --enable-examples
./ns3 build
```

## Chạy

```bash
# một lần chạy: 50 nút, B = 1,5 s, pause 0
./ns3 run "scratch/gpsr-paper-scenario --nNodes=50 --helloInterval=1.5 --pauseTime=0 --runIndex=1"

# kiểm thử perimeter (RNG và GG)
./ns3 run "scratch/gpsr-void-test --planarMode=0"
./ns3 run "scratch/gpsr-void-test --planarMode=1"

# toàn bộ thí nghiệm GPSR (102 lần chạy, 900 s mỗi lần) và vẽ đồ thị
python3 run_sim_and_plot.py --jobs 6 --scaling --no-dsr
```

Kết quả được ghi vào `results/`. Nếu thư mục này đã có tệp `simulation_raw_results.csv`, script sẽ bỏ qua các cấu hình đã chạy.

## Lưu ý

- Kịch bản mặc định dùng bảng ARP tĩnh (`--staticArp=1`); xem mục 2.3 của báo cáo. Đặt `--staticArp=0` để dùng ARP động của ns-3.
- Kết quả trong báo cáo chỉ chạy GPSR (`--no-dsr`); lý do không dùng DSR của ns-3 làm đối chứng ở mục 2.4 của báo cáo.
- Mỗi lần chạy in thêm dòng `DIAG` thống kê gói bị bỏ theo nguyên nhân (ARP, IP, MAC).
