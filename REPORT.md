MSSV: 202602445

# Lab 11 · Scenario Forge — Báo cáo

**Học viên:** Nguyễn Văn Ước  
**MSSV:** 2A202602445 (Mã số chạy pipeline: `202602445`)  
**Thị trấn của bạn (`meta.json: _run.town`):** **D**  

---

## 1. Phát lại log có trung thực không? (bước A)

### Bảng số liệu trích xuất từ `evidence/replay.json`:

| Chỉ số | Giá trị | Trích xuất từ `replay.json` |
| :--- | :--- | :--- |
| Tổng số log kịch bản tự nhiên | 400 | `n = 400` |
| Kịch bản bị bỏ vì người lái gây va chạm | **1** | `dropped_human_crash_logs = 1` |
| Số kịch bản đưa vào kiểm thử | 399 | `n_used = 399` |
| Va chạm khi planner chạy trên log phát lại (replay) | **91** (84 rear, 7 side, 0 front) | `replay.crashes = 91`, `replay.contact = {"rear": 84, "side": 7}` |
| Va chạm trong thế giới phản ứng (reactive) | **0** (0 rear, 0 side, 0 front) | `reactive.crashes = 0`, `reactive.contact = {}` |
| Va chạm xảy ra ở cả hai môi trường | 0 | `crash_in_both = 0` |
| Số kịch bản chỉ va chạm ở log phát lại | 91 | `len(replay_only) = 91` |

### Giải thích cơ chế va chạm `rear` và vì sao đó là va chạm GIẢ:
1. **Nguyên nhân va chạm đâm đuôi (`rear` chiếm 84/91 vụ = 92.3%):**
   - Trong chế độ phát lại log mở (open-loop replay), quỹ đạo của các xe khác được "tua lại" một cách mù quáng theo đúng những gì tài xế con người từng đi trong quá khứ khi log được ghi lại.
   - Khi xe tự hành (ego) do planner điều khiển chạy trên kịch bản đó, planner đưa ra quyết định thận trọng hơn tài xế người trong quá khứ (ví dụ: phanh sớm hơn 0.5 s, nhường người đi bộ hoặc giảm tốc cẩn trọng trước điểm mù xe buýt). Hệ quả là xe ego đi chậm hơn hoặc dừng lại tại vị trí mà tài xế người trước đây không dừng.
   - Xe phía sau trong log không hề "nhìn thấy" sự thay đổi này của ego, nó tiếp tục di chuyển tới với tốc độ trong quá khứ và húc thẳng vào đuôi xe ego (`rear`).
2. **Vì sao đây là va chạm giả (phantom/artifact collision):**
   - Trong thế giới thực (và trong mô phỏng closed-loop phản ứng `reactive`), tài xế xe phía sau có mắt quan sát và tuân theo quy luật bám đuôi (mô hình IDM). Khi xe trước phanh lại, xe sau sẽ phanh theo để giữ khoảng cách an toàn. 
   - Bằng chứng thực nghiệm đanh thép: Khi chuyển sang môi trường tương tác phản ứng (`reactive`), số va chạm từ **91 giảm ngay về đúng 0** (`min_ttc_reactive` của các cảnh này đều an toàn > 2.0 s). Do đó, 91 vụ va chạm trên log phát lại hoàn toàn là ảo ảnh do vi phạm tính phản ứng (distribution shift / lack of reactivity).

### Phân tích một kịch bản trong `replay_only`:
- **Kịch bản `scenario: 7` (`min_ttc_reactive = 2.5897 s`, tiếp xúc `rear`):**
  - Tham số: `pedT0 = 10.493 s`, `pedV = 1.380 m/s`, `busGo = 17.729 s`, `mu = 0.5`.
  - Diễn biến: Người đi bộ bắt đầu băng qua đường từ giây thứ 10.49 với vận tốc 1.38 m/s trên mặt đường trơn $\mu = 0.5$. Planner của ego phát hiện người đi bộ và chủ động phanh dừng êm ái trước vạch để nhường đường.
  - Trong thế giới phản ứng (`reactive`), xe bám đuôi phía sau nhìn thấy ego giảm tốc nên cũng phanh theo, khoảng cách luôn giữ an toàn với `min TTC = 2.59 s`. 
  - Trong log phát lại (`replay`), xe phía sau bị "mù" và lao theo vết xe cũ của người lái trong dữ liệu log, đâm sầm vào đuôi chiếc xe ego đang dừng ngay ngắn nhường đường.

---

## 2. Kịch bản bất khả thi (bước B)

### Danh sách các luật trong `is_feasible` và căn cứ vật lý:
1. **Gia tốc của xe ego dọc theo hành trình:**
   - Luật: $\max(|a_{\text{ego}}|) \le \mu \cdot 9.81\text{ m/s}^2$.
   - Căn cứ vật lý: Định luật II Newton kết hợp định luật ma sát Coulomb. Lực ma sát cực đại giữa lốp xe và mặt đường là $F_{\text{fric}} = \mu m g$, suy ra gia tốc cực đại mà xe có thể đạt được (cả khi tăng tốc lẫn phanh khẩn cấp) không thể vượt quá $\mu g$. Kịch bản đòi hỏi xe phanh vượt quá $\mu g$ là phi thực tế vì xe sẽ bị trượt lốp không thể kiểm soát.
2. **Xe cắt đầu (`cut_in`):**
   - Gia tốc ngang: Xe chuyển làn hình sin (cosine smooth step) với độ lách ngang `cutDs` trong thời gian `cutDur`. Đạo hàm bậc hai cực đại của quỹ đạo ngang cho gia tốc ngang đỉnh: $a_{\text{lat}} = \frac{\pi^2 \cdot |\text{cutDs}|}{2 \cdot (\text{cutDur})^2}$. Luật yêu cầu $a_{\text{lat}} \le \mu \cdot 9.81\text{ m/s}^2$. Căn cứ: Lực bám ngang của lốp trên mặt đường có hệ số ma sát $\mu$.
   - Gia tốc phanh dọc: $\text{cutBrake} \le \mu \cdot 9.81\text{ m/s}^2$. Căn cứ: Lực phanh dọc tối đa bị chặn bởi giới hạn bám của lốp.
3. **Người đi bộ (`ped`):**
   - Vận tốc: $0.6\text{ m/s} \le \text{pedV} \le 2.6\text{ m/s}$.
   - Căn cứ vật lý: Giới hạn cơ sinh học vận động người (human biomechanics). Người bình thường đi bộ từ tốn ở mức $\approx 0.6\text{ m/s}$ (người già, trẻ nhỏ) và chạy nước rút đô thị đạt tối đa $\approx 2.6\text{ m/s}$. Tốc độ ngoài khoảng này (ví dụ chạy nhanh như vận động viên Olympic $> 4\text{ m/s}$ hay bò lết) là phi lý trong ngữ cảnh giao thông phố thông thường.
4. **Không chồng lấn khi xuất phát (`overlap_at_start`):**
   - Luật: Không có đối tượng nào va chạm hoặc lọt vào lòng nhau ở frame 0 ($t = 0$). Căn cứ: Tính bất khả xâm phạm của vật rắn (solid body impenetrability).

### Cuộc săn sinh nhiều kịch bản bất khả thi nhất (`hunt.json`):
- `rear_ended` sinh ra nhiều kịch bản bất khả thi nhất: **19 kịch bản** (`infeasible = 19`, trong khi `bus_bypass` có 3, `cut_in` có 2, `crosswalk_block` có 8).
- **Vì sao CEM bị kéo về phía đó:**
  - Thuật toán Cross-Entropy Method (CEM) tối ưu hóa hàm phân phối đề xuất bằng cách chọn lọc nhóm mẫu tinh hoa (elite samples) có mức độ nguy hiểm cao nhất.
  - Để ép xe phía sau đâm vào đuôi xe ego một cách dễ nhất, CEM bị "cám dỗ" đẩy các tham số tới các vùng cực đoan: ép hệ số ma sát $\mu$ xuống mức rất thấp trong khi đòi hỏi gia tốc phanh dọc/ngang của xe sau cực cao, hoặc ép khoảng cách bám đuôi cực sát vượt quá giới hạn dừng xe vật lý ($a > \mu g$). 
  - Tại vùng biên phi vật lý này, xác suất đâm xe là 100%, đem lại điểm thưởng (reward) tuyệt đối cho CEM. Do đó, bộ tối ưu hóa tự nhiên bị hút mạnh về vùng tham số vi phạm định luật vật lý này.

### Điều gì xảy ra với ước lượng nếu đếm chúng là va chạm:
- Nếu đếm cả các kịch bản bất khả thi, xác suất rủi ro $\hat{p}$ sẽ bị **thổi phồng giả tạo (overestimated / biased upwards)** một cách nghiêm trọng bởi những "vụ tai nạn ma thuật" vốn không bao giờ xảy ra ngoài đời thực.
- Hậu quả kỹ thuật: Đội ngũ phát triển sẽ lãng phí tài nguyên để khắc phục những tình huống vi phạm định luật bảo toàn/ma sát, hoặc buộc planner phải lái xe theo kiểu "robot đóng băng" (frozen robot) — rụt rè thái quá, không dám di chuyển trên đường vì sợ những nguy cơ ma quái vi phạm vật lý.

---

## 3. Monte Carlo cho bạn biết gì (bước C1)

### Số liệu từ `evidence/mc.json`:
- Số va chạm quan sát được: $k = 3$ (`crashes = 3`).
- Tổng số lần chạy: $n = 3000$ (`n = 3000`).
- Ước lượng điểm: $\hat{p} = \frac{k}{n} = 0.001 = 1.00 \times 10^{-3}$ (`p = 0.001`).
- Khoảng tin cậy Wilson 95%: $[0.00034014, 0.00293620] \approx [3.40 \times 10^{-4}, 2.94 \times 10^{-3}]$ (`lo = 0.0003401410630165598`, `hi = 0.002936196810458527`).

### Tính toán số lần chạy để khoảng tin cậy hẹp lại còn $\pm 20\%$ của $p$:
- Nửa độ rộng khoảng tin cậy 95% của tỷ lệ nhị thức đối với sự kiện hiếm ($p \ll 1$) xấp xỉ:
  $$E = 1.96 \sqrt{\frac{p(1-p)}{N}} \approx 1.96 \sqrt{\frac{p}{N}}$$
- Yêu cầu sai số tương đối $\le 20\%$:
  $$\frac{E}{p} \le 0.20 \iff \frac{1.96 \sqrt{p/N}}{p} \le 0.20 \iff \frac{1.96}{\sqrt{N p}} \le 0.20 \iff \sqrt{N p} \ge \frac{1.96}{0.20} = 9.8$$
  $$\implies N p \ge 9.8^2 \approx 96.04$$
  *(Nghĩa là phương pháp Monte Carlo cần quan sát được ít nhất khoảng 96 vụ va chạm để đạt độ chính xác tương đối $\pm 20\%$)*.
- Với $\hat{p} = 1.00 \times 10^{-3}$:
  $$N \ge \frac{96.04}{0.001} \approx 96,040 \text{ lần chạy}$$
  Cần gần **100 000 lần mô phỏng** — một con số khổng lồ không khả thi nếu chạy mô phỏng chi tiết độ phân giải cao!

### Kiểu thất bại chưa hề xuất hiện trong 3 000 lần chạy:
- Toàn bộ 3 vụ va chạm trong `mc.json` đều thuộc kiểu `bus_bypass` ($k=3$).
- Các kiểu thất bại sau **chưa hề xuất hiện ($k = 0$)** trong 3 000 lần chạy:
  1. `cut_in`: $k = 0$
  2. `crosswalk_block`: $k = 0$
  3. `rear_ended`: $k = 0$
  4. `other`: $k = 0$

### Con số đúng cần báo cáo cho các kiểu chưa xuất hiện:
- **Tuyệt đối KHÔNG ĐƯỢC báo 0.0** (vì 0 vụ va chạm trên 3 000 mẫu không chứng minh xác suất bằng 0, mà chỉ chứng minh đây là biến cố hiếm).
- Con số khoa học và trung thực cần báo là **cận trên của khoảng tin cậy Wilson 95%** (hoặc quy tắc Rule of Three cho phân phối Poisson $3/N$):
  - Giá trị cận trên Wilson 95% trích từ `mc.json` cho cả 4 kiểu có $k=0$:
    $$\text{Upper Bound (Wilson 95\%)} = \mathbf{0.00127889566} \approx \mathbf{1.28 \times 10^{-3}}$$
  - Báo cáo chuẩn xác: $p_{\text{cut\_in}} \le 1.28 \times 10^{-3}$, $p_{\text{crosswalk\_block}} \le 1.28 \times 10^{-3}$, $p_{\text{rear\_ended}} \le 1.28 \times 10^{-3}$, $p_{\text{other}} \le 1.28 \times 10^{-3}$ (với độ tin cậy 95%).

---

## 4. Importance sampling, ESS và độ suy biến (bước C3)

### Bảng số liệu trích xuất từ `evidence/estimate.json` ($n = 600$ mỗi mode):

| Kiểu thất bại | Seed | $p$ | `hits` | `ess` | `top_share` | Ghi chú & Cận trên hợp lệ |
| :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **bus_bypass** | 0 | $4.04 \times 10^{-4}$ | 31 | 114.6 | 0.472 | Khoảng 95%: $[9.66 \times 10^{-6}, 7.98 \times 10^{-4}]$ |
| | 1 | $3.35 \times 10^{-4}$ | 21 | 119.3 | 0.243 | Khoảng 95%: $[1.04 \times 10^{-4}, 5.66 \times 10^{-4}]$ |
| | 2 | $1.27 \times 10^{-3}$ | 23 | 118.4 | **0.702** | Khoảng 95%: $[0, 3.07 \times 10^{-3}]$ |
| **cut_in** | 0 | 0.0 | 0 | 113.7 | 0.0 | Báo cận trên hợp lệ: $p \le 3/(\beta n) = \mathbf{0.025}$ |
| | 1 | 0.0 | 0 | 128.5 | 0.0 | Báo cận trên hợp lệ: $p \le 3/(\beta n) = \mathbf{0.025}$ |
| | 2 | 0.0 | 0 | 122.2 | 0.0 | Báo cận trên hợp lệ: $p \le 3/(\beta n) = \mathbf{0.025}$ |
| **crosswalk_block** | 0 | $4.43 \times 10^{-7}$ | 85 | 116.0 | **0.518** | Khoảng 95%: $[0, 9.27 \times 10^{-7}]$ |
| | 1 | $4.23 \times 10^{-7}$ | 98 | 114.0 | **0.608** | Khoảng 95%: $[0, 9.32 \times 10^{-7}]$ |
| | 2 | $2.24 \times 10^{-6}$ | 106 | 130.0 | **0.916** | Khoảng 95%: $[0, 6.26 \times 10^{-6}]$ |
| **rear_ended** | 0 | 0.0 | 0 | 128.1 | 0.0 | Báo cận trên hợp lệ: $p \le 3/(\beta n) = \mathbf{0.025}$ |
| | 1 | 0.0 | 0 | 116.9 | 0.0 | Báo cận trên hợp lệ: $p \le 3/(\beta n) = \mathbf{0.025}$ |
| | 2 | 0.0 | 0 | 124.2 | 0.0 | Báo cận trên hợp lệ: $p \le 3/(\beta n) = \mathbf{0.025}$ |
| **Tổng toàn bộ** | 0 | **$4.04 \times 10^{-4}$** | - | - | - | Khoảng 95%: $[1.01 \times 10^{-5}, 2.08 \times 10^{-3}]$ |
| | 1 | **$3.36 \times 10^{-4}$** | - | - | - | Khoảng 95%: $[1.04 \times 10^{-4}, 1.85 \times 10^{-3}]$ |
| | 2 | **$1.27 \times 10^{-3}$** | - | - | - | Khoảng 95%: $[0, 4.35 \times 10^{-3}]$ |

### Nhận xét & Đánh giá:
1. **Ba seed có đồng ý với nhau không? Có đồng ý với Monte Carlo không?**
   - **Đồng ý giữa 3 seed:** Cả 3 seed đều cho tổng xác suất va chạm nằm trong cùng bậc độ lớn $10^{-4} - 10^{-3}$ (Seed 0: $4.04 \times 10^{-4}$, Seed 1: $3.36 \times 10^{-4}$, Seed 2: $1.27 \times 10^{-3}$). Các khoảng tin cậy của 3 seed hoàn toàn giao thoa mạnh mẽ với nhau.
   - **Đồng ý với Monte Carlo:** Hoàn toàn đồng ý! Monte Carlo ở bước C1 cho $\hat{p} = 1.00 \times 10^{-3}$ với khoảng Wilson 95% $[3.40 \times 10^{-4}, 2.94 \times 10^{-3}]$. Tất cả các ước lượng từ 3 seed của Importance Sampling ($0.40 \times 10^{-3}$, $0.34 \times 10^{-3}$, $1.27 \times 10^{-3}$) đều nằm gọn bên trong khoảng tin cậy 95% của Monte Carlo!
2. **Ở đâu `top_share` lớn (> 0.5) và ý nghĩa thống kê:**
   - `top_share > 0.5` xuất hiện ở:
     - `crosswalk_block`: Seed 0 (0.518), Seed 1 (0.608), Seed 2 (0.916).
     - `bus_bypass`: Seed 2 (0.702).
   - **Giải thích:** Khi `top_share > 0.5` (thậm chí 0.916 ở Seed 2), hơn một nửa đến 91% tổng trọng số $\sum w_i y_i$ bị chi phối bởi một mẫu duy nhất mang tỷ số khả dĩ $w_k = \frac{p(x_k)}{q(x_k)}$ cực lớn (mẫu nằm xa ở đuôi của phân phối đề xuất nhưng lại va chạm). Khi đó, con số $\hat{p}$ thực chất chỉ phản ánh trọng số của đúng một mẫu may mắn đó chia cho $n$, chứ không phải là giá trị trung bình hội tụ ổn định của một tập hợp lớn. Đây là hiện tượng **suy biến trọng số (weight degeneracy)** kinh điển trong Importance Sampling.
3. **So sánh ngân sách (`budget`) với Monte Carlo:**
   - Ngân sách thực tế trong `estimate.json`: `budget = 16,800` lần mô phỏng ($4\text{ modes} \times (8\times 300 + 3\times 600) = 16,800$).
   - Trong khi đó, để Monte Carlo đo lường được các kiểu lỗi cực hiếm như `crosswalk_block` ($p \sim 10^{-7} - 10^{-6}$) với độ chính xác tương đối $\pm 20\%$, Monte Carlo cần $N \approx \frac{96}{10^{-6}} \approx \mathbf{960,000,000}$ lần chạy! Importance Sampling giúp giảm ngân sách tính toán hàng chục nghìn lần!
4. **Phần `other` trong tổng lấy từ đâu, và vì sao không săn nó?**
   - Trích từ `estimate.json: other`: $k = 0, n = 3000, p = 0.0, \text{hi} = 0.00127889566$, `source = "mc.json"`. Phần này lấy từ lần chạy Monte Carlo ở bước C1.
   - **Vì sao không săn nó:** "Other" không phải là một kiểu lỗi cụ thể với cấu trúc tham số hay hàm mục tiêu xác định (không có hàm `danger(R, "other")`), mà là phần bù (residual partition) của tất cả các vụ va chạm không thuộc 4 failure modes đã định nghĩa. Vì không có mục tiêu đặc thù, ta không thể hướng phân phối đề xuất của CEM tới nó mà bắt buộc phải dựa vào tần suất tự nhiên từ Monte Carlo để đảm bảo tính đầy đủ của phân hoạch không gian sự cố.

---

## 5. Các kiểu thất bại (bước C2 + 3D)

### Mô tả từng kiểu thất bại tìm thấy và chuỗi sự kiện:

1. **Kiểu 1: `bus_bypass` (tìm thấy 62 vụ trong cuộc săn `bus_bypass`):**
   - *Chuỗi sự kiện:* Xe ego di chuyển trên làn E1 với tốc độ $\approx 12.5\text{ m/s}$. Tại trạm dừng, chiếc xe buýt lớn đỗ sát mép đường che khuất hoàn toàn tầm nhìn góc trước bên phải của cảm biến ego. Đúng lúc ego tiến đến gần đầu xe buýt, một người đi bộ bước nhanh ra từ điểm mù phía trước xe buýt (`pedLook = 0`, không ngó nhìn). Cảm biến ego phát hiện chướng ngại vật quá muộn khi khoảng cách chỉ còn vài mét, khoảng cách dừng không đủ khiến xe đâm trực diện vào người đi bộ.
   - *Hình ảnh minh chứng 3D (Chương 9):*  
     ![Bus Bypass Collision](file:///e:/Antigravity/ai20k_lab/Track4-Day11-NguyenVanUoc-2A202602445Scenario-Forge-Student/evidence/shots/ch9-lab-bus-bypass.jpg)

2. **Kiểu 2: `crosswalk_block` (tìm thấy 43 vụ trong cuộc săn `crosswalk_block`):**
   - *Chuỗi sự kiện:* Xe ego tiếp cận vạch sang đường ngã tư. Một người đi bộ đang đi chậm trên vạch, trong khi điều kiện giao thông và tín hiệu khoảng trống khiến ego bị ép vào tình thế khó xử giữa việc vượt qua hay nhường đường; ego không kịp dừng trước vạch cản và đâm/chắn ngang người đi bộ ngay trên vạch qua đường.
   - *Hình ảnh minh chứng 3D (Chương 9):*  
     ![Crosswalk Block Collision](file:///e:/Antigravity/ai20k_lab/Track4-Day11-NguyenVanUoc-2A202602445Scenario-Forge-Student/evidence/shots/ch9-lab-crosswalk-block.jpg)

### Kiểu cuộc săn KHÔNG tìm thấy:
- `cut_in` và `rear_ended` không tìm thấy va chạm trực tiếp thuộc đúng nhãn của chúng:
  - Cuộc săn `cut_in` tìm thấy 14 vụ `bus_bypass`.
  - Cuộc săn `rear_ended` tìm thấy 1 vụ `bus_bypass` và 5 vụ `crosswalk_block` (và 19 kịch bản bị loại do `infeasible`).
- **Mức độ tin cậy:** Ta tin điều này ở mức rất cao. Lý do: Mô hình IDM car-following của xe phía sau và logic chuyển làn của xe cắt đầu có cơ chế chấp nhận khoảng hở an toàn (`gap acceptance`). Trong giới hạn gia tốc khả thi của vật lý ($\le \mu g$), xe cắt đầu và xe bám đuôi luôn có đủ điều kiện phanh để tránh va chạm trực tiếp với ego, trừ phi ego bị chặn đứng đột ngột bởi người đi bộ phía trước xe buýt.

### Điều gì xảy ra nếu chỉ chạy MỘT cuộc săn chung cho mọi kiểu?
- Hiện tượng **Mode Collapse (suy biến kiểu / sụp đổ phân phối)**!
- *Gợi ý từ `hunt.json: found`:* Ngay cả khi săn nhắm riêng vào `cut_in` và `rear_ended`, CEM vẫn bị trôi dạt sang tìm thấy `bus_bypass` và `crosswalk_block`.
- Nếu dùng một hàm mục tiêu chung (ví dụ: tổng số va chạm hoặc TTC nhỏ nhất chung), thuật toán CEM sẽ nhanh chóng dồn toàn bộ mật độ xác suất vào hố hấp dẫn sâu nhất — tức kiểu thất bại dễ xảy ra nhất là `bus_bypass` (vốn có tới 62 vụ). Toàn bộ các kiểu thất bại hiếm khác như `crosswalk_block` sẽ hoàn toàn bị "bỏ rơi" và biến mất khỏi phân phối đề xuất. Do đó, việc phân tách và săn riêng từng mode bằng hàm nguy hiểm chuyên biệt `danger(R, mode)` là bắt buộc.

---

## 6. Sửa planner (bước E + 3D)

### Bảng đối chiếu trích xuất từ `evidence/fix.json`:

| Đại lượng / Kịch bản | Chính sách gốc (Baseline) | Chính sách được chọn (`chosen`) | Ghi chú so sánh |
| :--- | :---: | :---: | :--- |
| **Cấu hình Policy** | Mặc định (`cutinLat: 2.15`) | **`{'cutinLat': 2.6}`** | Mở rộng hành lang nhận diện cắt đầu sang 2.6 m |
| **TRAIN: fail** | $5.15 \times 10^{-5}$ | **$1.98 \times 10^{-7}$** | Giảm rủi ro trên 250 lần |
| **TRAIN: fail_hi** | $9.07 \times 10^{-5}$ | **$3.78 \times 10^{-7}$** | Cận trên TRAIN cực kỳ an toàn |
| **TRAIN: progress** | 218.2 m | **218.2 m** | Giữ vững 100% hiệu năng di chuyển |
| **TRAIN: harsh** | 7.92 lần | **7.62 lần** | Giảm số lần phanh gấp |
| **TEST: fail** | $6.84 \times 10^{-4}$ | **$1.82 \times 10^{-7}$** | Giảm rủi ro trên TEST gần 3,700 lần! |
| **TEST: fail_hi_seen** | $1.41 \times 10^{-3}$ | **$3.76 \times 10^{-7}$** | Cận trên của các mode thực sự quan sát được |
| **TEST: fail_hi (valid)** | $2.69 \times 10^{-3}$ | **$1.28 \times 10^{-3}$** | Cận trên 95% hợp lệ toàn diện |
| **TEST: progress** | 217.3 m | **217.3 m** | Hiệu năng di chuyển tương đương tuyệt đối |
| **TEST: harsh** | 8.06 lần | **7.73 lần** | Lái êm hơn, ít phanh gấp hơn |

### Vì sao `fail_hi` của bản sửa có thể lớn hơn `fail` của policy gốc dù bản sửa tốt hơn thật?
- Trong quy chuẩn thống kê an toàn của bước TEST, `fail_hi` là **cận trên tin cậy 95% hợp lệ toàn diện**.
- Khi chính sách mới khắc phục triệt để một kiểu thất bại khiến số vụ va chạm của kiểu đó trong mẫu TEST giảm về 0 (`hits = 0`), ta không được phép coi xác suất đó bằng 0. Theo nguyên lý đánh giá phòng thủ (defensive importance sampling bound), mỗi kiểu chưa quan sát thấy (`unseen`) phải cộng thêm cận trên bảo thủ $3 / (\beta n)$ (với $\beta = 0.2, n = 400 \implies 3 / 80 = 0.0375$).
- Tương tự, thành phần `other_crashes` trên tập holdout tự nhiên ($n = 3000$) dù có $k=0$ vẫn mang cận trên Wilson là $1.28 \times 10^{-3}$. Cận trên này được cộng dồn vào `fail_hi` của bản sửa.
- Trong khi đó, `fail` của chính sách gốc là **ước lượng điểm thực nghiệm (empirical point estimate)** chỉ tính trên số mẫu va chạm quan sát được. Do đó, `fail_hi` (mang tính chứng nhận an toàn bảo thủ) có thể lớn hơn ước lượng điểm `fail` của bản gốc. Ngược lại, khi so sánh chuẩn mực trên `fail_hi_seen` (những gì dữ liệu thực sự quan sát), bản sửa đạt $3.76 \times 10^{-7}$, vượt trội hoàn toàn so với mức $1.41 \times 10^{-3}$ của baseline!

### Núm chỉnh "nghe thì an toàn hơn" nhưng thực tế làm tăng thất bại:
- **Núm chỉnh:** Tăng thời gian bám đuôi an toàn $T$ lên $2.0\text{ s}$ (`{'T': 2.0}`) hoặc giảm vận tốc thận trọng `cautionV` xuống quá thấp ($5.0\text{ m/s}$).
- **Giải thích bằng cách thế giới phản ứng lại:**
  - Nhìn vào bảng TRAIN trong `fix.json`: Candidate `{'T': 2.0}` có `TRAIN fail = 7.02e-04` (cao hơn gấp 13 lần so với baseline $5.15 \times 10^{-5}$!).
  - *Cơ chế phản ứng xã hội của các phương tiện khác:* Khi ego tăng $T$ lên 2.0 s hoặc bò quá chậm (`cautionV = 5.0`), khoảng cách đệm phía trước xe ego bị kéo giãn ra rất lớn. Xe ở làn bên cạnh coi đây là một khoảng trống thênh thang (`large gap`) để lập tức tạt đầu chuyển làn (`cut_in`). 
  - Khi xe khác cắt đầu ở cự ly gần trong khi ego di chuyển chậm chạp và thiếu linh hoạt, khoảng cách phanh bị bóp nghẹt đột ngột, dẫn đến nguy cơ va chạm cắt đầu và va chạm dây chuyền từ phía sau tăng vọt.

---

## 7. Từ mô phỏng ra đường thật

Phân phối tự nhiên trong `space.py` là các giả định giảng dạy. Ba giả định ảnh hưởng mạnh nhất đến ước lượng:

1. **Hành vi quan sát của người đi bộ tại điểm mù xe buýt (`pedLook`):**
   - *Trong `space.py`:* Được mô hình hóa bằng biến nhị phân Bernoulli với xác suất cố định (trong Town D, người đi bộ có xác suất nhìn quanh cố định). Khi `pedLook = 0`, người đi bộ bước thẳng ra đường từ đầu xe buýt với vận tốc không đổi, hoàn toàn không giảm tốc hay ngoái nhìn.
   - *Dữ liệu thực tế thay thế:* Tập dữ liệu hành vi người đi bộ tự nhiên đô thị quy mô lớn như **PIE (Pedestrian Intent Estimation)** hoặc **JAAD (Joint Attention in Autonomous Driving)**, trích xuất phân bố thời gian dừng lại quan sát (head pose, gaze angle, hesitation time).
   - *Hướng sai lệch:* Ngoài đời thực, đa số người đi bộ có bản năng sinh tồn tự nhiên, họ sẽ thò đầu ngó hoặc đi chậm lại khi bước qua một vật cản khổng lồ như xe buýt. Giả định trong mô phỏng khiến tỷ lệ va chạm `bus_bypass` bị **thổi phồng cao hơn thực tế (conservative overestimation)**.
2. **Khe chấp nhận và thời gian chuyển làn của xe cắt đầu (`cutGap`, `cutDur`, `cutDs`):**
   - *Trong `space.py`:* Sử dụng phân phối chuẩn tắc độc lập $N(0, 1)$ biến đổi qua hàm tuyến tính để sinh khoảng cách khe hở và thời gian lách làn.
   - *Dữ liệu thực tế thay thế:* Tập dữ liệu quỹ đạo xe tự nhiên cao tốc và đô thị như **highD dataset** hoặc **Waymo Open Dataset / Argoverse 2 / NGSIM**, trích xuất ma trận hiệp phương sai thực tế giữa vận tốc tương đối và khoảng cách chuyển làn an toàn (time headway acceptance).
   - *Hướng sai lệch:* Nếu mô phỏng giả định các pha cắt đầu xảy ra ở khoảng cách quá ngắn so với hành vi của tài xế văn minh, xác suất va chạm ước tính sẽ bị thiên lệch cao hơn thực tế.
3. **Phân phối hệ số ma sát mặt đường ($\mu$):**
   - *Trong `space.py`:* Hệ số ma sát được gán giá trị rời rạc $\mu \in \{0.5, 0.8\}$ với xác suất 50/50.
   - *Dữ liệu thực tế thay thế:* Thống kê khí tượng thủy văn địa phương (tỷ lệ ngày mưa/khô trong năm) kết hợp dữ liệu đo trượt lốp thực tế từ cảm biến hệ thống cân bằng điện tử ESC/ABS của đội xe thương mại tại thành phố vận hành.
   - *Hướng sai lệch:* Tỷ lệ đường trơn $\mu = 0.5$ lên tới 50% là quá cao so với điều kiện đường sá đô thị thông thường (thực tế đường khô ráo $\mu \approx 0.85$ chiếm $> 85\%$ thời gian). Điều này khiến quãng đường phanh yêu cầu bị kéo dài giả tạo, làm xác suất tai nạn trong mô phỏng bị **ước tính quá bi quan**.

---

## 8. Giới hạn

### Ba điều mà xác suất báo cáo KHÔNG nói (giới hạn của mô hình):
1. **Hình học không gian đường phố quá tối giản:** Mô phỏng chỉ có một đoạn phố thẳng đơn trục dài từ $x = -220\text{ m}$ đến $x = 122\text{ m}$ với một trạm xe buýt và một vạch qua đường. Xác suất này hoàn toàn không phản ánh rủi ro tại các nút giao phức tạp, vòng xuyến, ngã ba không đèn tín hiệu, đường dốc quanh co hay khu vực công trường thi công.
2. **Hành vi tác nhân xung quanh (NPCs) bị cơ giới hóa:** Các xe khác chỉ tuân theo mô hình bám đuôi IDM thuần túy và quỹ đạo lách làn định trước; người đi bộ chỉ chuyển động tịnh tiến đều. Thiếu hoàn toàn các tương tác xã hội phi ngôn ngữ (nháy đèn, bấm còi, vẫy tay nhường đường, hoặc người đi bộ giật mình lùi lại khi thấy ô tô lao tới).
3. **Mô hình cảm biến và nhận thức (Perception) lý tưởng hóa:** Cảm biến cung cấp ground-truth bounding box hoàn hảo không có độ trễ (latency), không có nhiễu đo lường, không có hiện tượng mất dấu mục tiêu (occlusion tracking loss) hay phát hiện nhầm (false positive/negative), ngoại trừ góc khuất 2D hình học của xe buýt.

### Nếu được thêm một tuần, bạn sẽ đo gì tiếp?
- **Đo lường tác động của độ trễ nhận thức (Perception Latency) và sai số ước lượng vận tốc:** Thay thế ground-truth state bằng một pipeline mô phỏng nhận thức có độ trễ 100–200 ms và nhiễu bounding box jitter để đo lường độ nhạy cảm của planner đối với chất lượng cảm biến.
- **Mở rộng sang kịch bản đa tác nhân hỗn hợp không bảo thủ:** Mô phỏng các tình huống có xe máy, xe đạp điện luồn lách từ phía sau xe buýt tại các góc cua đô thị đông đúc.

---

## 9. Rắc rối của bạn (bước G + chương 10)

### 1. Rắc rối bạn dựng và tình huống ngoài đời:
- **Tình huống ngoài đời:** Trên đường phố hỗn hợp đô thị (như tại Việt Nam), một người đi xe đạp di chuyển ở làn mép phải (làn E2), bất ngờ đánh lái lách sang trái một khoảng rộng (2.8 – 3.6 m) để tránh chướng ngại vật sát lề (ổ gà, nắp cống hở, hoặc cửa xe ô tô đỗ bên đường đột ngột mở ra) và nhập thẳng vào trước đầu xe ô tô tự hành trên làn E1 ngay trước nút giao.
- **Cấu hình trong `hazard.json`:**
  - Loại: `bike`, xuất phát làn `E2`, $v = 4.0\text{ m/s}$, lách `swerve = 3.4 m` trong thời gian `swDur = 2.5 s`.
  - Vị trí kích hoạt: Khi xe ego cách xe đạp khoảng cách `trig = 16.0 m`.

### 2. Căn cứ vì sao planner xử lý kém:
- Xe đạp bắt đầu lách khi khoảng cách giữa ego và xe đạp chỉ còn khoảng 12 – 20 m (tương ứng với time headway chỉ $\approx 1.0 - 1.5\text{ s}$ ở vận tốc thông thường).
- Mô hình IDM của planner chỉ kích hoạt phanh dọc khi vật cản đã nằm trọn vẹn trong hành lang làn đường của nó. Khi xe đạp đang ở làn E2 và bắt đầu lượn chéo sang, planner coi xe đạp là vật thể ở làn bên cạnh và không phanh trước. Đến khi xe đạp cắt qua vạch làn E1 ở cự ly quá gần, gia tốc phanh yêu cầu vượt quá giới hạn phanh êm ái $b = 2.0\text{ m/s}^2$ của IDM, khiến planner không kịp dừng xe dù về mặt vật lý nếu phanh khẩn cấp hết lực ngay khi nhìn thấy (`oracle avoid`) thì hoàn toàn tránh được!

### 3. Giả định `space` và `exposure` (`hazard.json: why`):
- `space`:
  - `h0_x`: `[-5.0, 25.0]` m (đoạn đường tiếp cận trước vạch ngã tư).
  - `h0_swerve`: `[2.8, 3.6]` m (khoảng cách lách tương đương bề rộng một làn xe đô thị tiêu chuẩn 3.0 – 3.5 m).
  - `h0_trig`: `[12.0, 20.0]` m (khoảng cách phản xạ của người đi xe đạp khi gặp chướng ngại vật bất ngờ).
- `exposure = 0.04`: Tần suất trung bình xuất hiện tình huống này được ước tính là 4% (tương đương 1 lần mỗi 25 chuyến đi) trong môi trường giao thông đô thị hỗn hợp.
- Tính khả thi vật lý: Gia tốc ngang cực đại của xe đạp khi lách:
  $$a_{\text{lat}} = \frac{\pi^2 \cdot \text{swerve}}{2 \cdot (\text{swDur})^2} = \frac{\pi^2 \cdot 3.4}{2 \cdot 2.5^2} \approx 2.68\text{ m/s}^2 \le 3.0\text{ m/s}^2 \le \mu g$$
  Hoàn toàn khả thi về mặt động học người lái xe đạp!

### 4. Kết quả trích xuất từ `evidence/mine.json`:

| Chỉ số đánh giá | Chính sách gốc (Baseline) | Chính sách chọn (`chosen: cutinLat = 2.6`) |
| :--- | :---: | :---: |
| Tỉ lệ kịch bản khả thi (`accepted`) | **99.67%** (1495/1500) | **99.67%** (1495/1500) |
| Tổng số va chạm với xe ego | 789 | 441 |
| Va chạm tránh được bằng phanh hết cỡ (`avoidable`) | **789** | **438** |
| Va chạm không thể tránh (`unavoidable`) | **0** | **0** |
| Va chạm không do rắc rối (`unattributed`) | **0** | **3** |
| Va chạm ở tốc độ thấp (`low_speed` < 1 m/s) | **0** | **2** |
| $P(\text{lỗi planner} \mid \text{rắc rối})$ | **$5.28 \times 10^{-1}$** (52.8%) | **$2.93 \times 10^{-1}$** (29.3%) |
| Khoảng tin cậy Wilson 95% | **$[0.5024, 0.5530]$** | **$[0.2705, 0.3166]$** |
| Tần suất lỗi mỗi chuyến ($\times \text{exposure } 0.04$) | **$2.11 \times 10^{-2}$** (cận trên $2.21 \times 10^{-2}$) | **$1.17 \times 10^{-2}$** (cận trên $1.27 \times 10^{-2}$) |

- **So sánh với tần suất lách xe buýt:**
  - Tần suất tai nạn do lách xe buýt trong phân phối tự nhiên (`estimate.json`) là $\sim 3.35 \times 10^{-4}$ đến $1.27 \times 10^{-3}$ mỗi chuyến.
  - Tần suất lỗi do xe đạp lạng lách bất ngờ gây ra là **$1.17 \times 10^{-2}$** mỗi chuyến — **cao gấp hơn 10 đến 30 lần** so với rủi ro lách xe buýt! Điều này chứng minh đây là một lỗ hổng an toàn nghiêm trọng ngoài đời thực mà planner hiện tại chưa được trang bị đầy đủ để xử lý.

### 5. Hình ảnh minh chứng (Chương 10):
![Hazard Studio Bike Swerve](file:///e:/Antigravity/ai20k_lab/Track4-Day11-NguyenVanUoc-2A202602445Scenario-Forge-Student/evidence/shots/ch10-studio.jpg)

### 6. Nếu bạn tự lái thì sao:
- Khi người lái tự điều khiển trong chương 10, người lái **hoàn toàn tránh được va chạm**.
- *Khác biệt của con người so với planner:* Người lái có khả năng phán đoán ý định (intent prediction) và nhận thức ngữ cảnh. Khi nhìn thấy xe đạp đi gần chướng ngại vật hoặc có xu hướng nghiêng xe lạng ra làn ngoài, tài xế người sẽ lập tức nhả chân ga, rà nhẹ chân phanh trước (defensive braking) và hơi đánh lái chếch sang trái trong làn để nới rộng khoảng cách an toàn, thay vì tiếp tục giữ ga lao tới và chờ xe đạp cắt ngang tim đường mới phanh gấp như planner.

### 7. Cách sửa:
- **Núm chỉnh có sẵn:** Việc tăng `cutinLat` lên `2.6 m` ở chính sách đã chọn giúp mở rộng hành lang phát hiện sớm vật thể lấn làn, giúp giảm tỷ lệ lỗi từ 52.8% xuống 29.3% (giảm gần một nửa).
- **Logic planner cần bổ sung thêm (mô tả kiến trúc):**
  - **Module dự đoán quỹ đạo ngang (Lateral Trajectory & Intention Forecasting):** Thay vì chỉ kiểm tra toạ độ vị trí tức thời bên trong hành lang làn (static lane containment), planner cần theo dõi vận tốc ngang ($v_y$) và gia tốc ngang ($a_y$) của các phương tiện thô sơ ở làn kế cận.
  - **Cơ chế TTC hai chiều (Time-to-Lane-Crossing):** Khi phát hiện xe đạp có $v_y > 0.5\text{ m/s}$ hướng về làn xe ego, planner phải tính toán thời điểm cắt qua vạch làn ($t_{\text{cross}} = \Delta y / v_y$) và kích hoạt giảm tốc chủ động trước khi vật thể thực sự bước vào làn đường.
