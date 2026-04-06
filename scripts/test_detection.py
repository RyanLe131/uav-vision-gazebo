#!/usr/bin/env python3
"""
test_detection.py

Mục đích: kiểm tra YOLOv8s có detect được person_1 trong video orbit không.

- Đọc video orbit mới nhất từ ~/orbit_videos/  (hoặc chỉ định --video)
- Chạy YOLOv8s inference trên TOÀN BỘ video
- Xuất video annotated với bounding box vào ~/orbit_videos/detection_test/
- In summary: bao nhiêu % frame detect được person, confidence trung bình

Usage:
  conda activate drone_cv
  python test_detection.py
  python test_detection.py --video ~/orbit_videos/orbit_9m_20260404_213046.mp4
"""

import argparse
import os
import glob
import cv2
from ultralytics import YOLO

# ── Config ────────────────────────────────────────────────────────────────────
VIDEO_DIR   = os.path.expanduser('~/orbit_videos')
OUTPUT_DIR  = os.path.join(VIDEO_DIR, 'detection_test')
MODEL_NAME  = 'yolov8s.pt'
CONF_THRESH = 0.25      # ngưỡng confidence tối thiểu
PERSON_CLS  = 0         # class index của 'person' trong COCO


def pick_latest_video(video_dir: str) -> str:
    """Tìm video mp4 mới nhất trong thư mục (không lấy trong thư mục con)."""
    videos = glob.glob(os.path.join(video_dir, '*.mp4'))
    if not videos:
        raise FileNotFoundError(f'Không tìm thấy video trong {video_dir}')
    return max(videos, key=os.path.getmtime)


def annotate_frame(frame, detections: list, frame_idx: int) -> tuple:
    """Vẽ bounding box lên frame. Trả về (annotated_frame, label_color)."""
    img = frame.copy()
    if detections:
        for d in detections:
            x1, y1, x2, y2 = d['bbox']
            conf = d['conf']
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(img,
                f"person {conf:.2f}",
                (x1, max(y1 - 8, 0)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        label = f'Frame {frame_idx}  DETECTED ({len(detections)})'
        color = (0, 255, 0)
    else:
        label = f'Frame {frame_idx}  --'
        color = (200, 200, 200)

    cv2.putText(img, label, (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
    return img, color


def process_video(video_path: str, model: YOLO, output_dir: str) -> list:
    """
    Đọc toàn bộ video, chạy inference mỗi frame,
    xuất video annotated, trả về list kết quả mỗi frame.
    """
    os.makedirs(output_dir, exist_ok=True)

    cap   = cv2.VideoCapture(video_path)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps   = cap.get(cv2.CAP_PROP_FPS)
    w     = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h     = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f'  Video  : {os.path.basename(video_path)}')
    print(f'  Frames : {total}  |  FPS: {fps:.1f}  |  '
          f'Duration: {total/fps:.1f}s  |  Size: {w}x{h}')

    # ── Output video path ─────────────────────────────────────────────────────
    base     = os.path.splitext(os.path.basename(video_path))[0]
    out_path = os.path.join(output_dir, f'{base}_detected.mp4')
    fourcc   = cv2.VideoWriter_fourcc(*'mp4v')
    writer   = cv2.VideoWriter(out_path, fourcc, fps, (w, h))

    results_list = []
    frame_idx    = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Inference
        results    = model(frame, classes=[PERSON_CLS],
                           conf=CONF_THRESH, verbose=False)
        boxes      = results[0].boxes
        detections = []
        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = float(box.conf[0])
                detections.append({
                    'bbox': (int(x1), int(y1), int(x2), int(y2)),
                    'conf': conf,
                })

        annotated, _ = annotate_frame(frame, detections, frame_idx)
        writer.write(annotated)

        results_list.append({
            'frame_idx' : frame_idx,
            'detections': detections,
        })

        frame_idx += 1
        if frame_idx % 50 == 0:
            pct = frame_idx / total * 100 if total > 0 else 0
            print(f'  ... {frame_idx}/{total} ({pct:.0f}%)', end='\r')

    cap.release()
    writer.release()
    print()
    print(f'  Video annotated : {out_path}')
    return results_list


def print_summary(results_list: list, video_path: str):
    """In summary kết quả detection."""
    total      = len(results_list)
    detected   = [r for r in results_list if r['detections']]
    n_detected = len(detected)

    all_confs = [d['conf']
                 for r in detected
                 for d in r['detections']]
    avg_conf = sum(all_confs) / len(all_confs) if all_confs else 0.0

    print()
    print('=' * 55)
    print('DETECTION SUMMARY')
    print('=' * 55)
    print(f'  Model          : {MODEL_NAME}')
    print(f'  Conf threshold : {CONF_THRESH}')
    print(f'  Total frames   : {total}')
    print(f'  Person detected: {n_detected}/{total} '
          f'({100*n_detected/total:.1f}%)')
    if all_confs:
        print(f'  Confidence     : avg={avg_conf:.3f}  '
              f'min={min(all_confs):.3f}  max={max(all_confs):.3f}')
    else:
        print('  Confidence     : N/A (không detect được frame nào)')
    print('=' * 55)
    print()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--video', default=None,
                        help='Path đến video (mặc định: video mới nhất)')
    args = parser.parse_args()

    video_path = args.video or pick_latest_video(VIDEO_DIR)

    print()
    print('=' * 55)
    print('TEST DETECTION — YOLOv8s on orbit video')
    print('=' * 55)

    # ── Load model ────────────────────────────────────────────────────────────
    print(f'\n[1/3] Load model {MODEL_NAME} ...')
    model = YOLO(MODEL_NAME)
    print('  OK')

    # ── Process full video ────────────────────────────────────────────────────
    print(f'\n[2/3] Process video ...')
    results_list = process_video(video_path, model, OUTPUT_DIR)

    # ── Summary ───────────────────────────────────────────────────────────────
    print(f'\n[3/3] Summary ...')
    print_summary(results_list, video_path)


if __name__ == '__main__':
    main()
