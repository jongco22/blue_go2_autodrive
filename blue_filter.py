import cv2
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

# ---- segmentation 함수 ----
def segment_blue_rgb(img_rgb,
                     rgb_thresh=((0, 0, 80), (120, 120, 255)),
                     hsv_thresh=((90, 50, 50), (140, 255, 255))):
    # RGB 임계
    rgb_min = np.array(rgb_thresh[0], np.uint8)
    rgb_max = np.array(rgb_thresh[1], np.uint8)
    mask_rgb = cv2.inRange(img_rgb, rgb_min, rgb_max)

    # HSV 임계
    img_hsv = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)
    hsv_min = np.array(hsv_thresh[0], np.uint8)
    hsv_max = np.array(hsv_thresh[1], np.uint8)
    mask_hsv = cv2.inRange(img_hsv, hsv_min, hsv_max)

    # 결합 + 잡음 제거
    mask = cv2.bitwise_or(mask_rgb, mask_hsv)
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    segmented = cv2.bitwise_and(img_rgb, img_rgb, mask=mask)
    return mask, segmented

# ---- 메인 실행부 ----
if __name__ == "__main__":
    # 1. 이미지 읽기 (파일 경로 변경)
    img_path = "test.jpeg"   # 사진 경로 입력
    img = np.array(Image.open(img_path).convert("RGB"))

    # 2. 세그멘테이션 실행
    mask, segmented = segment_blue_rgb(img)

    # 3. 결과 보여주기
    plt.subplot(1,2,1)
    plt.imshow(mask, cmap="gray")
    plt.title("Blue Mask")

    plt.subplot(1,2,2)
    plt.imshow(segmented)
    plt.title("Blue Segmented")

    plt.show()

    # 4. 결과 저장
    Image.fromarray(mask).save("blue_mask.png")
    Image.fromarray(segmented).save("blue_segmented.png")
    print("success: blue_mask.png, blue_segmented.png")
