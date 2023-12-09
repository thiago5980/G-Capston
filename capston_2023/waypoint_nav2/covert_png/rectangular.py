import cv2
import numpy as np

# 마우스 콜백 함수와 좌표를 저장할 리스트
coordinates = []

def get_coordinates(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        coordinates.append((x, y))
        print(f"클릭한 좌표: x = {x}, y = {y}")

# 이미지 읽기
image_path = "final_outside_smoke.png"  # 이미지 경로를 적절히 수정하세요.
image = cv2.imread(image_path)

# 윈도우 생성 및 마우스 콜백 함수 설정
cv2.namedWindow('image')
cv2.setMouseCallback('image', get_coordinates)

# 사용자가 지정한 RGB 색상
color = (205, 205, 205)  # Green. 색상을 원하는대로 변경하세요.

while True:
    # 이미지 출력
    cv2.imshow('image', image)
    
    # 4개의 점이 선택되면 다각형 그리기
    if len(coordinates) == 4:
        # NumPy 배열로 변환
        pts = np.array(coordinates, np.int32)
        
        # 다각형 경계 그리기
        cv2.polylines(image, [pts], isClosed=True, color=color, thickness=2)
        
        # 다각형 내부 채우기
        cv2.fillPoly(image, [pts], color=color)
        
        # 좌표 초기화
        coordinates.clear()

    # 'q' 키를 누르면 루프 종료 및 이미지 저장
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.imwrite('final_outside_smoke.png', image)  # 최종 이미지 저장
        print("이미지가 'outside_final.png'로 저장되었습니다.")
        break

cv2.destroyAllWindows()
