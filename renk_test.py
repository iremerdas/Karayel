import cv2
import numpy as np
from collections import deque  # tespit edilen objenin merkezini depolamak için kullanılır
import colorDetection as color

# nesne merkezini depolayacak veri tipi
buffer_size = 16  # deque nun boyutu
pts = deque(maxlen=buffer_size)  # nesnenin merkez noktaları

# renk aralıkları HSV formatında
lower_red1 = np.array([1, 170, 100])
upper_red1 = np.array([5, 255, 255])
lower_red2 = np.array([170, 170, 100])
upper_red2 = np.array([179, 255, 255])
lower_yellow = np.array([25, 100, 100])
upper_yellow = np.array([30, 255, 255])
lower_green = np.array([40, 100, 100])
upper_green = np.array([70, 255, 255])

# capture
cap = cv2.VideoCapture(0)
cap.set(3, 960)
cap.set(4, 480)
cap.set(10, 100)


def draw_contour(frame, rect, color, label):
    ((x, y), (width, height), rotation) = rect
    box = cv2.boxPoints(rect)
    box = np.int64(box)
    cv2.drawContours(frame, [box], 0, color, 2)
    cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 0), -1)
    cv2.putText(frame, label, (int(x) - 50, int(y) - 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 255), 2)


def merge_contours(contours, threshold):
    merged_contours = []
    merged_flags = [False] * len(contours)
    for i, cnt1 in enumerate(contours):
        if merged_flags[i]:
            continue
        new_contour_group = [cnt1]
        merged_flags[i] = True
        for j, cnt2 in enumerate(contours):
            if i == j or merged_flags[j]:
                continue
            M1 = cv2.moments(cnt1)
            M2 = cv2.moments(cnt2)
            if M1["m00"] != 0 and M2["m00"] != 0:
                cX1 = int(M1["m10"] / M1["m00"])
                cY1 = int(M1["m01"] / M1["m00"])
                cX2 = int(M2["m10"] / M2["m00"])
                cY2 = int(M2["m01"] / M2["m00"])
                dist = np.sqrt((cX1 - cX2) ** 2 + (cY1 - cY2) ** 2)
                if dist < threshold:
                    new_contour_group.append(cnt2)
                    merged_flags[j] = True
        if len(new_contour_group) > 1:
            combined_contour = np.vstack(new_contour_group)
            hull = cv2.convexHull(combined_contour)
            merged_contours.append(hull)
        else:
            merged_contours.append(cnt1)
    return merged_contours


def find_largest_contour(contours):
    if contours:
        return max(contours, key=cv2.contourArea)
    return None


def main():
    if cap.isOpened():
        try:
            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    print("Frame okunmuyor")
                    break
                detected_objects = []

                contours = color.color_detect(frame)

                for contour in contours:
                    contours_red, contours_yellow, contours_green = contour
                    merge_red = merge_contours(contours_red, 500)
                    for c in merge_red:
                        rect = cv2.minAreaRect(c)
                        ((x, y), (width, height), rotation) = rect
                        detected_objects.append(("Red", x, y, width, height, rotation))
                        draw_contour(frame, rect, (0, 0, 255), "Red")

                    merge_yellow = merge_contours(contours_yellow, 500)
                    for c in merge_yellow:
                        rect = cv2.minAreaRect(c)
                        ((x, y), (width, height), rotation) = rect
                        detected_objects.append(("Yellow", x, y, width, height, rotation))
                        draw_contour(frame, rect, (0, 255, 255), "Yellow")

                    merge_green = merge_contours(contours_green, 500)
                    for c in merge_green:
                        rect = cv2.minAreaRect(c)
                        ((x, y), (width, height), rotation) = rect
                        detected_objects.append(("Green", x, y, width, height, rotation))
                        draw_contour(frame, rect, (0, 255, 0), "Green")

                    largest_red = find_largest_contour(merge_red)
                    largest_yellow = find_largest_contour(merge_yellow)
                    largest_green = find_largest_contour(merge_green)

                    largest_contour = None
                    largest_area = 0
                    largest_color = None

                    if largest_red is not None and cv2.contourArea(largest_red) > largest_area:
                        largest_contour = largest_red
                        largest_area = cv2.contourArea(largest_red)
                        largest_color = "Red"
                    if largest_yellow is not None and cv2.contourArea(largest_yellow) > largest_area:
                        largest_contour = largest_yellow
                        largest_area = cv2.contourArea(largest_yellow)
                        largest_color = "Yellow"
                    if largest_green is not None and cv2.contourArea(largest_green) > largest_area:
                        largest_contour = largest_green
                        largest_area = cv2.contourArea(largest_green)
                        largest_color = "Green"

                    detected_objects.clear()

                    if largest_contour is not None:
                        rect = cv2.minAreaRect(largest_contour)
                        draw_contour(frame, rect, (0, 255, 255), largest_color)
                        M = cv2.moments(largest_contour)
                        if M["m00"] != 0:
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])
                            detected_objects.append((largest_color, cX, cY, largest_area))

                    for obj in detected_objects:
                        label, x, y, area = obj
                        if label == "Red":
                            print(
                                f"{label}: Object Coordinates - x: {x}, y: {y}, area: {area}")
                            print("Sağa dön")
                        elif label == "Green":
                            print(
                                f"{label}: Object Coordinates - x: {x}, y: {y}, area: {area}")
                            print("Sola dön")
                        else:
                            print(
                                f"{label}: Object Coordinates - x: {x}, y: {y}, area: {area}")
                            if x < 320:
                                print("Sarı nesne kameranın solunda, sağa yönel")
                            elif x > 640:
                                print("Sarı nesne kameranın sağında, sola yönel")
                            else:
                                print("Sarı nesne kameranın ortasında, zikzak yönel")

                cv2.imshow('Original', frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            cap.release()
            cv2.destroyAllWindows()
    else:
        print("Error: Unable to open camera")


if __name__ == "__main__":
    main()