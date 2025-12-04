import pytesseract
import numpy as np
import cv2
from imutils.object_detection import non_max_suppression

class TextRecognition:
    def __init__(self, east_model_path, min_confidence=0.5, width=320, height=320, padding=0.0):
        self.east_model_path = east_model_path
        self.min_confidence = min_confidence
        self.width = width
        self.height = height
        self.padding = padding
        self.net = cv2.dnn.readNet(self.east_model_path)
        self.layer_names = ['feature_fusion/Conv_7/Sigmoid', 'feature_fusion/concat_3']
        pytesseract.pytesseract.tesseract_cmd = '/usr/bin/tesseract'

    def forward_passer(self, image):
        blob = cv2.dnn.blobFromImage(image, 1.0, (image.shape[1], image.shape[0]), 
                                     (123.68, 116.78, 103.94), swapRB=True, crop=False)
        self.net.setInput(blob)
        scores, geometry = self.net.forward(self.layer_names)
        return scores, geometry

    def box_extractor(self, scores, geometry):
        (num_rows, num_cols) = scores.shape[2:4]
        rectangles = []
        confidences = []

        for y in range(0, num_rows):
            scores_data = scores[0, 0, y]
            x_data_0 = geometry[0, 0, y]
            x_data_1 = geometry[0, 1, y]
            x_data_2 = geometry[0, 2, y]
            x_data_3 = geometry[0, 3, y]
            angles_data = geometry[0, 4, y]

            for x in range(0, num_cols):
                if scores_data[x] < self.min_confidence:
                    continue

                (offset_x, offset_y) = (x * 4.0, y * 4.0)
                angle = angles_data[x]
                cos = np.cos(angle)
                sin = np.sin(angle)

                h = x_data_0[x] + x_data_2[x]
                w = x_data_1[x] + x_data_3[x]

                end_x = int(offset_x + (cos * x_data_1[x]) + (sin * x_data_2[x]))
                end_y = int(offset_y - (sin * x_data_1[x]) + (cos * x_data_2[x]))
                start_x = int(end_x - w)
                start_y = int(end_y - h)

                rectangles.append((start_x, start_y, end_x, end_y))
                confidences.append(float(scores_data[x]))

        return rectangles, confidences

    def recognize_text(self, image):
        orig_image = image.copy()
        orig_h, orig_w = orig_image.shape[:2]

        # resizing image
        image = cv2.resize(image, (self.width, self.height))
        ratio_w = orig_w / float(self.width)
        ratio_h = orig_h / float(self.height)

        # forward pass
        scores, geometry = self.forward_passer(image)

        # extract boxes
        rectangles, confidences = self.box_extractor(scores, geometry)

        # applying non-max suppression to get boxes depicting text regions
        boxes = non_max_suppression(np.array(rectangles), probs=confidences)

        results = []

        # text recognition main loop
        for (start_x, start_y, end_x, end_y) in boxes:
            start_x = int(start_x * ratio_w)
            start_y = int(start_y * ratio_h)
            end_x = int(end_x * ratio_w)
            end_y = int(end_y * ratio_h)

            dx = int((end_x - start_x) * self.padding)
            dy = int((end_y - start_y) * self.padding)

            start_x = max(0, start_x - dx)
            start_y = max(0, start_y - dy)
            end_x = min(orig_w, end_x + (dx * 2))
            end_y = min(orig_h, end_y + (dy * 2))

            # ROI to be recognized
            roi = orig_image[start_y:end_y, start_x:end_x]

            # recognizing text
            config = '-l eng --oem 1 --psm 7'
            text = pytesseract.image_to_string(roi, config=config)

            # collating results
            results.append(((start_x, start_y, end_x, end_y), text))

        # sorting results top to bottom
        results.sort(key=lambda r: r[0][1])

        return results
