import cv2 as cv, numpy as np, json

cap = cv.VideoCapture(2)

def controller(img, brightness=255, contrast=127):
	brightness = int((brightness - 0) * (255 - (-255)) / (510 - 0) + (-255))

	contrast = int((contrast - 0) * (127 - (-127)) / (254 - 0) + (-127))

	if brightness != 0:

		if brightness > 0:

			shadow = brightness

			max = 255

		else:

			shadow = 0
			max = 255 + brightness

		al_pha = (max - shadow) / 255
		ga_mma = shadow

		# The function addWeighted
		# calculates the weighted sum
		# of two arrays
		cal = cv.addWeighted(img, al_pha,
							img, 0, ga_mma)

	else:
		cal = img

	if contrast != 0:
		Alpha = float(131 * (contrast + 127)) / (127 * (131 - contrast))
		Gamma = 127 * (1 - Alpha)

		# The function addWeighted calculates
		# the weighted sum of two arrays
		cal = cv.addWeighted(cal, Alpha,
							cal, 0, Gamma)

	# putText renders the specified
	# text string in the image.
	cv.putText(cal, 'B:{},C:{}'.format(brightness,
										contrast),
				(10, 30), cv.FONT_HERSHEY_SIMPLEX,
				1, (0, 0, 255), 2)

	return cal


while True:
    ret, frame = cap.read()
    if not ret:
        print("Empty frame")
        continue

    cv.imshow('original', frame)
    # frame_dilation = cv.dilate(frame, np.ones((5,5), np.uint8))
    # cv.imshow('dilated', frame_dilation)
    # med_blurred = cv.medianBlur(frame_dilation, 5)
    # cv.imshow('blurred', med_blurred)

    rgb_planes = cv.split(frame)

    result_planes = []
    result_norm_planes = []
    for plane in rgb_planes:
        dilated_img = cv.dilate(plane, np.ones((7, 7), np.uint8))
        bg_img = cv.medianBlur(dilated_img, 21)
        diff_img = 255 - cv.absdiff(plane, bg_img, None)
        norm_img = cv.normalize(diff_img, None, alpha=0, beta=255, norm_type=cv.NORM_MINMAX, dtype=cv.CV_8UC1)
        result_planes.append(diff_img)
        result_norm_planes.append(norm_img)

    result = cv.merge(result_planes)
    result_norm = cv.merge(result_norm_planes)

    cv.imshow('shadows_out.png', result)
    cv.imshow('shadows_out_norm.png', result_norm)


    # diff_img = 255-cv.absdiff()
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()