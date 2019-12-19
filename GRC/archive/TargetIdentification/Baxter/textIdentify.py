import numpy as np
import subprocess
import pytesseract
import cv2 as cv
from matplotlib import pyplot as plt

img = cv.imread('input7.jpg')

gf_img = cv.fastNlMeansDenoisingColored(img, None, 10, 10, 7, 21)
cv.imwrite('sampleTargetFiltered.jpg', gf_img)
pass_arg = []
pass_arg.append("./whiteboardClean.sh")
pass_arg.append("sampleTargetFiltered.jpg")
pass_arg.append("cleanedTarget.jpg")
subprocess.check_call(pass_arg)

pass_arg2 = []
pass_arg2.append("./textclean.sh")
subprocess.check_call(pass_arg2)
cln_img = cv.imread('output.jpg')
cv.imwrite('output2.jpg', cln_img)
cln_img = cv.imread('output2.jpg')
conf = ('-l eng --oem 3 --psm 11')
text = pytesseract.image_to_string(cln_img, config=conf)
print(text)
plt.subplot(121), plt.imshow(img)
plt.subplot(122), plt.imshow(cln_img)
plt.show()
