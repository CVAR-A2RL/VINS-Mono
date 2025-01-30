import cv2
import numpy as np
import os

def main():
    folder = 'calibration_frames'
    output_folder = 'undistorted_images'

# Camera calibration matrix: 
# [[578.78812127   0.         623.128289  ]
#  [  0.         578.78812127 353.09562861]
#  [  0.           0.           1.        ]]
# Camera distortion coefficients: 
# [-0.06651397  0.05124578  0.00042515 -0.00014606  0.03284797  0.22609471
#  -0.03786378]


    # Camera intrinsic and distortion parameters

    mtx = np.array([[576.47290634, 0,            621.58926236],
                    [0,            576.47290634, 351.36882130],
                    [0,            0,            1           ]])
    dist = np.array([11.97829777, 9.94853639, 0.00034557, -0.00001785, 0.76452355, 12.26980541, 13.39036658, 2.77419623])

    os.makedirs(output_folder, exist_ok=True)

    for filename in os.listdir(folder):
        image_path = os.path.join(folder, filename)
        image = cv2.imread(image_path)

        h, w = image.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

        # Undistort the image
        dst = cv2.undistort(image, mtx, dist, None, newcameramtx)
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        h, w = image.shape[:2]
        dst = cv2.resize(dst, (w, h))
        cv2.imwrite(os.path.join(output_folder, filename), dst)
        print(f'Image {filename} processed.')

    print('All images processed.')

if __name__ == '__main__':
    main()
