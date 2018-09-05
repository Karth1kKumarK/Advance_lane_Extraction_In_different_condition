---
layout: post
title : Advance lane detection using opencv
---
This post describes process and code implementation required to achieve lane detection with rpi camera using opencv pipeline. The code is in jupyter notebook is available <a href="https://github.com/Karth1kKumarK/Advance_lane_detection_code" target="_blank">here</a>.
This project was inspired by Udacity's CarND-Advanced-Lane-Lines project.
<table style="width:100%; border:0px;">
  <tr>
    <th>Track</th>
    <th>Platform</th> 
  </tr>
  <tr>
    <td><img src="{{ site.baseurl }}/assets/images/IMG_20180824_170612.jpg" width="580px"></td>
    <td><img src="{{ site.baseurl }}/assets/images/car.png" width="280px"></td>
  </tr>
</table>
### The steps to be followed to achieve objective are:
-   calibration of rpi camera to obtain intrinsic camera parameters and distortion      coefficients.
-  Apply distortion correction to image from rpi camera.
-  Apply color mask,ROI to image to filter out unnecessary information
-  Conversion to gray scale image and apply canny edge detection
-  Apply a perspective transform to get bird eye view.
-  Detect lane pixels and fit to find the lane boundary.
-  Warp the lane boundary back on to original image

### Rpi Camera calibration:
The image captured is 2D representation of 3D world,Hence this transformation from 3D to 2D is not ideal, This leads to distortion in image.Therefore camera calibration is required to obtain camera matrix and Distortion coefficients.for more information 
refer <a href="https://docs.opencv.org/3.1.0/dc/dbb/tutorial_py_calibration.html" target="_blank">this.</a>


For camera calibration,I have used 7X9 checkerboard available <a href="https://www.mrpt.org/downloads/camera-calibration-checker-board_9x7.pdf" target="_blank">here</a>,capture the images of checker board in different orientation and place   them in a folder camera_01.Run the cameracalib.py(__place camera_01 in the same directory as cameracalib.py__)

<div class="code-block">
{% highlight python %}
import numpy as np
import cv2
import glob
import sys
import argparse
nRows = 7
nCols = 9
dimension = 23
workingFolder   = "./camera_01"
imageType       = 'jpg'
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((nRows*nCols,3), np.float32)
objp[:,:2] = np.mgrid[0:nRows,0:nCols].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
filename    = workingFolder + "/*." + imageType
images      = glob.glob(filename)
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (nRows,nCols), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2=cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv2.drawChessboardCorners(img, (nRows,nCols), corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)
cv2.destroyAllWindows()
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
img = cv2.imread(images[1])
h,  w = img.shape[:2]
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
 # undistort
mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
dst = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)

    # crop the image
x,y,w,h = roi
dst = dst[y:y+h, x:x+w]
cv2.imwrite(workingFolder + "/calibresult.png",dst)
filename = workingFolder + "/cameraMatrix.txt"
np.savetxt(filename, mtx, delimiter=',')
filename = workingFolder + "/cameraDistortion.txt"
np.savetxt(filename, dist, delimiter=',')
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error
print(mean_error/len(objpoints))

{% endhighlight %}
</div>

After successful, Execution the cameraMatrix.txt and cameraDistortion.txt are saved in camera_01 folder.We will load this matrix into python space and pass them as argument 
into __undistort__ function.
{% highlight python %}
mtx=np.loadtxt('/YOUR DIRECTORY/camera_01/cameraMatrix.txt', delimiter=',', dtype=None)
dist=np.loadtxt('/YOUR DIRECTORY/camera_01/cameraDistortion.txt', delimiter=',', dtype=None)
{% endhighlight %}

<table style="width:100%; border:0px;">
  <tr>
    <th>Distorted</th>
    <th>Undistorted</th> 
  </tr>
  <tr>
    <td><img src="{{ site.baseurl }}/assets/images/snapshot_640_480_20.jpg" width="280px"></td>
    <td><img src="{{ site.baseurl }}/assets/images/calibresult.png" width="280px"></td>
  </tr>
</table>

The undistort function applies distortion correction to the image
(__the image returned from this function is croped after distortion correction__)
{% highlight python %}
def undistort(img,mtx,dist):#function for un distorting the image wrt to camera                                        #parameters 
                             #obtained from camera calibration
        h,w = img.shape[:2]
        newcameramtx,roi=cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h)) 
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
        # crop the image
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
        return dst
{% endhighlight %}


### Apply color mask,roi to image to filter out unnecessary information

In the the current project because of the Color of the left,right lane are same.
Hence color mask and the Region of interest)( __ROI__) is applied to the image, The information other than the ROI and bandwidth of the color mask  is eliminated.The bandwidth for the upper and lower limit of color mask was found through trail and error.
<table style="width:100%; border:0px;">
  <tr>
    <th>Undisort image</th>
    <th>ROI image</th> 
  </tr>
  <tr>
    <td><img src="{{ site.baseurl }}/assets/images/Undistort.png" width="480px"></td>
    <td><img src="{{ site.baseurl }}/assets/images/ROIimage.png" width="280px"></td>
  </tr>
</table>
<div class="code-block"> 
{% highlight python %}
def color_filter(image):
    #convert to HLS to mask based on HLS
    hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    lower = np.array([105,60,50])
    upper = np.array([125,110,115])
    yellower = np.array([255,215,0]) #lower limit 
    yelupper = np.array([50,255,255])
    yellowmask = cv2.inRange(hls, yellower, yelupper)
    whitemask = cv2.inRange(hls, lower, upper)
    mask = cv2.bitwise_or(yellowmask, whitemask)
    masked = cv2.bitwise_and(image, image, mask = mask)
    return masked
    
    def ROI(img1):   # function to get region of interest in a image
    img=maskedimage=color_filter(img1)
    #change the ppoly coordinate according the camera mount
    shape = np.array([[0,250],[640,250],[640,480],[0,480]])
    #define a numpy array with the dimensions of img, but comprised of zeros
    mask = np.zeros_like(img)
    #Uses 3 channels or 1 channel for color depending on input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
    #creates a polygon with the mask color
    cv2.fillPoly(mask, np.int32([shape]), ignore_mask_color)
    #returns the image only where the mask pixels are not zero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image
{% endhighlight %}
</div>

### conversion to gray scale image and apply canny edge detection
The ROI image obtained is now converted into grayscale and canny edge detection algorithm is applied.
<table style="width:100%; border:0px;">
  <tr>
    <th>Gray scale image </th>
    <th>Canny edge</th> 
  </tr>
  <tr>
    <td><img src="{{ site.baseurl }}/assets/images/greyscale.png" width="280px"></td>
    <td><img src="{{ site.baseurl }}/assets/images/canny.png" width="280px"></td>
  </tr>
</table>
{% highlight python %}
greyimage=cv2.cvtColor(ROIimage, cv2.COLOR_RGB2GRAY)
cannyimage=cv2.Canny(greyimage,100 ,80)
{% endhighlight %}
### Apply a perspective transform to get bird eye view(BEV)
The perspective transformation will give us top down view of the track.
This is important for estimation of curvature radius in case of curved 
track. To get BEV we want to select four source point's in trapezoidal shape
which we want to get the top down view
<table style="width:100%; border:0px;">
  <tr>
    <th>Canny edge </th>
    <th>Bird Eye view</th> 
  </tr>
  <tr>
    <td><img src="{{ site.baseurl }}/assets/images/canny.png" width="280px"></td>
    <td><img src="{{ site.baseurl }}/assets/images/birdeye.png" width="280px"></td>
  </tr>
</table>
{% highlight python %}
def birdeyeview(frame):
    #cv2.circle(frame, (155, 280), 5, (0, 0, 255), -1)
    ##these  points are for  visualization
    #cv2.circle(frame, (500, 280), 5, (0, 0, 255), -1)
    #cv2.circle(frame, (5, 435), 5, (0, 0, 255), -1)
    #cv2.circle(frame, (605, 435), 5, (0, 0, 255), -1)
    pts1 = np.float32([[155, 280],[500, 280],[605, 435],[5,435]])
    pts2 = np.float32([[0, 0], [605, 0],[605,440],[0,440]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    result = cv2.warpPerspective(frame, matrix, (605,440))
    return result,matrix
{% endhighlight %}

### Detect lane pixels and fit to find the lane boundary
The lane detection relies on extraction of lane pixel and to  fit the curve to  get the  curvature and that in turn can we used to compute steering angles.
The approach used here is  to get histogram of the lower half of the
image to get the range of position of pixels in the image.
Then dividing the entire image into n(current n=9) of windows.
Using the function __Yourimage.nonzero()__ to get x ,y co ordinates of non zero pixels is determined.In the current window with width =100,Window boundaries in x and y is determined, then within this window non zero pixel location is found.these pixels are categorized  based on the boundaries of window,Hence extracting left and right lane pixels.
Now x and y co ordinate of right and left lane pixels are determined, we use Curve fitting to obtain the polynomial.
<img src="{{ site.baseurl }}/assets/images/lane_detect.png" width="480px">
<div class="code-block"> 
{% highlight python %}
def extract_lanes_pixels(binary_warped):

        # Take a histogram of the bottom half of the image
        histogram = np.sum(binary_warped[binary_warped.shape[0]/2:,:], axis=0)
        #plt.plot(histogram)
        #plt.show()
        # Create an output image to draw on and  visualize the result
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        midpoint = np.int(histogram.shape[0]/2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # Choose the number of sliding windows
        nwindows = 9
        # Set height of windows
        window_height = np.int(binary_warped.shape[0]/nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Current positions to be updated for each window
        leftx_current = leftx_base
        rightx_current = rightx_base
        # Set the width of the windows +/- margin
        margin = 100
        # Set minimum number of pixels found to recenter window
        
        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        right_lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = binary_warped.shape[0] - (window+1)*window_height
            win_y_high = binary_warped.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            # Draw the windows on the visualization image
            cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),(0,255,0), 2) 
            cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),(0,255,0), 2) 
            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:        
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds] 

        return leftx, lefty, rightx, righty, left_lane_inds, right_lane_inds

def poly_fit(leftx, lefty, rightx, righty, left_lane_inds, right_lane_inds, binary_warped, plot:False):  

        # Fit a second order polynomial to each
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        # Generate x and y values for plotting
        ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
        out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
        out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

        if(plot):
            plt.imshow(cv2.cvtColor(out_img, cv2.COLOR_BGR2RGB))
            plt.plot(left_fitx, ploty, color='yellow')
            plt.plot(right_fitx, ploty, color='yellow')
            plt.xlim(0, 640)
            plt.ylim(540, 0)
            plt.show()

        return left_fit, right_fit, ploty, left_fitx, right_fitx
{% endhighlight %}
</div>

### Warp the lane boundary back on to original image
Now that lane curves are detected we will use 
cv2.fillPoly(color_warp, np.int_([pts]), (0,255,0))
to fill image along curve. After this step,Image is 
inverse perspective transformed into original view plane and 
combined with the  original undistorted image.
<img src="{{ site.baseurl }}/assets/images/final.png">
<div class="code-block">
{% highlight python %}
def plain_lane(undist, warped, M, left_fitx, right_fitx, ploty, plot=False):
        
        Minv = inv (M)

        # Create an image to draw the lines on
        warp_zero = np.zeros_like(warped).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        
        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))
        
        # Draw the lane onto the warped blank image
        cv2.fillPoly(color_warp, np.int_([pts]), (0,255,0))
        
        # Warp the blank back to original image space using inverse perspective matrix (Minv)
        newwarp = cv2.warpPerspective(color_warp, Minv, (warped.shape[1], warped.shape[0])) 
        #newwarp=undistorth(newwarp)
        # Combine the result with the original image
        result = cv2.addWeighted(undist, 1, newwarp, 0.3, 0)
        if(plot):
            plt.imshow(cv2.cvtColor(result, cv2.COLOR_BGR2RGB))
            
        
        return result


 def render_curvature_and_offset(rundist_image, curverad, offset, plot=False):   
        # Add curvature and offset information
        offst_text = 'offset: {:.2f}m'.format(offset)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(rundist_image, offst_text, (24, 50), font, 1, (255, 255, 255), 2)
        curverad_text = 'curverad: {:.2f}m'.format(curverad)
        cv2.putText(rundist_image, curverad_text, (19, 90), font, 1, (255, 255, 255), 2)
        if(plot):
            plt.imshow(cv2.cvtColor(rundist_image, cv2.COLOR_BGR2RGB))
        return rundist_image
{% endhighlight %}
</div>

