---
layout: post
title : Extraction and Detection of Lanes in different illumination Condition
---
The first post <a href="https://karth1kkumark.github.io/Advance_lane_detection/" target="_blank">here</a>.Describes the lane detection using a color mask that has a major drawback. The Rpi camera that I am using has no IR filter, Therefore this greatly affects the color perception of the image based on illumination condition, This alters the color range of lane pixels, So the challenge was to develop a technique which is impervious to these drawbacks. This post describes the methodology to achieve lane extraction in various illumination condition with considerable accuracy.

<img src="{{ site.baseurl }}/assets/images/IMG_0028.jpg" width="300px">

### The hardware Utlilized
- RC car
- RPI NOIR camera
- Raspiberry PI
- L298N motor driver

### Streaming Images from Rpi to Laptop Through ROS

Ros has built-in packages for image streaming and control of a robot, which we will be using in the future. I have installed ubuntu mate flavor of Ubiquity robotics available  <a href="https://downloads.ubiquityrobotics.com/pi.html" target="_blank">here</a>, which has ros pre-installed. The instruction for compiling and running raspicam_node is available <a href="https://github.com/UbiquityRobotics/raspicam_node" target="_blank">available here.</a>

{% highlight python %}
<launch>
<node type="raspicam_node" pkg="raspicam_node" name="raspicam_node" output="screen">

<param name="camera_info_url" value="package://raspicam_node/camera_info/camerav2_640X480.yaml"/>
<param name="width" value="640"/>
<param name="height" value="480"/>

<param name="framerate" value="30"/>
<param name="exposure_mode" value="antishake"/>
<param name="shutter_speed" value="0"/>
<param name="camera_frame_id" value="raspicam"/>
</node>
</launch>
{% endhighlight %}

The image streamed by raspicam_node is in a compressed format. Hence rosrun the following to decompress and publish the image  on  __/raspicam_node/image__ topic
{% highlight python %}
rosrun image_transport republish compressed in:=/raspicam_node/image raw out:=/raspicam_node/image
{% endhighlight %}

The image is published  on  the  __/raspicam_node/image__ topic is of type sensor_msgs/Image. This need to be converted into opencv format for further operation.
{% highlight python %}
image_sub = rospy.Subscriber("/raspicam_node/image",Image,callback)
def callback(data):
    cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
{% endhighlight %}
### Preprocess for thresholding 
The First step in the process of extraction of lane pixel is to undistort the image
{% highlight python %}
def undistorth(img,mtx,dist):#function for un distorting the image wrt to camera parameters 
                             #obtained from camera calibration
        h,w = img.shape[:2]
        print(h,w)
        newcameramtx,roi=cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h)) 
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
        #plt.imshow(dst)
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
        return dst
{% endhighlight %}

Camera intrinsic matrix and distortion coefficient are passed as arguments  with the image.

In order to increase the contrast of the image, Adaptive histogram equalization is applied

<table style="width:100%; border:0px;">
  <tr>
    <th>Undisort image</th>
    <th>Enhanced image</th> 
  </tr>
  <tr>
    <td><img src="{{ site.baseurl }}/assets/images/Undistort.png" width="480px"></td>
    <td><img src="{{ site.baseurl }}/assets/images/Enhancement_screenshot_07.09.2018.png" width="480px"></td>
  </tr>
</table>
<div class="code-block"> 

{% highlight python %}
def enhancement(img):

#-----Converting image to LAB Color model----------------------------------- 
    lab= cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

#-----Splitting the LAB image to different channels-------------------------
    l, a, b = cv2.split(lab)
    #cv2.imshow('l_channel', l)
    #cv2.imshow('a_channel', a)
    #cv2.imshow('b_channel', b)

#-----Applying CLAHE to L-channel-------------------------------------------
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    cl = clahe.apply(l)
    #cv2.imshow('CLAHE output', cl)

#-----Merge the CLAHE enhanced L-channel with the a and b channel-----------
    limg = cv2.merge((cl,a,b))
    #cv2.imshow('limg', limg)

#-----Converting image from LAB Color model to RGB model--------------------
    final = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
    #cv2.imshow('final', final)
    #cv2.waitKey()
    return final
{% endhighlight %}
</div>


The contrast-enhanced image is converted to <a href="https://en.wikipedia.org/wiki/CIELAB_color_space" target="_blank">CIELAB color space.</a> The image is split into the corresponding channel(L* for the lightness and a* and b* for the green–red and blue-yellow color components). As the color of the track falls into blue-yellow, We choose b channel image(A) and apply the __bitwise_not__ operation to obtain the inverse image(B). Now we subtract b channel image(A) from bit not image to obtain difference image C, Only those pixel which undergoes a change of more than 150-pixel value is retained .rest of the pixels are set to 0. The difference image(C) is element-wise multiplied with b channel image(A), This amplifies the pixel value of pixels which undergoes max change.
<table style="width:100%; border:0px;">
  <tr>
    <th>b channel image</th>
    <th>Bitnot image</th> 
  </tr>
  <tr>
    <td><img src="{{ site.baseurl }}/assets/images/B channel_screenshot_07.09.2018.png" width="480px"></td>
    <td><img src="{{ site.baseurl }}/assets/images/Bitnot_screenshot_07.09.2018.png" width="480px"></td>
  </tr>
</table>

<table style="width:100%; border:0px;">
  <tr>
    <th>Difference image</th>
    <th>Element wise multipled image</th> 
  </tr>
  <tr>
    <td><img src="{{ site.baseurl }}/assets/images/Diff_screenshot_07.09.2018.png" width="480px"></td>
    <td><img src="{{ site.baseurl }}/assets/images/multipy_screenshot_07.09.2018.png" width="480px"></td>
  </tr>
</table>

{% highlight python %}
    lab= cv2.cvtColor(frame11, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    bitnotimage=cv2.bitwise_not(b)
    diff=bitnotimage-b
    diff[diff< 150] = 0
    multipyimage=np.multiply(diff,b)
    multipyimage[multipyimage<110] = 0
{% endhighlight %}


### Dynamic Thresholding

The implementation is inspired by this <a href="https://ieeexplore.ieee.org/document/6889285/" target="_blank">paper.</a> The image obtained from element-wise multiplication is applied dynamic thresholding to extract Lane pixel. Dynamic thresholding assumes that a vehicle runs in the middle of lanes. The image is divided in half along the width resulting in Dl(left half) and DR(right half) For both Dl and DR are scanned along the rows and max value in each row is obtained along with its position in the Row. For Dl the positional max value is selected as this resides closer to the middle of the lane, similarly, For DR the positional min value is selected.

<table style="width:100%; border:0px;">
  <tr>
    <th>Element wise multipled image</th>
    <th>Dynamic Thresholding</th> 
  </tr>
  <tr>
    <td><img src="{{ site.baseurl }}/assets/images/multipy_screenshot_07.09.2018.png" width="480px"></td>
    <td><img src="{{ site.baseurl }}/assets/images/Dynamic thres_screenshot_07.09.2018.png" width="480px"></td>
  </tr>
</table>

<div class="code-block"> 
{% highlight python %}
   def dynamicrange(image):
        (iH, iW) = image.shape[:2]
        Dl=image[:,:iW/2]
        DR=image[:,iW/2:]
        imagecopy2=np.zeros((iH,iW),dtype="uint8")
        for k in np.arange(0,iH):
        maxv=np.max(Dl[k,:])
        c=[i for i, j in enumerate(Dl[k,:]) if j == maxv]
        if(len(c)):
            imagecopy2[k][c[len(c)-1]]=Dl[k,c[len(c)-1]]
        
        maxvR=np.max(DR[k,:])
        d=[m for m, n in enumerate(DR[k,:]) if n == maxvR]
        if(len(d)):
            imagecopy2[k][iW/2+d[0]]=DR[k,d[0]]
        return imagecopy2
{% endhighlight %}
</div>

### Detect lane pixels and fit to find the lane boundary
The Image after dynamic thresholding is passed as an argument to ROI function which crops the image leaving out only lane pixel. The ROI image is then perspectively transformed to obtain Bird eye view. The histogram of the lower half of the image is used to get the range of the position of pixels in the image. Then dividing the entire image into n(current n=9) windows. Using the function Your image.nonzero() to get x, y coordinates of non zero pixels is determined. Now x and y coordinate of right and left lane pixels are determined.we use Curve fitting to obtain the polynomial.
<table style="width:100%; border:0px;">
  <tr>
    <th>ROI image</th>
    <th>Bird Eye View</th> 
  </tr>
  <tr>
    <td><img src="{{ site.baseurl }}/assets/images/ROI_screenshot_07.09.2018.png" width="480px"></td>
    <td><img src="{{ site.baseurl }}/assets/images/BIE_screenshot_07.09.2018.png" width="480px"></td>
  </tr>
</table>
<div class="code-block"> 
{% highlight python %}
def ROI(img):   # function to get region of interest in a                           image
    #img=color_filter(img1)
    h=frame.shape[0]
    w=frame.shape[1]
    #change the ppoly coordinate according the camera mount
    shape = np.array([[5,200 ],[w,200],[w, h],[5,h]])
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
def birdeyeview(frame):
    h=frame.shape[0]
    w=frame.shape[1]
    pts1 = np.float32([[5,200 ],[w,200],[w, h],[5,h]])
    pts2 = np.float32([[0, 0], [w, 0],[w,h],[0,h]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    result = cv2.warpPerspective(frame, matrix, (w,h))
    return result,matrix
def extract_lanes_pixels(binary_warped):

        # Take a histogram of the bottom half of the image
        m=int(binary_warped.shape[0]/2)
        histogram = np.sum(binary_warped[m:,:], axis=0)
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
        minpix=50
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

def poly_fit(leftx, lefty, rightx, righty, left_lane_inds, right_lane_inds, binary_warped, plot):  

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

       
        return left_fit, right_fit, ploty, left_fitx, right_fitx
{% endhighlight %}
</div>

### Warp the lane boundary back on to original image
Now that lane curves are detected we will use cv2.fillPoly(color_warp, np.int_([pts]), (0,255,0)) to fill image along curve. After this step, Image is inverse perspective transformed into original view plane and combined with the original undistorted image.

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

### Result
 The results obtained from dynamic thresholding in both daylight and artificial illumination scenario.
 <iframe width="817" height="297" src="https://www.youtube.com/embed/qstuhDxod5s" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

<iframe width="817" height="297" src="https://www.youtube.com/embed/3ekDr2XvUK8" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

### Resources:
Source code:<a
href="https://github.com/Karth1kKumarK/Lane_detection_code" target="_blank">https://github.com/Karth1kKumarK/Lane_detection_code</a>

Reference Paper:
<a
href="https://ieeexplore.ieee.org/abstract/document/6889285/" target="_blank">https://ieeexplore.ieee.org/abstract/document/6889285/</a>

