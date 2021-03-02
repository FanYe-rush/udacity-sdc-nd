### Reflection

### 1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

Pipeline:
1. Apply gaussian blur to smooth out the image
2. Convert the image to gray scale
3. Apply canny edge detection to find the edges
4. Extract the lines among detected edges
5. Apply the bounding box to find out the lane lines in the detected lines
    *. While scanning through the frames, storing the lanes detected, for each frame, use the aveage image of last 10 images of deteceted lanes as the image to  use for aggregation. (image of detected lane lines only, not overlaied on original image)
    *. Run the average image through the pipeline again (blue, gray scale, canny detection, and hough line transformation), but in draw_lines(), an additional "aggregate=True" is passed in.
    *. To aggregate lines, separate all lines into 2 group: those with positive and those with negative slopes.
    *. Get the average slope and average constant term for each line
    *. Get the intersections of those 2 lines with top and bottom horizontal lines of the bounding box
6. Apply the aggregated lane to the original image

### 2. Identify potential shortcomings with your current pipeline

1. Bounding box is rough, could easily include non-relavent lines / edges to increase noise
2. When turns are sharp, lanes lines might change from positive slope to negative (or the other way around), aggregation by positive/negative slopes is not very robust
3. Always assuming straight lane lines can also produce badly fitted lane lines when in sharp turns.
4. Right now I detect lines, draw to image, cache them, then average the image caches to extract the lines again for aggregation, it's running 2 passes of the pipeline for each image


### 3. Suggest possible improvements to your pipeline

1. Improve line aggregation, on top of line slopes, also consider approximity of the line segments to form lines (or even curves) based on how close those segments are with each other
2. for the 4th point, could cache the detected lines instead and run aggregation with previously detected lines directly to save the second pass of line detection.
