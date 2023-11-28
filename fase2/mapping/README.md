# Drone mapping

The intention of phase 2 was to map the competition area. To do this, we created a "survey" mission in QGroundControl and adjusted the height according to the overlap we wanted and the time interval between shots. Ideally, the number of photos should not exceed 200. That's because processing in the Metashape software takes time and we had 20 minutes to do all the post-processing. Therefore, the mapping was aimed more at speed than at the quality of the map. We used the Picamera camera for the task. It has good quality and enough for what we wanted.



<figure><img src="mapa_odonto_resized.jpg" alt=""><figcaption><p><em>One of the maps we created when testing in USP. Although the quality was not the best, the mission was fast and the processing took less than 10 minutes.</em></p></figcaption></figure>



### Instructions

#### How to adjust parameters

There are many parameters to adjust in mapping. One of them is the camera setup. That's why we use the `parameters.sh` code to swap the best combinations of ISO and shutter speed depending on the day. We tested more parameters of the PiCamera camera, but we thought the most relevant ones were those. To use the `parameters.sh` code, simply type `bash parameters.sh` in the terminal of your on-board computer connected with the camera at the desired height and speed (with the camera already attached to the drone). This will better simulate the flight conditions.  **Remembering that all codes are based on PiCamera. More settings for it, see the** `sd_card` **tab.**

**Calculating height**

To calculate the flight height, you can simulate the GSD with the `ground_size.py` script to adjust the image quality. Overlap can be simulated in QGroundControl.

#### Flying

In the code `fase2_completa.py` there is a state machine. To begin mapping, you wait to arrive at the desired waypoint (this can be adjusted in the code depending on your mission). The mapping is finalized after reaching another waypoint and from there the precision landing must begin. Be sure to simulate your mission with the dronekit using `simulation_dronekit.py` before the flight to check that everything is correct.

#### Processing the map

With the images and their coordinates in hand, it's time to put together the map. First, run the script `resized.py` on the images, if you want the processing to be faster. This code only decreases the size of the images, making them lighter for Agisoft Metashape Pro. With the new images, all you have to do is open the software and upload them, align them, assemble the depth map or point cloud and then create an orthomosaic on top of them. This process can give some errors, especially in the alignment of the images. Make sure there are enough images and that they are of relatively good quality. To save your orthomosaic, simply export it in tiff format and then use the `tiff_to_jpeg.py` script to turn it into an image

