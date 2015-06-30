Registration - DisCODe Component Library
===========================================

Description
-----------

DCL containing components responsible for registration of models of objects (SIFTObjctModels - SOMs), their saving to files, loading models from files etc.

Dependencies
------------

DCL depends on the following DCLs:
- CvCoreTypes - contains basic OpenCV-related types
- PCLCoreTypes - contains basic PCL-related types
- PCL - contains algorithms responsible for processing of point clouds (3D vision)
- CvBasic - contains components responsible for "classical" (2D) computer vision
- CvStereo - required for estimation of depth from stereo pairs
- CameraNUI - required for acquisition from Kinect-like sensors
- ModelViewAcquisition - contains component responsible for extraction of masks
- SIFTObjectModel - contains components responsible for generation of SIFT Object Model

Tasks
------------
Registration of models of objects from stereo camera images:

   * __TexturedStereoLRSequenceToCloudViewer__ - Reads two pairs of left and right images from files (2 with and 2 without additional projection of texture), computes depth and displays resulting XYZRGB point cloud and stores RGB and XYZ files. 

   * __RGBXYZWideSequenceSOMCollector__ - Reads RGB and XYZ files along with image acquired from wide camera in order to generate "fragments of object" - views consisting of pairs of XYZRGB and XYZSIFT clouds.

   * __CloudXYZRGBSIFTLUMCollector__ - Loads views (pairs of XYZRGB and XYZSIFT clouds) and registers them, with the use of LUM-based loop closure.


Registration of models of objects from Kinect images:

   * __RGBDSequenceSOMCollector__ - Reads RGB and Depth images from files and generates on their basis "fragments of object" - views consisting of pairs of XYZRGB and XYZSIFT.

   * __CloudXYZRGBSIFTLUMCollector__ - Loads views (pairs of XYZRGB and XYZSIFT clouds) and registers them, with the use of LUM-based loop closure (the same task as for stereo).



Task for calibration of our multi-camera setup:

   * __LeftWideSetupCalibration__ - Task for calibration of a dual-camera  setup (i.e. left and wide cameras) with the use of classical chessboard pattern.

   * __LeftWideSetupCalibrationCirclesGrid__ - Task for calibration of a dual-camera  setup (i.e. left and wide cameras) with the use of circles grid pattern.


Additional tasks for displaying images - kinect RGB-D images:

   * __KinectRGBDRainbowViewer__ - Displays images acquired on-the-fly from Kinect, enables writing of images to files.


Additional tasks for displaying images - stereo camera images:

   * __StereoLRSequenceViewer__ - Displays left and right images from stereo pair (loaded from files).

   * __StereoLRSequenceToRGBXYZViewer__ - Loads left and right images loaded from files, estimates depth and displays results. Additionally enables to save resulting images to files.


Additional tasks for displaying images - RGB-XYZ pairs of images:

   * __RGBXYZSequenceViewer__ - loads RGB-XYZ pairs of images, displays rainbow (depth) and generated XYZRGB point cloud.


Additional tasks for displaying clouds:


   * __CloudXYZRGBSequenceViewer__ - loads and displays sequence of XYZRGB clouds.

   * __CloudXYZRGBSIFTSequenceViewer__ - loads and displays pairs of XYZRGB and XYZSIFT clouds

   * __CloudXYZRGBSIFTCorrespondencesViewer__ - loads and displays two sequences of pairs of XYZRGB and XYZSIFT clouds, estimates and displays correspondences.





Maintainer
----------

- [Tomasz Kornuta](https://github.com/tkornuta)

