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
Registration from stereo camera:

   * __TexturedStereoLRSequenceToCloudViewer__ - Reads two pairs of left and right images from files (2 with and 2 without additional projection of texture), computes depth and displays resulting XYZRGB point cloud and stores RGB and XYZ files. 

   * __RGBXYZWideSequenceSOMCollector__ - Reads RGB and XYZ files along with image acquired from wide camera in order to generate fragments of object - pairs of XYZRGB and XYZSIFT clouds.

   * __CloudXYZRGBSIFTLUMCollector__ - Loads pairs of XYZRGB and XYZSIFT clouds and registers them, with the use of LUM-based loop closure.


Registration from Kinect:

   * __RGBDSequenceSOMCollector

Additional stereo camera tasks:

   * __StereoLRSequenceViewer__ - Displays left and right images from stereo pair (loaded from files).

   * __StereoLRSequenceToRGBXYZViewer__ - Loads left and right images loaded from files, estimates depth and displays results. Additionally enables to save resulting images to files.


Maintainer
----------

- [Tomasz Kornuta](https://github.com/tkornuta)

