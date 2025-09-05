# Seeing A 3D World in A Grain of Sand

Yufan Zhang<sup>1</sup>, Yu Ji<sup>2</sup>, Yu Guo<sup>1</sup>, Jinwei Ye<sup>1</sup>
<br>
<sup>1</sup>George Mason University, <sup>2</sup>LightThought LLC
<br>

[![Project](https://img.shields.io/badge/Project%20Page-Website-blue)](https://miniature-3dgs.github.io)
[![arXiv](https://img.shields.io/badge/arXiv-Paper-red)](https://arxiv.org/pdf/2503.00260)
[![GitHub](https://img.shields.io/badge/GitHub-Code-black)](https://github.com/YufanZhang82/miniGS)

## Introduction

See our [Project Page](https://miniature-3dgs.github.io) for more demonstration. 

## Abstract
We present a snapshot imaging technique for recovering 3D surrounding views of miniature scenes. Due to their intricacy, miniature scenes with objects sized in millimeters are difficult to reconstruct, yet miniatures are common in life and their 3D digitalization is desirable. We design a catadioptric imaging system with a single camera and eight pairs of planar mirrors for snapshot 3D reconstruction from a dollhouse perspective. We place paired mirrors on nested pyramid surfaces for capturing surrounding multi-view images in a single shot. Our mirror design is customizable based on the size of the scene for optimized view coverage. We use the 3D Gaussian Splatting (3DGS) representation for scene reconstruction and novel view synthesis. We overcome the challenge posed by our sparse view input by integrating visual hull-derived depth constraint. Our method demonstrates state-of-the-art performance on a variety of synthetic and real miniature scenes.

## Quick Start

You can find the pre-processed data in the ./input_data folder, which you can use directly for 3DGS training.
The directory is shown below.

```
input_data/
└── bear/
    ├── depth
    ├── images
    ├── masks
    ├── monodepth
    ├── sparse
    └── fused.ply
...
```
depth is the depth map obtained from the visual hull.
monodepth is the monocular depth map obtained by running [Depth Anything V2](https://github.com/DepthAnything/Depth-Anything-V2).
fused.ply is the dense point cloud.

## Image Preprocessing

### Calibration

You can skip this step and use the calibrated camera intrinsic and extrinsic parameters directly, which are located in the ./image_preprocessing/calib/5_cameraParameters folder.

Use MATLAB to run calib.m in the ./image_preprocessing folder. This will calibrate the intrinsic and extrinsic parameters of the eight virtual cameras.

### Raw Image Processing


## ✅ TODOs

- [x] release calibration code and 3DGS input data
- [ ] release raw image processing code
- [ ] release 3D printed prototype model

## Citation
If you find our work useful for your project please cite:
```
@InProceedings{Zhang_2025_CVPR,
    author    = {Zhang, Yufan and Ji, Yu and Guo, Yu and Ye, Jinwei},
    title     = {Seeing A 3D World in A Grain of Sand},
    booktitle = {Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)},
    month     = {June},
    year      = {2025},
    pages     = {11187-11196}
}
```

# Website License
<a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-sa/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/">Creative Commons Attribution-ShareAlike 4.0 International License</a>.
