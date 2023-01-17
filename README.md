
<h1 align="center">
 Optimization of Humanoid Robot Designs for Human-Robot Ergonomic Payload Lifting
</h1>


<div align="center">

C.Sartore, L.Rapetti, D.Pucci _"Optimization of Humanoid Robot Designs for Human-Robot Ergonomic Payload Lifting"_
in 2022 IEEE-RAS International Conference on Humanoid Robotics (Humanoids)

</div>

<p align="center">

 
 

https://user-images.githubusercontent.com/56030908/212954125-6fbbb7e9-ec35-4d0e-b718-e91360bdb34a.mp4


 
</p>

<div align="center">
  IEEE-RAS International Conference on Humanoid Robotics
</div>

<div align="center">
  <a href="#installation"><b>Installation</b></a> |
  <a href="https://ieeexplore.ieee.org/document/10000222"><b>Paper</b></a> |
  <a href=><b>Video</b></a>
</div>

### Installation
:warning: The repo depends on HSL routine, to correctly link the library please substitute this line of the docker image with the absolute path to the coinhsl.zip


To install the repo on a Linux termminal follow the following steps 

```
git clone https://github.com/ami-iit/paper_sartore_2022_humanoids_ergonomic_design.git  
cd paper_sartore_2022_humanoids_ergonomic_design
docker build --tag Sartore2022Results . 
```
For running the script you have first to disable the control the acess control via the following command 
```
xhost +
```
⚠️ We kindly suggest to re-activate once you have collected the results via the following command 
```
xhost -
```

For running the docker image use the following command 

```
docker run --net=host --env="DISPLAY=$DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix" --privileged -it Sartore2022Results
```

### Citing this work

```bibtex
@INPROCEEDINGS{Sartore2022Optimization,
  author={Sartore, Carlotta and Rapetti, Lorenzo and Pucci, Daniele},
  booktitle={2022 IEEE-RAS 21st International Conference on Humanoid Robots (Humanoids)}, 
  title={Optimization of Humanoid Robot Designs for Human-Robot Ergonomic Payload Lifting}, 
  year={2022},
  volume={},
  number={},
  pages={722-729},
  doi={10.1109/Humanoids53995.2022.10000222}}
```

### Maintainer

This repository is maintained by:

| | |
|:---:|:---:|
| [<img src="https://github.com/CarlottaSartore.png" width="40">](https://github.com/GitHubUserName) | [@CarlottaSartore](https://github.com/CarlottaSartore) |
