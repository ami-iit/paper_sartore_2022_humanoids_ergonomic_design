
<h1 align="center">
 Optimization of Humanoid Robot Designs for Human-Robot Ergonomic Payload Lifting
</h1>


<div align="center">

C.Sartore, L.Rapetti, D.Pucci _"Optimization of Humanoid Robot Designs for Human-Robot Ergonomic Payload Lifting"_
in 2022 IEEE-RAS International Conference on Humanoid Robotics (Humanoids)

</div>

<p align="center">

[![Video](https://user-images.githubusercontent.com/56030908/213148311-0147d036-41c5-4124-aa90-341ad09b03f8.jpg)](https://user-images.githubusercontent.com/56030908/213148149-e539dc85-93d3-4d7a-94b7-54945c591f3e.mp4)

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


:warning: The repository depends on [HSL Mathematical Software Library](https://www.hsl.rl.ac.uk/), to correctly link the library please substitute [this](https://github.com/ami-iit/paper_sartore_2022_humanoids_ergonomic_design/blob/fc5083ca619d9c0dfe4e333fadad6d0f000c0dbf/Dockerfile#L26) line of the docker image with the absolute path to the `coinhsl.zip`

⚠️ This repository depends on [docker](https://docs.docker.com/)


To install the repo on a Linux terminal follow the following steps 

```
git clone https://github.com/ami-iit/paper_sartore_2022_humanoids_ergonomic_design.git  
cd paper_sartore_2022_humanoids_ergonomic_design
docker build --tag sartore2022results . 
```
Before running the docker image, you have to disable the acess control to allow the visualization of the optimization output, this can be done via the following command 
```
xhost +
```
For running the docker image use the following command 

```
docker run --net=host --env="DISPLAY=$DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix" --privileged -it sartore2022results
```

⚠️  Once you have collected the results, we suggest to re-activate the acess control via the following command  

```
xhost -
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
