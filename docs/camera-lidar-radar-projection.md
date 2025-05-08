# Perception Fusion

## Clone repo 

```
git clone --recursive git@github.com:ARG-NCTU/perception-fusion.git
``` 

## Update repo and submodules

```bash
git pull
git submodule sync --recursive
git submodule update --init --recursive
```

## Data preparation

Make the data dir
```bash
cd ~/perception-fusion
make -p data
cd data
```

Download and extract sample data
```bash
wget ftp://140.113.148.83/arg-projectfile-download/south-tw-maritime-multi-modal-dataset/south-tw-maritime-multi-modal-dataset.zip
unzip south-tw-maritime-multi-modal-dataset.zip
```

## Usage

Enter the repo

```bash
cd ~/perception-fusion
```

1. Docker Run

Run this script to pull docker image to your workstation.

```bash
source Docker/cpu/run.sh
```

2. Pointcloud projected on Camera Left, Mid, Right, Back Image

![image](docs/example/CAMERA_BACK_lidar.png)

Lidar pointcloud only for example:

```bash
python3 -m PerceptionFusion.PerceptionFusion --frame 10 --proj lidar --data /home/arg/perception-fusion/data/argnctu-perception --save_dir visualization
```

Radar pointcloud only for example:

```bash
python3 -m PerceptionFusion.PerceptionFusion --frame 10 --proj radar --data /home/arg/perception-fusion/data/argnctu-perception --save_dir visualization
```

Lidar pointcloud only for example:

```bash
python3 -m PerceptionFusion.PerceptionFusion --frame 10 --proj both --data /home/arg/perception-fusion/data/argnctu-perception --save_dir visualization
```
