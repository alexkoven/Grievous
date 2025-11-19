We are building a cheap, human-like, fully-autonomous testbed. The testbed design is based on Mobile Aloha and XLeRobot. The software is built on top of LeRobot. Follow along for future updates!

![Short Demo](media/short_demo_920.gif)

We will update this repo as we go.


## Installation
1. Create conda environment and activate
```
conda create -y -n grievous python=3.10
conda activate grievous
```

2. Install ffmpeg in conda environment with
`conda install ffmpeg -c conda-forge`

3. Clone this repo and install directory in editable mode
```
git clone git@github.com:alexkoven/Grievous.git
cd ./Grievous
pip install -e .
```


## Running Grievous

On RPi5, activate conda environment and run
```
cd ~/Grievous
python -m lerobot.robots.grievous.grievous_host
```

On remote PC, activate conda environment and run 
```
./test_record.sh
```

