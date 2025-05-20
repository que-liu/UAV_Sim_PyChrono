# PyChrono Installation Guide on macOS

This guide walks you through the installation of [PyChrono](https://projectchrono.org/) on macOS.


## Install Anaconda or Miniconda distribution

check the [installation guide](https://www.anaconda.com/docs/getting-started/miniconda/install#macos-linux-installation)

## Step 1: Add the conda-forge channel

```bash
conda config --add channels http://conda.anaconda.org/conda-forge
```

## Step 2: Create a new environment for PyChrono

```bash
conda create -n chrono python=3.9
```
You can replace "chrono" with the name you want.

To enter the environment:
```bash
conda activate chrono
```
To leave the environment:
```bash
conda deactivate
```
In the following steps, please stay in the created environment.


## Step 3: Install dependencies for PyChrono
```bash
conda install -c conda-forge numpy=1.24.0
conda install -c conda-forge matplotlib
conda install -c conda-forge irrlicht=1.8.5
```

## Step 4: Download Pychrono package and install
Visit the [package repo](https://anaconda.org/projectchrono/pychrono/files?page=3)

Click to download osx-arm64/pychrono-7.0.0-py39_2455.tar.bz2

After the download completes, navigate to the Downloads folder and install PyChrono
```bash
cd Downloads/
conda install pychrono-7.0.0-py39_2455.tar.bz2
export PYTHONPATH=$HOME/miniconda3/envs/pychrono/share/chrono/python
```

## Step 5: Copy the demos and test the installation
First, navigate to the folder you want to keep the demos files.
```bash
mkdir pychrono_demos
cd pychrono_demos/
cp -r $PYTHONPATH/pychrono/demos/* .
```
Now you can try to run the demos to check if the installation is correct.
For example,
```bash
cd mbs/
python3.9 demos_MBS_revolute.py
```