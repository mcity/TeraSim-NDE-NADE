## Pre-requirements

- System: Ubuntu 22.04
- Python: 3.10
- Anaconda is preferred

```python
conda create -n terasim python=3.10
conda activate terasim 
```

## Installation

### Install required packages

```bash
sudo apt update
sudo apt upgrade
sudo apt install redis-server libxext-dev libxrender-dev libgl1-mesa-glx  -y
```

### TeraSim

- install sumo and related packages

```python
pip install eclipse-sumo traci libsumo sumolib
```

- clone the repo

```python
git clone https://github.com/michigan-traffic-lab/TeraSim.git
```

- install the repo as a package

```python
cd TeraSim
pip install .
cd ..
```

### TeraSim-MR

- install and initialize redis-server
    
    [Install Redis on Linux](https://redis.io/docs/getting-started/installation/install-redis-on-linux/)
    
- Clone TeraSim-MR repo

```python
git clone https://github.com/michigan-traffic-lab/TeraSim-MR.git
```

- Install requirements

```python
pip install -r requirements.txt
```

- install package

```python
cd TeraSim-MR
pip install .
cd ..
```

- Set environment variable (for communication with McityOS)

```python
export MCITY_OCTANE_KEY=mcity
export MCITY_OCTANE_SERVER=https://atrium-simulation.um.city
```

### TeraSim-NDE-ITE

- Clone the repo

```python
git clone https://github.com/michigan-traffic-lab/TeraSim-NDE-ITE
git checkout ITE-ASCS
```

## Usage

Two modules are required to run the ITE simulation with remote access. This two modules should run in two individual terminals

### Communication
This will communicate with McityOS to publish the simulation status and receive the AV-related information

```bash
```bash
bash communication.bash
```

### ITE simulation

This will run the ITE simulation forever with 120s per iteration. The result will be saved at `output/"ITE_test_$(date +%Y%m%d-%H%M%S)"`.

```bash
bash ITE.bash
```