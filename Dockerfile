FROM ubuntu:latest

RUN apt update && apt upgrade -y

RUN apt install python3-dev python3-pip redis-server libxext-dev libxrender-dev libgl1-mesa-glx  -y


COPY TeraSim /app/TeraSim

COPY TeraSim-MR /app/TeraSim-MR

COPY TeraSim-NDE-ITE /app/TeraSim-NDE-ITE


WORKDIR /app

RUN pip3 --default-timeout=2000 install eclipse-sumo traci libsumo sumolib

WORKDIR /app/TeraSim

RUN pip3 install .

WORKDIR /app/TeraSim-MR

RUN pip3 install -r requirements.txt

RUN pip3 install .

WORKDIR /app/TeraSim-NDE-ITE

RUN pip3 install -r requirements.txt

RUN chmod -R 777 ITE_remote.bash

CMD ["bash","ITE_remote.bash"]