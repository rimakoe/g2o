FROM public.ecr.aws/lts/ubuntu:jammy

RUN apt-get update && \
    apt-get -y install \
    cmake \
    build-essential \
    python3-dev \
    libeigen3-dev \ 
    git \
    python3-venv \
    pip \
    net-tools && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip setuptools wheel mypy numpy flags

WORKDIR /home/workspace/g2o

#ENV PYTHONPATH="/usr/local/lib/python3.10/dist-packages/g2opy:${PYTHONPATH}"
#ENTRYPOINT ["./entrypoint.bash"]
