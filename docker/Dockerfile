FROM public.ecr.aws/lts/ubuntu:jammy AS deps

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

FROM deps AS install

COPY .. /home/workspace/g2o
RUN /home/workspace/g2o/entrypoint.bash

CMD ["/bin/bash"]