#!/bin/bash

mkdir -p /home/workspace/g2o/build && \
cmake -S/home/workspace/g2o -B/home/workspace/g2o/build -DG2O_BUILD_PYTHON=ON -DG2O_USE_CSPARSE=OFF -DG2O_USE_CHOLMOD=OFF && \
cd /home/workspace/g2o/build && \
make -j10 && \
make install

cd /home/workspace/g2o
python3 setup.py build bdist_wheel
pip install /home/workspace/g2o/build/python/dist/g2opy-1.0.0-py3-none-any.whl