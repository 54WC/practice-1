#! /bin/bash
echo "Welcome"
export PYTHONPATH=/users/wc/underactuated/src:${PYTHONPATH}
export PYTHONPATH=/users/wc/underactuated/src/underactuated:${PYTHONPATH}
export PYTHONPATH=/opt/drake/lib/python3.7/site-packages:${PYTHONPATH}
export PATH=/usr/local/opt/python/libexec/bin:${PATH}
echo ${PYTHONPATH}
echo ${PATH}
echo "Welcome"
