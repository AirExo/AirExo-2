#!/bin/bash

base_path="data/train"
scene_name="scene_0001"

python -m airexo.adaptor.render +path=${base_path}/${scene_name}
