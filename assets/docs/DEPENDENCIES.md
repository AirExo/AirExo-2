# 📁 Dependencies Installation Guide

Please follow the following steps to install necessary dependencies for ***AirExo*-2**.

1. Clone [ControlNet](https://github.com/lllyasviel/ControlNet), [ProPainter](https://github.com/sczhou/ProPainter) and [SAM2](https://github.com/facebookresearch/sam2) into the `dependencies` folder.

    ```bash
    git submodule init
    git submodule update
    cd dependencies
    ```

2.  Install [pytorch3d](https://github.com/facebookresearch/pytorch3d).
    ```bash
    cd pytorch3d
    pip install -e .
    cd ..
    ```

3.  Install [redner](https://github.com/BachiLi/redner). For GPU version, 
    ```bash
    pip install redner-gpu==0.4.28
    ```

    For CPU version,
    ```bash
    pip install redner==0.4.28
    ```

4.  For [SAM2](https://github.com/facebookresearch/sam2), you might need to install a new conda environment for SAM-2 annotations.
    ```bash
    conda create -n sam2 python=3.10
    conda activate sam2
    pip install -r sam2_requirements.txt
    ```
    