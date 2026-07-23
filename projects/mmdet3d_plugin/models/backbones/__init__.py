# ------------------------------------------------------------------------
# Copyright (c) 2022 megvii-model. All Rights Reserved.
# ------------------------------------------------------------------------
# Modified from DETR3D (https://github.com/WangYueFt/detr3d)
# Copyright (c) 2021 Wang, Yue
# ------------------------------------------------------------------------
from .vovnet import VoVNet
from .vovnetcp import VoVNetCP
try:
    from .eva_vit import EVAViT
except ImportError:
    # EVA-only dependencies (for example fvcore) are not required by the R50
    # CARLA training profile.
    EVAViT = None
