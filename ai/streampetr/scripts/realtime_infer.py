import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import numpy as np
import time
import os
import cv2
import torch
import open3d as o3d

from pyquaternion import Quaternion
from nuscenes.nuscenes import NuScenes
from nuscenes.utils.data_classes import LidarPointCloud

# logger and stream
trt_logger = trt.Logger(trt.Logger.WARNING)
cuda_stream = cuda.Stream()

# format output when print
np.set_printoptions(suppress=True, precision=6)


# ---- Some configs ----
CAMERA_CHANNELS = [
    'CAM_FRONT', 'CAM_FRONT_RIGHT', 'CAM_FRONT_LEFT', 
    'CAM_BACK', 'CAM_BACK_LEFT', 'CAM_BACK_RIGHT'
]

# MODE = "one-stage"
MODE = "two-stage"

MONOLITH_ONNX_PATH = "simplify_monolith.onnx"
MONOLITH_PRECISION = 'fp16'

BACKBONE_ONNX_PATH = "simplify_extract_img_feat.onnx"
DETECTOR_ONNX_PATH = "simplify_pts_head_memory.onnx"
BACKBONE_PRECISION = 'fp16'
DETECTOR_PRECISION = 'fp16'

WEIGHT_PATH = '../weights/'

NUSCENES_PATH = '../data/nuscenes'
NUSCENES_VER = 'v1.0-mini'

ENABLE_VISUALIZE = True


def inverse_sigmoid(x: np.ndarray, eps=1e-5):
    """Inverse function of sigmoid.

    Args:
        x (np.ndarray): The tensor to do the
            inverse.
        eps (float): EPS avoid numerical
            overflow. Defaults 1e-5.
    Returns:
        x_inverse (np.ndarray): The x has passed the inverse
            function of sigmoid, has same shape as input.
    """
    x = x.clip(min=0, max=1)
    x1 = x.clip(min=eps)
    x2 = (1 - x).clip(min=eps)
    return np.log(x1 / x2)


def build_transform_matrix(translation, rotation):
    """Helper to build 4x4 matrix"""
    mat = np.eye(4, dtype=np.float32)
    mat[:3, :3] = rotation.rotation_matrix
    mat[:3, 3] = translation
    return mat

def invert_transform_matrix(egopose):
    """ Compute the inverse transformation of a 4x4 egopose numpy matrix."""
    inverse_matrix = np.zeros((4, 4), dtype=np.float32)
    rotation = egopose[:3, :3]
    translation = egopose[:3, 3]
    inverse_matrix[:3, :3] = rotation.T
    inverse_matrix[:3, 3] = -np.dot(rotation.T, translation)
    inverse_matrix[3, 3] = 1.0
    return inverse_matrix


def transform_reference_points(reference_points: np.ndarray, egopose: np.ndarray, reverse=False, translation=True):
    reference_points = np.concatenate([reference_points, np.ones_like(reference_points[..., 0:1])], axis=-1)
    if reverse:
        matrix = np.linalg.inv(egopose)
    else:
        matrix = egopose.copy()
    if not translation:
        matrix[..., :3, 3] = 0.0
    reference_points = matrix[:, np.newaxis, ...] @ reference_points[..., np.newaxis]
    return reference_points.squeeze(-1)[..., :3]


def topk_gather_np(feat, topk_indexes):
    """
    NumPy equivalent of torch.gather for the topk dimension.
    Assumes feat is (B, N, ...) and topk_indexes is (B, K)
    """
    batch_idx = np.arange(feat.shape[0])[:, np.newaxis]
    return feat[batch_idx, topk_indexes]


def denormalize_bbox(normalized_bboxes: np.ndarray):
    # rotation 
    rot_sine = normalized_bboxes[..., 6:7]
    rot_cosine = normalized_bboxes[..., 7:8]
    rot = np.arctan2(rot_sine, rot_cosine)

    # size (w, l, h)
    s = normalized_bboxes[..., 3:6]
    s = np.exp(s)

    # center in the bev (x, y, z)
    c = normalized_bboxes[..., 0:3]

    if normalized_bboxes.shape[-1] > 8:
        # velocity (vx, vy)
        v = normalized_bboxes[..., 8:10]
        denormalized_bboxes = np.concatenate([c, s, rot, v], axis=-1)
    else:
        denormalized_bboxes = np.concatenate([c, s, rot], axis=-1)

    return denormalized_bboxes

def get_bbox_corners(bboxes):
    """
    bboxes: (N, 9) numpy array [cx, cy, cz, w, l, h, yaw, vx, vy]
    returns: (N, 8, 3) corners
    """
    if len(bboxes) == 0:
        return np.zeros((0, 8, 3))

    # 1. Define the 8 corners in local object space
    # (x_min, x_max) x (y_min, y_max) x (z_min, z_max)
    x_corners = np.array([-0.5, 0.5, 0.5, -0.5, -0.5, 0.5, 0.5, -0.5]) # [8]
    y_corners = np.array([-0.5, -0.5, 0.5, 0.5, -0.5, -0.5, 0.5, 0.5]) # [8]
    z_corners = np.array([-0.5, -0.5, -0.5, -0.5, 0.5, 0.5, 0.5, 0.5])                     # [8]

    # Scale corners by w, l, h
    corners = np.stack([x_corners, y_corners, z_corners], axis=0) # [3, 8]
    corners = corners[None, :, :] * bboxes[:, 3:6, None]         # [N, 3, 8]

    # 2. Rotate corners by yaw
    yaw = bboxes[:, 6]
    c, s = np.cos(yaw), np.sin(yaw)
    # Rotation matrix around Z-axis
    rot_mats = np.array([
        [c, -s, np.zeros_like(c)],
        [s,  c, np.zeros_like(c)],
        [np.zeros_like(c), np.zeros_like(c), np.ones_like(c)]
    ]).transpose(2, 0, 1) # [N, 3, 3]

    corners = rot_mats @ corners # [N, 3, 8]

    # 3. Translate to (cx, cy, cz)
    corners += bboxes[:, :3, None]
    
    return corners.transpose(0, 2, 1) # [N, 8, 3]


def prepare_location(input_shape: np.ndarray, stride: int):
    """
    Arguments:
        input_shape (np.ndarray): (B, N, C, H, W)
        stride (int): downsampling stride
    Returns:
        locations (np.ndarray): (H, W, 2)
    """

    B, N, C, H, W = input_shape
    feat_h, feat_w = H // stride, W // stride
    shifts_x = (np.arange(0, feat_w * stride, stride, dtype=np.float32) + stride // 2) / W
    shifts_y = (np.arange(0, feat_h * stride, stride, dtype=np.float32) + stride // 2) / H
    shift_x, shift_y = np.meshgrid(shifts_x, shifts_y)
    shift_x = shift_x.reshape(-1)
    shift_y = shift_y.reshape(-1)
    locations = np.stack((shift_x, shift_y), axis=1)
    locations = locations.reshape(1, feat_h, feat_w, 2)
    locations = np.tile(locations, (B * N, 1, 1, 1))
    
    return locations


def create_data_dict():
    data_dict = {
        # sensors' data
        "timestamp": None,
        "raw_img": None,
        "ego_pose": None,
        "ego_pose_inv": None,
        "lidar2ego": None,
        "lidar2img": None,
        "lidar_pc": None,
        # backbone's input
        "img": None,
        # detector's input
        "img_feats": None,
        "pos_embed": None,
        "cone": None,
        "memory_embedding": None,
        "memory_reference_point": None,
        "memory_timestamp": None,
        "memory_egopose": None,
        "memory_velo": None,
        # detector's output
        "all_cls_scores": None,
        "all_bbox_preds": None,
        "rec_reference_points": None,
        "rec_memory": None,
        "rec_velo": None,
        # bbox output
        "preds": None,
    }
    return data_dict


class StreamPETR:
    """
    Wrapper to run tensorrt version of StreamPETR

    This one is highly custom, please check carefully before modify anything

    NOTE: ONLY ACCEPT BATCH SHAPE OF ONE
    """

    def __init__(self, camera_cfgs=[]):
        # configs
        self.stride = 16
        self.input_shape = (1, 6, 3, 256, 704)  # (B, N, C, H, W)
        self.camera_cfgs = camera_cfgs
        self.weights_path = WEIGHT_PATH

        self.position_range = np.array([-61.2, -61.2, -10.0, 61.2, 61.2, 10.0], dtype=np.float32)
        self.depth_start = 1
        self.depth_num = 64
        self.LID = True

        if self.LID:
            index  = np.arange(start=0, stop=self.depth_num, dtype=np.float32)
            bin_size = (self.position_range[3] - self.depth_start) / (self.depth_num * (self.depth_num + 1))
            coords_d = self.depth_start + bin_size * index * (index + 1)
        else:
            index  = np.arange(start=0, stop=self.depth_num, dtype=np.float32)
            bin_size = (self.position_range[3] - self.depth_start) / self.depth_num
            coords_d = self.depth_start + bin_size * index

        self.coords_d = coords_d

        # static info to be calculated
        self.ida_mats = None
        self.preprocessed_intrinsic = None
        self.lidar2imgs = None

        self.static_pos_embed = None
        self.static_cone = None
    
        # networks
        if MODE == "one-stage":
            self.model = TensorRTBenchmark(MONOLITH_ONNX_PATH, precision=MONOLITH_PRECISION)
            assert self.model.build_engine(), "Failed to init detector engine"

        elif MODE == "two-stage":
            self.backbone = TensorRTBenchmark(BACKBONE_ONNX_PATH, precision=BACKBONE_PRECISION)
            assert self.backbone.build_engine(), "Failed to init backbone engine"

            self.pts_head = TensorRTBenchmark(DETECTOR_ONNX_PATH, precision=DETECTOR_PRECISION)
            assert self.pts_head.build_engine(), "Failed to init detector engine"

        # memory
        self.embed_dims = 256
        self.topk_proposals = 128
        self.num_propagated = 128
        self.memory_len = 512

        # decoder
        post_center_range=[-61.2, -61.2, -10.0, 61.2, 61.2, 10.0]
        threshold = 0.5
        self.bbox_coder = NMSFreeCoder(post_center_range, self.topk_proposals, threshold)
    

    def _calc_img_lidars_transform(self):
        preprocessed_intrinsic = []
        lidar2img_mats = []

        for (intrinsic, extrinsic), ida_mat in zip(self.camera_cfgs, self.ida_mats):
            # compensate for resize, crop
            compensated_intrinsic = ida_mat @ intrinsic
            preprocessed_intrinsic.append(compensated_intrinsic)

            # create 4x4 camera to image transformation matrix
            cam2img = np.eye(4)
            cam2img[:intrinsic.shape[0], :intrinsic.shape[1]] = compensated_intrinsic

            # lidar to cam is extrinsic 
            lidar2cam = extrinsic

            # get the lidar to image pixel matrix
            lidar2img = (cam2img @ lidar2cam)
            lidar2img_mats.append(lidar2img)

        # hardcoded for batch = 1, if change batch number, update this
        self.preprocessed_intrinsic = np.stack(preprocessed_intrinsic, axis=0)[np.newaxis, ...]
        self.lidar2imgs = np.stack(lidar2img_mats, axis=0)[np.newaxis, ...]


    def position_encoder_np(self, x):
        weights = {
            "w1": np.fromfile(self.weights_path + "position_encoder_w1.bin", dtype=np.float32).reshape(self.depth_num * 3, self.embed_dims * 4),
            "b1": np.fromfile(self.weights_path + "position_encoder_b1.bin", dtype=np.float32).reshape(self.embed_dims * 4),
            "w2": np.fromfile(self.weights_path + "position_encoder_w2.bin", dtype=np.float32).reshape(self.embed_dims * 4, self.embed_dims),
            "b2": np.fromfile(self.weights_path + "position_encoder_b2.bin", dtype=np.float32).reshape(self.embed_dims)
        }
        # Layer 1: Linear + ReLU
        x = np.matmul(x, weights['w1']) + weights['b1']
        x = np.maximum(0, x) 
        
        # Layer 2: Linear
        x = np.matmul(x, weights['w2']) + weights['b2']
        return x


    def position_embeding(self, memory_centers: np.ndarray):
        eps = 1e-5
        B, N, C, H, W = self.input_shape
        BN, mem_H, mem_W, _ = memory_centers.shape
        D = self.coords_d.shape[0]
    
        intrinsic = self.preprocessed_intrinsic
        intrinsic = np.stack([intrinsic[..., 0, 0], intrinsic[..., 1, 1]], axis=-1)
        intrinsic = np.abs(intrinsic) / 1e3
        intrinsic = np.tile(intrinsic, (1, mem_H * mem_W, 1)).reshape(B, -1, 2)
        LEN = intrinsic.shape[1]
    
        num_sample_tokens = LEN
    
        memory_centers[..., 0] *= W
        memory_centers[..., 1] *= H
        memory_centers = memory_centers.reshape(B, LEN, 1, 2)
        memory_centers = np.tile(memory_centers, (1, 1, D, 1))

        coords_d = self.coords_d.reshape(1, 1, D, 1)
        coords_d = np.tile(coords_d, (B, num_sample_tokens, 1, 1))
        coords = np.concatenate([memory_centers, coords_d], axis=-1)
        coords = np.concatenate((coords, np.ones_like(coords[..., :1])), axis=-1)
        coords[..., :2] *= np.maximum(coords[..., 2:3], eps)
        coords = coords[..., np.newaxis]
    
        img2lidars = np.linalg.inv(self.lidar2imgs)
        img2lidars = img2lidars.reshape(B * N, 1, 1, 4, 4)
        img2lidars = np.tile(img2lidars, (1, mem_H*mem_W, D, 1, 1)).reshape(B, LEN, D, 4, 4)
    
        coords3d = (img2lidars @ coords).squeeze(-1)[..., :3]
        coords3d[..., :3] = (coords3d[..., :3] - self.position_range[:3]) / (self.position_range[3:6] - self.position_range[:3])
        coords3d = coords3d.reshape(B, -1, D * 3)
    
        pos_embed = inverse_sigmoid(coords3d)
        coords_position_embeding = self.position_encoder_np(pos_embed)
    
        # for spatial alignment in focal petr
        cone = np.concatenate([intrinsic, coords3d[..., -3:], coords3d[..., -90:-87]], axis=-1)
    
        return coords_position_embeding, cone


    def _calc_frustum(self):
        self._calc_img_lidars_transform()
        locations = prepare_location(self.input_shape, self.stride)
        pos_embed, cone = self.position_embeding(locations)
        self.static_pos_embed = pos_embed
        self.static_cone = cone


    def image_preprocess(self, images:list):
        """
        Operations: Resize, Crop (bottom-center), Convert to RGB, Normalize, Reorder to CWH
        Also calculate IDA matrix on first run

        Arguments:
            images(list[np.ndarray]): list of N images in HWC order

        Returns:
            processed_images(np.ndarray): images after being preprocessed
        """

        B, N, C, H, W = self.input_shape
        ida_mats = []
        preprocessed_images = []
        for img in images:
            img_H, img_W, img_C = img.shape

            # resize
            scale_factor = max(H / img_H, W / img_W)
            resize_H, resize_W = (int(img_H * scale_factor), int(img_W * scale_factor))
            img = cv2.resize(img, (resize_W, resize_H), interpolation=cv2.INTER_LINEAR)

            # crop (bottom center)
            crop_y = resize_H - H
            crop_x = (resize_W - W) // 2
            img = img[crop_y:crop_y+H, crop_x:crop_x+W]

            # convert to RGB
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            
            # normalize
            mean = np.array([123.675, 116.28, 103.53], dtype=np.float32)
            std = np.array([58.395, 57.12, 57.375], dtype=np.float32)
            img = img.astype(np.float32)
            img = (img - mean) / std

            # HWC to CHW
            img = img.transpose(2, 0, 1)

            # store preprocessed image
            preprocessed_images.append(img)

            # store IDA matrix (first run only)
            if self.ida_mats is None:
                ida_mat = np.eye(3)
                ida_mat[:2, :2] *= scale_factor
                ida_mat[:2, 2] = (-crop_x, -crop_y)
                ida_mats.append(ida_mat)
        
        if self.ida_mats is None:
            self.ida_mats = ida_mats

        preprocessed_images = np.stack(preprocessed_images, axis=0)
        
        return preprocessed_images


    def preprocess(self, data: dict):
        # hardcoded for batch = 1, if change batch number, update this
        raw_images = data["raw_img"][0]
        data["img"] = self.image_preprocess(raw_images)[np.newaxis, ...]

        # calculate the frustum (first run)
        if self.static_pos_embed is None or self.static_cone is None:
            self._calc_frustum()

        data["pos_embed"] = self.static_pos_embed
        data["cone"] = self.static_cone

        return data


    def init_memory(self, timestamp, ego_pose):
        # zero init
        B = self.input_shape[0]
        self.memory_embedding = np.zeros((B, self.memory_len, self.embed_dims), dtype=np.float32)
        self.memory_reference_point = np.zeros((B, self.memory_len, 3), dtype=np.float32)
        self.memory_egopose = np.zeros((B, self.memory_len, 4, 4), dtype=np.float32)
        self.memory_velo = np.zeros((B, self.memory_len, 2), dtype=np.float32)

        # init timestamp
        self.memory_timestamp = np.full((B, self.memory_len, 1), fill_value=timestamp, dtype=np.float64)

        # init reference point
        pseudo_reference_points = np.fromfile(self.weights_path + "pseudo_reference_points.bin", dtype=np.float32)
        pseudo_reference_points = pseudo_reference_points.reshape(self.num_propagated, 3)
        self.memory_reference_point[:, :self.num_propagated] = pseudo_reference_points

        # init ego pose
        self.memory_egopose[:, :self.num_propagated] = np.eye(4)
    
        # transform back to global
        self.memory_reference_point = transform_reference_points(self.memory_reference_point, ego_pose)
        self.memory_egopose = ego_pose[:, np.newaxis, ...] @ self.memory_egopose

    def read_memory(self, data: dict):
        """
        Get memory data.
        Perform conversion from global to ego.
        """

        timestamp = data["timestamp"]
        ego_pose_inv = data["ego_pose_inv"]

        if not hasattr(self, 'memory_timestamp') or self.memory_timestamp is None:
            ego_pose = data["ego_pose"]
            self.init_memory(timestamp, ego_pose)

        # convert clock time to time delta
        memory_timestamp = timestamp - self.memory_timestamp

        # convert global to ego coord
        memory_egopose = ego_pose_inv[:, np.newaxis] @ self.memory_egopose
        memory_reference_point = transform_reference_points(self.memory_reference_point, ego_pose_inv, reverse=False)

        memory_embedding = self.memory_embedding
        memory_velo = self.memory_velo

        # assign data
        data["memory_embedding"] = memory_embedding
        data["memory_reference_point"] = memory_reference_point
        data["memory_timestamp"] = memory_timestamp
        data["memory_egopose"] = memory_egopose
        data["memory_velo"] = memory_velo

        return data


    def update_memory(self, data: dict):
        timestamp = data["timestamp"]
        ego_pose = data["ego_pose"]
        rec_reference_points = data["rec_reference_points"]
        rec_memory = data["rec_memory"]
        rec_velo = data["rec_velo"]

        B, topk = rec_velo.shape[:2]

        # convert from ego to global coord
        rec_reference_points = transform_reference_points(rec_reference_points, ego_pose, reverse=False)

        # create timestamp and ego pose for new memory entries
        rec_ego_pose = np.broadcast_to(ego_pose[:, np.newaxis], (B, topk, 4, 4))
        rec_timestamp = np.full((B, topk, 1), fill_value=timestamp, dtype=np.float64)

        # insert new topk memory
        self.memory_embedding = np.concatenate([rec_memory, self.memory_embedding], axis=1)
        self.memory_timestamp = np.concatenate([rec_timestamp, self.memory_timestamp], axis=1)
        self.memory_egopose = np.concatenate([rec_ego_pose, self.memory_egopose], axis=1)
        self.memory_reference_point = np.concatenate([rec_reference_points, self.memory_reference_point], axis=1)
        self.memory_velo = np.concatenate([rec_velo, self.memory_velo], axis=1)

        # trim entries of old memory
        self.memory_embedding = self.memory_embedding[:, :self.memory_len]
        self.memory_timestamp = self.memory_timestamp[:, :self.memory_len]
        self.memory_egopose = self.memory_egopose[:, :self.memory_len]
        self.memory_reference_point = self.memory_reference_point[:, :self.memory_len]
        self.memory_velo = self.memory_velo[:, :self.memory_len]


    def infer_two_stage(self, data: dict):
        if not hasattr(self, "average_total"):
            self.average_total = 0
            self.average_pre = 0
            self.average_backbone = 0
            self.average_memread = 0
            self.average_detector = 0
            self.average_memupdate = 0
            self.average_decoder = 0

        start_cp = time.time()
        # preprocess
        data = self.preprocess(data)
        pre_cp = time.time()

        # stage 1: back bone
        backbone_output = self.backbone.infer(data)
        backbone_cp = time.time()
        data.update(backbone_output)

        # stage 2: transformer detector
        data = self.read_memory(data)
        read_cp = time.time()
        detector_output = self.pts_head.infer(data)
        detector_cp = time.time()
        data.update(detector_output)
        self.update_memory(data)
        update_cp = time.time()
        
        # postprocess
        preds_list = self.bbox_coder.decode(data)
        data['preds'] = preds_list
        post_cp = time.time()

        
        self.average_total = 0.8 * self.average_total + 0.2 * (post_cp - start_cp)
        self.average_pre = 0.8 * self.average_pre + 0.2 * (pre_cp - start_cp)
        self.average_backbone = 0.8 * self.average_backbone + 0.2 * (backbone_cp - pre_cp)
        self.average_memread = 0.8 * self.average_memread + 0.2 * (read_cp - backbone_cp)
        self.average_detector = 0.8 * self.average_detector + 0.2 * (detector_cp - read_cp)
        self.average_memupdate = 0.8 * self.average_memupdate + 0.2 * (update_cp - detector_cp)
        self.average_decoder = 0.8 * self.average_decoder + 0.2 * (post_cp - update_cp)

        print(f"""
              total:     {self.average_total * 1e3: 3.3f}
              pre:       {self.average_pre * 1e3: 3.3f}
              backbone:  {self.average_backbone * 1e3: 3.3f}
              memread:   {self.average_memread * 1e3: 3.3f}
              detector:  {self.average_detector * 1e3: 3.3f}
              memupdate: {self.average_memupdate * 1e3: 3.3f}
              decoder:   {self.average_decoder * 1e3: 3.3f}
              """)

        return data

    def infer_one_stage(self, data: dict):
        if not hasattr(self, "average_total"):
            self.average_total = 0
            self.average_pre = 0
            self.average_detector = 0
            self.average_memupdate = 0
            self.average_decoder = 0

        start_cp = time.time()
        # preprocess
        data = self.preprocess(data)
        data = self.read_memory(data)
        pre_cp = time.time()

        # transformer detector
        detector_output = self.model.infer(data)
        detector_cp = time.time()
        data.update(detector_output)
        self.update_memory(data)
        update_cp = time.time()
        
        # postprocess
        preds_list = self.bbox_coder.decode(data)
        data['preds'] = preds_list
        post_cp = time.time()

        
        self.average_total = 0.8 * self.average_total + 0.2 * (post_cp - start_cp)
        self.average_pre = 0.8 * self.average_pre + 0.2 * (pre_cp - start_cp)
        self.average_detector = 0.8 * self.average_detector + 0.2 * (detector_cp - pre_cp)
        self.average_memupdate = 0.8 * self.average_memupdate + 0.2 * (update_cp - detector_cp)
        self.average_decoder = 0.8 * self.average_decoder + 0.2 * (post_cp - update_cp)

        print(f"""
              total:     {self.average_total * 1e3: 3.3f}
              pre:       {self.average_pre * 1e3: 3.3f}
              detector:  {self.average_detector * 1e3: 3.3f}
              memupdate: {self.average_memupdate * 1e3: 3.3f}
              decoder:   {self.average_decoder * 1e3: 3.3f}
              """)

        return data

    def infer(self, data: dict):
        if MODE == "one-stage":
            return self.infer_one_stage(data)
        elif MODE == "two-stage":
            return self.infer_two_stage(data)


class NUScenes_reader:
    def __init__(self, version=NUSCENES_VER):
        self.nusc = NuScenes(version=version, dataroot=NUSCENES_PATH, verbose=False)

        self.CAMERA_CHANNELS = CAMERA_CHANNELS
        self.camera_cfgs = None
        self.lidar2imgs = None
        
        assert self.select_scene(0)
        self._calc_camera_calibration()


    def select_scene(self, target_scene):
        scene_changed = False
        if isinstance(target_scene, str):
            for scene in self.nusc.scene:
                if target_scene in scene['name']:
                    self.scene = scene
                    scene_changed = True
                    break
        else:
            self.scene = self.nusc.scene[target_scene]
            scene_changed = True

        if scene_changed:
            self.next_sample_token = self.scene['first_sample_token']
            print(f"Changed to scene {self.scene['name']}!!!")
            return True
        
        print(f"Failed to change scene!!!")
        return False


    def _calc_camera_calibration(self, sample_token = None):
        # 0. Default to next sample token
        if sample_token is None:
            sample_token = self.next_sample_token

        # 1. Get the sample
        sample_rec = self.nusc.get('sample', sample_token)

        # 2. Get lidar calib
        # Pull lidar data
        lidar_token = sample_rec['data']['LIDAR_TOP']
        lidar_data = self.nusc.get('sample_data', lidar_token)

        # Get lidar2ego (calib)
        lidar_rec = self.nusc.get('calibrated_sensor', lidar_data['calibrated_sensor_token'])
        lidar2ego = build_transform_matrix(lidar_rec['translation'], Quaternion(lidar_rec['rotation']))

        # 3. Calculate intrinsic and extrinsic of each camera
        camera_calibs = []
        for cam_channel in self.CAMERA_CHANNELS:
            # Pull camera data
            cam_token = sample_rec['data'][cam_channel]
            cam_data = self.nusc.get('sample_data', cam_token)

            # Get intrinsic and cam2ego matrix (calib)
            calib_rec = self.nusc.get('calibrated_sensor', cam_data['calibrated_sensor_token'])
            intrinsic = np.array(calib_rec['camera_intrinsic'])
            cam2ego = build_transform_matrix(calib_rec['translation'], Quaternion(calib_rec['rotation']))

            # Calculate lidar2cam
            ego2cam = invert_transform_matrix(cam2ego)
            lidar2cam = ego2cam @ lidar2ego

            extrinsic = lidar2cam
            
            camera_calibs.append((intrinsic, extrinsic))

        self.camera_cfgs = camera_calibs
        
        # 4. Calculate lidar to images transform matrix
        lidar2imgs = []
        for intrinsic, extrinsic in self.camera_cfgs:
            # create 4x4 camera to image transformation matrix
            cam2img = np.eye(4)
            cam2img[:intrinsic.shape[0], :intrinsic.shape[1]] = intrinsic

            # lidar to cam is extrinsic 
            lidar2cam = extrinsic

            # get the lidar to image pixel matrix
            lidar2img = (cam2img @ lidar2cam)
            lidar2imgs.append(lidar2img)

        self.lidar2imgs = lidar2imgs

        return camera_calibs


    def get_lidar_to_global_matrix(self, sample_token):
        sample_rec = self.nusc.get('sample', sample_token)
        
        # 1. Get the Lidar Sample Data
        lidar_token = sample_rec['data']['LIDAR_TOP']
        lidar_data = self.nusc.get('sample_data', lidar_token)
        
        # 2. Get Lidar to Ego (Calibration)
        l2e_rec = self.nusc.get('calibrated_sensor', lidar_data['calibrated_sensor_token'])
        l2e_matrix = build_transform_matrix(l2e_rec['translation'], Quaternion(l2e_rec['rotation']))
        
        # 3. Get Ego to Global (Pose)
        e2g_rec = self.nusc.get('ego_pose', lidar_data['ego_pose_token'])
        e2g_matrix = build_transform_matrix(e2g_rec['translation'], Quaternion(e2g_rec['rotation']))
        
        # 4. Final Lidar-to-Global Matrix
        l2g_matrix = e2g_matrix @ l2e_matrix

        return l2g_matrix, l2e_matrix


    def get_lidar_points(self, sample_token):
        # 1. Get the sample record
        sample_rec = self.nusc.get('sample', sample_token)
        
        # 2. Get the lidar data token
        lidar_token = sample_rec['data']['LIDAR_TOP']
        lidar_record = self.nusc.get('sample_data', lidar_token)
        
        # 3. Get the absolute path to the .bin file
        pcl_path = os.path.join(self.nusc.dataroot, lidar_record['filename'])
        
        # 4. Use nuScenes util to load the point cloud
        # nusc_pc.points is shape (4, N)
        nusc_pc = LidarPointCloud.from_file(str(pcl_path))
        points = nusc_pc.points.T.astype(np.float32) # to (N, 4)
        
        return points


    def get_sample_data(self, sample_token):
        # 1. Get sample
        sample_rec = self.nusc.get('sample', sample_token)
        
        # 2. Load Images from 6 Cameras
        image_list = []
        for cam_channel in self.CAMERA_CHANNELS:
            cam_token = sample_rec['data'][cam_channel]
            cam_data = self.nusc.get('sample_data', cam_token)
            
            # Get absolute file path
            im_path = os.path.join(self.nusc.dataroot, cam_data['filename'])
            
            # Load image (OpenCV loads as BGR)
            img = cv2.imread(im_path)
            image_list.append(img)

        # 3. Get Timestamp
        timestamp = sample_rec['timestamp']

        # 4. Get Ego pose
        ego_pose, l2e_mat = self.get_lidar_to_global_matrix(sample_token)

        # 5. Get lidar scan
        lidar_pc = self.get_lidar_points(sample_token)
        
        return image_list, ego_pose, l2e_mat, timestamp, lidar_pc


    def __iter__(self):
        return self

    def __next__(self):
        if self.next_sample_token == "":
            raise StopIteration
        
        # pull data
        image_list, ego_pose, l2e_mat, timestamp, lidar_pc = self.get_sample_data(self.next_sample_token)

        # create data dict
        data = create_data_dict()
        data["timestamp"] = timestamp / 1e6
        data["raw_img"] = [image_list]
        data["ego_pose"] = ego_pose[np.newaxis, ...]
        data["ego_pose_inv"] = invert_transform_matrix(ego_pose)[np.newaxis, ...]
        data["lidar2ego"] = l2e_mat[np.newaxis, ...]
        data["lidar2img"] = np.stack(self.lidar2imgs, axis=0)[np.newaxis, ...]
        data["lidar_pc"] = lidar_pc[np.newaxis, ...]
        
        # advance sample token
        sample_rec = self.nusc.get('sample', self.next_sample_token)
        self.next_sample_token = sample_rec['next']

        return data


class NMSFreeCoder:
    """Bbox coder for NMS-free detector.
    Args:
        post_center_range (list[float]): Limit of the center.
            Default: None.
        topk (int): Max number to be kept. Default: None (keep all).
        score_threshold (float): Threshold to filter boxes based on score.
            Default: None.
    """

    def __init__(self,
                 post_center_range=None,
                 topk=None,
                 score_threshold=None,):

        self.post_center_range = post_center_range
        self.topk = topk
        self.score_threshold = score_threshold

    def encode(self):
        pass

    def decode_single(self, cls_scores: np.ndarray, bbox_preds: np.ndarray):
        """Decode bboxes.
        Args:
            cls_scores (Tensor): Outputs from the classification head, \
                shape [num_query, cls_out_channels]. Note \
                cls_out_channels should includes background.
            bbox_preds (Tensor): Outputs from the regression \
                head with normalized coordinate format (cx, cy, cz, w, l, h, rot_sine, rot_cosine, vx, vy). \
                Shape [num_query, 10].
        Returns:
            list[dict]: Decoded boxes.
        """

        # get class of each query
        labels = np.argmax(cls_scores, axis=1)
        scores = np.max(cls_scores, axis=1)

        # filter topk results
        if self.topk is not None:
            idx = np.argsort(scores)[-1:-self.topk-1:-1]
            scores = scores[idx]
            labels = labels[idx]
            bbox_preds = bbox_preds[idx]

        # filter with score threshold
        if self.score_threshold is not None:
            mask = scores >= self.score_threshold
            scores = scores[mask]
            labels = labels[mask]
            bbox_preds = bbox_preds[mask]

        final_box_preds = denormalize_bbox(bbox_preds)
        final_scores = scores
        final_labels = labels

        if self.post_center_range is not None:
            mask = np.all((final_box_preds[..., :3] >= self.post_center_range[:3]), axis=1)
            mask &= np.all((final_box_preds[..., :3] <= self.post_center_range[3:]), axis=1)

            final_box_preds = final_box_preds[mask]
            final_scores = final_scores[mask]
            final_labels = final_labels[mask]
            predictions_dict = {
                'bboxes': final_box_preds,
                'scores': final_scores,
                'labels': final_labels
            }

        else:
            raise NotImplementedError(
                'Need to reorganize output as a batch, '
                'only support post_center_range is not None for now!')

        return predictions_dict

    def decode(self, preds_dicts):
        """Decode bboxes.
        Args:
            all_cls_scores (Tensor): Outputs from the classification head, \
                shape [nb_dec, bs, num_query, cls_out_channels]. Note \
                cls_out_channels should includes background.
            all_bbox_preds (Tensor): Sigmoid outputs from the regression \
                head with normalized coordinate format (cx, cy, cz, w, l, h, rot_sine, rot_cosine, vx, vy). \
                Shape [nb_dec, bs, num_query, 10].
        Returns:
            list[dict]: Decoded boxes.
        """
        all_cls_scores = preds_dicts['all_cls_scores'][-1]
        all_bbox_preds = preds_dicts['all_bbox_preds'][-1]
        
        batch_size = all_cls_scores.shape[0]
        predictions_list = []
        for i in range(batch_size):
            predictions_list.append(self.decode_single(all_cls_scores[i], all_bbox_preds[i]))

        return predictions_list


class LiveVisualizer:
    def __init__(self):
        # 1. Create windows
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="Live BEV Detection", width=480, height=480)
        self.first_frame = True

        # 2. Add a Ground Grid
        self.grid = self._create_grid(size=100, step=10) # 100m grid
        self.vis.add_geometry(self.grid)
        self.vis.reset_view_point(True)

        # 3. Initialize an empty point cloud object
        self.pcd = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.pcd)
        
        # 4. Others variables
        # Track active box geometries to remove them later
        self.active_boxes = []

        # Drawing orders
        self.cam_order = CAMERA_CHANNELS
        self.stitch_order = [
            ['CAM_FRONT_LEFT', 'CAM_FRONT', 'CAM_FRONT_RIGHT'],  # front
            ['CAM_BACK_RIGHT', 'CAM_BACK', 'CAM_BACK_LEFT']      # back
        ]
        self.box_edges = [
            [0,1], [1,2], [2,3], [3,0],  # bottom face
            [4,5], [5,6], [6,7], [7,4],  # top face
            [0,4], [1,5], [2,6], [3,7]   # pillars
        ]

    def _create_grid(self, size=100, step=10):
        """Creates a lineset representing a grid on the XY plane."""
        lines = []
        points = []
        # Create horizontal and vertical lines
        for i in range(-size, size + step, step):
            points.append([i, -size, 0])
            points.append([i, size, 0])
            lines.append([len(points)-2, len(points)-1])
            
            points.append([-size, i, 0])
            points.append([size, i, 0])
            lines.append([len(points)-2, len(points)-1])
            
        ls = o3d.geometry.LineSet()
        ls.points = o3d.utility.Vector3dVector(np.array(points))
        ls.lines = o3d.utility.Vector2iVector(np.array(lines))
        ls.paint_uniform_color([0.5, 0.5, 0.5]) # Grey grid
        return ls

    def draw_bboxes_on_img(self, img, bboxes):
        """
        img: The OpenCV image (H, W, 3)
        pixels: [N, 8, 2] Projected pixel coordinates for N boxes
        """
        color=(0, 255, 0)
        thickness=2

        for box_points in bboxes:
            
            # Draw the 12 edges
            for start, end in self.box_edges:
                cv2.line(img, tuple(box_points[start]), tuple(box_points[end]), color, thickness)
            
            # nuScenes front is usually the 0-1-5-4 face or 1-2-6-5 depending on your get_corners logic
            # Let's assume 0-1-5-4 is the front:
            front_mid_top = (box_points[5] + box_points[6]) // 2
            front_mid_bottom = (box_points[1] + box_points[2]) // 2
            cv2.line(img, tuple(front_mid_top), tuple(front_mid_bottom), (0, 0, 255), thickness)

        return img

    def project_to_image(self, images, bbox3d, l2i_mat):
        """
        corners_3d: [N, 8, 3]
        lidar2img_rt: [4, 4] (Projection matrix including intrinsic and extrinsic)
        """

        N = bbox3d.shape[0]

        if N <= 0:
            return images
        
        # Add homogeneous coordinate: [N, 8, 4]
        bbox3d_padded = np.concatenate([bbox3d, np.ones((N, 8, 1))], axis=-1)

        # Loop through each camera channel
        visualized_images = []
        for img, l2i in zip(images, l2i_mat):
            # Project: [N, 8, 4] @ [4, 4].T -> [N, 8, 4]
            bbox2d = bbox3d_padded @ l2i.T
            
            # 1. Depth Filter: The depth (3rd value) of all points must be positive
            depths = bbox2d[..., 2:3]
            is_infront = np.all(depths > 0.1, axis=1).reshape(-1) # [N]
            bbox2d = bbox2d[is_infront]
            depths = depths[is_infront]
            
            # 2. Perspective Division
            bbox2d = bbox2d[..., :2] / depths  # [N, 8, 2]
            
            # 3. Boundary Filter: Any point of bbox is inside the image?
            img_h, img_w, img_c = img.shape
            point_inview = (0 <= bbox2d[..., 0]) & (bbox2d[..., 0] < img_w) & \
                    (0 <= bbox2d[..., 1]) & (bbox2d[..., 1] < img_h)
            is_visible = np.any(point_inview, axis=1).reshape(-1)
            bbox2d = bbox2d[is_visible].astype(np.int32)

            viz_img = self.draw_bboxes_on_img(img, bbox2d)
            
            visualized_images.append(viz_img)
        
        return visualized_images

    def stitch_images(self, images: list):
        """
        Arguments:
            images(list): list of visualized images
        Returns:
            combined_image(np.ndarray):
        """
        # Resize to avoid big images
        resized = [cv2.resize(img, (0, 0), fx=(1/4), fy=(1/4)) for img in images]
        
        top_row = np.hstack([resized[self.cam_order.index(channel)] for channel in self.stitch_order[0]])
        bottom_row = np.hstack([resized[self.cam_order.index(channel)] for channel in self.stitch_order[1]])
        
        combined = np.vstack([top_row, bottom_row])
        return combined

    def update(self, data: dict):
        """
        bboxes: (N, 9) numpy array [cx, cy, cz, w, l, h, yaw, vx, vy]
        points: (M, 3) numpy array
        """
            
        # NOTE: Only visualize first sample in batch
        # get raw images
        raw_imgs = output["raw_img"][0]
        
        # # get lidar point cloud
        # lidar_pc = output["lidar_pc"][0]
        # lidar_pc = lidar_pc[::2]  # throw half away to lower demand
        lidar_pc = None

        # grab prediction
        preds = output["preds"][0]
        bboxes = preds["bboxes"]
        scores = preds["scores"]
        labels = preds["labels"]

        # get some configs
        l2e_mat = output["lidar2ego"][0]
        l2i_mat = output["lidar2img"][0]

        # 1. Update Point Cloud
        if lidar_pc is not None and len(lidar_pc) > 0:
            num_points = lidar_pc.shape[0]
            if l2e_mat is not None:
                ones_padded = np.hstack((lidar_pc[:, :3], np.ones((num_points, 1))))
                transformed_points = (l2e_mat @ ones_padded.T).T
                lidar_pc[:, :3] = transformed_points[:, :3]

            self.pcd.points = o3d.utility.Vector3dVector(lidar_pc[:, :3])
            
            # Paint points based on distance
            distance = np.linalg.norm(lidar_pc[:, :3], axis=1)
            color_shift = np.clip(distance / 40, 0, 1)
            colors = np.zeros((num_points, 3))
            colors[:, 0] = 1 - color_shift
            colors[:, 1] = 1 - np.abs(color_shift * 2 - 1)
            colors[:, 2] = color_shift
            self.pcd.colors = o3d.utility.Vector3dVector(colors)
        else:
            # Properly clear the point cloud data
            self.pcd.points = o3d.utility.Vector3dVector(np.zeros((0, 3)))
            self.pcd.colors = o3d.utility.Vector3dVector(np.zeros((0, 3)))
        
        self.vis.update_geometry(self.pcd)

        # 2. Update bboxes
        # Clear old boxes
        for box in self.active_boxes:
            self.vis.remove_geometry(box, reset_bounding_box=False)
        self.active_boxes = []

        # Add new boxes
        if bboxes is not None:
            corners = get_bbox_corners(bboxes)  # convert bbox to 3D corner points

            # Update 2D view
            visualized_images = self.project_to_image(raw_imgs, corners, l2i_mat)
            combined_images = self.stitch_images(visualized_images)
            cv2.imshow('2D_view', combined_images)

            if len(bboxes) > 0:
                # Update 3D view
                if l2e_mat is not None:
                    corners_points = corners.reshape(-1, 3)
                    ones_padded = np.hstack((corners_points, np.ones_like(corners_points[:, 0:1])))
                    transformed_points = (l2e_mat @ ones_padded.T).T
                    corners = transformed_points[:, :3].reshape(corners.shape)
                
                for i in range(len(corners)):
                    # Define lines for the wireframe box
                    line_set = o3d.geometry.LineSet()
                    line_set.points = o3d.utility.Vector3dVector(corners[i])
                    line_set.lines = o3d.utility.Vector2iVector(self.box_edges)
                    
                    # Color code based on label (e.g., Red for Car, Green for Pedestrian)
                    color = [1, 0, 0] if (labels is not None and labels[i] == 0) else [0, 1, 0]
                    line_set.paint_uniform_color(color)
                    
                    self.vis.add_geometry(line_set, reset_bounding_box=False)
                    self.active_boxes.append(line_set)

        # 3. Reset view first frame
        if self.first_frame and ((lidar_pc is not None and len(lidar_pc) > 0) or (bboxes is not None and len(bboxes) > 0)):
            # This centers the camera on the data and sets a reasonable distance
            self.vis.reset_view_point(True)
            self.first_frame = False

        # 4. Handle events and render
        self.vis.poll_events()
        self.vis.update_renderer()
        cv2.pollKey()

    def is_active(self):
        return self.vis.poll_events()

    def close(self):
        self.vis.destroy_window()


class TensorRTBenchmark:
    def __init__(self, onnx_path, engine_path=None, precision="fp32", logger=None, stream=None):
        self.trt_logger = logger if logger is not None else trt.Logger(trt.Logger.WARNING)
        self.onnx_path = onnx_path
        self.engine_path = engine_path
        self.precision = precision
        self.engine = None
        self.context = None
        self.inputs = []
        self.outputs = []
        self.allocations = []
        self.stream = stream if stream is not None else cuda.Stream()

        # Check hardware support
        builder = trt.Builder(self.trt_logger)
        if self.precision == "int8" and not builder.platform_has_fast_int8:
            self.precision = "fp16"
            print("INT8 not supported on this GPU, falling back to FP16.")
        if self.precision == "fp16" and not builder.platform_has_fast_fp16:
            self.precision = "fp32"
            print("FP16 not supported on this GPU, falling back to FP32.")

        # create default engine path if none is given
        if self.engine_path is None:
            self.engine_path = self.onnx_path + "_" + self.precision + ".engine"

    def _layernorm_force_fp32(self, network):
        # Find Pow node (indicate the start of LayerNorm)
        for i in range(network.num_layers):
            pow_layer = network.get_layer(i)
            if pow_layer.type != trt.LayerType.ELEMENTWISE:
                continue

            pow_layer.__class__ = trt.IElementWiseLayer
            if pow_layer.op != trt.ElementWiseOperation.POW:
                continue

            # Find downstream reduce layer
            reduce_layer = None
            for j in range(network.num_layers):
                pow_output = pow_layer.get_output(0)
                next_layer = network.get_layer(j)
                next_layer_inputs = [next_layer.get_input(id) for id in range(next_layer.num_inputs)]

                # check if use pow output
                if not pow_output in next_layer_inputs:
                    continue

                if next_layer.type != trt.LayerType.REDUCE:
                    continue

                reduce_layer = next_layer
                break

            if reduce_layer is None:
                continue

            # Find downstream add layer
            add_layer = None
            for j in range(network.num_layers):
                reduce_output = reduce_layer.get_output(0)
                next_layer = network.get_layer(j)
                next_layer_inputs = [next_layer.get_input(id) for id in range(next_layer.num_inputs)]

                # check if use reduce output
                if not reduce_output in next_layer_inputs:
                    continue

                if next_layer.type != trt.LayerType.ELEMENTWISE:
                    continue

                next_layer.__class__ = trt.IElementWiseLayer
                if next_layer.op != trt.ElementWiseOperation.SUM:
                    continue

                add_layer = next_layer
                break

            if add_layer is None:
                continue

            # Find downstream sqrt layer
            sqrt_layer = None
            for j in range(network.num_layers):
                add_output = add_layer.get_output(0)
                next_layer = network.get_layer(j)
                next_layer_inputs = [next_layer.get_input(id) for id in range(next_layer.num_inputs)]

                # check if use add output
                if not add_output in next_layer_inputs:
                    continue

                if next_layer.type != trt.LayerType.UNARY:
                    continue

                next_layer.__class__ = trt.IUnaryLayer
                if next_layer.op != trt.UnaryOperation.SQRT:
                    continue

                sqrt_layer = next_layer
                break

            if sqrt_layer is None:
                continue

            # Find downstream div layer
            div_layer = None
            for j in range(network.num_layers):
                sqrt_output = sqrt_layer.get_output(0)
                next_layer = network.get_layer(j)
                next_layer_inputs = [next_layer.get_input(id) for id in range(next_layer.num_inputs)]

                # check if use sqrt output
                if not sqrt_output in next_layer_inputs:
                    continue

                if next_layer.type != trt.LayerType.ELEMENTWISE:
                    continue

                next_layer.__class__ = trt.IElementWiseLayer
                if next_layer.op != trt.ElementWiseOperation.DIV:
                    continue

                div_layer = next_layer
                break

            if div_layer is None:
                continue
            
            # Force the layer to compute and output in FP32
            for layer in [pow_layer, reduce_layer, add_layer, sqrt_layer, div_layer]:
                layer.precision = trt.DataType.FLOAT
                layer.set_output_type(0, trt.DataType.FLOAT)

            print(f"Forced {pow_layer.name} and subsequences to FP32")

    def build_engine(self):
        """Builds a TensorRT engine from an ONNX file."""
        if os.path.exists(self.engine_path):
            print(f"Loading existing engine from {self.engine_path}...")
            with open(self.engine_path, "rb") as f, trt.Runtime(self.trt_logger) as runtime:
                self.engine = runtime.deserialize_cuda_engine(f.read())
        else:
            print(f"Building engine from {self.onnx_path}...")
            builder = trt.Builder(self.trt_logger)
            network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
            parser = trt.OnnxParser(network, self.trt_logger)
            
            with open(self.onnx_path, 'rb') as model:
                if not parser.parse(model.read()):
                    for error in range(parser.num_errors):
                        print(parser.get_error(error))
                    return False

            config = builder.create_builder_config()
            # Set workspace limit (e.g., 1GB)
            config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 2 << 30)

            # if self.precision == "int8":
            #     config.set_flag(trt.BuilderFlag.INT8)

            if self.precision == "fp16":
                config.set_flag(trt.BuilderFlag.FP16)

                # Force FP32 for LayerNorm layers to avoid losing accuracy
                self._layernorm_force_fp32(network)
                config.set_flag(trt.BuilderFlag.OBEY_PRECISION_CONSTRAINTS)

            serialized_engine = builder.build_serialized_network(network, config)
            with open(self.engine_path, "wb") as f:
                f.write(serialized_engine)
            
            runtime = trt.Runtime(self.trt_logger)
            self.engine = runtime.deserialize_cuda_engine(serialized_engine)

        self.context = self.engine.create_execution_context()
        self._allocate_buffers()
        return True

    def _allocate_buffers(self):
        """Allocates memory on GPU for inputs and outputs."""
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            shape = self.engine.get_tensor_shape(name)
            size = trt.volume(shape)
            
            # Host (CPU) and Device (GPU) memory
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)
            
            self.allocations.append(int(device_mem))
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.inputs.append({'host': host_mem, 'device': device_mem, 'name': name, 'shape': shape})
            else:
                self.outputs.append({'host': host_mem, 'device': device_mem, 'name': name, 'shape': shape})

    def run_benchmark(self, iterations=100, warmup=10):
        print(f"Starting benchmark for {iterations} iterations...")
        
        # 1. Prepare random input
        for inp in self.inputs:
            np.copyto(inp['host'], np.random.random(inp['host'].shape).astype(inp['host'].dtype))

        # 2. Warmup
        for _ in range(warmup):
            self.infer()

        # 3. Timed Loop
        cuda.Context.synchronize()
        start_time = time.time()
        for _ in range(iterations):
            self.infer()
        cuda.Context.synchronize()
        end_time = time.time()

        total_time = end_time - start_time
        avg_latency = (total_time / iterations) * 1000
        fps = iterations / total_time
        
        print("-" * 30)
        print(f"Average Latency: {avg_latency:.3f} ms")
        print(f"Average FPS: {fps:.2f}")
        print("-" * 30)

    def infer(self, input_dict:dict=None):
        """Single inference step."""
        # Assign inputs if given
        if input_dict is not None:
            for input in self.inputs:
                input_name = input['name']
                np.copyto(input['host'], input_dict[input_name].flat)

        # Host to Device
        for inp in self.inputs:
            cuda.memcpy_htod_async(inp['device'], inp['host'], self.stream)
        
        # Set tensor addresses
        for i, addr in enumerate(self.allocations):
            self.context.set_tensor_address(self.engine.get_tensor_name(i), addr)
            
        # Execute
        self.context.execute_async_v3(self.stream.handle)
        
        # Device to Host
        for out in self.outputs:
            cuda.memcpy_dtoh_async(out['host'], out['device'], self.stream)
        
        self.stream.synchronize()

        # Return output
        output_dict = {}
        for output in self.outputs:
            output_name = output['name']
            output_dict[output_name] = np.copy(output['host']).reshape(output['shape'])

        return output_dict


if __name__ == "__main__":
    visualize = ENABLE_VISUALIZE

    dataset = NUScenes_reader()
    model = StreamPETR(camera_cfgs=dataset.camera_cfgs)
    if visualize:
        viz = LiveVisualizer()

    for data in dataset:
        start_time = time.time()
        output = model.infer(data)

        if visualize:
            # stop if visualizer is closed
            if not viz.is_active():
                break

            viz.update(output)
        
        # measure total run time
        end_time = time.time()
        elapsed = end_time - start_time

        # pace the loop speed
        sleep_for = 0.5 - elapsed  # sec
        time.sleep(max(0, sleep_for))

    if visualize:
        # Wait till user close the visualizer
        while viz.is_active():
            time.sleep(0.1)
        viz.close()
        cv2.destroyAllWindows()