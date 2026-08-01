# ------------------------------------------------------------------------
# Copyright (c) 2022 megvii-model. All Rights Reserved.
# ------------------------------------------------------------------------
# Modified from DETR3D (https://github.com/WangYueFt/detr3d)
# Copyright (c) 2021 Wang, Yue
# ------------------------------------------------------------------------
# Modified from mmdetection3d (https://github.com/open-mmlab/mmdetection3d)
# Copyright (c) OpenMMLab. All rights reserved.
# ------------------------------------------------------------------------
#  Modified by Shihao Wang
# ------------------------------------------------------------------------
import math
import random

import mmcv
import numpy as np
import torch
from mmcv.parallel import DataContainer as DC
from mmcv.utils import print_log
from mmdet.datasets import DATASETS
from mmdet3d.core.bbox import Box3DMode, LiDARInstance3DBoxes
from mmdet3d.core.evaluation.indoor_eval import indoor_eval
from mmdet3d.datasets import NuScenesDataset
from nuscenes.eval.common.utils import Quaternion

from projects.mmdet3d_plugin.core.evaluation.carla_eval import (
    evaluate_carla_center_distance,
)


@DATASETS.register_module()
class CustomNuScenesDataset(NuScenesDataset):
    r"""NuScenes Dataset.

    This datset only add camera intrinsics and extrinsics to the results.
    """

    def __init__(
        self,
        collect_keys,
        seq_mode=False,
        seq_split_num=1,
        num_frame_losses=1,
        queue_length=8,
        random_length=0,
        sequence_order_by_scene=False,
        *args,
        **kwargs
    ):
        super().__init__(*args, **kwargs)
        if sequence_order_by_scene:
            # NuScenesDataset sorts every sample globally by timestamp. That is
            # fine for real nuScenes recordings, but independent CARLA clips
            # can have overlapping simulator timestamps. Restore contiguous
            # clip order so the streaming sampler and prev_exists state never
            # cross scene boundaries.
            self.data_infos.sort(
                key=lambda info: (
                    str(info.get("scene_token", "")),
                    int(info.get("frame_idx", 0)),
                    int(info.get("timestamp", 0)),
                )
            )
        self.queue_length = queue_length
        self.collect_keys = collect_keys
        self.random_length = random_length
        self.num_frame_losses = num_frame_losses
        self.seq_mode = seq_mode
        if seq_mode:
            self.num_frame_losses = 1
            self.queue_length = 1
            self.seq_split_num = seq_split_num
            self.random_length = 0
            self._set_sequence_group_flag() # Must be called after load_annotations b/c load_annotations does sorting.

    def _set_sequence_group_flag(self):
        """
        Set each sequence to be a different group
        """
        res = []

        curr_sequence = 0
        for idx in range(len(self.data_infos)):
            if (
                idx != 0
                and self.data_infos[idx]["scene_token"]
                != self.data_infos[idx - 1]["scene_token"]
            ):
                # Camera-only CARLA samples have no LiDAR sweeps on any frame.
                # Scene tokens are the reliable temporal boundary for both
                # CARLA and nuScenes-derived info files.
                curr_sequence += 1
            res.append(curr_sequence)

        self.flag = np.array(res, dtype=np.int64)

        if self.seq_split_num != 1:
            if self.seq_split_num == 'all':
                self.flag = np.array(range(len(self.data_infos)), dtype=np.int64)
            else:
                bin_counts = np.bincount(self.flag)
                new_flags = []
                curr_new_flag = 0
                for curr_flag in range(len(bin_counts)):
                    curr_sequence_length = np.array(
                        list(range(0, 
                                bin_counts[curr_flag], 
                                math.ceil(bin_counts[curr_flag] / self.seq_split_num)))
                        + [bin_counts[curr_flag]])

                    for sub_seq_idx in (curr_sequence_length[1:] - curr_sequence_length[:-1]):
                        for _ in range(sub_seq_idx):
                            new_flags.append(curr_new_flag)
                        curr_new_flag += 1

                assert len(new_flags) == len(self.flag)
                expected_groups = sum(
                    min(self.seq_split_num, count) for count in bin_counts
                )
                assert len(np.bincount(new_flags)) == expected_groups
                self.flag = np.array(new_flags, dtype=np.int64)


    def prepare_train_data(self, index):
        """
        Training data preparation.
        Args:
            index (int): Index for accessing the target data.
        Returns:
            dict: Training data dict of the corresponding index.
        """
        queue = []
        index_list = list(range(index-self.queue_length-self.random_length+1, index))
        random.shuffle(index_list)
        index_list = sorted(index_list[self.random_length:])
        index_list.append(index)
        prev_scene_token = None
        for i in index_list:
            i = max(0, i)
            input_dict = self.get_data_info(i)
            
            if not self.seq_mode: # for sliding window only
                if input_dict['scene_token'] != prev_scene_token:
                    input_dict.update(dict(prev_exists=False))
                    prev_scene_token = input_dict['scene_token']
                else:
                    input_dict.update(dict(prev_exists=True))

            self.pre_pipeline(input_dict)
            example = self.pipeline(input_dict)

            queue.append(example)

        for k in range(self.num_frame_losses):
            if self.filter_empty_gt and \
                (queue[-k-1] is None or ~(queue[-k-1]['gt_labels_3d']._data != -1).any()):
                return None
        return self.union2one(queue)

    def prepare_test_data(self, index):
        """Prepare data for testing.

        Args:
            index (int): Index for accessing the target data.

        Returns:
            dict: Testing data dict of the corresponding index.
        """
        input_dict = self.get_data_info(index)
        self.pre_pipeline(input_dict)
        example = self.pipeline(input_dict)
        return example
        
    def union2one(self, queue):
        for key in self.collect_keys:
            if key != 'img_metas':
                queue[-1][key] = DC(torch.stack([each[key].data for each in queue]), cpu_only=False, stack=True, pad_dims=None)
            else:
                queue[-1][key] = DC([each[key].data for each in queue], cpu_only=True)
        if not self.test_mode:
            for key in ['gt_bboxes_3d', 'gt_labels_3d', 'gt_bboxes', 'gt_labels', 'centers2d', 'depths']:
                if key == 'gt_bboxes_3d':
                    queue[-1][key] = DC([each[key].data for each in queue], cpu_only=True)
                else:
                    queue[-1][key] = DC([each[key].data for each in queue], cpu_only=False)

        queue = queue[-1]
        return queue

    def get_data_info(self, index):
        """Get data info according to the given index.

        Args:
            index (int): Index of the sample data to get.

        Returns:
            dict: Data information that will be passed to the data \
                preprocessing pipelines. It includes the following keys:

                - sample_idx (str): Sample index.
                - pts_filename (str): Filename of point clouds.
                - sweeps (list[dict]): Infos of sweeps.
                - timestamp (float): Sample timestamp.
                - img_filename (str, optional): Image filename.
                - lidar2img (list[np.ndarray], optional): Transformations \
                    from lidar to different cameras.
                - ann_info (dict): Annotation info.
        """
        info = self.data_infos[index]
        # standard protocal modified from SECOND.Pytorch

        e2g_rotation = Quaternion(info['ego2global_rotation']).rotation_matrix
        e2g_translation = info['ego2global_translation']
        l2e_rotation = Quaternion(info['lidar2ego_rotation']).rotation_matrix
        l2e_translation = info['lidar2ego_translation']
        e2g_matrix = convert_egopose_to_matrix_numpy(e2g_rotation, e2g_translation)
        l2e_matrix = convert_egopose_to_matrix_numpy(l2e_rotation, l2e_translation)
        ego_pose =  e2g_matrix @ l2e_matrix # lidar2global

        ego_pose_inv = invert_matrix_egopose_numpy(ego_pose)
        input_dict = dict(
            sample_idx=info['token'],
            pts_filename=info['lidar_path'],
            sweeps=info['sweeps'],
            ego_pose=ego_pose,
            ego_pose_inv = ego_pose_inv,
            prev_idx=info['prev'],
            next_idx=info['next'],
            scene_token=info['scene_token'],
            frame_idx=info['frame_idx'],
            timestamp=info['timestamp'] / 1e6,
        )

        if self.modality['use_camera']:
            image_paths = []
            lidar2img_rts = []
            intrinsics = []
            extrinsics = []
            img_timestamp = []
            for cam_type, cam_info in info['cams'].items():
                img_timestamp.append(cam_info['timestamp'] / 1e6)
                image_paths.append(cam_info['data_path'])
                # obtain lidar to image transformation matrix
                cam2lidar_r = cam_info['sensor2lidar_rotation']
                cam2lidar_t = cam_info['sensor2lidar_translation']
                cam2lidar_rt = convert_egopose_to_matrix_numpy(cam2lidar_r, cam2lidar_t)
                lidar2cam_rt = invert_matrix_egopose_numpy(cam2lidar_rt)

                intrinsic = cam_info['cam_intrinsic']
                viewpad = np.eye(4)
                viewpad[:intrinsic.shape[0], :intrinsic.shape[1]] = intrinsic
                lidar2img_rt = (viewpad @ lidar2cam_rt)
                intrinsics.append(viewpad)
                extrinsics.append(lidar2cam_rt)
                lidar2img_rts.append(lidar2img_rt)
                
            if not self.test_mode: # for seq_mode
                prev_exists  = not (index == 0 or self.flag[index - 1] != self.flag[index])
            else:
                prev_exists = None

            input_dict.update(
                dict(
                    img_timestamp=img_timestamp,
                    img_filename=image_paths,
                    lidar2img=lidar2img_rts,
                    intrinsics=intrinsics,
                    extrinsics=extrinsics,
                    prev_exists=prev_exists,
                ))
        if not self.test_mode:
            annos = self.get_ann_info(index)
            annos.update( 
                dict(
                    bboxes=info['bboxes2d'],
                    labels=info['labels2d'],
                    centers2d=info['centers2d'],
                    depths=info['depths'],
                    bboxes_ignore=info['bboxes_ignore'])
            )
            input_dict['ann_info'] = annos
            
        return input_dict


    def __getitem__(self, idx):
        """Get item from infos according to the given index.
        Returns:
            dict: Data dictionary of the corresponding index.
        """
        if self.test_mode:
            return self.prepare_test_data(idx)
        while True:

            data = self.prepare_train_data(idx)
            if data is None:
                idx = self._rand_another(idx)
                continue
            return data

    def evaluate(
        self,
        results,
        metric="bbox",
        logger=None,
        jsonfile_prefix=None,
        result_names=("pts_bbox",),
        show=False,
        out_dir=None,
        pipeline=None,
        distance_thresholds=(0.5, 1.0, 2.0, 4.0),
        point_cloud_range=None,
        **kwargs
    ):
        """Evaluate CARLA without requiring nuScenes database tables."""
        if metric in ("carla", "carla_center_distance"):
            metrics = evaluate_carla_center_distance(
                results=results,
                data_infos=self.data_infos,
                class_names=self.CLASSES,
                distance_thresholds=distance_thresholds,
                point_cloud_range=point_cloud_range,
                jsonfile_prefix=jsonfile_prefix,
            )
            summary = ["CARLA center-distance evaluation:"]
            summary.extend(
                "{}={:.4f}".format(name, value)
                for name, value in sorted(metrics.items())
                if name.endswith("/mAP") or name == "carla/mATE_2m"
            )
            print_log(" ".join(summary), logger=logger)
            return metrics
        return super().evaluate(
            results=results,
            metric=metric,
            logger=logger,
            jsonfile_prefix=jsonfile_prefix,
            result_names=result_names,
            show=show,
            out_dir=out_dir,
            pipeline=pipeline,
            **kwargs
        )

@DATASETS.register_module()
class CarlaStreamPetrDataset(CustomNuScenesDataset):
    """StreamPETR CARLA dataset with a simulator-native 3D IoU evaluator.

    CARLA samples intentionally do not depend on a nuScenes database. Reusing
    ``NuScenesDataset.evaluate`` would therefore fail while trying to open
    nuScenes tables. This evaluator consumes the boxes already embedded in the
    CARLA info files and reports class-wise 3D IoU AP/recall.
    """

    def __init__(self, point_cloud_range=None, *args, **kwargs):
        # Validation must use the same BEV support as ObjectRangeFilter during
        # training. The collector intentionally keeps a wider 61.2 m radial
        # label range, while the CARLA StreamPETR head predicts within +/-51.2
        # m. Counting unreachable GT boxes would artificially depress recall.
        self.point_cloud_range = (
            np.asarray(point_cloud_range, dtype=np.float32)
            if point_cloud_range is not None
            else None
        )
        super().__init__(*args, **kwargs)

    def load_annotations(self, ann_file):
        """Load CARLA scenes without interleaving simulator-local clocks.

        Every CARLA server restart resets elapsed simulation time. The
        NuScenesDataset implementation globally sorts by timestamp, which
        would alternate frames from many CARLA scenes. The collector already
        writes scenes contiguously and frame indices monotonically, so retain
        that canonical order.
        """
        data = mmcv.load(ann_file, file_format="pkl")
        data_infos = list(data["infos"])[:: self.load_interval]
        self.metadata = data["metadata"]
        self.version = self.metadata["version"]

        previous_scene = None
        expected_frame = 0
        closed_scenes = set()
        for info in data_infos:
            scene = info["scene_token"]
            if scene != previous_scene:
                if scene in closed_scenes:
                    raise ValueError(f"Non-contiguous CARLA scene {scene}")
                if previous_scene is not None:
                    closed_scenes.add(previous_scene)
                previous_scene = scene
                expected_frame = 0
            if info["frame_idx"] != expected_frame:
                raise ValueError(
                    f"CARLA scene {scene} expected frame {expected_frame}, "
                    f"received {info['frame_idx']}"
                )
            expected_frame += 1
        return data_infos

    def evaluate(
        self,
        results,
        metric=(0.25, 0.5),
        logger=None,
        **kwargs,
    ):
        if isinstance(metric, (int, float)):
            metric = [float(metric)]
        elif isinstance(metric, (list, tuple)):
            try:
                metric = [float(value) for value in metric]
            except (TypeError, ValueError):
                # MMDetection's CLI passes its conventional ``bbox`` token.
                metric = [0.25, 0.5]
        else:
            metric = [0.25, 0.5]

        if not torch.cuda.is_available():
            raise RuntimeError(
                "CARLA validation requires CUDA; CPU evaluator fallback is disabled"
            )
        eval_device = torch.device("cuda", torch.cuda.current_device())
        mmcv.print_log(
            f"CARLA rotated 3D IoU backend: {eval_device}",
            logger=logger,
        )

        detections = []
        for result in results:
            detection = result.get("pts_bbox", result)
            boxes = detection["boxes_3d"]
            # StreamPETR predicts x/y velocity as the final two box values.
            # The generic 3D IoU evaluator operates on the geometric 7-vector.
            boxes = LiDARInstance3DBoxes(
                boxes.tensor[:, :7].detach().to(
                    eval_device, non_blocking=True
                ),
                box_dim=7,
                origin=(0.5, 0.5, 0.0),
            )
            detections.append(
                {
                    "boxes_3d": boxes,
                    "scores_3d": detection["scores_3d"].detach().cpu(),
                    "labels_3d": detection["labels_3d"].detach().cpu(),
                }
            )

        gt_annos = []
        present_labels = set()
        for info in self.data_infos:
            mask = info["valid_flag"].astype(bool)
            if self.point_cloud_range is not None:
                x_min, y_min, _, x_max, y_max, _ = self.point_cloud_range
                centers = info["gt_boxes"][:, :2]
                mask &= (
                    (centers[:, 0] >= x_min)
                    & (centers[:, 0] <= x_max)
                    & (centers[:, 1] >= y_min)
                    & (centers[:, 1] <= y_max)
                )
            names = info["gt_names"][mask]
            boxes = info["gt_boxes"][mask]
            # NumPy infers float64 for an empty list. Empty-GT CARLA frames
            # would then fail here because float arrays are invalid indices.
            known = np.asarray(
                [name in self.CLASSES for name in names],
                dtype=np.bool_,
            )
            boxes = boxes[known]
            labels = np.array(
                [self.CLASSES.index(name) for name in names[known]],
                dtype=np.int64,
            )
            present_labels.update(labels.tolist())
            gt_annos.append(
                {
                    "gt_num": len(labels),
                    "gt_boxes_upright_depth": torch.as_tensor(
                        boxes,
                        dtype=torch.float32,
                        device=eval_device,
                    ),
                    "class": labels,
                }
            )

        # Predictions for a category with no ground truth have undefined AP
        # and would otherwise make the aggregate metric NaN on small splits.
        for detection in detections:
            labels = detection["labels_3d"]
            keep = torch.zeros_like(labels, dtype=torch.bool)
            for label in present_labels:
                keep |= labels == label
            detection["boxes_3d"] = detection["boxes_3d"][
                keep.to(eval_device)
            ]
            detection["scores_3d"] = detection["scores_3d"][keep]
            detection["labels_3d"] = labels[keep]

        label2cat = {index: name for index, name in enumerate(self.CLASSES)}
        return indoor_eval(
            gt_annos,
            detections,
            metric,
            label2cat,
            logger=logger,
            box_type_3d=LiDARInstance3DBoxes,
            box_mode_3d=Box3DMode.LIDAR,
        )

def invert_matrix_egopose_numpy(egopose):
    """ Compute the inverse transformation of a 4x4 egopose numpy matrix."""
    inverse_matrix = np.zeros((4, 4), dtype=np.float32)
    rotation = egopose[:3, :3]
    translation = egopose[:3, 3]
    inverse_matrix[:3, :3] = rotation.T
    inverse_matrix[:3, 3] = -np.dot(rotation.T, translation)
    inverse_matrix[3, 3] = 1.0
    return inverse_matrix

def convert_egopose_to_matrix_numpy(rotation, translation):
    transformation_matrix = np.zeros((4, 4), dtype=np.float32)
    transformation_matrix[:3, :3] = rotation
    transformation_matrix[:3, 3] = translation
    transformation_matrix[3, 3] = 1.0
    return transformation_matrix
