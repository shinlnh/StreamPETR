import os
import json
import logging

import numpy as np
from tqdm import tqdm

import utils.carla_metric as carla_metric

from .lane_dataset_loader import LaneDatasetLoader

SPLITS = ['train', 'val', 'test']


class CarlaDataset(LaneDatasetLoader):
    def __init__(self, max_lanes=None, split='train', root: str=None, official_metric=False):
        self.split = split
        self.root = root
        self.official_metric = official_metric
        self.logger = logging.getLogger(__name__)

        if root is None:
            raise Exception('Please specify the path to raw json label.')
        if split.lower() not in SPLITS:
            raise Exception('Split `{}` does not exist.'.format(split))

        self.list = root.replace('.json', f'_{split}.json')

        self.img_w, self.img_h = 1280, 720
        self.ys, self.annotations = self.load_annotations(self.list, self.img_w, self.img_h)
        self.max_lanes = 4 if max_lanes is None else max_lanes
        
        self.logger.info('%d annotations loaded, with a maximum of %d lanes in an image.',
                         len(self.annotations), self.max_lanes)


    def get_img_heigth(self, _):
        return self.img_h


    def get_img_width(self, _):
        return self.img_w


    def load_annotations(self, label_path: str, img_w: int, img_h: int):
        annotations = []
        
        f = open(label_path)
        ys = json.loads(f.readline())['Ys']  # Read first line which contains y anchors
        for line in f:
            data = json.loads(line)
            if not 'lines' in data:
                continue
            if len(data['lines']) == 0:
                continue
            
            lane_marks = [list(zip(xs, ys)) for xs in data['lines']] # Load Xs and combine with Ys
            lane_marks = [list(filter(lambda x: 0 <= x[0] <= img_w, lane_mark)) for lane_mark in lane_marks] # Filter invalid points
            mask = [len(lane_mark) > 1 for lane_mark in lane_marks] # Mask to filter lane with less than 2 points
            lane_marks = [lane_mark for i, lane_mark in enumerate(lane_marks) if mask[i]]
            lane_marks = [sorted(lane_mark, key= lambda x: x[1]) for lane_mark in lane_marks] # Sort by y value

            lanes_type = data['types'] # Load lane marking's class
            lanes_type = [lane_type for i, lane_type in enumerate(lanes_type) if mask[i]] # Filter lane with less than 2 points

            anno = {'path': os.path.join(os.path.dirname(label_path), data['image']),
                    'org_path': data['image'],
                    'lanes': lane_marks,
                    'types': lanes_type}
            annotations.append(anno)
        f.close()

        return ys, annotations


    def save_prediction(self, predictions, output_basedir):
        output_dir = os.path.join(output_basedir, os.path.dirname(self.root))
        output_file = os.path.join(output_dir, 'predictions.json')
        os.makedirs(output_dir, exist_ok=True)
        if os.path.isfile(output_file):
            os.remove(output_file)
        with open(output_file, 'a') as out_f:
            out_f.write(json.dumps({'Ys': self.ys}) + '\n') # save y anchor values
            ys = np.array(self.ys) / self.img_h # convert to np array with range [0, 1)
            for idx, pred in enumerate(tqdm(predictions)):
                # info to collects
                lanes_marking = []
                lanes_type = []
                image_path = self.annotations[idx]['old_anno']['org_path']
                
                # collect xs and types
                for lane in pred:
                    xs = lane(ys)
                    xs = [round(x * self.img_w) if 0. <= x < 1. else -2 for x in xs]
                    lanes_marking.append(xs)
                    lanes_type.append(int(lane.metadata['type']))

                # dump to json and write
                output = {'lines': lanes_marking,
                          'types': lanes_type,
                          'image': image_path}
                output = json.dumps(output)
                out_f.write(output + '\n')
        return output_file


    def get_metrics(self, prediction, idx):
        ys = np.array(self.ys) / self.img_h # convert to np array with range [0, 1)
        lanes = []
        for lane in prediction:
            xs = lane(ys)
            lane = [(round(x * self.img_w), y * self.img_h) for x, y in zip(xs, ys) if 0. <= x < 1.]
            if len(lane) < 2:
                continue
            lanes.append(lane)
        anno = self.annotations[idx]['old_anno']['lanes']
        _, fp, fn, ious, matches = carla_metric.carla_metric(lanes, anno, img_shape=(self.img_h, self.img_w, 3))

        return fp, fn, matches, ious


    def eval_predictions(self, predictions, output_basedir):
        print('Generating prediction output...')
        # Save prediction to json file
        saved_path = self.save_prediction(predictions, output_basedir)

        return carla_metric.eval_predictions(saved_path, self.list,
                                             (self.img_h, self.img_w, 3),
                                             official= self.official_metric)


    def transform_annotations(self, transform):
        self.annotations = list(map(transform, self.annotations))


    def __getitem__(self, idx):
        return self.annotations[idx]


    def __len__(self):
        return len(self.annotations)
