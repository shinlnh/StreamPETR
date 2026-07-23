import cv2
import numpy as np
import os
from PIL import Image
from utils.label_loader import LabelLoader
import config as cfg

loading_directory = cfg.loading_directory
saving_directory = cfg.saving_directory


class ImageSaver():
    """
    Class used to load all the .npy files with LabelLoader and then convert into an image.
    Also draws the corresponding ground truth lanepoints on the images and saves the images seperately.
    """

    def __init__(self):
        self.colormap = [(0, 255, 0),
                        (255, 0, 0),
                        (255, 255, 0),
                        (0, 0, 255)]

        self.label_loader = LabelLoader(cfg.label_raw)
        self.label_loader2 = LabelLoader(cfg.label_raw, cfg.label_split)
        self.image_name = 0
        self.image_name_gt = 0

        self.categories_count = {}
        self.split_list = ['train/', 'val/', 'test/']
        self.split_ratio = [cfg.split_ratio['train'], cfg.split_ratio['val'], cfg.split_ratio['test']] # filter 
        self.split_ratio = [max(0, x) for x in self.split_ratio]
        sum_ratio = sum(self.split_ratio)
        if sum_ratio == 1.:
            self.split_ratio = [x/sum_ratio for x in self.split_ratio] # normalize to [0,1]
        else:
            raise ValueError('Split ratio is incorrect, please check split_ratio in config.py')


    def execute(self):
        for i, path in enumerate(self.find_imagesets()):
            image_set = np.load(path)
            if(image_set.shape[0] == cfg.number_of_images):
                for image in image_set:
                    self.save_images_with_annotation(image)
                    self.save_image_to_split(image)
                print('Saving imageset', i)
        
        self.label_loader.close_file()
        self.label_loader2.close_file()

    def find_imagesets(self):
        """
        Find all the .npy files from the specified directory and appends these path to a list.

        Returns:
            imageset_list: a list of imageset paths
        """
        
        imageset_list = []
        filenames = sorted(os.listdir(loading_directory))
        for filename in filenames:
            if filename.endswith('.npy'):
                file_path = os.path.join(loading_directory, filename)
                if os.path.isfile(file_path):
                    imageset_list.append(file_path)
                    print('Found', file_path)
                
        return imageset_list
    

    def save_image_to_split(self, image_array):
        """
        Converts the numpy array into .jpg images and saves it to a specific directory.
        Changes the label according to its calculated category and appends the label to the final json-file if the category is not already 
        full (defined by max_files_per_classification). This balances the input data for the algorithm
        In the actual implementation a image with the category nolanes wont be saved
        
        Args:
            image_array: numpy array
        """
        
        image = Image.fromarray(cv2.cvtColor(image_array, cv2.COLOR_BGR2RGB), 'RGB')
        
        category = str(self.label_loader2.calculate_folder())
        if not "nolanes" in category:
            if category not in self.categories_count:
                self.categories_count[category] = 0
            split = np.random.choice(self.split_list, p=self.split_ratio)
            if self.categories_count[category] < cfg.max_files_per_classification:
                relative_path = os.path.join(category, split, f'{self.categories_count[category] :06d}' + '.jpg')
                save_name = os.path.join(saving_directory, relative_path)
                folder = os.path.dirname(save_name)
                if not os.path.isdir(folder):
                    os.makedirs(folder)
                image.save(save_name, 'JPEG')

                # Choose a split
                self.label_loader2.update_rawfile_in(relative_path, split)
                self.categories_count[category] += 1


    def save_images_with_annotation(self, image_array):
        """
        Saves the images with its corresponding ground truth labels as .jpg images to a specific directory.
        
        Args:
            image_array: numpy array.
        """
        
        image_rgb = cv2.cvtColor(image_array, cv2.COLOR_BGR2RGB)
        for i, lane in enumerate(self.label_loader.load_lanepoints()):
            for lanepoint in lane:
                if lanepoint[0] >= 0:
                    cv2.circle(image_rgb, (lanepoint[0], lanepoint[1]), 3, self.colormap[i], thickness=1)
        
        image = Image.fromarray(image_rgb)
        save_name = os.path.join(cfg.visualize_directory, f'{self.image_name_gt:06d}')
        self.image_name_gt += 1
        
        folder = os.path.dirname(save_name)
        if not os.path.isdir(folder):
            os.makedirs(folder)
        
        image.save(save_name + '.jpg', 'JPEG')
