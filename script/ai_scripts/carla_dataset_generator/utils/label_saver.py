import os
import json
import config as cfg

class LabelSaver():
    """
    Helper class to save all the lanedata (labels). Each label contains a list 
    of the x values of a lane, their corresponding predefined y-values and 
    their path to the image.
    """
    def __init__(self, label_file):
        self.image_name = 0
        
        folder = os.path.dirname(label_file)
        if not os.path.isdir(folder):
            os.makedirs(folder)
        
        self.file = open(label_file, 'w')
        self.file.write(json.dumps({"Ys": cfg.h_samples}) + '\n') # Write y coordinates
        

    def add_label(self, line_list, line_type):
        filestring = {"lines": line_list,
                      "types": line_type,
                      "image": f'{self.image_name:06d}' + '.jpg'}
        
        jsonstring = json.dumps(filestring)
        self.file.write(jsonstring + '\n')
        self.image_name += 1
    
    
    def close_file(self):
        self.file.close()