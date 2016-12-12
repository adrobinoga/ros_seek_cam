import sys
import pyseek


class SeekAPI():

    def __init__(self):
        self.seek = pyseek.PySeek()
        
    def find_cam(self):
        self.seek.open() 
        
    def get_image(self):
        return self.seek.get_image()
    
