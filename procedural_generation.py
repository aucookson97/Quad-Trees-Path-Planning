# -*- coding: utf-8 -*-
"""
Created on Fri Nov 20 19:38:34 2020

@author: aidan
"""

import noise
import numpy as np
from PIL import Image

LAND = (8, 150, 27)
WATER = (37, 106, 217) 


class IslandGenerator():
    
    def __init__(self, map_size, threshold):
        self.map_size = map_size
        self.noise_arr = self._perlin_noise()
        self.noise_arr[self.noise_arr >= threshold] = 1
        self.noise_arr[self.noise_arr < threshold] = 0
        self.map = self.noise_arr.astype(int) # 1s are land, 0s are water
        self.map_layer = np.repeat(self.map[...,np.newaxis], repeats=3, axis=2)

        # Create an image for visual purposes. TODO: Do with numpy
        #img = np.zeros((self.map_size, self.map_size, 3))
        img = np.multiply(self.map_layer,LAND) + np.multiply(np.logical_not(self.map_layer),WATER)
        self.img = Image.fromarray(img.astype('uint8'), 'RGB')


    def _perlin_noise(self, scale=.2, octaves=1, persistence=1, lacunarity=2.0):
        seed = np.random.randint(0, 100)
        noise_arr = np.zeros((self.map_size,self.map_size))
        
        x_idx = np.linspace(0, 1, self.map_size)
        y_idx = np.linspace(0, 1, self.map_size)
        arr_x, arr_y = np.meshgrid(x_idx, y_idx)
        
        noise_arr = np.vectorize(noise.pnoise2)(arr_x/scale,
                                                arr_y/scale,
                                                octaves=octaves,
                                                persistence=persistence,
                                                lacunarity=lacunarity,
                                                repeatx=1024,
                                                repeaty=1024,
                                                base=seed)
        return noise_arr
        
    def show_map(self):
        self.img.show()
