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
        self.map = np.repeat(self.map[...,np.newaxis],repeats=3,axis=2)

        # Create an image for visual purposes. TODO: Do with numpy
<<<<<<< HEAD
        img = np.zeros((self.map_size, self.map_size, 3))
        for y in range(self.map_size):
            for x in range(self.map_size):
                img[y, x] = LAND if self.map[y, x] else WATER
        self.img = Image.fromarray(img.astype('uint8'), 'RGB')
    
    def _perlin_noise(self, scale=.2, octaves=3, persistence=.5, lacunarity=2.0):
=======
        self.img = np.zeros((self.map_size, self.map_size, 3))
        self.img = np.multiply(self.map,LAND) + np.multiply(np.logical_not(self.map),WATER)

    def _perlin_noise(self, scale=.2, octaves=3, persistence=3, lacunarity=2.0):
>>>>>>> beb9b5ace8f6644bc7f54fe70417b08d241d2df5
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
        
<<<<<<< HEAD
    def show_map(self):
        self.img.show()
        
    
    
# ig = IslandGenerator(100, .1)
# mapp = ig.map
# img = ig.img
# ig._show_map()
=======
    def _show_map(self):
        img_map = Image.fromarray(self.img.astype('uint8'), 'RGB')
        img_map.show()
        return img_map
        
    
    
ig = IslandGenerator(1000, .05)
mapp = ig.map
img = ig.img
img_map = ig._show_map()
img_map.save('example.png')
>>>>>>> beb9b5ace8f6644bc7f54fe70417b08d241d2df5
