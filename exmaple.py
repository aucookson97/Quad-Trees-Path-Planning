from procedural_generation import IslandGenerator
from PIL import Image, ImageDraw
import pickle, os
import numpy as np
from matplotlib import pyplot as plt


ig = pickle.load(open('ig.pkl','rb'))


fig = plt.figure()
img = np.zeros(ig.img.size)
plt_img = plt.imshow(img)

while True:       

    image = np.array(ig.img)
    plt_img.set_data(image)
    fig.canvas.draw_idle()
    plt.pause(1)

