import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

def image_to_binary_matrix(image_path, threshold=143):
    image = Image.open(image_path).convert('L')
    width, height = image.size
    pixel_data = list(image.getdata())
    
    binary_matrix = [[1 if pixel > threshold else 0 for pixel in pixel_data[row * width:(row + 1) * width]] for row in range(height)]    
    return binary_matrix

def crop_matrix(matrix, left, top, right, bottom):
    cropped_matrix = [row[left:right] for row in matrix[top:bottom]]
    
    plt.imshow(cropped_matrix, cmap='gray', interpolation='bilinear')
    plt.show()
    
    
image_path = "CNU_library.png"

result_matrix = image_to_binary_matrix(image_path)
crop_matrix(result_matrix, left=140, top=75, right=440, bottom=280)