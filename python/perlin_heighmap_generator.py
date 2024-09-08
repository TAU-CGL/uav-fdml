import cv2
import trimesh
import numpy as np
from perlin_noise import PerlinNoise


def generate_perlin_noise(width, height, octaves=10):
    noise = PerlinNoise(octaves=octaves)
    img = np.array([[noise([x / width, y / height]) for x in range(width)] for y in range(height)])
    return img

def normalize_image(img):
    return 255 * (img - np.min(img)) / (np.max(img) - np.min(img))

def image2mesh(img: np.array, scale):
    vertices = []
    faces = []

    for y in range(img.shape[0]):
        for x in range(img.shape[1]):
            vertices.append([x / img.shape[1] * 5, img[y, x] * scale,  y / img.shape[0] * 5])
    
    for y in range(img.shape[0] - 1):
        for x in range(img.shape[1] - 1):
            i = y * img.shape[1] + x
            faces.append([i, i + 1, i + img.shape[1]])
            faces.append([i + 1, i + img.shape[1], i + img.shape[1] + 1])
    
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
    mesh.fix_normals()
    mesh = mesh.unwrap()
    return mesh


if __name__ == "__main__":
    for i in range(3):
        prefix = f"resources/fdml/scans/perlin/perlin{i+1}"
        img = generate_perlin_noise(128, 128)
        cv2.imwrite(prefix + ".png", normalize_image(img))
        mesh = image2mesh(img, 0.25)
        mesh.export(prefix + ".obj")