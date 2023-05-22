# imports
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import cv2
import glob
from tqdm import tqdm
import numpy as np

# Use Agg backend for canvas
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
import argparse

parser = argparse.ArgumentParser(description="Draw output directories from eval_Orbeez_SLAM to video")
parser.add_argument("--images_dir", help="The saved render images and infos directory", required=True)
args = parser.parse_args()

frames = []
depths = []

for frame in sorted(glob.glob(args.images_dir+"/frame/*.png")):
    frames.append(frame)

for depth in sorted(glob.glob(args.images_dir+"/depth/*.png")):
    depths.append(depth)

outputs = []
for frame, depth in zip(frames, depths):
    outputs.append([frame, depth])

print(outputs)
frame, depth = outputs[0]
frame = cv2.imread(frame)
depth = cv2.imread(depth)

fig, axarr = plt.subplots(2,1) 
axarr[0].imshow(frame)
axarr[1].imshow(depth, cmap="plasma")

# put pixel buffer in numpy array
canvas = FigureCanvas(fig)
canvas.draw()
mat = np.array(canvas.renderer._renderer)
print(mat.shape)

video = cv2.VideoWriter(args.images_dir+'/video.mp4', cv2.VideoWriter_fourcc('M','P','4','V'), 10, (mat.shape[1],mat.shape[0]), True)

# loop over your images
for output in tqdm(outputs):

    frame, depth = output

    frame = cv2.imread(frame)
    depth = cv2.imread(depth)

    fig, axarr = plt.subplots(2,1) 
    axarr[0].imshow(depth, cmap="plasma")
    axarr[1].imshow(frame)
    # plt.show()

    # put pixel buffer in numpy array
    canvas = FigureCanvas(fig)
    canvas.draw()
    mat = np.array(canvas.renderer._renderer)
    # mat = cv2.cvtColor(mat, cv2.COLOR_RGB2BGR)

    # write frame to video
    video.write(mat[:,:,:3])
    plt.close()

# close video writer
cv2.destroyAllWindows()
video.release()