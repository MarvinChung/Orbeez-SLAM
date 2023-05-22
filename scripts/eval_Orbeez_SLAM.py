import argparse
import os
import commentjson as json

import numpy as np

import shutil
import re
import time

import sys
sys.path.insert(1, "./Thirdparty/instant-ngp-kf/scripts")
from common import *
from scenes import *

from tqdm import tqdm
# import moviepy.video.io.ImageSequenceClip
import cv2
import yaml
import math
sys.path.insert(1, "./build/Thirdparty/instant-ngp-kf")
import pyngp as ngp # noqa
import json

import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def parse_args():

    parser = argparse.ArgumentParser(description="Run neural graphics primitives testbed with additional configuration & output options")
    parser.add_argument("--dataset_dir", default="./Replica/office0", help="The directory of the dataset.")
    parser.add_argument("--dataset_type", default="Replica", choices=["Replica", "ScanNet"])
    parser.add_argument("--dataset_config", default="configs/RGB-D/Replica/office0.yaml", help="Path to the scene yaml config.")
    parser.add_argument("--out_dir", help="Save the render images and infos", required=True)
    parser.add_argument("--load_snapshot", default="", help="Load this snapshot before training. recommended extension: .msgpack", required=True)

    parser.add_argument("--nerf_compatibility", action="store_true", help="Matches parameters with original NeRF. Can cause slowness and worse results on some scenes.")
    parser.add_argument("--near_distance", default=-1, type=float, help="set the distance from the camera at which training rays start for nerf. <0 means use ngp default")
    parser.add_argument("--sharpen", default=0, help="Set amount of sharpening applied to NeRF training images.")
    parser.add_argument("--debug_show", action="store_true", help="show the image and depth for user to check.")
    parser.add_argument("--save_rgb", action="store_true")
    parser.add_argument("--save_depth", action="store_true")

    args = parser.parse_args()
    return args

def load_replica(config, args):

    # scale  = config["NeRF"]["scale"]
    # offset = config["NeRF"]["offset"]

    Two = np.identity(4, dtype=float)
    
    transforms = []
    frames = []
    depths = []

    with open(args.dataset_dir + "/traj.txt", "r") as f:
        lines=f.readlines()
        for i, line in enumerate(lines):
            xform_array = np.fromstring(line, dtype=float, sep=' ')
            xform_mat = np.array([xform_array[:4],xform_array[4:8],xform_array[8:12],xform_array[12:]])

            if i == 0:
                # set first frame to origin
                R_inv = np.linalg.inv(xform_mat[:3,:3])
                t_inv = -R_inv @ xform_mat[:3,3]
                Two[:3,:3] = R_inv 
                Two[:3,3] = t_inv

            # transform to origin coordinate
            xform_mat =  Two @ xform_mat

            #transform to nerf coordinate (load dataset will do this)
            #xform_mat[:3,3] = xform_mat[:3,3] * scale + offset
            transforms.append(xform_mat)

    for file_name in sorted(glob.glob(args.dataset_dir +"/frame/*.jpg")):
        frames.append(file_name)

    for file_name in sorted(glob.glob(args.dataset_dir +"/depth/*.png")):
        depths.append(file_name)
    
    datas = []
    for xform, frame, depth in zip(transforms, frames, depths):
        datas.append({"xform":xform, "frame_path": frame, "depth_path":depth})

    return datas

def load_scannet(config, args):
    Two = np.identity(4, dtype=float)
    
    transforms = []
    frames = []
    depths = []

    with open(args.dataset_dir + "/trajectory.txt", "r") as f:
        lines=f.readlines()
        for i, line in enumerate(lines):
            xform_array = np.fromstring(line, dtype=float, sep=' ')
            xform_mat = xform_array[1:].reshape(4,4)

            if i == 0:
                # set first frame to origin
                R_inv = np.linalg.inv(xform_mat[:3,:3])
                t_inv = -R_inv @ xform_mat[:3,3]
                Two[:3,:3] = R_inv 
                Two[:3,3] = t_inv

            # transform to origin coordinate
            xform_mat =  Two @ xform_mat

            #transform to nerf coordinate (load dataset will do this)
            #xform_mat[:3,3] = xform_mat[:3,3] * scale + offset
            transforms.append(xform_mat)

    for file_name in sorted(glob.glob(args.dataset_dir +"/color/*.jpg")):
        frames.append(file_name)

    for file_name in sorted(glob.glob(args.dataset_dir +"/depth/*.png")):
        depths.append(file_name)
    
    datas = []
    for xform, frame, depth in zip(transforms, frames, depths):
        datas.append({"xform":xform, "frame_path": frame, "depth_path":depth})

    return datas

def load_tum(config, args):
    raise NotImplementedError("There is no specific gt image for the timestamp of gt traj")
    # Two = np.identity(4, dtype=float)
    
    # transforms = []
    # frames = []
    # depths = []

    # with open(args.dataset_dir + "/groundtruth.txt", "r") as f:
    #     lines=f.readlines()
    #     for i, line in enumerate(lines[3:]):
    #         xform_array = np.fromstring(line, dtype=float, sep=' ')
    #         t = xform_array[1:4]
    #         rot = R.from_quat(xform_array[4:]).as_matrix()
    #         xform_mat = np.identity(4, dtype=float)
    #         xform_mat[:3,:3] = rot
    #         xform_mat[:3, 3] = t

    #         if i == 0:
    #             # set first frame to origin
    #             R_inv = np.linalg.inv(xform_mat[:3,:3])
    #             t_inv = -R_inv @ xform_mat[:3,3]
    #             Two[:3,:3] = R_inv 
    #             Two[:3,3] = t_inv

    #         # transform to origin coordinate
    #         xform_mat =  Two @ xform_mat

    #         #transform to nerf coordinate (load dataset will do this)
    #         #xform_mat[:3,3] = xform_mat[:3,3] * scale + offset
    #         transforms.append(xform_mat)

    # for file_name in sorted(glob.glob(args.dataset_dir +"/rgb/*.png")):
    #     frames.append(file_name)

    # for file_name in sorted(glob.glob(args.dataset_dir +"/depth/*.png")):
    #     depths.append(file_name)
    
    # datas = []
    # for xform, frame, depth in zip(transforms, frames, depths):
    #     datas.append({"xform":xform, "frame_path": frame, "depth_path":depth})

    # return datas

def to_angle(r, x):
    return 2 * 180 / math.pi * math.atan(r/(2*x))

if __name__ == "__main__":
    args = parse_args()

    mode = ngp.TestbedMode.NerfSlam
    with open(args.dataset_config, 'r') as stream:
        config = yaml.safe_load(stream)

    # network = config["NeRF"]["network_config_path"]
    w              = config["Camera"]["width"]
    h              = config["Camera"]["height"]
    fl_x           = config["Camera"]["fx"]
    fl_y           = config["Camera"]["fy"]
    cx             = config["Camera"]["cx"]
    cy             = config["Camera"]["cy"]
    k1             = config["Camera"]["k1"]
    k2             = config["Camera"]["k2"]
    p1             = config["Camera"]["p1"]
    p2             = config["Camera"]["p2"]
    DepthMapFactor = config["DepthMapFactor"]
    aabb_scale     = config["NeRF"]["aabb_scale"]
    offset         = config["NeRF"]["offset"]
    scale          = config["NeRF"]["scale"]

    json_dict = {}
    json_dict["w"]                   = w
    json_dict["h"]                   = h
    json_dict["fl_x"]                = fl_x
    json_dict["fl_y"]                = fl_y
    json_dict["cx"]                  = cx
    json_dict["cy"]                  = cy
    json_dict["k1"]                  = k1
    json_dict["k2"]                  = k2
    json_dict["p1"]                  = p1
    json_dict["p2"]                  = p2
    json_dict["integer_depth_scale"] = 1.0/DepthMapFactor
    json_dict["aabb_scale"]          = aabb_scale
    json_dict["offset"]              = offset
    json_dict["scale"]               = scale

    out_dir = args.out_dir

    if not os.path.exists(out_dir):
        os.makedirs(out_dir)

    if not os.path.exists(out_dir+"/depth"):
        os.makedirs(out_dir+"/depth")

    if not os.path.exists(out_dir+"/frame"):
        os.makedirs(out_dir+"/frame")

    if args.dataset_type == "Replica":
        datas = load_replica(config, args)
    elif args.dataset_type == "ScanNet":
        datas = load_scannet(config, args)
    elif args.dataset_type == "TUM":
        datas = load_tum(config, args)

    testbed = ngp.Testbed(mode)
    testbed.nerf.sharpen = float(args.sharpen)

    # This is a hack. (The load snapshot dataset is broken. Bug need to be fixed!)
    with open(out_dir+'/transforms.json', 'w') as f:
        json.dump(json_dict, f, indent=4)

    testbed.load_training_data(out_dir+'/transforms.json')

    if args.load_snapshot:
        print("Loading snapshot ", args.load_snapshot)
        testbed.load_snapshot(args.load_snapshot)

    testbed.shall_train = False
    testbed.nerf.render_with_camera_distortion = True

    if args.near_distance >= 0.0:
        print("NeRF training ray near_distance ", args.near_distance)
        testbed.nerf.training.near_distance = args.near_distance

    if args.nerf_compatibility:
        print(f"NeRF compatibility mode enabled")

        # Prior nerf papers accumulate/blend in the sRGB
        # color space. This messes not only with background
        # alpha, but also with DOF effects and the likes.
        # We support this behavior, but we only enable it
        # for the case of synthetic nerf data where we need
        # to compare PSNR numbers to results of prior work.
        testbed.color_space = ngp.ColorSpace.SRGB

        # No exponential cone tracing. Slightly increases
        # quality at the cost of speed. This is done by
        # default on scenes with AABB 1 (like the synthetic
        # ones), but not on larger scenes. So force the
        # setting here.
        testbed.nerf.cone_angle_constant = 0

        # Optionally match nerf paper behaviour and train on a
        # fixed white bg. We prefer training on random BG colors.
        # testbed.background_color = [1.0, 1.0, 1.0, 1.0]
        # testbed.nerf.training.random_bg_color = False

    totmse = 0
    totpsnr = 0
    totssim = 0
    totcount = 0
    minpsnr = 1000
    maxpsnr = 0

    totl1depth = 0

    # Evaluate metrics on black background
    testbed.background_color = [0.0, 0.0, 0.0, 1.0]

    # Prior nerf papers don't typically do multi-sample anti aliasing.
    # So snap all pixels to the pixel centers.
    testbed.snap_to_pixel_centers = True
    spp = 8

    testbed.nerf.rendering_min_transmittance = 1e-4



    testbed.fov_axis = 0
    # testbed.fov = test_transforms["camera_angle_x"] * 180 / np.pi
    testbed.fov = to_angle(w, fl_x)
    # testbed.fov_xy = [to_angle(w, fl_x), to_angle(h, fl_y)]

    testbed.shall_train = False
    
    with tqdm(list(enumerate(datas)), unit="images", desc=f"Rendering test frame") as t:
        for ct, data in t:
            xform      = data["xform"]
            frame_path = data["frame_path"]
            depth_path = data["depth_path"]

            ref_image = read_image(frame_path)
            ref_depth = imageio.imread(depth_path) 
            ref_depth = np.asarray(ref_depth).astype(np.float32) / DepthMapFactor

            # NeRF blends with background colors in sRGB space, rather than first
            # transforming to linear space, blending there, and then converting back.
            # (See e.g. the PNG spec for more information on how the `alpha` channel
            # is always a linear quantity.)
            # The following lines of code reproduce NeRF's behavior (if enabled in
            # testbed) in order to make the numbers comparable.
            if testbed.color_space == ngp.ColorSpace.SRGB and ref_image.shape[2] == 4:
                # Since sRGB conversion is non-linear, alpha must be factored out of it
                ref_image[...,:3] = np.divide(ref_image[...,:3], ref_image[...,3:4], out=np.zeros_like(ref_image[...,:3]), where=ref_image[...,3:4] != 0)
                ref_image[...,:3] = linear_to_srgb(ref_image[...,:3])
                ref_image[...,:3] *= ref_image[...,3:4]
                ref_image += (1.0 - ref_image[...,3:4]) * testbed.background_color
                ref_image[...,:3] = srgb_to_linear(ref_image[...,:3])

            testbed.set_nerf_camera_matrix_from_slam(np.matrix(xform)[:-1,:])

            # ###############
            # render rgb #
            # ###############

            testbed.render_mode = ngp.Shade
            image = testbed.render(ref_image.shape[1], ref_image.shape[0], spp, True)

            # Debug code #
            if args.debug_show:
                f, axarr = plt.subplots(2,1) 
                axarr[0].imshow(image)
                axarr[1].imshow(ref_image)
                plt.show()

            if args.save_rgb:
                file_name = out_dir+"/frame/" + str(ct).zfill(5) + ".png"         
                write_image(file_name, image)

            A = np.clip(linear_to_srgb(image[...,:3]), 0.0, 1.0)
            R = np.clip(linear_to_srgb(ref_image[...,:3]), 0.0, 1.0)
            mse = float(compute_error("MSE", A, R))
            ssim = float(compute_error("SSIM", A, R))
            totssim += ssim
            totmse += mse
            psnr = mse2psnr(mse)
            totpsnr += psnr
            minpsnr = psnr if psnr<minpsnr else minpsnr
            maxpsnr = psnr if psnr>maxpsnr else maxpsnr

            # ###############
            # render depth #
            # ###############

            testbed.render_mode = ngp.Depth
            # The testbed has already considered the scale for depth
            depth = testbed.render(ref_depth.shape[1], ref_depth.shape[0], spp, True)

            # Debug code #
            if args.debug_show:
                f, axarr = plt.subplots(2,1) 
                axarr[0].imshow(depth[:,:,0], cmap="plasma")
                axarr[1].imshow(ref_depth, cmap="plasma")
                plt.show()

            l1depth = float(compute_error("MAE", depth[:,:,0], ref_depth))
            totl1depth += l1depth

            if args.save_depth:
                file_name = out_dir+"/depth/" + str(ct).zfill(5) + ".png"         
                plt.imsave(file_name, depth[:,:,0], cmap="plasma")

            t.set_postfix({"psnr": totpsnr/(totcount or 1), "l1depth(m)": totl1depth/(totcount or 1)})
            totcount = totcount+1


    psnr_avgmse = mse2psnr(totmse/(totcount or 1))
    psnr = totpsnr/(totcount or 1)
    ssim = totssim/(totcount or 1)
    l1depth = 100*totl1depth/(totcount or 1)
    print(f"PSNR={psnr} [min={minpsnr} max={maxpsnr}] SSIM={ssim} l1depth(cm)={l1depth}")


    with open(out_dir+'/output.txt', 'w') as f:
        f.write(f"PSNR={psnr} [min={minpsnr} max={maxpsnr}] SSIM={ssim} l1depth(cm)={l1depth}")
