import numpy as np
import cv2
from lib.pysixd_stuff import view_sampler
import math
from lib.meshrenderer import meshrenderer_phong
import random
from lib.pysixd.misc import calc_xyz_bp_fast


# ##############################################   config：start  ############################################
FOR_R = True
VIS = True
bbox_VIS = False
random_light = True
render_near = 0.1
render_far = 10000 # unit: should be mm

### intrinsic_matrix
K = np.array(  [[567.53720406, 0, 312.66570357],
              [0,569.36175922, 257.1729701],
              [0, 0, 1]])

# image size
IM_W, IM_H = 640, 480
## obj CAD path
ply_model_paths = [str('/home/sunh/github/Renders_for_6D/opengl_render/obj_000001.ply')]
max_rel_offset = 0.2  # used change the abs bbox
# ##############################################   config：end  ############################################

Renderer = meshrenderer_phong.Renderer(ply_model_paths,samples=1,vertex_scale=float(1)) # float(1) for some models

#  get the bbox from depth
def calc_2d_bbox(xs, ys, im_size):
    bbTL = (max(xs.min() - 1, 0),
            max(ys.min() - 1, 0))
    bbBR = (min(xs.max() + 1, im_size[0] - 1),
            min(ys.max() + 1, im_size[1] - 1))
    return [bbTL[0], bbTL[1], bbBR[0] - bbTL[0], bbBR[1] - bbTL[1]]

def angle2Rmat(theta):
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])

    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])

    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R

## sample poses
R1 = view_sampler.sample_views(100, radius=100,azimuth_range=(0,  2 * math.pi),elev_range=( 0, math.pi)    )
t_all = np.array([[0,0,900]])

for i in range(1):
    for idx in range(len(R1[0]) ):

        R = R1[0][idx]['R']
        t = t_all[i]

        bgr, depth = Renderer.render(
            obj_id=0,
            W=IM_W,
            H=IM_H,
            K=K.copy(),
            R=R,
            t=t,
            near=render_near,
            far=render_far,
            random_light=random_light
        )
        mask = (depth > 1e-8).astype('uint8')
        show_msk = (mask / mask.max() * 255).astype("uint8")

        xyz_np = calc_xyz_bp_fast(depth, R, t, K)

        # normalization the xyz map
        # "1": {"diameter": 102.099, "min_x": -37.9343, "min_y": -38.7996, "min_z": -45.8845, "size_x": 75.8686,
        #       "size_y": 77.5992, "size_z": 91.769},
        xyz_np[:,:,0] = ((xyz_np[:,:,0] + 37.9343) / 75.8686)
        xyz_np[:,:,1] =   ((xyz_np[:,:,1] + 38.7996) / 77.5992)
        xyz_np[:,:,2] =  ((xyz_np[:,:,2] + 45.8845) / 91.769)
        xyz_np[show_msk==0] = 0

        cv2.imshow('bgr', bgr)
        cv2.imshow('xyz_np', xyz_np)
        cv2.waitKey()














