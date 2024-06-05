import copy
import time
import numpy as np
import open3d as o3d
import teaserpp_python
import argparse
import multiprocessing as mp

NOISE_BOUND = 0.05
N_OUTLIERS = 1700
OUTLIER_TRANSLATION_LB = 5
OUTLIER_TRANSLATION_UB = 10


def get_angular_error(R_exp, R_est):
    """
    Calculate angular error
    """
    return abs(np.arccos(min(max(((np.matmul(R_exp.T, R_est)).trace() - 1) / 2, -1.0), 1.0)));


def python_teaser(queue: mp.Queue, output: str = ""):
    print("==================================================")
    print("        TEASER++ Python registration example      ")
    print("==================================================")

    # Load bunny ply file
    print("reading file 1")
    src_cloud = o3d.io.read_point_cloud("/input-data/src_pcd.ply")
    src = np.transpose(np.asarray(src_cloud.points))
    print("reading file 2")
    dst_cloud = o3d.io.read_point_cloud("/input-data/trg_pcd.ply")
    dst = np.transpose(np.asarray(dst_cloud.points))
    print("read files")
    # Populating the parameters
    print("setting solver params")
    solver_params = teaserpp_python.RobustRegistrationSolver.Params()
    solver_params.cbar2 = 1
    solver_params.noise_bound = NOISE_BOUND
    solver_params.estimate_scaling = False
    solver_params.rotation_estimation_algorithm = teaserpp_python.RobustRegistrationSolver.ROTATION_ESTIMATION_ALGORITHM.GNC_TLS
    solver_params.rotation_gnc_factor = 5
    solver_params.rotation_max_iterations = 500
    solver_params.rotation_cost_threshold = 1e-30

    print("starting solver")
    solver = teaserpp_python.RobustRegistrationSolver(solver_params)
    start = time.time()
    print("solving")
    solver.solve(src, dst)
    end = time.time()

    solution = solver.getSolution()

    transformation = np.hstack((solution.rotation, (solution.translation).reshape(-1, 1)))
    transformation = np.concatenate((transformation, ((np.array([0, 0, 0, 1])).reshape(-1, 1)).T), axis=0)

    src_cloud.transform(transformation)


    print("=====================================")
    print("          TEASER++ Results           ")
    print("=====================================")

    print("Estimated rotation: ")
    print(solution.rotation)

    print("Estimated translation: ")
    print(solution.translation)

    if output:
        with open(output, 'w') as file:
            file.write(str(solution.rotation) + "\n" + str(solution.translation))
    print("Time taken (s): ", end - start)

    queue.put(solution)

