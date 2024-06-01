import copy
import time
import numpy as np
import open3d as o3d
import teaserpp_python
import argparse

NOISE_BOUND = 0.05
N_OUTLIERS = 1700
OUTLIER_TRANSLATION_LB = 5
OUTLIER_TRANSLATION_UB = 10


def get_angular_error(R_exp, R_est):
    """
    Calculate angular error
    """
    return abs(np.arccos(min(max(((np.matmul(R_exp.T, R_est)).trace() - 1) / 2, -1.0), 1.0)));


if __name__ == "__main__":
    print("==================================================")
    print("        TEASER++ Python registration example      ")
    print("==================================================")

    parser = argparse.ArgumentParser(description="Run TEASER++ on two point clouds.")

    # Add arguments
    parser.add_argument("input1", type=str, help="Path to the first input point cloud file.")
    parser.add_argument("input2", type=str, help="Path to the second input point cloud file.")
    parser.add_argument("--output", type=str, help="Path to the output file to save results.", default=None)
    args = parser.parse_args()

    # Load bunny ply file
    src_cloud = o3d.io.read_point_cloud(args.input1)
    src = np.transpose(np.asarray(src_cloud.points))

    dst_cloud = o3d.io.read_point_cloud(args.input2)
    dst = np.transpose(np.asarray(dst_cloud.points))

    # Populating the parameters
    solver_params = teaserpp_python.RobustRegistrationSolver.Params()
    solver_params.cbar2 = 1
    solver_params.noise_bound = NOISE_BOUND
    solver_params.estimate_scaling = False
    solver_params.rotation_estimation_algorithm = teaserpp_python.RobustRegistrationSolver.ROTATION_ESTIMATION_ALGORITHM.GNC_TLS
    solver_params.rotation_gnc_factor = 5
    solver_params.rotation_max_iterations = 500
    solver_params.rotation_cost_threshold = 1e-30

    solver = teaserpp_python.RobustRegistrationSolver(solver_params)
    start = time.time()
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

    if args.output:
        with open(args.output, 'w') as file:
            file.write(str(solution.rotation) + "\n" + str(solution.translation))
    print("Time taken (s): ", end - start)

